#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::fmt::Debug;
use core::str::{from_utf8, FromStr};

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_net::{Config, Ipv4Address, Stack, StackResources};
use embassy_net::dns::Socket;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_rp::{bind_interrupts, Peripherals};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::{ThreadModeRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_io_async::Write;
use rand::RngCore;
use static_cell::StaticCell;
use trouble_host::Address;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// 应用配置参数
const WIFI_NETWORK: &str = "EXKIDS";
const WIFI_PASSWORD: &str = "tb-yk-zk!";


/////初始化状态
static mut MY_CLIENT: i32 = 1;
static mut MY_GROUP: i32 = 1;
static mut NOW_GROUP: i32 = 1;
static mut NOW_DING: bool = true;
/////

static mut CORE1_STACK: embassy_rp::multicore::Stack<4096> = embassy_rp::multicore::Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let server_port: u16 = 1234;
    let server_addr: Ipv4Address = Ipv4Address::from_str("10.189.15.230").unwrap();

    let mut p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;
    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi: PioSpi<PIO0, 0, DMA_CH0> = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    let config = Config::dhcpv4(Default::default());

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    loop {
        //control.join_open(WIFI_NETWORK).await;
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    let my_ip = stack.config_v4();
    info!("DHCP IP: {:?}", my_ip.unwrap().address);


    /////初始化控制器
    let mut ding = Output::new(p.PIN_17, Level::Low);
    let mut ping = Input::new(p.PIN_16, embassy_rp::gpio::Pull::Up);
    // let g_up = Input::new(&p.PIN_16, embassy_rp::gpio::Pull::Down);
    // let g_down = Input::new(&p.PIN_16, embassy_rp::gpio::Pull::Down);
    // let g_reset = Input::new(&p.PIN_16, embassy_rp::gpio::Pull::Down);
    /////
    let mut rx_buffer = [0; 512];
    let mut tx_buffer = [0; 512];


    let mut socket: TcpSocket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    info!("try connect...");
    if let Err(e) = socket.connect((server_addr, server_port)).await {
        warn!("connect error: {:?}", e);
        return;
    }

    let socket = Mutex::<NoopRawMutex, _>::from(socket);

    let key_control = async {
        let mut last_state = ping.is_low();
        loop {
            ping.wait_for_any_edge().await;
            if ping.is_low() == last_state {
                continue;
            }
            let mut socket = socket.lock().await;
            if ping.is_low() {
                info!("low");
                last_state = true;
                socket.write_all(&[0xFF, 0x02, 0x00, 0x01, 0x00, 0x00]).await.expect("send fail");
            } else {
                info!("high");
                last_state = false;
                socket.write_all(&[0xFF, 0x02, 0x00, 0x01, 0x01, 0x00]).await.expect("send fail");
            }
        }
    };

    /** 收报代码 */
    let client_recv = async {
        let mut buf = [0; 4096];
        let mut no_recv = false;
        loop {
            if no_recv {
                Timer::after(Duration::from_millis(1)).await;
                no_recv = false;
            }
            let mut socket = socket.lock().await;
            if !socket.can_recv() {
                no_recv = true;
                continue;
            }
            match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(recv_len) => {
                    if recv_len < 5 {
                        continue;
                    }
                    let mut offset = 0;
                    while offset < recv_len {
                        let chunk_len = (recv_len - offset).min(6);
                        let packet = &buf[offset..offset + chunk_len];

                        if chunk_len < 5 {
                            break; //结束处理
                        }

                        // 处理固定长度的 packet
                        if packet[0] != 0xFF {
                            offset += 1; // 跳过1个字节
                            continue;
                        }
                        let command = packet[1];
                        let body_length = ((packet[2] << 2) + packet[3]) as usize;
                        let body = &packet[4..(4 + body_length)];
                        match command {
                            1 => unsafe { // 切换分组
                                NOW_GROUP = body[0] as i32
                            }
                            2 => { // 发报
                                if body[0] == 0 {
                                    ding.set_low()
                                } else {
                                    ding.set_high()
                                }
                            }
                            8 => unsafe { // 工作模式
                                NOW_DING = body[0] == 1
                            }
                            9 => {} // Ping
                            _ => {}
                        }

                        offset += chunk_len;
                    }
                }
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };
        }
    };


    // embassy_rp::multicore::spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
    //     let executor1 = EXECUTOR1.init(Executor::new());
    //     executor1.run(|spawner| spawner.spawn(key_control(ping, &socket)).unwrap());
    // });
    // unwrap!(spawner.spawn(key_control));

    join(key_control, client_recv).await;
}
