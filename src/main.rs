#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

mod led;

use core::any::Any;
use core::str::FromStr;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::{join3};
use embassy_futures::select::{select4, Either4};
use embassy_net::{Config, Ipv4Address, Stack, StackResources};
use embassy_net::tcp::{TcpReader, TcpSocket};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_io_async::Write;
use rand::RngCore;
use static_cell::StaticCell;
use crate::led::Led;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// 应用配置参数
const WIFI_NETWORK: &str = "EXKIDS";
const WIFI_PASSWORD: &str = "tb-yk-zk!";
const SERVER_PORT: u16 = 10080;
// const SERVER_ADDR: &str = "10.189.0.11";
const SERVER_ADDR: &str = "10.189.15.230";

const MY_CLIENT: i32 = 1;
static mut MY_GROUP: i32 = 1;

// 初始化状态
static mut NOW_GROUP: i32 = 1;
static mut NOW_DING: bool = true;
static mut LED: Option<Led> = None;

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
    let p = embassy_rp::init(Default::default());

    /////初始化控制器
    let ding = Output::new(p.PIN_17, Level::Low);
    // controller
    let mut ping = Input::new(p.PIN_16, embassy_rp::gpio::Pull::Up);
    let mut g_up = Input::new(p.PIN_18, embassy_rp::gpio::Pull::Up);
    let mut g_down = Input::new(p.PIN_19, embassy_rp::gpio::Pull::Up);
    let mut g_reset = Input::new(p.PIN_20, embassy_rp::gpio::Pull::Up);
    // LED
    let dio = Output::new(p.PIN_11, Level::Low);    //数据引脚 SDI
    let rclk = Output::new(p.PIN_12, Level::Low);   //时钟引脚 SCLK
    let sclk = Output::new(p.PIN_13, Level::Low);   //锁存引脚 LOAD
    unsafe { LED = Some(Led { data_pin: dio, clock_pin: rclk, latch_pin: sclk }); } // 初始化LED
    
    
    unsafe { LED.as_mut().unwrap().show("0.0.0.0").await; } //启动中
    //// 初始化网络
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

    unsafe { LED.as_mut().unwrap().show(" . . .0").await; } //连接WiFi
    loop {
        //control.join_open(WIFI_NETWORK).await;
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    unsafe { LED.as_mut().unwrap().show(" . .0.8").await; } //获取IP
    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    let my_ip = stack.config_v4();
    info!("DHCP IP: {:?}", my_ip.unwrap().address);


    unsafe { LED.as_mut().unwrap().show(" .0.8.8").await; } //尝试接入
    /////
    let mut rx_buffer = [0; 512];
    let mut tx_buffer = [0; 512];
    let mut socket: TcpSocket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    info!("try connect...");
    loop {
        let mut err_count = 0;
        match socket.connect((Ipv4Address::from_str(SERVER_ADDR).unwrap(), SERVER_PORT)).await {
            Err(e) => {
                warn!("connect error: {:?}", e);
                err_count += 1;
                if (err_count < 30) {
                    warn!("wait time reconnect");
                    Timer::after(Duration::from_secs(10)).await;
                } else {
                    //reset to reboot
                    cortex_m::peripheral::SCB::sys_reset();
                }
            }
            Ok(_) => {
                break;
            }
        }
    }

    unsafe { LED.as_mut().unwrap().show("8.8.8.8").await; } //启动完成

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_millis(8_300));

    // split reader\write
    let (reader, writer_source) = socket.split();

    let writer = Mutex::<NoopRawMutex, _>::from(writer_source);

    // 控制器实现
    let key_control = async {
        loop {
            match select4(key_button(&mut ping), key_button(&mut g_up), key_button(&mut g_down), key_button(&mut g_reset)).await {
                Either4::First(state) => {
                    let mut w = writer.lock().await;
                    if state {
                        w.write_all(&[0xFF, 0x02, 0x00, 0x01, 0x01, 0x00]).await.expect("send fail");
                    } else {
                        w.write_all(&[0xFF, 0x02, 0x00, 0x01, 0x00, 0x00]).await.expect("send fail");
                    }
                }
                Either4::Second(state) => unsafe {
                    if state {
                        let mut w = writer.lock().await;
                        let tg = (NOW_GROUP + 1) as u8;
                        w.write_all(&[0xFF, 0x01, 0x00, 0x01, tg, 0x00]).await.expect("send fail");
                    }
                }
                Either4::Third(state) => unsafe {
                    if state {
                        let mut w = writer.lock().await;
                        let tg = (NOW_GROUP + 1) as u8;
                        w.write_all(&[0xFF, 0x01, 0x00, 0x01, tg, 0x00]).await.expect("send fail");
                    }
                }
                Either4::Fourth(state) => unsafe {
                    if state {
                        let mut w = writer.lock().await;
                        let tg = MY_GROUP as u8;
                        w.write_all(&[0xFF, 0x01, 0x00, 0x01, tg, 0x00]).await.expect("send fail");
                    }
                }
            };
        }
    };

    let ping_timeout = async {
        loop {
            Timer::after(Duration::from_millis(5_000)).await;
            let mut w = writer.lock().await;
            w.write_all(&[0xFF, 0x09, 0x00, 0x01, 0x01, 0x00]).await.expect("send fail");
        }
    };

    join3(key_control, ping_timeout, client_recv(reader, ding, watchdog)).await;
}

async fn key_button(ping: &mut Input<'_>) -> bool {
    let mut last_state = ping.is_low();
    loop {
        ping.wait_for_any_edge().await;
        Timer::after(Duration::from_millis(50)).await; // 等待防抖动延迟
        if ping.is_low() == last_state {
            continue;
        }
        if ping.is_low() {
            last_state = true;
        } else {
            last_state = false;
        }
        return last_state;
    }
}

/** 收报代码 */
async fn client_recv(mut reader: TcpReader<'_>, mut ding: Output<'_>, mut watchdog: Watchdog) {
    let mut buf = [0; 256];
    loop {
        match reader.read(&mut buf).await {
            Ok(0) => {
                warn!("read EOF");
                break;
            }
            Ok(recv_len) => {
                // 收到过消息就feed watchdog
                watchdog.feed();
                // 处理消息
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
                            NOW_GROUP = body[0] as i32;
                            shuffle_led().await;
                        }
                        2 => unsafe { // 发报
                            if NOW_DING {
                                if body[0] == 0 {
                                    ding.set_low()
                                } else {
                                    ding.set_high()
                                }
                            }
                        }
                        8 => unsafe { // 工作模式
                            NOW_DING = body[0] == 1;
                            shuffle_led().await;
                        }
                        9 => {}
                        _ => {}
                    }

                    offset += chunk_len;
                }
            }
            Err(e) => {
                warn!("read error: {:?}", e);
                // 这里可能要重启，由于break了不再接收，一会watchdog会重启
                break;
            }
        };
    }
}

async unsafe fn shuffle_led()  {
    LED.as_mut().unwrap().show("8.8.8.8").await;
}