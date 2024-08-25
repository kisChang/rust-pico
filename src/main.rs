#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]
mod led;
use crate::led::Led;
use core::str::FromStr;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::join3;
use embassy_futures::select::{select4, Either4};
use embassy_net::tcp::{TcpReader, TcpSocket};
use embassy_net::{Config, Ipv4Address, Stack, StackResources};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::multicore::spawn_core1;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::flash::{Async, ERASE_SIZE, FLASH_BASE};
use embassy_rp::pio::Pio;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use heapless::String;
use rand::RngCore;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct Irqusb {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

// 应用配置参数
const WIFI_NETWORK: &str = "EXKIDS";
const WIFI_PASSWORD: &str = "tb-yk-zk!";
const MY_CLIENT: i32 = 1;

////// 固定算出的常量
static mut MY_GROUP: i32 = 1;

const SERVER_PORT: u16 = 10080;
// 板内写入信息
const ADDR_OFFSET: u32 = 0x100000;
const FLASH_SIZE: usize = 2 * 1024 * 1024;
// 初始化状态
static mut NOW_GROUP: i32 = 1;
static mut NOW_DING: bool = true;
static LED_CHANNEL: Channel<ThreadModeRawMutex, &str, 64> = Channel::new();

static mut CORE1_STACK: embassy_rp::multicore::Stack<4096> = embassy_rp::multicore::Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[cfg(debug_assertions)]
fn get_firmware() -> &'static [u8] {
    unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) }
}
#[cfg(debug_assertions)]
fn get_firmware_clm() -> &'static [u8] {
    unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) }
}

#[cfg(not(debug_assertions))]
fn get_firmware() -> &'static [u8] {
    include_bytes!("../src/lib/cyw43-firmware/43439A0.bin")
}
#[cfg(not(debug_assertions))]
fn get_firmware_clm() -> &'static [u8] {
    include_bytes!("../src/lib/cyw43-firmware/43439A0_clm.bin")
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //加载配置信息
    unsafe {
        MY_GROUP = (MY_CLIENT + 1) / 2;
    }
    // TODO WebUSB 方式提供配置管理功能
    // let driver = embassy_rp::usb::Driver::new(p.USB, Irqusb);
    // TODO flash 的方式存储配置信息
    // let mut flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH0);


    /////初始化控制器
    let ding = Output::new(p.PIN_17, Level::Low);
    // controller
    let mut work_led = Output::new(p.PIN_2, Level::Low);
    let mut ping = Input::new(p.PIN_14, embassy_rp::gpio::Pull::Up);
    let mut g_up = Input::new(p.PIN_18, embassy_rp::gpio::Pull::Up);
    let mut g_down = Input::new(p.PIN_19, embassy_rp::gpio::Pull::Up);
    let mut g_reset = Input::new(p.PIN_20, embassy_rp::gpio::Pull::Up);
    // LED
    let dio = Output::new(p.PIN_11, Level::Low);    //数据引脚 SDI
    let rclk = Output::new(p.PIN_12, Level::Low);   //时钟引脚 SCLK
    let sclk = Output::new(p.PIN_13, Level::Low);   //锁存引脚 LOAD
    let mut led = Led { data_pin: dio, clock_pin: rclk, latch_pin: sclk }; // 初始化LED

    led.show("0.0.0.0").await; //启动中
    //// 初始化网络
    let mut rng = RoscRng;
    let fw = get_firmware();
    let clm = get_firmware_clm();

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

    control.gpio_set(0, true).await;
    work_led.set_high();
    led.show(" . . .0").await; //连接WiFi
    loop {
        //control.join_open(WIFI_NETWORK).await;
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    led.show(" . .0.8").await; //获取IP
    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    let my_ip = stack.config_v4();
    info!("DHCP IP: {:?}", my_ip.unwrap().address);


    led.show(" .0.8.8").await; //尝试寻找服务器IP
    let server_addr: &str;
    let mut buf_udp = [0; 50];
    {
        let mut rx_buffer = [0; 50];
        let mut tx_buffer = [0; 50];
        let mut rx_meta = [PacketMetadata::EMPTY; 16];
        let mut tx_meta = [PacketMetadata::EMPTY; 16];

        let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
        socket.bind(16667).unwrap();
        loop {
            // 给 255.255.255.255:16666 发送消息 telegram
            socket.send_to("telegram".as_bytes(), (Ipv4Address::from_str("255.255.255.255").unwrap(), 16666)).await.unwrap();
            // 等待回音
            let (n, ep) = socket.recv_from(&mut buf_udp).await.unwrap();
            if let Ok(s) = core::str::from_utf8(&buf_udp[..n]) {
                info!("Telegram rxd from {}: {}", ep, s);
                server_addr = s;
                break;
            }
        }
    }

    led.show("0.8.8.8").await; //尝试接入
    /////
    let mut rx_buffer = [0; 512];
    let mut tx_buffer = [0; 512];
    let mut socket: TcpSocket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    info!("try connect: {}", server_addr);
    loop {
        let mut err_count = 0;
        match socket.connect((Ipv4Address::from_str(server_addr).unwrap(), SERVER_PORT)).await {
            Err(e) => {
                warn!("connect error: {:?}", e);
                err_count += 1;
                if err_count < 30 {
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

    led.show("8.8.8.8").await; //启动完成

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_millis(8_300));

    // split reader\write
    let (reader, mut writer_source) = socket.split();

    // init join client
    writer_source.write_all(&[0xFF, 0x00, 0x00, 0x01, MY_CLIENT as u8, 0x00]).await.expect("send fail");
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
                        let mut tg = (NOW_GROUP + 1) as u8;
                        if tg >= 99 {
                            tg = 1
                        }
                        w.write_all(&[0xFF, 0x01, 0x00, 0x01, tg, 0x00]).await.expect("send fail");
                    }
                }
                Either4::Third(state) => unsafe {
                    if state {
                        let mut w = writer.lock().await;
                        let mut tg = (NOW_GROUP - 1) as u8;
                        if tg <= 0 {
                            tg = 99
                        }
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
        let mut state = false;
        let mut count = 0;
        loop {
            control.gpio_set(0, state).await;
            state = !state;
            work_led.toggle();
            Timer::after(Duration::from_millis(1_000)).await;
            count = count + 1;
            if count >= 5 { // 5次执行一轮上报
                count = 0;
                let mut w = writer.lock().await;
                w.write_all(&[0xFF, 0x09, 0x00, 0x01, 0x01, 0x00]).await.expect("send fail");
            }
        }
    };

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| unwrap!(spawner.spawn(shuffle_led(led))));
        },
    );

    join3(key_control, ping_timeout, client_recv(reader, ding, watchdog, LED_CHANNEL.sender())).await;
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
async fn client_recv(
    mut reader: TcpReader<'_>,
    mut ding: Output<'_>,
    mut watchdog: Watchdog,
    sender: Sender<'_, ThreadModeRawMutex, &str, 64>
) {
    // 先发一次sender,刷新LED
    sender.send("").await;
    // 正式进入收报代码
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
                            sender.send("").await;
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
                            sender.send("").await;
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

#[embassy_executor::task]
async fn shuffle_led(mut led: Led) {
    loop {
        match LED_CHANNEL.receive().await {
            _ => unsafe {
                if NOW_GROUP == 0 {
                    led.show("LoAd").await;
                } else {
                    let mut str: String<5> = String::try_from(NOW_GROUP).unwrap();
                    if MY_GROUP == NOW_GROUP {
                        str.push_str(" @").unwrap();
                    } else {
                        str.push_str(" $").unwrap();
                    }
                    info!("show: {}", str);
                    led.show(&str).await;
                }
                // info!("show: {}", str);
                // led.show(str).await;
            }
        }
    }
}