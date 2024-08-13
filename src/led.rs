#![no_std]
#![no_main]

use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Timer};
use heapless::String;

const MSBFIRST: bool = true;  // High bit first
const LSBFIRST: bool = false; // Low bit first

// 7-segment display encoding
const TABLE: [u8; 26] = [
    0x3f, 0x06, 0x5b, 0x4f, // 0123
    0x66, 0x6d, 0x7d, 0x07, // 4567
    0x7f, 0x6f, 0x77, 0x7c, // 89Ab
    0x39, 0x5e, 0x79, 0x71, // C dE F
    0x76, 0x38, 0x37, 0x3E, // H L N U
    0x73, 0x5C, 0x40, 0x6B, 0x6A, 0x72 // P o - &@$  
];

fn buf_7seg_c(temp: char, has_dot: bool) -> u8 {
    let index = "0123456789AbCdEFHLNUPo-&@$".find(temp).unwrap_or(0);
    let mut buf = TABLE[index];
    buf = !buf;
    if has_dot {
        buf &= 0x7f;
    }
    buf
}

fn shift_out(data_pin: &mut Output, clock_pin: &mut Output, bit_order: bool, value: u8) {
    for i in (0..8).rev() {
        let bit = if bit_order {
            (value >> i) & 1
        } else {
            (value >> (7 - i)) & 1
        };
        data_pin.set_level(Level::from(bit != 0));
        clock_pin.set_level(Level::High);
        clock_pin.set_level(Level::Low);
    }
}

async fn ledshow_big(data_pin: &mut Output<'_>, clock_pin: &mut Output<'_>, latch_pin: &mut Output<'_>, s: &str) {
    let mut has_dot = false;
    for chr in s.chars().rev() {
        if chr == '.' {
            has_dot = true;
            continue;
        }
        shift_out(data_pin, clock_pin, MSBFIRST, buf_7seg_c(chr, has_dot));
        has_dot = false;
    }
    latch_pin.set_level(Level::High);
    Timer::after(Duration::from_millis(10)).await;
    latch_pin.set_level(Level::Low);
}

pub fn space_pad(text: &str, length: usize) -> String<20>  {
    let mut result = String::<20>::new();

    // 计算忽略小数点后的长度
    let mut actual_length = 0;
    for c in text.chars() {
        if c != '.' {
            actual_length += 1;
        }
    }
    // 根据计算的长度在前面追加空格
    while actual_length < length {
        result.push(' ').unwrap();
        actual_length += 1;
    }
    // 将 text 追加到尾部
    result.push_str(text).unwrap();

    result
}


async fn ledshow(data_pin: &mut Output<'_>, clock_pin: &mut Output<'_>, latch_pin: &mut Output<'_>, str: &str) {
    let padded_str = space_pad(str, 4);
    ledshow_big(data_pin, clock_pin, latch_pin, &padded_str).await;
}

pub struct Led {
    pub(crate) data_pin: Output<'static>,
    pub(crate) clock_pin: Output<'static>,
    pub(crate) latch_pin: Output<'static>,
}

impl Led {
    pub async fn show(&mut self, str: &str) {
        ledshow(&mut self.data_pin, &mut self.clock_pin, &mut self.latch_pin, str).await;
    }
}
