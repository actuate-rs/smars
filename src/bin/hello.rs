#![no_main]
#![no_std]

extern crate alloc;

use actuate::Diagram;

use alloc::vec::Vec;
use embedded_alloc::Heap;
use serde::{Deserialize, Serialize};
use smars as _; // global logger + panicking-behavior + memory layout
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Alternate, GpioExt, Output, Pin},
    pac::{self, USART3},
    prelude::*,
    rcc::RccExt,
    serial::{self, Serial},
    time::{Instant, MonoTimer},
};

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub struct Time(u32);

struct Epoch(Instant);

fn time_system(Time(time): &mut Time, Epoch(epoch): &Epoch) {
    *time = epoch.elapsed();
}

struct Led(Pin<'C', 13, Output>);

struct Blink {
    last_time: u32,
}

fn blink(Led(led): &mut Led, Time(time): &Time, blink: &mut Blink) {
    if *time - blink.last_time > 10000000 {
        blink.last_time = *time;

        defmt::println!("Blink! t={}", time);
        led.toggle();
    }
}

struct ControllerSerial(Serial<USART3, (Pin<'B', 10, Alternate>, Pin<'B', 11>)>);

#[derive(Default)]
struct Buffer(Vec<u8>);

#[derive(Deserialize)]
struct Command {
    throttle: f64,
}

#[derive(Default)]
struct State {
    left_throttle: f64,
    right_throttle: f64,
}

fn controller(
    ControllerSerial(serial): &mut ControllerSerial,
    Buffer(buf): &mut Buffer,
    state: &mut State,
) {
    let r: Result<u8, _> = serial.read();
    if let Ok(b) = r {
        buf.push(b);
    }

    if let Ok(msg) = postcard::from_bytes::<Command>(buf) {
        state.left_throttle = msg.throttle;
        state.right_throttle = msg.throttle;

        buf.clear();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 10000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut gpioc = dp.GPIOC.split();

    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let timer = MonoTimer::new(cp.DWT, cp.DCB, clocks);

    // USART3
    let mut gpiob = dp.GPIOB.split();
    // Configure pb10 as a push_pull output, this will be the tx pin
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // Take ownership over pb11
    let rx = gpiob.pb11;

    // Set up the usart device. Take ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let mut afio = dp.AFIO.constrain();
    let mut serial = Serial::new(
        dp.USART3,
        (tx, rx),
        &mut afio.mapr,
        serial::Config::default().baudrate(9600.bps()),
        &clocks,
    );

    let mut diagram = Diagram::builder()
        .add_input(Time(0))
        .add_state(Epoch(timer.now()))
        .add_system(time_system)
        .add_state(Led(led))
        .add_state(Blink { last_time: 0 })
        .add_state(ControllerSerial(serial))
        .add_state(Buffer::default())
        .add_state(State::default())
        .add_system(blink)
        .add_system(controller)
        .build();

    loop {
        diagram.run();
    }
}
