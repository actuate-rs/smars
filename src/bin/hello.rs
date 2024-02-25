#![no_main]
#![no_std]

extern crate alloc;

use actuate::Diagram;

use embedded_alloc::Heap;
use smars as _; // global logger + panicking-behavior + memory layout
use stm32f1xx_hal::{
    flash::FlashExt,
    gpio::{GpioExt, Output, Pin},
    pac::{self},
    rcc::RccExt,
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

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 2048;
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

    let mut diagram = Diagram::builder()
        .add_input(Time(0))
        .add_state(Epoch(timer.now()))
        .add_system(time_system)
        .add_state(Led(led))
        .add_state(Blink { last_time: 0 })
        .add_system(blink)
        .build();

    loop {
        diagram.run();
    }
}
