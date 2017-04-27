#![feature(lang_items, core_intrinsics)]
#![feature(start)]
#![feature(asm)]
#![feature(lang_items)]
#![feature(macro_reexport)]
#![feature(naked_functions)]
#![no_std]

#![crate_type="staticlib"]

pub mod bsp;
pub mod reg;

use core::intrinsics;
use bsp::gpio;

extern crate spin;
use spin::Mutex;

#[macro_use]
extern crate cortex_m;

pub extern crate stm32f427x as peripheral;


// pub static StateWriter: Mutex<i32> = Mutex::new(i32);

static mut state: i32 = 0;

// Entry point for this program.
#[no_mangle] // ensure that this symbol is called `main` in the output
pub extern "C" fn rust_main(_argc: i32, _argv: *const *const u8) -> i32 {
    unsafe {
        if state == 0 {
            gpio::writePin(gpio::Port::PortE, gpio::Pin::Pin7, gpio::PinState::Set);
            gpio::writePin(gpio::Port::PortF, gpio::Pin::Pin14, gpio::PinState::Set);
            unsafe {
                state = 1;
            }
        } else {
            gpio::writePin(gpio::Port::PortE, gpio::Pin::Pin7, gpio::PinState::Reset);
            gpio::writePin(gpio::Port::PortF, gpio::Pin::Pin14, gpio::PinState::Reset);
            unsafe {
                state = 0;
            }
        }
    }
    1344
}

// These functions are used by the compiler, but not
// for a bare-bones hello world. These are normally
// provided by libstd.
#[lang = "eh_personality"]
#[no_mangle]
pub extern "C" fn rust_eh_personality() {}

// This function may be needed based on the compilation target.
#[lang = "eh_unwind_resume"]
#[no_mangle]
pub extern "C" fn rust_eh_unwind_resume() {}

#[lang = "panic_fmt"]
#[no_mangle]
pub extern fn panic_fmt(fmt: core::fmt::Arguments, file: &'static str,
    line: u32) -> !
{
    // iprintln!("\n\nPANIC in {} at line {}:", file, line);
    // iprintln!("    {}", fmt);
    loop{}
}