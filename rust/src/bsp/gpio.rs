#![allow(dead_code)]

use reg;

#[repr(u32)]
pub enum Port {
    PortE = reg::GPIOE_BASE,
    PortF = reg::GPIOF_BASE,
}

pub enum PinState {
    Reset = 0,
    Set = 1
}

#[repr(u32)]
pub enum Pin {
    Pin0 = (1 << 0),
    Pin1 = (1 << 1),
    Pin2 = (1 << 2),
    Pin3 = (1 << 3),
    Pin4 = (1 << 4),
    Pin5 = (1 << 5),
    Pin6 = (1 << 6),
    Pin7 = (1 << 7),
    Pin8 = (1 << 8),
    Pin9 = (1 << 9),
    Pin10 = (1 << 10),
    Pin11 = (1 << 11),
    Pin12 = (1 << 12),
    Pin13 = (1 << 13),
    Pin14 = (1 << 14),
    Pin15 = (1 << 15),
    Pin16 = (1 << 16),
    Pin17 = (1 << 17),
    Pin18 = (1 << 18),
    Pin19 = (1 << 19),
    Pin20 = (1 << 20),
}

extern {
    // fn GPIOPinTypeGPIOOutput(base: *const u32, mask: u32);
    fn HAL_GPIO_WritePin(base: *const u32, mask: u32, value: u32);
}

// pub fn make_output(port: Port, pin: Pin) {
//     let mask = pin as u32;
//     let base = port as u32;
//     unsafe {
//         GPIOPinTypeGPIOOutput(base as *const u32, mask);
//     }
// }

pub fn writePin(port: Port, pin: Pin, value: PinState) {
    let base = port as u32;
    let shifted_val = pin as u32;
    unsafe {
        HAL_GPIO_WritePin(base as *const u32, shifted_val, value as u32);
    }
}