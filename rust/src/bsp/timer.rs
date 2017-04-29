#![allow(dead_code)]

use reg;

#[repr(u32)]
pub enum Timer {
    TIM1 = reg::TIM1_BASE,
    TIM2 = reg::TIM2_BASE,
    TIM3 = reg::TIM3_BASE,
    TIM4 = reg::TIM4_BASE,
    TIM5 = reg::TIM5_BASE,
    TIM6 = reg::TIM6_BASE,
    TIM7 = reg::TIM7_BASE,
    TIM8 = reg::TIM8_BASE,
    TIM9 = reg::TIM9_BASE,
    TIM10 = reg::TIM10_BASE,
    TIM11 = reg::TIM11_BASE,
    TIM12 = reg::TIM12_BASE,
    TIM13 = reg::TIM13_BASE,
    TIM14 = reg::TIM14_BASE
}

pub enum PWMState {
    On = 1,
    Off = 0
}

#[repr(u32)]
pub enum Channel {
    Channel1 = TIM_CHANNEL_1,
    Channel2 = TIM_CHANNEL_2,
    Channel3 = TIM_CHANNEL_3,
    Channel4 = TIM_CHANNEL_4
}

extern {
    // fn GPIOPinTypeGPIOOutput(base: *const u32, mask: u32);
    fn HAL_TIM_Base_Start(base: *const u32);
    fn HAL_TIM_PWM_Start(base: *const u32, mask: u32);
}

// pub fn make_output(port: Port, pin: Pin) {
//     let mask = pin as u32;
//     let base = port as u32;
//     unsafe {
//         GPIOPinTypeGPIOOutput(base as *const u32, mask);
//     }
// }

pub fn set(port: Port, pin: Pin, value: PinState) {
    let base = port as u32;
    let shifted_val = pin as u32;
    unsafe {
        HAL_GPIO_WritePin(base as *const u32, shifted_val, value as u32);
    }
}