#![no_std]
#![no_main]

use panic_halt as _;

use adf4158::prelude::*;
use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use longan_nano::hal::{delay::McycleDelay, pac, prelude::*, rcu::Clocks};
use riscv_rt::entry;

struct Adf4158<Clk, Data, Le> {
    clk: Clk,
    data: Data,
    le: Le,
    delay: McycleDelay,
    delay_us: u32,
}

macro_rules! adf4158_pins {
    ($clk:expr, $data:expr, $le:expr, $rcu:expr) => {
        Adf4158::new(
            $clk.into_push_pull_output(),
            $data.into_push_pull_output(),
            $le.into_push_pull_output(),
            &$rcu.clocks,
        )
    };
}

impl<Clk, Data, Le> Adf4158<Clk, Data, Le> {
    fn new(clk: Clk, data: Data, le: Le, clocks: &Clocks) -> Self {
        let delay = McycleDelay::new(clocks);
        let delay_us = 10;
        Self {
            clk,
            data,
            le,
            delay,
            delay_us,
        }
    }
}

impl<Clk, Data, Le> RegisterWrite for Adf4158<Clk, Data, Le>
where
    Clk: OutputPin<Error = Infallible>,
    Data: OutputPin<Error = Infallible>,
    Le: OutputPin<Error = Infallible>,
{
    fn write(&mut self, data: u32) {
        self.le.set_low().unwrap();
        for j in (0..32).rev() {
            self.clk.set_low().unwrap();
            self.data.set_state(((data >> j) & 1 != 0).into()).unwrap();
            self.delay.delay_us(self.delay_us);
            self.clk.set_high().unwrap();
            self.delay.delay_us(self.delay_us);
        }
        self.clk.set_low().unwrap();
        self.le.set_high().unwrap();
        self.delay.delay_us(self.delay_us);
        self.le.set_low().unwrap();
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
enum ConfigMode {
    /// CW carrier at 4.35 GHz.
    CW,
    /// Sawtooth covering 4.2 - 4.45 GHz in 60 usec.
    Sawtooth,
    /// Triangle covering 4.2 - 4.45 GHz in 60 usec.
    Triangle,
}

impl ConfigMode {
    fn adf4158_config(&self) -> Adf4158Config {
        let ramp_on = !matches!(self, ConfigMode::CW);

        // 10 MHz reference with doubler: 20 MHz PFD frequency
        let (int, frac) = match self {
            ConfigMode::CW => (217, 1 << 24),       // 4.35 GHz
            ConfigMode::Sawtooth => (210, 0),       // 4.2 GHz
            ConfigMode::Triangle => (210, 1 << 23), // 4.205 GHz
        };

        let ramp_mode = match self {
            ConfigMode::CW | ConfigMode::Sawtooth => RampMode::ContinuousSawtooth,
            ConfigMode::Triangle => RampMode::ContinuousTriangular,
        };

        let step_word1 = 1024;
        let dev_offset_word1 = 4;
        let deviation_word1 = match self {
            ConfigMode::CW => 0,
            ConfigMode::Sawtooth => 25600,
            ConfigMode::Triangle => 24576,
        };

        let ramp_delay = matches!(self, ConfigMode::Sawtooth);

        Adf4158Config {
            ramp_on,
            muxout_control: MuxoutControl::ReadbackToMux,
            int,
            frac,
            reference_doubler: true,
            ramp_mode,
            readback_to_muxout: ReadbackToMuxout::RampComplete,
            clock_divider_mode: ClkDivMode::RampDivider,
            dev_offset_word1,
            deviation_word1,
            step_word1,
            ramp_delay,
            delay_start_word: 200,
            ..Default::default()
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp.RCU.configure().freeze();

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);
    let mut adf4158 = adf4158_pins!(gpioc.pc13, gpioa.pa1, gpioa.pa2, rcu);
    let cw_en = gpiob.pb1.into_pull_up_input();
    let sawtooth_en = gpiob.pb10.into_pull_up_input();
    let triangle_en = gpiob.pb11.into_pull_up_input();

    let default_mode = ConfigMode::Triangle;
    default_mode.adf4158_config().write(&mut adf4158);

    let mut pressed_enable = None;
    loop {
        let new_pressed_enable = if cw_en.is_low().unwrap() {
            Some(ConfigMode::CW)
        } else if sawtooth_en.is_low().unwrap() {
            Some(ConfigMode::Sawtooth)
        } else if triangle_en.is_low().unwrap() {
            Some(ConfigMode::Triangle)
        } else {
            None
        };
        if let Some(new) = new_pressed_enable
            && pressed_enable != new_pressed_enable
        {
            new.adf4158_config().write(&mut adf4158);
        }
        pressed_enable = new_pressed_enable;
    }
}
