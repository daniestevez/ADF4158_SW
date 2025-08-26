#![no_std]
#![no_main]

use panic_halt as _;

use adf4158::prelude::*;
use core::convert::Infallible;
use embedded_hal::digital::v2::OutputPin;
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

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp.RCU.configure().freeze();

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);
    let mut adf4158 = adf4158_pins!(gpioc.pc13, gpioa.pa1, gpioa.pa2, rcu);

    let adf4158_config = Adf4158Config {
        ramp_on: false,
        muxout_control: MuxoutControl::ThreeStateOutput,
        // 4.35 GHz with 10 MHz reference (with reference doubler)
        int: 217,
        frac: 1 << 24,
        cycle_slip_reduction: false,
        cp_current_setting: ChargePumpCurrentSetting::MA0_31,
        prescaler: Prescaler::P8_9,
        r_divider: false,
        reference_doubler: true,
        r_counter: 1,
        clk1_divider: 0,
        n_sel: NWordLoad::OnSdclk,
        sigma_delta_enabled: true,
        ramp_mode: RampMode::ContinuousSawtooth,
        psk_enable: false,
        fsk_enable: false,
        ldp: Ldp::PFDCycles24,
        pd_polarity: PDPolarity::Positive,
        power_down: false,
        cp_three_state: false,
        counter_reset: false,
        le_select: LESelect::FromPin,
        sigma_delta_modulator_mode: SigmaDeltaModulatorMode::NormalOperation,
        negative_bleed_current: true, // TODO: optimize
        readback_to_muxout: false,
        clock_divider_mode: ClkDivMode::Off,
        clk2_divider: 0,
        tx_ramp_clk: TxRampClk::ClkDiv,
        parabolic_ramp: false,
        interrupt: Interrupt::Off,
        fsk_ramp_enable: false,
        ramp_2_enable: false,
        dev_offset_word1: 0,
        deviation_word1: 0,
        dev_offset_word2: 0,
        deviation_word2: 0,
        step_word1: 0,
        step_word2: 0,
        ramp_delay_fast_lock: false,
        ramp_delay: false,
        del_clk_sel: DelClkSel::PfdClk,
        del_start_enable: false,
        delay_start_word: 0,
    };

    adf4158_config.write(&mut adf4158);

    loop {}
}
