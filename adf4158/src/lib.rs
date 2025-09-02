#![no_std]

pub mod prelude {
    pub use super::{
        Adf4158Config, ChargePumpCurrentSetting, ClkDivMode, DelClkSel, Interrupt, LESelect, Ldp,
        MuxoutControl, NWordLoad, PDPolarity, Prescaler, RampMode, ReadbackToMuxout, RegisterWrite,
        SigmaDeltaModulatorMode, TxRampClk,
    };
}

pub trait RegisterWrite {
    fn write(&mut self, data: u32);
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Adf4158Config {
    // R0
    pub ramp_on: bool,
    pub muxout_control: MuxoutControl,
    pub int: u16, // 12 bits
    // R0 & R1
    pub frac: u32, // 25 bits
    // R2
    pub cycle_slip_reduction: bool,
    pub cp_current_setting: ChargePumpCurrentSetting,
    pub prescaler: Prescaler,
    pub r_divider: bool,
    pub reference_doubler: bool,
    pub r_counter: u8,     // 5 bits
    pub clk1_divider: u16, // 12 bits
    // R3
    pub n_sel: NWordLoad,
    pub sigma_delta_enabled: bool,
    pub ramp_mode: RampMode,
    pub psk_enable: bool,
    pub fsk_enable: bool,
    pub ldp: Ldp,
    pub pd_polarity: PDPolarity,
    pub power_down: bool,
    pub cp_three_state: bool,
    pub counter_reset: bool,
    // R4
    pub le_select: LESelect,
    pub sigma_delta_modulator_mode: SigmaDeltaModulatorMode,
    pub negative_bleed_current: bool,
    pub readback_to_muxout: ReadbackToMuxout,
    pub clock_divider_mode: ClkDivMode,
    pub clk2_divider: u16, // 12 bits
    // R5
    pub tx_ramp_clk: TxRampClk,
    pub parabolic_ramp: bool,
    pub interrupt: Interrupt,
    pub fsk_ramp_enable: bool,
    pub ramp_2_enable: bool,
    pub dev_offset_word1: u8, // 4 bits
    pub deviation_word1: i16,
    pub dev_offset_word2: u8, // 4 bits
    pub deviation_word2: i16,
    // R6
    pub step_word1: u32, // 20 bits
    pub step_word2: u32, // 20 bits
    // R7
    pub ramp_delay_fast_lock: bool,
    pub ramp_delay: bool,
    pub del_clk_sel: DelClkSel,
    pub del_start_enable: bool,
    pub delay_start_word: u16,
}

impl Default for Adf4158Config {
    fn default() -> Adf4158Config {
        Adf4158Config {
            ramp_on: false,
            muxout_control: MuxoutControl::ThreeStateOutput,
            int: 0,
            frac: 0,
            cycle_slip_reduction: false,
            cp_current_setting: ChargePumpCurrentSetting::MA0_31,
            prescaler: Prescaler::P8_9,
            r_divider: false,
            reference_doubler: false,
            r_counter: 1,
            clk1_divider: 1,
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
            negative_bleed_current: false,
            readback_to_muxout: ReadbackToMuxout::Disabled,
            clock_divider_mode: ClkDivMode::Off,
            clk2_divider: 1,
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
        }
    }
}

impl Adf4158Config {
    pub fn write<R: RegisterWrite>(&self, reg: &mut R) {
        assert!(self.frac < 1 << 25);
        reg.write(word_r7(
            self.ramp_delay_fast_lock,
            self.ramp_delay,
            self.del_clk_sel,
            self.del_start_enable,
            self.delay_start_word,
        ));
        reg.write(word_r6(StepSel::Word1, self.step_word1));
        reg.write(word_r6(StepSel::Word2, self.step_word2));
        reg.write(word_r5(
            self.tx_ramp_clk,
            self.parabolic_ramp,
            self.interrupt,
            self.fsk_ramp_enable,
            self.ramp_2_enable,
            DevSel::Word1,
            self.dev_offset_word1,
            self.deviation_word1,
        ));
        reg.write(word_r5(
            self.tx_ramp_clk,
            self.parabolic_ramp,
            self.interrupt,
            self.fsk_ramp_enable,
            self.ramp_2_enable,
            DevSel::Word2,
            self.dev_offset_word2,
            self.deviation_word2,
        ));
        reg.write(word_r4(
            self.le_select,
            self.sigma_delta_modulator_mode,
            self.negative_bleed_current,
            self.readback_to_muxout,
            self.clock_divider_mode,
            self.clk2_divider,
        ));
        reg.write(word_r3(
            self.n_sel,
            self.sigma_delta_enabled,
            self.ramp_mode,
            self.psk_enable,
            self.fsk_enable,
            self.ldp,
            self.pd_polarity,
            self.power_down,
            self.cp_three_state,
            self.counter_reset,
        ));
        reg.write(word_r2(
            self.cycle_slip_reduction,
            self.cp_current_setting,
            self.prescaler,
            self.r_divider,
            self.reference_doubler,
            self.r_counter,
            self.clk1_divider,
        ));
        let frac_lsbs = (self.frac & ((1 << 13) - 1)) as u16;
        let frac_msbs = (self.frac >> 13) as u16;
        reg.write(word_r1(frac_lsbs));
        reg.write(word_r0(
            self.ramp_on,
            self.muxout_control,
            self.int,
            frac_msbs,
        ));
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum MuxoutControl {
    ThreeStateOutput = 0b0000,
    DVDD = 0b0001,
    DGND = 0b0010,
    RDividerOutput = 0b0011,
    NDividerOutput = 0b0100,
    DigitalLockDetect = 0b0110,
    SerialDataOutput = 0b0111,
    ClkDividerOutput = 0b1010,
    FastLockSwitch = 0b1100,
    RDivider2 = 0b1101,
    NDivider2 = 0b1110,
    ReadbackToMux = 0b1111,
}

#[allow(clippy::identity_op)]
fn word_r0(ramp_on: bool, muxout_control: MuxoutControl, int: u16, frac_msb: u16) -> u32 {
    assert!(int < 1 << 12);
    assert!(frac_msb < 1 << 12);
    (u32::from(ramp_on) << 31)
        | (u32::from(muxout_control as u8) << 27)
        | (u32::from(int) << 15)
        | (u32::from(frac_msb) << 3)
        | 0b000
}

fn word_r1(frac_lsb: u16) -> u32 {
    assert!(frac_lsb < 1 << 13);
    (u32::from(frac_lsb) << 15) | 0b001
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum ChargePumpCurrentSetting {
    MA0_31 = 0,
    MA0_63 = 1,
    MA0_94 = 2,
    MA1_25 = 3,
    MA1_57 = 4,
    MA1_88 = 5,
    MA2_19 = 6,
    MA2_5 = 7,
    MA2_81 = 8,
    MA3_13 = 9,
    MA3_44 = 10,
    MA3_75 = 11,
    MA4_06 = 12,
    MA4_38 = 13,
    MA4_69 = 14,
    MA5 = 15,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum Prescaler {
    P4_5 = 0,
    P8_9 = 1,
}

fn word_r2(
    cycle_slip_reduction: bool,
    cp_current_setting: ChargePumpCurrentSetting,
    prescaler: Prescaler,
    r_divider: bool,
    reference_doubler: bool,
    r_counter: u8,
    clk1_divider: u16,
) -> u32 {
    assert!(r_counter < 1 << 5);
    assert!(clk1_divider < 1 << 12);
    (u32::from(cycle_slip_reduction) << 28)
        | (u32::from(cp_current_setting as u8) << 24)
        | (u32::from(prescaler as u8) << 22)
        | (u32::from(r_divider) << 21)
        | (u32::from(reference_doubler) << 20)
        | (u32::from(r_counter) << 15)
        | (u32::from(clk1_divider) << 3)
        | 0b010
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum NWordLoad {
    OnSdclk = 0,
    Delayed4Cycles = 1,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum RampMode {
    ContinuousSawtooth = 0,
    ContinuousTriangular = 1,
    SingleSawtooth = 2,
    SingleRampBurst = 3,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum Ldp {
    PFDCycles24 = 0,
    PFDCycles40 = 1,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum PDPolarity {
    Negative = 0,
    Positive = 1,
}

#[allow(clippy::too_many_arguments)]
fn word_r3(
    n_sel: NWordLoad,
    sigma_delta_enabled: bool,
    ramp_mode: RampMode,
    psk_enable: bool,
    fsk_enable: bool,
    ldp: Ldp,
    pd_polarity: PDPolarity,
    power_down: bool,
    cp_three_state: bool,
    counter_reset: bool,
) -> u32 {
    let sigma_delta_reset = !sigma_delta_enabled;
    (u32::from(n_sel as u8) << 15)
        | (u32::from(sigma_delta_reset) << 14)
        | (u32::from(ramp_mode as u8) << 10)
        | (u32::from(psk_enable) << 9)
        | (u32::from(fsk_enable) << 8)
        | (u32::from(ldp as u8) << 7)
        | (u32::from(pd_polarity as u8) << 6)
        | (u32::from(power_down) << 5)
        | (u32::from(cp_three_state) << 4)
        | (u32::from(counter_reset) << 3)
        | 0b011
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum LESelect {
    FromPin = 0,
    SynchWithRefIn = 1,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum SigmaDeltaModulatorMode {
    NormalOperation = 0,
    DisabledWhenFracZero = 0b01110,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum ReadbackToMuxout {
    Disabled = 0b00,
    Enabled = 0b01,
    RampComplete = 0b11,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum ClkDivMode {
    Off = 0b00,
    FastLockDivider = 0b01,
    RampDivider = 0b11,
}

fn word_r4(
    le_select: LESelect,
    sigma_delta_modulator_mode: SigmaDeltaModulatorMode,
    negative_bleed_current: bool,
    readback_to_muxout: ReadbackToMuxout,
    clock_divider_mode: ClkDivMode,
    clk2_divider: u16,
) -> u32 {
    assert!(clk2_divider < 1 << 12);
    let negative_bleed_current = if negative_bleed_current { 0b11 } else { 0 };
    (u32::from(le_select as u8) << 31)
        | (u32::from(sigma_delta_modulator_mode as u8) << 26)
        | (negative_bleed_current << 23)
        | (u32::from(readback_to_muxout as u8) << 21)
        | (u32::from(clock_divider_mode as u8) << 19)
        | (u32::from(clk2_divider) << 7)
        | 0b100
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum TxRampClk {
    ClkDiv = 0,
    TxData = 1,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum Interrupt {
    Off = 0b00,
    LoadChannelContinueSweep = 0b01,
    LoadChannelStopSweep = 0b10,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
enum DevSel {
    Word1 = 0,
    Word2 = 1,
}

#[allow(clippy::too_many_arguments)]
fn word_r5(
    tx_ramp_clk: TxRampClk,
    parabolic_ramp: bool,
    interrupt: Interrupt,
    fsk_ramp_enable: bool,
    ramp_2_enable: bool,
    dev_sel: DevSel,
    dev_offset: u8,
    deviation: i16,
) -> u32 {
    assert!(dev_offset <= 9);
    (u32::from(tx_ramp_clk as u8) << 29)
        | (u32::from(parabolic_ramp) << 28)
        | (u32::from(interrupt as u8) << 26)
        | (u32::from(fsk_ramp_enable) << 25)
        | (u32::from(ramp_2_enable) << 24)
        | (u32::from(dev_sel as u8) << 23)
        | (u32::from(dev_offset) << 19)
        | (u32::from(deviation as u16) << 3)
        | 0b101
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
enum StepSel {
    Word1 = 0,
    Word2 = 1,
}

fn word_r6(step_sel: StepSel, step_word: u32) -> u32 {
    assert!(step_word < 1 << 20);
    (u32::from(step_sel as u8) << 23) | (step_word << 3) | 0b110
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum DelClkSel {
    PfdClk = 0,
    PdfTimesClk1 = 1,
}

fn word_r7(
    ramp_delay_fast_lock: bool,
    ramp_delay: bool,
    del_clk_sel: DelClkSel,
    del_start_enable: bool,
    delay_start_word: u16,
) -> u32 {
    assert!(delay_start_word < 1 << 12);
    (u32::from(ramp_delay_fast_lock) << 18)
        | (u32::from(ramp_delay) << 17)
        | (u32::from(del_clk_sel as u8) << 16)
        | (u32::from(del_start_enable) << 15)
        | (u32::from(delay_start_word) << 3)
        | 0b111
}
