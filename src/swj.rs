use bitflags::bitflags;

bitflags! {
    /// Pin definitions in the SWJ_Pins command
    pub struct Pins: u8 {
        /// SWCLK
        const SWCLK = 1 << 0;
        /// SWDIO
        const SWDIO = 1 << 1;
        /// TDI
        const TDI = 1 << 2;
        /// TDO
        const TDO = 1 << 3;
        /// NTRST
        const NTRST = 1 << 5;
        /// NRESET
        const NRESET = 1 << 7;
    }
}

/// Trait for SWJ, as it is now only the SWJ_Pins command is handled here,
/// clock speed is part of the JTAG and SWD traits.
pub trait Swj {
    /// Runner for SWJ_Pins commands.
    fn pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins;

    /// Runner for SWJ_Pins commands.
    fn sequence(&mut self, data: &[u8], nbits: usize);

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;
}
