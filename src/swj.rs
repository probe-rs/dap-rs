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

// TODO: Change the name? Move it to a different module?
/// Trait describing the dependencies necessary to yield an instance of
/// [`crate::Dap`]
///
/// User has to provide implementations of SWJ_{Pins, Sequence, Clock} commands
pub trait Dependencies<SWD, JTAG>: From<SWD> + From<JTAG> {
    /// Runner for SWJ_Pins commands.
    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins;

    /// Runner for SWJ_Pins commands.
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize);

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool;

    /// Set pins in high impedance mode
    fn high_impedance_mode(&mut self);
}
