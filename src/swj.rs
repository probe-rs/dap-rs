/// Trait for SWJ, as it is now only the SWJ_Pins command is handled here,
/// clock speed is part of the JTAG and SWD traits.
pub trait Swj {
    /// Runner for SWJ_Pins commands.
    fn pins(&mut self, output: u8, mask: u8, wait_us: u32) -> u8;

    /// Runner for SWJ_Pins commands.
    fn sequence(&mut self, data: &[u8], nbits: usize);

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;
}
