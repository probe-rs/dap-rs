/// Trait for SWJ, as it is now only the SWJ_Pins command is handled here,
/// and sequences and clock speed is part of the JTAG and SWD traits.
pub trait Swj {
    /// Runner for SWJ_Pins commands.
    fn pins(&mut self, output: u8, mask: u8, wait_us: u32) -> u8;
}
