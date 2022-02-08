pub trait Jtag {
    /// Handle a JTAG sequence request.
    fn sequences(&mut self, data: &[u8], rxbuf: &mut [u8]) -> u32;

    /// Set the maximum clock frequency for the JTAG, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;

    /// A sequence of bits to send. Used by `SWJ_Sequence`.
    fn tms_sequence(data: &[u8], nbits: usize);

    // TODO: What is missing for the 2 other JTAG commands
}
