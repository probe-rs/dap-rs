use crate::dap::DapContext;

pub trait Jtag<CONTEXT: DapContext> {
    /// If JTAG is available or not.
    const AVAILABLE: bool;

    /// Create the JTAG from context
    fn new(context: CONTEXT) -> Self;

    /// Release the context from the JTAG
    fn release(self) -> CONTEXT;

    /// Handle a JTAG sequence request.
    fn sequences(&mut self, data: &[u8], rxbuf: &mut [u8]) -> u32;

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;

    // TODO: What is missing for the 2 other JTAG commands
}
