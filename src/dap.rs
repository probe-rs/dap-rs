use crate::*;

/// DAP handler.
/// TODO: Figure out how to structure this.
pub struct Dap<JTAG: jtag::Jtag, SWD: swd::Swd, SWJ: swj::Swj, SWO: swo::Swo> {
    jtag: Option<JTAG>,
    swd: Option<SWD>,
    swj: Option<SWJ>,
    swo: Option<SWO>,
}
