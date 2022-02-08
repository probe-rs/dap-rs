#![no_std]
#![warn(missing_docs)]

pub mod dap;
pub mod jtag;
pub mod swd;
pub mod swj;
pub mod swo;
// TODO: somewhere from where LEDs can be controlled based on host status
// pub mod host_status;
