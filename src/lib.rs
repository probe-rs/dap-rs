//! TODO: Crate docs

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

/// TODO: Dap docs
pub mod dap;
pub mod jtag;
pub mod swd;
pub mod swj;
pub mod swo;
pub mod usb;

#[cfg(test)]
mod mock_device;
