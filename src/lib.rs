//! TODO: Crate docs

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

/// TODO: Dap docs
pub mod dap;
pub mod driver;
pub mod jtag;
pub mod swd;
pub mod swj;
pub mod swo;
pub mod usb;

#[cfg(test)]
mod mock_device;

// Re-export the usb-device crate, so that crates depending on us can use it without
// having to track it as a separate dependency.
pub use usb_device;
