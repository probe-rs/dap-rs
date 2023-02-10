use num_enum::{IntoPrimitive, TryFromPrimitive};

/// The available errors for SWD.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Parity error.
    BadParity,
    /// Ack not yet ready.
    AckWait,
    /// A fault.
    AckFault,
    /// A protocol error.
    AckProtocol,
    /// Unkown error.
    AckUnknown(u8),
}

/// The definision of SWD results.
pub type Result<T> = core::result::Result<T, Error>;

/// Available DP registers.
#[repr(u8)]
#[derive(PartialEq, Eq, Copy, Clone, Debug, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DPRegister {
    DPIDR = 0,
    CTRLSTAT = 1,
    SELECT = 2,
    RDBUFF = 3,
}

/// Encode if a transaction is for AP or DP.
#[repr(u8)]
#[derive(PartialEq, Eq, Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum APnDP {
    /// For DP.
    DP = 0,
    /// For AP.
    AP = 1,
}

/// Encode if an SWD transaction is a read or a write.
#[repr(u8)]
#[derive(PartialEq, Eq, Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RnW {
    /// Write flag.
    W = 0,
    /// Read flag.
    R = 1,
}

/// The different kinds of SWD Ack.
#[repr(u8)]
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum Ack {
    Ok = 0b001,
    Wait = 0b010,
    Fault = 0b100,
    Protocol = 0b111,
}

impl Ack {
    /// Helper method for `Swd::read_inner` and `Swd::write_inner` to decode an SWD Ack.
    pub fn try_ok(ack: u8) -> Result<()> {
        match ack {
            v if v == (Ack::Ok as u8) => Ok(()),
            v if v == (Ack::Wait as u8) => Err(Error::AckWait),
            v if v == (Ack::Fault as u8) => Err(Error::AckFault),
            v if v == (Ack::Protocol as u8) => Err(Error::AckProtocol),
            _ => Err(Error::AckUnknown(ack)),
        }
    }
}

/// Turn around period configuration.
#[derive(Copy, Clone, Debug, TryFromPrimitive, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
#[repr(u8)]
pub enum TurnaroundPeriod {
    Cycles1 = 0b00,
    Cycles2 = 0b01,
    Cycles3 = 0b10,
    Cycles4 = 0b11,
}

/// Data phase configuration.
#[derive(Copy, Clone, Debug, TryFromPrimitive, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
#[repr(u8)]
pub enum DataPhase {
    NoDataPhase = 0,
    AlwaysDataPhase = 1,
}

/// Definition of SWD communication.
pub trait Swd<DEPS>: From<DEPS> {
    /// If SWD is available or not.
    const AVAILABLE: bool;

    /// Helper method over `read_inner` to retry during `AckWait`.
    fn read(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister) -> Result<u32> {
        for _ in 0..wait_retries {
            match self.read_inner(apndp, a) {
                Err(Error::AckWait) => continue,
                x => return x,
            }
        }

        Err(Error::AckWait)
    }

    /// Here the actual hardware implementation for an SWD read is made.
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> Result<u32>;

    /// Helper method over `write_inner` to retry during `AckWait`.
    fn write(&mut self, wait_retries: usize, apndp: APnDP, a: DPRegister, data: u32) -> Result<()> {
        for _ in 0..wait_retries {
            match self.write_inner(apndp, a, data) {
                Err(Error::AckWait) => continue,
                x => return x,
            }
        }

        Err(Error::AckWait)
    }

    /// Here the actual hardware implementation for an SWD write is made.
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> Result<()>;

    /// Shorthand helper to read DP registers
    fn read_dp(&mut self, wait_retries: usize, a: DPRegister) -> Result<u32> {
        self.read(wait_retries, APnDP::DP, a)
    }

    /// Shorthand helper to write DP registers
    fn write_dp(&mut self, wait_retries: usize, a: DPRegister, data: u32) -> Result<()> {
        self.write(wait_retries, APnDP::DP, a, data)
    }

    /// Shorthand helper to read AP registers
    fn read_ap(&mut self, wait_retries: usize, a: DPRegister) -> Result<u32> {
        self.read(wait_retries, APnDP::AP, a)
    }

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;
}

/// Helper used by `Swd::read_inner` and `Swd::write_inner` to make the request byte.
pub fn make_request(apndp: APnDP, rnw: RnW, a: DPRegister) -> u8 {
    let req = 1 | ((apndp as u8) << 1) | ((rnw as u8) << 2) | ((a as u8) << 3) | (1 << 7);
    let parity = (req.count_ones() & 1) as u8;
    req | (parity << 5)
}
