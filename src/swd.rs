use num_enum::IntoPrimitive;

/// The available errors for SWD.
#[derive(Copy, Clone, Debug)]
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
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum DPRegister {
    DPIDR = 0,
    CTRLSTAT = 1,
    SELECT = 2,
    RDBUFF = 3,
}

/// Encode if a transaction is for AP or DP.
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum APnDP {
    /// For DP.
    DP = 0,
    /// For AP.
    AP = 1,
}

/// Encode if an SWD transaction is a read or a write.
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum RnW {
    /// Write flag.
    W = 0,
    /// Read flag.
    R = 1,
}

/// The different kinds of SWD Ack.
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Ack {
    /// An ok Ack.
    Ok = 0b001,
    /// Not yet ready.
    Wait = 0b010,
    /// A fault has occurred.
    Fault = 0b100,
    /// Protocol error.
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

/// Definition of SWD communication.
pub trait Swd {
    /// Helper method over `read_inner` to retry during `AckWait`.
    fn read(&mut self, num_retries: usize, apndp: APnDP, a: DPRegister) -> Result<u32> {
        for _ in 0..num_retries {
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
    fn write(&mut self, num_retries: usize, apndp: APnDP, a: DPRegister, data: u32) -> Result<()> {
        for _ in 0..num_retries {
            match self.write_inner(apndp, a, data) {
                Err(Error::AckWait) => continue,
                x => return x,
            }
        }

        Err(Error::AckWait)
    }

    /// Here the actual hardware implementation for an SWD write is made.
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> Result<()>;

    /// Here is the actual hardware implementation to idle low.
    fn idle_low(&mut self);
}

/// Helper used by `Swd::read_inner` and `Swd::write_inner` to make the request byte.
pub fn make_request(apndp: APnDP, rnw: RnW, a: DPRegister) -> u8 {
    let req = 1 | ((apndp as u8) << 1) | ((rnw as u8) << 2) | ((a as u8) << 3) | (1 << 7);
    let parity = (req.count_ones() & 1) as u8;
    req | (parity << 5)
}
