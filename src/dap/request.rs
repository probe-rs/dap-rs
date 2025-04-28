use super::Command;

pub struct Request<'a> {
    /// The CMSIS-DAP command.
    pub command: Command,

    /// The request payload.
    pub data: &'a [u8],
}

impl<'a> Request<'a> {
    /// Returns None if the report is empty
    pub fn from_report(report: &'a [u8]) -> Option<Self> {
        let (command, data) = report.split_first()?;

        let command = (*command).try_into().unwrap_or(Command::Unimplemented);

        Some(Request { command, data })
    }

    /// Consumes the next byte and returns it as a `u8` value.
    pub fn next_u8(&mut self) -> u8 {
        let value = self.data[0];
        self.data = &self.data[1..];
        value
    }

    /// Consumes the next two bytes and returns them as a `u16` value.
    pub fn next_u16(&mut self) -> u16 {
        let value = u16::from_le_bytes(self.data[0..2].try_into().unwrap());
        self.data = &self.data[2..];
        value
    }

    /// Consumes the next four bytes and returns them as a `u32` value.
    pub fn next_u32(&mut self) -> u32 {
        let value = u32::from_le_bytes(self.data[0..4].try_into().unwrap());
        self.data = &self.data[4..];
        value
    }

    /// Consumes the first `count` bytes of the data.
    pub fn consume(&mut self, count: usize) {
        self.data = &self.data[count..];
    }

    /// Returns the remaining data.
    pub fn rest(self) -> &'a [u8] {
        &self.data
    }
}
