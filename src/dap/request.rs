use super::Command;

pub struct Request<'a> {
    pub command: Command,
    pub data: &'a [u8],
}

impl<'a> Request<'a> {
    /// Returns None if the report is empty
    pub fn from_report(report: &'a [u8]) -> Option<Self> {
        let (command, data) = report.split_first()?;

        let command = (*command).try_into().unwrap_or(Command::Unimplemented);

        Some(Request { command, data })
    }

    pub fn next_u8(&mut self) -> u8 {
        let value = self.data[0];
        self.data = &self.data[1..];
        value
    }

    pub fn next_u16(&mut self) -> u16 {
        let value = u16::from_le_bytes(self.data[0..2].try_into().unwrap());
        self.data = &self.data[2..];
        value
    }

    pub fn next_u32(&mut self) -> u32 {
        let value = u32::from_le_bytes(self.data[0..4].try_into().unwrap());
        self.data = &self.data[4..];
        value
    }

    pub fn rest(self) -> &'a [u8] {
        &self.data
    }
}
