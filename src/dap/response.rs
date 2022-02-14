use super::{Command, ResponseStatus};

pub struct ResponseWriter<'a> {
    pub buf: &'a mut [u8],
    pub idx: usize,
}

impl<'a> ResponseWriter<'a> {
    pub fn new(command: Command, buf: &'a mut [u8]) -> Self {
        buf[0] = command as u8;
        ResponseWriter { buf, idx: 1 }
    }

    pub fn write_u8(&mut self, value: u8) {
        self.buf[self.idx] = value;
        self.idx += 1;
    }

    pub fn write_u16(&mut self, value: u16) {
        let value = value.to_le_bytes();
        self.buf[self.idx..self.idx + 2].copy_from_slice(&value);
        self.idx += 2;
    }

    pub fn write_u32(&mut self, value: u32) {
        let value = value.to_le_bytes();
        self.buf[self.idx..self.idx + 4].copy_from_slice(&value);
        self.idx += 4;
    }

    pub fn write_slice(&mut self, data: &[u8]) {
        self.buf[self.idx..self.idx + data.len()].copy_from_slice(&data);
        self.idx += data.len();
    }

    pub fn write_ok(&mut self) {
        self.write_u8(ResponseStatus::DapOk.into());
    }

    pub fn write_err(&mut self) {
        self.write_u8(ResponseStatus::DapError.into());
    }

    pub fn write_u8_at(&mut self, idx: usize, value: u8) {
        self.buf[idx] = value;
    }

    pub fn write_u16_at(&mut self, idx: usize, value: u16) {
        let value = value.to_le_bytes();
        self.buf[idx..idx + 2].copy_from_slice(&value);
    }

    pub fn mut_at(&mut self, idx: usize) -> &mut u8 {
        &mut self.buf[idx]
    }

    pub fn read_u8_at(&self, idx: usize) -> u8 {
        self.buf[idx]
    }

    pub fn remaining(&mut self) -> &mut [u8] {
        &mut self.buf[self.idx..]
    }

    pub fn skip(&mut self, n: usize) {
        self.idx += n;
    }
}
