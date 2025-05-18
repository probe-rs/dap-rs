use crate::{jtag, swd, swj};

#[mockall::automock]
pub trait SwdJtagDevice {
    // swj
    fn high_impedance_mode(&mut self);
    fn process_swj_clock(&mut self, max_frequency: u32) -> bool;
    fn process_swj_pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> swj::Pins;
    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize);

    // swd
    fn swd_config(&mut self) -> &mut swd::Config;
    fn read_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister) -> swd::Result<u32>;
    fn write_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister, data: u32) -> swd::Result<()>;
    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()>;
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()>;

    // jtag
    fn jtag_config(&mut self) -> &mut jtag::Config;
    fn sequence(&mut self, info: jtag::SequenceInfo, tdi: &[u8], resp: &mut [u8]);
    fn tms_sequence(&mut self, tms: &[bool]);

    // swd/jtag
    fn set_clock(&mut self, max_frequency: u32) -> bool;
}

impl swj::Dependencies<Self, Self> for MockSwdJtagDevice {
    fn high_impedance_mode(&mut self) {
        SwdJtagDevice::high_impedance_mode(self)
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        SwdJtagDevice::process_swj_clock(self, max_frequency)
    }

    fn process_swj_pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> swj::Pins {
        SwdJtagDevice::process_swj_pins(self, output, mask, wait_us)
    }

    fn process_swj_sequence(&mut self, data: &[u8], nbits: usize) {
        SwdJtagDevice::process_swj_sequence(self, data, nbits)
    }

    fn jtag_config(&mut self) -> &mut jtag::Config {
        SwdJtagDevice::jtag_config(self)
    }

    fn swd_config(&mut self) -> &mut swd::Config {
        SwdJtagDevice::swd_config(self)
    }
}

impl swd::Swd<Self> for MockSwdJtagDevice {
    const AVAILABLE: bool = true;

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        SwdJtagDevice::set_clock(self, max_frequency)
    }

    fn read_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister) -> swd::Result<u32> {
        SwdJtagDevice::read_inner(self, apndp, a)
    }

    fn write_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister, data: u32) -> swd::Result<()> {
        SwdJtagDevice::write_inner(self, apndp, a, data)
    }

    fn read_sequence(&mut self, num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        SwdJtagDevice::read_sequence(self, num_bits, data)
    }

    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        SwdJtagDevice::write_sequence(self, num_bits, data)
    }

    fn config(&mut self) -> &mut swd::Config {
        Self::swd_config(self)
    }
}

impl jtag::Jtag<MockSwdJtagDevice> for MockSwdJtagDevice {
    const AVAILABLE: bool = true;

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        SwdJtagDevice::set_clock(self, max_frequency)
    }

    fn sequence(&mut self, info: jtag::SequenceInfo, tdi: &[u8], resp: &mut [u8]) {
        SwdJtagDevice::sequence(self, info, tdi, resp)
    }

    fn config(&mut self) -> &mut jtag::Config {
        SwdJtagDevice::jtag_config(self)
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        SwdJtagDevice::tms_sequence(self, tms)
    }
}
