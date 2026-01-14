//! Bitbanging (GPIO-driven) SWD and JTAG implementation.

use core::num::NonZeroU32;

use crate::{
    dap::DelayNs,
    jtag::{self, TapConfig},
    swd::{self, APnDP, DPRegister},
    swj::{Dependencies, Pins},
};

/// Clock cycle with data output
macro_rules! clock_out {
    ($self:expr, $bit:expr) => {
        $self.tms_swdio.set_high($bit);
        $self.tck_swclk.set_high(false);
        $self.wait();
        $self.tck_swclk.set_high(true);
        $self.wait();
    };
}

/// Clock cycle with data input
macro_rules! clock_in {
    ($self:expr, $var:ident, $bit:expr) => {
        $self.tck_swclk.set_high(false);
        $self.wait();
        $var |= ($self.tms_swdio.is_high() as u32) << $bit;
        $self.tck_swclk.set_high(true);
        $self.wait();
    };
}

/// Clock cycle without data capture (for turnaround)
macro_rules! clock_only {
    ($self:expr) => {
        $self.tck_swclk.set_high(false);
        $self.wait();
        $self.tck_swclk.set_high(true);
        $self.wait();
    };
}

/// A trait for a pin that can be used as an input or output.
pub trait InputOutputPin {
    fn set_as_output(&mut self);
    fn set_high(&mut self, high: bool);

    fn set_as_input(&mut self);
    fn is_high(&mut self) -> bool;
}

/// A trait for a delay implementation that can be used to delay for a number of CPU cycles.
pub trait DelayCycles: DelayNs {
    fn cpu_clock(&self) -> u32;
    fn delay_cycles(&mut self, cycles: u32);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Dir {
    Write = 0,
    Read = 1,
}

pub struct BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    nreset: IO,
    tdi: IO,
    tms_swdio: IO,
    tck_swclk: IO,
    tdo: IO,
    delay: D,
    bit_cycles: u32,
    target_clock: Option<NonZeroU32>,
    swd_config: swd::Config,
    jtag_config: jtag::Config,
}

impl<IO, D> BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    pub fn new(
        nreset: IO,
        tdi: IO,
        tms_swdio: IO,
        tck_swclk: IO,
        tdo: IO,
        delay: D,
        scan_chain: &'static mut [TapConfig],
    ) -> Self {
        let mut this = Self {
            nreset,
            tdi,
            tms_swdio,
            tck_swclk,
            tdo,
            delay,
            bit_cycles: 0,
            target_clock: None,
            swd_config: swd::Config::default(),
            jtag_config: jtag::Config::new(scan_chain),
        };

        // Start with pins configured to SWD - JTAG pins are a superset, and SWJ sequences need this.
        this.pins_to_swd();
        this
    }

    fn pins_to_swd(&mut self) {
        self.tms_swdio.set_high(true);
        self.tms_swdio.set_as_output();

        self.tck_swclk.set_as_output();

        self.nreset.set_as_input();
        self.tdi.set_as_input();
        self.tdo.set_as_input();
    }

    fn pins_to_jtag(&mut self) {
        self.nreset.set_high(true);
        self.nreset.set_as_output();

        self.tms_swdio.set_high(true);
        self.tms_swdio.set_as_output();

        self.tck_swclk.set_as_output();

        self.tdi.set_as_output();
        self.tdo.set_as_input();
    }

    #[inline(always)]
    fn req(&mut self, port: APnDP, dir: Dir, addr: DPRegister) {
        let req = (port as u32) | (dir as u32) << 1 | (addr as u32) << 2;
        let parity = req.count_ones() % 2;
        let val = 0b10000001 | req << 1 | parity << 5;

        // shift_out 8 bits
        // Note: pin may be in input mode after a previous read(), caller must ensure output mode
        clock_out!(self, val & (1 << 0) != 0);
        clock_out!(self, val & (1 << 1) != 0);
        clock_out!(self, val & (1 << 2) != 0);
        clock_out!(self, val & (1 << 3) != 0);
        clock_out!(self, val & (1 << 4) != 0);
        clock_out!(self, val & (1 << 5) != 0);
        clock_out!(self, val & (1 << 6) != 0);
        clock_out!(self, val & (1 << 7) != 0);
    }

    #[inline(always)]
    pub fn read(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.tms_swdio.set_as_output();
        self.req(port, Dir::Read, addr);

        // turnaround cycle (don't read)
        self.tms_swdio.set_as_input();
        clock_only!(self);

        // read 3 ack bits
        let mut ack = 0;
        clock_in!(self, ack, 0);
        clock_in!(self, ack, 1);
        clock_in!(self, ack, 2);

        match ack {
            0b001 => {} // ok
            0b010 => {
                clock_only!(self); // turnaround cycle
                return Err(swd::Error::AckWait);
            }
            0b100 => {
                clock_only!(self); // turnaround cycle
                return Err(swd::Error::AckFault);
            }
            _ => {
                clock_only!(self); // turnaround cycle
                return Err(swd::Error::AckUnknown(ack as u8));
            }
        }

        // shift_in 32 bits - data
        let mut data = 0;
        clock_in!(self, data, 0);
        clock_in!(self, data, 1);
        clock_in!(self, data, 2);
        clock_in!(self, data, 3);
        clock_in!(self, data, 4);
        clock_in!(self, data, 5);
        clock_in!(self, data, 6);
        clock_in!(self, data, 7);
        clock_in!(self, data, 8);
        clock_in!(self, data, 9);
        clock_in!(self, data, 10);
        clock_in!(self, data, 11);
        clock_in!(self, data, 12);
        clock_in!(self, data, 13);
        clock_in!(self, data, 14);
        clock_in!(self, data, 15);
        clock_in!(self, data, 16);
        clock_in!(self, data, 17);
        clock_in!(self, data, 18);
        clock_in!(self, data, 19);
        clock_in!(self, data, 20);
        clock_in!(self, data, 21);
        clock_in!(self, data, 22);
        clock_in!(self, data, 23);
        clock_in!(self, data, 24);
        clock_in!(self, data, 25);
        clock_in!(self, data, 26);
        clock_in!(self, data, 27);
        clock_in!(self, data, 28);
        clock_in!(self, data, 29);
        clock_in!(self, data, 30);
        clock_in!(self, data, 31);

        // shift_in 1 bit - parity
        let mut parity = 0;
        clock_in!(self, parity, 0);

        if parity != data.count_ones() % 2 {
            return Err(swd::Error::BadParity);
        }

        // turnaround cycle (clock only, keeps pin in input mode)
        clock_only!(self);

        Ok(data)
    }

    #[inline(always)]
    pub fn write(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.tms_swdio.set_as_output();
        self.req(port, Dir::Write, addr);

        // turnaround cycle (don't read)
        self.tms_swdio.set_as_input();
        clock_only!(self);

        // read 3 ack bits
        let mut ack = 0;
        clock_in!(self, ack, 0);
        clock_in!(self, ack, 1);
        clock_in!(self, ack, 2);

        // turnaround cycle (don't read)
        clock_only!(self);

        match ack {
            0b001 => {} // ok
            0b010 => return Err(swd::Error::AckWait),
            0b100 => return Err(swd::Error::AckFault),
            _ => return Err(swd::Error::AckUnknown(ack as _)),
        }

        // shift_out 32 bits - data
        self.tms_swdio.set_as_output();
        clock_out!(self, data & (1 << 0) != 0);
        clock_out!(self, data & (1 << 1) != 0);
        clock_out!(self, data & (1 << 2) != 0);
        clock_out!(self, data & (1 << 3) != 0);
        clock_out!(self, data & (1 << 4) != 0);
        clock_out!(self, data & (1 << 5) != 0);
        clock_out!(self, data & (1 << 6) != 0);
        clock_out!(self, data & (1 << 7) != 0);
        clock_out!(self, data & (1 << 8) != 0);
        clock_out!(self, data & (1 << 9) != 0);
        clock_out!(self, data & (1 << 10) != 0);
        clock_out!(self, data & (1 << 11) != 0);
        clock_out!(self, data & (1 << 12) != 0);
        clock_out!(self, data & (1 << 13) != 0);
        clock_out!(self, data & (1 << 14) != 0);
        clock_out!(self, data & (1 << 15) != 0);
        clock_out!(self, data & (1 << 16) != 0);
        clock_out!(self, data & (1 << 17) != 0);
        clock_out!(self, data & (1 << 18) != 0);
        clock_out!(self, data & (1 << 19) != 0);
        clock_out!(self, data & (1 << 20) != 0);
        clock_out!(self, data & (1 << 21) != 0);
        clock_out!(self, data & (1 << 22) != 0);
        clock_out!(self, data & (1 << 23) != 0);
        clock_out!(self, data & (1 << 24) != 0);
        clock_out!(self, data & (1 << 25) != 0);
        clock_out!(self, data & (1 << 26) != 0);
        clock_out!(self, data & (1 << 27) != 0);
        clock_out!(self, data & (1 << 28) != 0);
        clock_out!(self, data & (1 << 29) != 0);
        clock_out!(self, data & (1 << 30) != 0);
        clock_out!(self, data & (1 << 31) != 0);

        // shift_out 1 bit - parity
        clock_out!(self, data.count_ones() % 2 != 0);

        Ok(())
    }

    #[inline(always)]
    fn shift_out(&mut self, val: u32, num_bits: usize) {
        self.tms_swdio.set_as_output();
        for i in 0..num_bits {
            clock_out!(self, val & (1 << i) != 0);
        }
    }

    #[inline(always)]
    fn shift_in(&mut self, num_bits: usize) -> u32 {
        self.tms_swdio.set_as_input();
        let mut val = 0;
        for i in 0..num_bits {
            clock_in!(self, val, i);
        }
        val
    }

    #[inline(always)]
    fn shift_jtag(&mut self, tms: bool, tdi: u32, num_bits: usize) -> u32 {
        self.tms_swdio.set_as_output();
        self.tms_swdio.set_high(tms);
        let mut tdo = 0;
        for i in 0..num_bits {
            self.tdi.set_high(tdi & (1 << i) != 0);
            self.tck_swclk.set_high(false);
            self.wait();
            tdo |= (self.tdo.is_high() as u32) << i;
            self.tck_swclk.set_high(true);
            self.wait();
        }
        tdo
    }

    #[inline(always)]
    fn wait(&mut self) {
        self.delay.delay_cycles(self.bit_cycles);
    }

    fn apply_clock(&mut self) {
        if let Some(target_clock) = self.target_clock {
            self.bit_cycles = (self.delay.cpu_clock() / 2)
                .div_ceil(target_clock.get())
                .max(1);

            self.target_clock = None;
        }
        // TODO implement proper setup when converting into swd/jtag
        self.tck_swclk.set_as_output();
        self.tdi.set_as_output();
    }

    fn set_target_clock(&mut self, max_frequency: u32) -> bool {
        debug!("set frequency({})", max_frequency);
        match NonZeroU32::new(max_frequency) {
            Some(frequency) => {
                self.target_clock = Some(frequency);
                true
            }
            _ => false,
        }
    }
}

impl<IO: InputOutputPin, D: DelayCycles> Dependencies<SwdAdapter<IO, D>, JtagAdapter<IO, D>>
    for BitbangAdapter<IO, D>
{
    fn high_impedance_mode(&mut self) {
        self.tms_swdio.set_as_input();
        self.tck_swclk.set_as_input();
        self.nreset.set_as_input();
        self.tdi.set_as_input();
        self.tdo.set_as_input();
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.set_target_clock(max_frequency)
    }

    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        if mask.contains(Pins::SWCLK) {
            self.tck_swclk.set_high(output.contains(Pins::SWCLK));
        }
        if mask.contains(Pins::SWDIO) {
            self.tms_swdio.set_high(output.contains(Pins::SWDIO));
        }
        if mask.contains(Pins::NRESET) {
            self.nreset.set_high(output.contains(Pins::NRESET));
        }
        if mask.contains(Pins::TDO) {
            self.tdo.set_high(output.contains(Pins::TDO));
        }

        if wait_us != 0 {
            self.delay.delay_us(wait_us);
        }

        let mut read = Pins::empty();

        read.set(Pins::SWCLK, self.tck_swclk.is_high());
        read.set(Pins::SWDIO, self.tms_swdio.is_high());
        read.set(Pins::NRESET, self.nreset.is_high());
        read.set(Pins::TDO, self.tdo.is_high());
        read.set(Pins::TDI, self.tdi.is_high());
        read.set(Pins::NTRST, true);

        read
    }

    fn process_swj_sequence(&mut self, data: &[u8], mut num_bits: usize) {
        self.apply_clock();
        debug!("write_sequence({})", num_bits);
        for b in data.iter().copied() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            self.shift_out(b as u32, bits);
            num_bits -= bits;
        }
    }

    fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.swd_config
    }

    fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.jtag_config
    }
}

pub struct SwdAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    inner: BitbangAdapter<IO, D>,
}

impl<IO, D> From<BitbangAdapter<IO, D>> for SwdAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    fn from(mut inner: BitbangAdapter<IO, D>) -> Self {
        inner.pins_to_swd();

        Self { inner }
    }
}
impl<IO, D> From<SwdAdapter<IO, D>> for BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    fn from(inner: SwdAdapter<IO, D>) -> Self {
        inner.inner
    }
}

impl<IO, D> swd::Swd<BitbangAdapter<IO, D>> for SwdAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    fn available(deps: &BitbangAdapter<IO, D>) -> bool {
        // TODO: Check if the SWD pins are available
        true
    }

    fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        debug!("read_inner({:?}, {:?})", port, addr);
        self.inner.read(port, addr)
    }

    fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        debug!("write_inner({:?}, {:?}, {:x})", port, addr, data);
        self.inner.write(port, addr, data)
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_target_clock(max_frequency)
    }

    /// Write a sequence of bits using SWDIO and the clock line running at the configured freq.
    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.inner.process_swj_sequence(data, num_bits);
        Ok(())
    }

    /// Read a sequence of bits using SWDIO and the clock line running at the configured freq.
    fn read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.inner.apply_clock();
        debug!("read_sequence({})", num_bits);
        for b in data.iter_mut() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            *b = self.inner.shift_in(bits) as u8;
            num_bits -= bits;
        }
        Ok(())
    }

    fn config(&mut self) -> &mut swd::Config {
        &mut self.inner.swd_config
    }
}

pub struct JtagAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    inner: BitbangAdapter<IO, D>,
}

impl<IO, D> From<BitbangAdapter<IO, D>> for JtagAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    fn from(mut inner: BitbangAdapter<IO, D>) -> Self {
        inner.pins_to_jtag();

        Self { inner }
    }
}

impl<IO, D> From<JtagAdapter<IO, D>> for BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    fn from(mut inner: JtagAdapter<IO, D>) -> Self {
        // Reset pin configuration to default.
        inner.inner.pins_to_swd();
        inner.inner
    }
}

impl<IO, D> jtag::Jtag<BitbangAdapter<IO, D>> for JtagAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    fn available(deps: &BitbangAdapter<IO, D>) -> bool {
        // TODO: Check if the JTAG pins are actually available
        true
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.inner.set_target_clock(max_frequency)
    }

    fn config(&mut self) -> &mut jtag::Config {
        &mut self.inner.jtag_config
    }

    fn sequence(&mut self, info: jtag::SequenceInfo, tdi: &[u8], mut rxbuf: &mut [u8]) {
        let mut n_bits = info.n_bits;
        for &byte in tdi {
            let bits = n_bits.min(8) as usize;
            n_bits -= bits as u8;
            let tdo = self.inner.shift_jtag(info.tms, byte as u32, bits);
            if info.capture && !rxbuf.is_empty() {
                rxbuf[0] = tdo as u8;
                rxbuf = &mut rxbuf[1..];
            }
        }
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        for &bit in tms.iter() {
            self.inner.tms_swdio.set_high(bit);
            self.inner.tck_swclk.set_high(false);
            self.inner.wait();
            self.inner.tck_swclk.set_high(true);
            self.inner.wait();
        }
    }
}
