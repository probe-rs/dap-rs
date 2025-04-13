use crate::{
    dap,
    swd::{APnDP, RnW},
};

/// Describes a JTAG sequence request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SequenceInfo {
    /// The number of bits to shift in/out.
    pub n_bits: u8,

    /// If the TDO data should be captured.
    pub capture: bool,

    /// The TMS level to output.
    pub tms: bool,
}

impl From<u8> for SequenceInfo {
    fn from(byte: u8) -> Self {
        const JTAG_SEQUENCE_TCK: u8 = 0x3F;
        const JTAG_SEQUENCE_TMS: u8 = 0x40;
        const JTAG_SEQUENCE_TDO: u8 = 0x80;

        let n_bits = match byte & JTAG_SEQUENCE_TCK {
            // CMSIS-DAP says 0 means 64 bits
            0 => 64,
            // Other integers are normal.
            n => n,
        };

        Self {
            n_bits: n_bits as u8,
            capture: (byte & JTAG_SEQUENCE_TDO) != 0,
            tms: (byte & JTAG_SEQUENCE_TMS) != 0,
        }
    }
}

/// Describes a JTAG sequence request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferInfo {
    pub ap_ndp: APnDP,
    pub r_nw: RnW,
    pub a2a3: u8,
    pub match_value: bool,
    pub match_mask: bool,
    pub timestamp: bool,
}

impl From<u8> for TransferInfo {
    fn from(byte: u8) -> Self {
        const DAP_TRANSFER_APNDP: u8 = 1 << 0;
        const DAP_TRANSFER_RNW: u8 = 1 << 1;
        const DAP_TRANSFER_A2: u8 = 1 << 2;
        const DAP_TRANSFER_A3: u8 = 1 << 3;
        const DAP_TRANSFER_MATCH_VALUE: u8 = 1 << 4;
        const DAP_TRANSFER_MATCH_MASK: u8 = 1 << 5;
        const DAP_TRANSFER_TIMESTAMP: u8 = 1 << 7;

        Self {
            ap_ndp: if byte & DAP_TRANSFER_APNDP != 0 {
                APnDP::AP
            } else {
                APnDP::DP
            },
            r_nw: if byte & DAP_TRANSFER_RNW != 0 {
                RnW::R
            } else {
                RnW::W
            },
            a2a3: ((byte & DAP_TRANSFER_A2) >> 2) | ((byte & DAP_TRANSFER_A3) >> 3),
            match_value: byte & DAP_TRANSFER_MATCH_VALUE != 0,
            match_mask: byte & DAP_TRANSFER_MATCH_MASK != 0,
            timestamp: byte & DAP_TRANSFER_TIMESTAMP != 0,
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapConfig {
    /// The number of bits in the IR register.
    pub ir_length: u8,
    /// The number of bypass bits before the IR register.
    pub ir_before: u16,
    /// The number of bypass bits after the IR register.
    pub ir_after: u16,
}

impl TapConfig {
    /// Empty value for array initialization
    pub const INIT: Self = Self {
        ir_length: 0,
        ir_before: 0,
        ir_after: 0,
    };
}

/// JTAG interface configuraiton.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The number of devices on the JTAG chain.
    pub device_count: u8,
    /// The position of the selected device.
    pub index: u8,
    /// TAPs on the scan chain.
    pub scan_chain: &'static mut [TapConfig],
}

impl Config {
    pub fn new(chain_buffer: &'static mut [TapConfig]) -> Self {
        Self {
            device_count: 0,
            index: 0,
            scan_chain: chain_buffer,
        }
    }

    /// Returns information about the currently selected TAP.
    pub fn current_tap(&self) -> TapConfig {
        self.scan_chain[self.index as usize]
    }

    pub(crate) fn update_device_count(&mut self, count: u8) -> bool {
        if count as usize >= self.scan_chain.len() {
            return false;
        }
        self.device_count = count;
        true
    }
}

impl Config {
    /// Selects the device at the given index.
    ///
    /// If the index is out of bounds, it returns `false` and does not change the
    /// selected device.
    pub fn select_index(&mut self, index: u8) -> bool {
        if index >= self.device_count {
            return false;
        }
        self.index = index;
        true
    }
}

pub trait Jtag<DEPS>: From<DEPS> {
    /// If JTAG is available or not.
    const AVAILABLE: bool;

    /// Returns a mutable reference to the JTAG interface configuration.
    fn config(&mut self) -> &mut Config;

    /// Handle a JTAG sequence request.
    ///
    /// The implementor is responsible for parsing the request. The first byte contains
    /// the number of sequences to process. Each sequence is described by a byte, which
    /// contains the number of bits to shift in/out, whether to capture TDO, and the TMS
    /// level to output. This info byte may be parsed by [`SequenceInfo`]'s `From<u8>`
    /// implementation.
    fn sequences(&mut self, mut req: dap::Request, resp: &mut dap::ResponseWriter) {
        // Run requested JTAG sequences. Cannot fail.
        let sequence_count = req.next_u8();

        for _ in 0..sequence_count {
            let sequence_info = SequenceInfo::from(req.next_u8());
            let n_bytes = sequence_info.n_bits.div_ceil(8) as usize;
            self.sequence(sequence_info, &req.data[..n_bytes], resp.remaining());

            req.consume(n_bytes);
            if sequence_info.capture {
                resp.skip(n_bytes);
            }
        }
    }

    /// Handle a single JTAG sequence.
    ///
    /// This function handles the individual JTAG sequences that are broken up by
    /// the `sequences` function. It is called for each sequence in the request.
    ///
    /// For better performance, you may choose to provide an empty implementation
    /// of this function and handle the sequences by overriding `sequences` instead.
    fn sequence(&mut self, info: SequenceInfo, tdi: &[u8], rxbuf: &mut [u8]);

    /// Send out a sequence of TMS bits, while keeping TDI unchanged.
    fn tms_sequence(&mut self, tms: &[bool]);

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;

    /// Shift out the instruction register (IR).
    fn shift_ir(&mut self, ir: u32) {
        const IDLE_TO_SHIFT_IR: &[bool] = &[true, true, false, false];
        const EXIT1_IR_TO_IDLE: &[bool] = &[true, false];
        self.tms_sequence(IDLE_TO_SHIFT_IR);

        let tap = self.config().current_tap();
        let ir_length = tap.ir_length;
        let bypass_before = tap.ir_before;
        let bypass_after = tap.ir_after;

        // Send the bypass bits before the IR.
        bypass_bits(self, bypass_before, false);

        // Send the IR bits.
        let mut ir = ir;

        // All bits except last with TMS = 0
        let mut ir_bits = ir_length;
        while ir_bits > 1 {
            let bits = ir_bits.min(8);

            self.sequence(
                SequenceInfo {
                    n_bits: bits,
                    capture: false,
                    tms: false,
                },
                &[ir as u8],
                &mut [],
            );

            ir >>= bits;
            ir_bits -= bits;
        }

        // Last bit with TMS = 1
        self.sequence(
            SequenceInfo {
                n_bits: 1,
                capture: false,
                tms: bypass_after == 0,
            },
            &[ir as u8],
            &mut [],
        );

        if bypass_after > 0 {
            if bypass_after > 1 {
                // Send the bypass bits after the IR.
                bypass_bits(self, bypass_after.saturating_sub(1), false);
            }

            // Send the last bypass bit with TMS = 1
            bypass_bits(self, bypass_after, true);
        }

        self.tms_sequence(EXIT1_IR_TO_IDLE);
    }

    /// Shift out the data register (DR) and return the captured bits.
    fn shift_dr(&mut self, dr: u32) -> u32 {
        const IDLE_TO_SHIFT_DR: &[bool] = &[true, false, false];
        const EXIT1_DR_TO_IDLE: &[bool] = &[true, false];
        self.tms_sequence(IDLE_TO_SHIFT_DR);

        let device_index = self.config().index as usize;
        let device_count = self.config().device_count as usize;
        let bypass_before = device_index as u16;
        let bypass_after = device_count as u16 - bypass_before - 1;

        // Send the bypass bits before the DR.
        bypass_bits(self, bypass_before, false);

        // Send the DR bits.
        shift_dr(self, dr, bypass_after)
    }

    /// Shift out the data register (DR) for an ABORT command.
    fn write_abort(&mut self, data: u32) {
        const IDLE_TO_SHIFT_DR: &[bool] = &[true, false, false];
        const EXIT1_DR_TO_IDLE: &[bool] = &[true, false];
        self.tms_sequence(IDLE_TO_SHIFT_DR);

        let device_index = self.config().index as usize;
        let device_count = self.config().device_count as usize;
        let bypass_before = device_index as u16;
        let bypass_after = device_count as u16 - bypass_before - 1;

        // Send the bypass bits before the DR.
        bypass_bits(self, bypass_before, false);

        // RnW=0, A2=0, A3=0, ignore ACK
        self.sequence(
            SequenceInfo {
                n_bits: 3,
                capture: false,
                tms: false,
            },
            &[0],
            &mut [],
        );

        shift_dr(self, data, bypass_after);
    }

    /// Shift out the data register (DR) for an ABORT command.
    fn transfer(&mut self, req: TransferInfo, data: u32) -> TransferResult {
        const IDLE_TO_SHIFT_DR: &[bool] = &[true, false, false];
        const SHIFT_DR_TO_IDLE: &[bool] = &[true, true, false];
        const EXIT1_DR_TO_IDLE: &[bool] = &[true, false];
        self.tms_sequence(IDLE_TO_SHIFT_DR);

        let device_index = self.config().index as usize;
        let device_count = self.config().device_count as usize;
        let bypass_before = device_index as u16;
        let bypass_after = device_count as u16 - bypass_before - 1;

        // Send the bypass bits before the DR.
        bypass_bits(self, bypass_before, false);

        // RnW=0, A2=0, A3=0
        let mut ack = 0;
        self.sequence(
            SequenceInfo {
                n_bits: 3,
                capture: true,
                tms: false,
            },
            &[req.a2a3 << 1 | req.r_nw as u8],
            core::slice::from_mut(&mut ack),
        );

        const DAP_TRANSFER_OK: u8 = 0x01;
        const DAP_TRANSFER_WAIT: u8 = 0x02;
        const DAP_TRANSFER_FAULT: u8 = 0x03;

        if ack == DAP_TRANSFER_OK {
            let captured_dr = shift_dr(self, data, bypass_after);

            TransferResult::Ok(captured_dr)
        } else {
            self.tms_sequence(SHIFT_DR_TO_IDLE);
            if ack == DAP_TRANSFER_WAIT {
                TransferResult::Wait
            } else {
                TransferResult::Fault
            }
        }
    }
}

fn shift_dr<DEPS>(jtag: &mut impl Jtag<DEPS>, data: u32, bypass_after: u16) -> u32 {
    // Send the DR bits.
    let mut dr = data;

    // All bits except last with TMS = 0
    let mut captured_dr = 0;
    let mut dr_bits = 32;
    while dr_bits > 1 {
        let bits = dr_bits.min(8);

        let mut captured_byte = 0;
        jtag.sequence(
            SequenceInfo {
                n_bits: bits,
                capture: true,
                tms: false,
            },
            &[dr as u8],
            core::slice::from_mut(&mut captured_byte),
        );
        captured_dr >>= bits;
        captured_dr |= (captured_byte as u32) << (32 - bits);

        dr >>= bits;
        dr_bits -= bits;
    }

    // Last bit (with TMS = 1 if bypass_after == 0)
    let mut captured_byte = 0;
    jtag.sequence(
        SequenceInfo {
            n_bits: 1,
            capture: true,
            tms: bypass_after == 0,
        },
        &[dr as u8],
        core::slice::from_mut(&mut captured_byte),
    );

    captured_dr >>= 1;
    captured_dr |= (captured_byte as u32) << 31;

    if bypass_after > 0 {
        if bypass_after > 1 {
            // Send the bypass bits after the DR.
            bypass_bits(jtag, bypass_after.saturating_sub(1), false);
        }

        // Send the last bypass bit with TMS = 1
        bypass_bits(jtag, bypass_after, true);
    }

    captured_dr
}

fn bypass_bits<DEPS>(jtag: &mut impl Jtag<DEPS>, mut bypass: u16, tms: bool) {
    while bypass > 0 {
        let bits = bypass.min(8);
        bypass -= bits;

        jtag.sequence(
            SequenceInfo {
                n_bits: bits as u8,
                capture: false,
                tms,
            },
            &[0xFF],
            &mut [],
        );
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferResult {
    Ok(u32),
    Wait,
    Nack,
    Fault,
    Mismatch,
}
impl TransferResult {
    pub(crate) fn status(&self) -> u8 {
        match self {
            TransferResult::Ok(_) => 0x1,
            TransferResult::Wait => 0x2,
            TransferResult::Nack => 0x4,
            TransferResult::Fault => 0x8,
            TransferResult::Mismatch => 0x10,
        }
    }
}
