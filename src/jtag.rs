use crate::dap;

/// Describes a JTAG sequence request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

pub trait Jtag<DEPS>: From<DEPS> {
    /// If JTAG is available or not.
    const AVAILABLE: bool;

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
            self.sequence(sequence_info, req.data, resp.remaining());

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

    /// Set the maximum clock frequency, return `true` if it is valid.
    fn set_clock(&mut self, max_frequency: u32) -> bool;

    // TODO: What is missing for the 2 other JTAG commands
}
