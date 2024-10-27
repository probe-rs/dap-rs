use crate::{jtag, swd, swj, swo, usb};

mod command;
mod request;
mod response;
mod state;

pub use command::*;
pub use request::*;
pub use response::*;

pub use embedded_hal::delay::DelayNs;

use state::State;

/// LED control trait.
pub trait DapLeds {
    /// React to host status, usually setting some LEDs
    fn react_to_host_status(&mut self, host_status: HostStatus);
}

/// DAP handler.
pub struct Dap<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO> {
    state: State<DEPS, SWD, JTAG>,
    swo: Option<SWO>,
    swo_streaming: bool,
    swd_wait_retries: usize,
    match_retries: usize,
    version_string: &'a str,
    // mode: Option<DapMode>,
    leds: LEDS,
    wait: WAIT,
}

impl<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO> Dap<'a, DEPS, LEDS, WAIT, JTAG, SWD, SWO>
where
    DEPS: swj::Dependencies<SWD, JTAG>,
    LEDS: DapLeds,
    WAIT: DelayNs,
    JTAG: jtag::Jtag<DEPS>,
    SWD: swd::Swd<DEPS>,
    SWO: swo::Swo,
{
    /// Create a Dap handler
    pub fn new(
        dependencies: DEPS,
        leds: LEDS,
        wait: WAIT,
        swo: Option<SWO>,
        version_string: &'a str,
    ) -> Self {
        // TODO: Replace with const assert
        assert!(SWD::AVAILABLE || JTAG::AVAILABLE);

        Dap {
            state: State::new(dependencies),
            swo,
            swo_streaming: false,
            swd_wait_retries: 5,
            match_retries: 8,
            version_string,
            // mode: None,
            leds,
            wait,
        }
    }

    /// Process a new CMSIS-DAP command from `report`.
    ///
    /// Returns number of bytes written to response buffer.
    pub fn process_command(
        &mut self,
        report: &[u8],
        rbuf: &mut [u8],
        version: DapVersion,
    ) -> usize {
        let req = match Request::from_report(report) {
            Some(req) => req,
            None => return 0,
        };

        let resp = &mut ResponseWriter::new(req.command, rbuf);

        // defmt::trace!("Dap command: {}", req.command);

        match req.command {
            Command::DAP_Info => self.process_info(req, resp, version),
            Command::DAP_HostStatus => self.process_host_status(req, resp),
            Command::DAP_Connect => self.process_connect(req, resp),
            Command::DAP_Disconnect => self.process_disconnect(req, resp),
            Command::DAP_WriteABORT => self.process_write_abort(req, resp),
            Command::DAP_Delay => self.process_delay(req, resp),
            Command::DAP_ResetTarget => self.process_reset_target(req, resp),
            Command::DAP_SWJ_Pins => self.process_swj_pins(req, resp),
            Command::DAP_SWJ_Clock => self.process_swj_clock(req, resp),
            Command::DAP_SWJ_Sequence => self.process_swj_sequence(req, resp),
            Command::DAP_SWD_Configure => self.process_swd_configure(req, resp),
            Command::DAP_SWD_Sequence => self.process_swd_sequence(req, resp),
            Command::DAP_SWO_Transport => self.process_swo_transport(req, resp),
            Command::DAP_SWO_Mode => self.process_swo_mode(req, resp),
            Command::DAP_SWO_Baudrate => self.process_swo_baudrate(req, resp),
            Command::DAP_SWO_Control => self.process_swo_control(req, resp),
            Command::DAP_SWO_Status => self.process_swo_status(req, resp),
            Command::DAP_SWO_ExtendedStatus => self.process_swo_extended_status(req, resp),
            Command::DAP_SWO_Data => self.process_swo_data(req, resp),
            Command::DAP_JTAG_Configure => self.process_jtag_configure(req, resp),
            Command::DAP_JTAG_IDCODE => self.process_jtag_idcode(req, resp),
            Command::DAP_JTAG_Sequence => self.process_jtag_sequence(req, resp),
            Command::DAP_TransferConfigure => self.process_transfer_configure(req, resp),
            Command::DAP_Transfer => self.process_transfer(req, resp),
            Command::DAP_TransferBlock => self.process_transfer_block(req, resp),
            Command::DAP_TransferAbort => {
                self.process_transfer_abort();
                // Do not send a response for transfer abort commands
                return 0;
            }
            Command::DAP_ExecuteCommands => self.process_execute_commands(req, resp),
            Command::DAP_QueueCommands => self.process_queue_commands(req, resp),
            Command::Unimplemented => {}
        }

        resp.idx
    }

    /// Suspend the interface.
    pub fn suspend(&mut self) {
        self.state.to_none();

        if let State::None { deps, .. } = &mut self.state {
            deps.high_impedance_mode();
        } else {
            unreachable!();
        }
    }

    fn process_info(&mut self, mut req: Request, resp: &mut ResponseWriter, version: DapVersion) {
        match DapInfoID::try_from(req.next_u8()) {
            // Return 0-length string for VendorID, ProductID, SerialNumber
            // to indicate they should be read from USB descriptor instead
            Ok(DapInfoID::VendorID) => resp.write_u8(0),
            Ok(DapInfoID::ProductID) => resp.write_u8(0),
            Ok(DapInfoID::SerialNumber) => resp.write_u8(0),
            // Return git version as firmware version
            Ok(DapInfoID::FirmwareVersion) => {
                resp.write_u8(self.version_string.len() as u8);
                resp.write_slice(self.version_string.as_bytes());
            }
            // Return 0-length string for TargetVendor and TargetName to indicate
            // unknown target device.
            Ok(DapInfoID::TargetVendor) => resp.write_u8(0),
            Ok(DapInfoID::TargetName) => resp.write_u8(0),
            Ok(DapInfoID::Capabilities) => {
                resp.write_u8(1);
                // Bit 0: SWD supported
                // Bit 1: JTAG supported
                // Bit 2: SWO UART supported
                // Bit 3: SWO Manchester not supported
                // Bit 4: Atomic commands not supported
                // Bit 5: Test Domain Timer not supported
                // Bit 6: SWO Streaming Trace supported
                let swd = (SWD::AVAILABLE as u8) << 0;
                let jtag = (JTAG::AVAILABLE as u8) << 1;
                let swo = match &self.swo {
                    Some(swo) => {
                        let support = swo.support();
                        (support.uart as u8) << 2 | (support.manchester as u8) << 3
                    }
                    None => 0,
                };
                let atomic = 0 << 4;
                let swo_streaming = 1 << 6;
                resp.write_u8(swd | jtag | swo | atomic | swo_streaming);
            }
            Ok(DapInfoID::SWOTraceBufferSize) => {
                resp.write_u8(4);
                let size = match &self.swo {
                    Some(swo) => swo.buffer_size(),
                    None => 0,
                };
                resp.write_u32(size as u32);
            }
            Ok(DapInfoID::MaxPacketCount) => {
                resp.write_u8(1);
                // Maximum of one packet at a time
                resp.write_u8(1);
            }
            Ok(DapInfoID::MaxPacketSize) => {
                resp.write_u8(2);
                match version {
                    DapVersion::V1 => {
                        // Maximum of 64 bytes per packet
                        resp.write_u16(usb::DAP1_PACKET_SIZE);
                    }
                    DapVersion::V2 => {
                        // Maximum of 512 bytes per packet
                        resp.write_u16(usb::DAP2_PACKET_SIZE);
                    }
                }
            }
            _ => resp.write_u8(0),
        }
    }

    fn process_host_status(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let status_type = req.next_u8();
        let status_status = req.next_u8();
        // Use HostStatus to set our LED when host is connected to target
        if let Ok(status) = HostStatusType::try_from(status_type) {
            let status_value = status_status != 0;
            let status = match status {
                HostStatusType::Connect => HostStatus::Connected(status_value),
                HostStatusType::Running => HostStatus::Running(status_value),
            };

            self.leds.react_to_host_status(status);
        }
        resp.write_u8(0);
    }

    fn process_connect(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let port = req.next_u8();
        let port = match ConnectPort::try_from(port) {
            Ok(port) => port,
            Err(_) => {
                resp.write_u8(ConnectPortResponse::Failed as u8);
                return;
            }
        };

        // defmt::info!(
        // "DAP connect: {}, SWD: {}, JTAG: {}",
        // port,
        // SWD::AVAILABLE,
        // JTAG::AVAILABLE
        // );

        match (SWD::AVAILABLE, JTAG::AVAILABLE, port) {
            // SWD
            (true, true, ConnectPort::Default)
            | (true, true, ConnectPort::SWD)
            | (true, false, ConnectPort::Default)
            | (true, false, ConnectPort::SWD) => {
                self.state.to_swd();
                resp.write_u8(ConnectPortResponse::SWD as u8);
            }

            // JTAG
            (true, true, ConnectPort::JTAG)
            | (false, true, ConnectPort::Default)
            | (false, true, ConnectPort::JTAG) => {
                self.state.to_jtag();
                resp.write_u8(ConnectPortResponse::JTAG as u8);
            }

            // Error (tried to connect JTAG or SWD when not available)
            (true, false, ConnectPort::JTAG) | (false, true, ConnectPort::SWD) => {
                resp.write_u8(ConnectPortResponse::Failed as u8);
            }

            // Checked by `new`
            (false, false, _) => unreachable!(),
        }
    }

    fn process_disconnect(&mut self, _req: Request, resp: &mut ResponseWriter) {
        self.state.to_none();

        if let State::None { deps, .. } = &mut self.state {
            deps.high_impedance_mode();
        } else {
            unreachable!();
        }

        resp.write_ok();
    }

    fn process_write_abort(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        self.state.to_last_mode();

        let idx = req.next_u8();
        let word = req.next_u32();
        match (SWD::AVAILABLE, JTAG::AVAILABLE, &mut self.state) {
            (_, true, State::Jtag(_jtag)) => {
                // TODO: Implement one day.
                let _ = idx;
                resp.write_err();
            }
            (true, _, State::Swd(swd)) => {
                match swd.write_dp(self.swd_wait_retries, swd::DPRegister::DPIDR, word) {
                    Ok(_) => resp.write_ok(),
                    Err(_) => resp.write_err(),
                }
            }
            _ => {
                resp.write_err();
            }
        }
    }

    fn process_delay(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let delay = req.next_u16() as u32;
        self.wait.delay_us(delay);
        resp.write_ok();
    }

    fn process_reset_target(&mut self, _req: Request, resp: &mut ResponseWriter) {
        resp.write_ok();
        // "No device specific reset sequence is implemented"
        resp.write_u8(0);
    }

    fn process_swj_pins(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let output = swj::Pins::from_bits_truncate(req.next_u8());
        let mask = swj::Pins::from_bits_truncate(req.next_u8());
        let wait_us = req.next_u32().min(3_000_000); // Defined as max 3 seconds

        self.state.to_none();

        if let State::None { deps, .. } = &mut self.state {
            resp.write_u8(deps.process_swj_pins(output, mask, wait_us).bits());
        } else {
            unreachable!();
        }
    }

    fn process_swj_clock(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let max_frequency = req.next_u32();
        let valid = self.state.set_clock(max_frequency);

        if valid {
            resp.write_ok();
        } else {
            resp.write_err();
        }
    }

    fn process_swj_sequence(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let nbits: usize = match req.next_u8() {
            // CMSIS-DAP says 0 means 256 bits
            0 => 256,
            // Other integers are normal.
            n => n as usize,
        };

        let payload = req.rest();
        let nbytes = (nbits + 7) / 8;
        let seq = if nbytes <= payload.len() {
            &payload[..nbytes]
        } else {
            resp.write_err();
            return;
        };

        self.state.to_none();

        if let State::None { deps, .. } = &mut self.state {
            deps.process_swj_sequence(seq, nbits);
        } else {
            unreachable!();
        }

        resp.write_ok();
    }

    fn process_swd_configure(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        // TODO: Do we want to support other configs?
        let config = req.next_u8();
        let clk_period = config & 0b011;
        let always_data = (config & 0b100) != 0;
        if clk_period == 0 && !always_data {
            resp.write_ok();
        } else {
            resp.write_err();
        }
    }

    fn process_swd_sequence(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let sequence_info = req.next_u8();

        let nbits: usize = match sequence_info & 0x3F {
            // CMSIS-DAP says 0 means 64 bits
            0 => 64,
            // Other integers are normal.
            n => n as usize,
        };
        let output = (sequence_info & 0x80) == 0;

        if output {
            self.process_swd_sequence_output(req, resp, nbits);
        } else {
            self.process_swd_sequence_input(req, resp, nbits);
        }
    }

    fn process_swd_sequence_output(
        &mut self,
        req: Request,
        resp: &mut ResponseWriter,
        nbits: usize,
    ) {
        let payload = req.rest();
        let nbytes = (nbits + 7) / 8;
        let seq = if nbytes <= payload.len() {
            &payload[..nbytes]
        } else {
            resp.write_err();
            return;
        };

        self.state.to_swd();

        match &mut self.state {
            State::Swd(swd) => {
                if swd.write_sequence(nbits, seq).is_ok() {
                    resp.write_ok();
                } else {
                    resp.write_err();
                }
            }
            _ => resp.write_err(),
        }
    }

    fn process_swd_sequence_input(
        &mut self,
        _req: Request,
        resp: &mut ResponseWriter,
        nbits: usize,
    ) {
        let payload = resp.remaining();
        let nbytes = (nbits + 7) / 8;
        let buf = if nbytes + 1 <= payload.len() {
            &mut payload[1..nbytes + 1]
        } else {
            resp.write_err();
            return;
        };

        self.state.to_swd();

        match &mut self.state {
            State::Swd(swd) => {
                if swd.read_sequence(nbits, buf).is_ok() {
                    resp.write_ok();
                    resp.skip(nbytes);
                } else {
                    resp.write_err();
                }
            }
            _ => resp.write_err(),
        }
    }

    fn process_swo_transport(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let transport = req.next_u8();
        match swo::SwoTransport::try_from(transport) {
            Ok(swo::SwoTransport::None) | Ok(swo::SwoTransport::DAPCommand) => {
                self.swo_streaming = false;
                resp.write_ok();
            }
            Ok(swo::SwoTransport::USBEndpoint) => {
                self.swo_streaming = true;
                resp.write_ok();
            }
            _ => resp.write_err(),
        }
    }

    fn process_swo_mode(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let mode = req.next_u8();

        let swo = if let Some(swo) = &mut self.swo {
            swo
        } else {
            resp.write_err();
            return;
        };

        match swo::SwoMode::try_from(mode) {
            Ok(mode) => {
                swo.set_mode(mode);
                resp.write_ok();
            }
            _ => resp.write_err(),
        }
    }

    fn process_swo_baudrate(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let target = req.next_u32();
        let actual = if let Some(swo) = &mut self.swo {
            swo.set_baudrate(target)
        } else {
            0
        };
        resp.write_u32(actual);
    }

    fn process_swo_control(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let swo = if let Some(swo) = &mut self.swo {
            swo
        } else {
            resp.write_err();
            return;
        };

        match swo::SwoControl::try_from(req.next_u8()) {
            Ok(control) => {
                swo.set_control(control);
                resp.write_ok();
            }
            _ => resp.write_err(),
        }
    }

    fn process_swo_status(&mut self, _req: Request, resp: &mut ResponseWriter) {
        // Trace status:
        // Bit 0: trace capture active
        // Bit 6: trace stream error (always written as 0)
        // Bit 7: trace buffer overflow (always written as 0)
        let (active, len) = if let Some(swo) = &self.swo {
            (swo.is_active(), swo.bytes_available())
        } else {
            (false, 0)
        };

        resp.write_u8(active as u8);
        // Trace count: remaining bytes in buffer
        resp.write_u32(len);
    }

    fn process_swo_extended_status(&mut self, _req: Request, resp: &mut ResponseWriter) {
        // Trace status:
        // Bit 0: trace capture active
        // Bit 6: trace stream error (always written as 0)
        // Bit 7: trace buffer overflow (always written as 0)
        let (active, len) = if let Some(swo) = &self.swo {
            (swo.is_active(), swo.bytes_available())
        } else {
            (false, 0)
        };
        resp.write_u8(active as u8);
        // Trace count: remaining bytes in buffer.
        resp.write_u32(len);
        // Index: sequence number of next trace. Always written as 0.
        resp.write_u32(0);
        // TD_TimeStamp: test domain timer value for trace sequence
        resp.write_u32(0);
    }

    fn process_swo_data(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        let active = if let Some(swo) = &mut self.swo {
            swo.is_active()
        } else {
            false
        };

        // Write status byte to response
        resp.write_u8(active as u8);

        // Skip length for now
        resp.skip(2);

        let mut buf = resp.remaining();

        // Limit maximum return size to maximum requested bytes
        let n = req.next_u16() as usize;
        if buf.len() > n {
            buf = &mut buf[..n];
        }

        // Read data from UART
        let len = if let Some(swo) = &mut self.swo {
            swo.polling_data(&mut buf)
        } else {
            0
        };
        resp.skip(len as _);

        // Go back and write length
        resp.write_u16_at(2, len as u16);
    }

    fn process_jtag_sequence(&mut self, req: Request, resp: &mut ResponseWriter) {
        self.state.to_jtag();

        match &mut self.state {
            State::Jtag(jtag) => {
                // Run requested JTAG sequences. Cannot fail.
                let size = jtag.sequences(req.rest(), resp.remaining());
                resp.skip(size as _);

                resp.write_ok();
            }
            _ => resp.write_err(),
        };
    }

    fn process_jtag_configure(&self, _req: Request, _resp: &mut ResponseWriter) {
        // TODO: Implement one day (needs proper JTAG support)
    }

    fn process_jtag_idcode(&self, _req: Request, _resp: &mut ResponseWriter) {
        // TODO: Implement one day (needs proper JTAG support)
    }

    fn process_transfer_configure(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        // We don't support variable idle cycles
        // TODO: Should we?
        let _idle_cycles = req.next_u8();

        // Send number of wait retries through to SWD
        self.swd_wait_retries = req.next_u16() as usize;

        // Store number of match retries
        self.match_retries = req.next_u16() as usize;

        resp.write_ok();
    }

    fn process_transfer(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        self.state.to_last_mode();

        let _idx = req.next_u8();
        let ntransfers = req.next_u8();
        let mut match_mask = 0xFFFF_FFFFu32;

        match &mut self.state {
            State::Jtag(_jtag) => {
                // TODO: Implement one day.
            }
            State::Swd(swd) => {
                // Skip two bytes in resp to reserve space for final status,
                // which we update while processing.
                resp.write_u16(0);

                for transfer_idx in 0..ntransfers {
                    // Store how many transfers we execute in the response
                    resp.write_u8_at(1, transfer_idx + 1);

                    // Parse the next transfer request
                    let transfer_req = req.next_u8();
                    let apndp = swd::APnDP::try_from(transfer_req & (1 << 0)).unwrap();
                    let rnw = swd::RnW::try_from((transfer_req & (1 << 1)) >> 1).unwrap();
                    let a = swd::DPRegister::try_from((transfer_req & (3 << 2)) >> 2).unwrap();
                    let vmatch = (transfer_req & (1 << 4)) != 0;
                    let mmask = (transfer_req & (1 << 5)) != 0;
                    let _ts = (transfer_req & (1 << 7)) != 0;

                    if rnw == swd::RnW::R {
                        // Issue register read
                        let mut read_value = if apndp == swd::APnDP::AP {
                            // Reads from AP are posted, so we issue the
                            // read and subsequently read RDBUFF for the data.
                            // This requires an additional transfer so we'd
                            // ideally keep track of posted reads and just
                            // keep issuing new AP reads, but our reads are
                            // sufficiently fast that for now this is simpler.
                            let rdbuff = swd::DPRegister::RDBUFF;
                            if swd
                                .read_ap(self.swd_wait_retries, a)
                                .check(resp.mut_at(2))
                                .is_none()
                            {
                                break;
                            }
                            match swd
                                .read_dp(self.swd_wait_retries, rdbuff)
                                .check(resp.mut_at(2))
                            {
                                Some(v) => v,
                                None => break,
                            }
                        } else {
                            // Reads from DP are not posted, so directly read the register.
                            match swd.read_dp(self.swd_wait_retries, a).check(resp.mut_at(2)) {
                                Some(v) => v,
                                None => break,
                            }
                        };

                        // Handle value match requests by retrying if needed.
                        // Since we're re-reading the same register the posting
                        // is less important and we can just use the returned value.
                        if vmatch {
                            let target_value = req.next_u32();
                            let mut match_tries = 0;
                            while (read_value & match_mask) != target_value {
                                match_tries += 1;
                                if match_tries > self.match_retries {
                                    break;
                                }

                                read_value = match swd
                                    .read(self.swd_wait_retries, apndp.into(), a)
                                    .check(resp.mut_at(2))
                                {
                                    Some(v) => v,
                                    None => break,
                                }
                            }

                            // If we didn't read the correct value, set the value mismatch
                            // flag in the response and quit early.
                            if (read_value & match_mask) != target_value {
                                resp.write_u8_at(1, resp.read_u8_at(1) | (1 << 4));
                                break;
                            }
                        } else {
                            // Save read register value
                            resp.write_u32(read_value);
                        }
                    } else {
                        // Write transfer processing

                        // Writes with match_mask set just update the match mask
                        if mmask {
                            match_mask = req.next_u32();
                            continue;
                        }

                        // Otherwise issue register write
                        let write_value = req.next_u32();
                        if swd
                            .write(self.swd_wait_retries, apndp, a, write_value)
                            .check(resp.mut_at(2))
                            .is_none()
                        {
                            break;
                        }
                    }
                }
            }
            _ => return,
        }
    }

    fn process_transfer_block(&mut self, mut req: Request, resp: &mut ResponseWriter) {
        self.state.to_last_mode();

        let _idx = req.next_u8();
        let ntransfers = req.next_u16();
        let transfer_req = req.next_u8();
        let apndp = swd::APnDP::try_from(transfer_req & (1 << 0)).unwrap();
        let rnw = swd::RnW::try_from((transfer_req & (1 << 1)) >> 1).unwrap();
        let a = swd::DPRegister::try_from((transfer_req & (3 << 2)) >> 2).unwrap();

        match &mut self.state {
            State::Jtag(_jtag) => {
                // TODO: Implement one day.
            }
            State::Swd(swd) => {
                // Skip three bytes in resp to reserve space for final status,
                // which we update while processing.
                resp.write_u16(0);
                resp.write_u8(0);

                // Keep track of how many transfers we executed,
                // so if there is an error the host knows where
                // it happened.
                let mut transfers = 0;

                // If reading an AP register, post first read early.
                if rnw == swd::RnW::R
                    && apndp == swd::APnDP::AP
                    && swd
                        .read_ap(self.swd_wait_retries, a)
                        .check(resp.mut_at(3))
                        .is_none()
                {
                    // Quit early on error
                    resp.write_u16_at(1, 1);
                    return;
                }

                for transfer_idx in 0..ntransfers {
                    transfers = transfer_idx;
                    if rnw == swd::RnW::R {
                        // Handle repeated reads
                        let read_value = if apndp == swd::APnDP::AP {
                            // For AP reads, the first read was posted, so on the final
                            // read we need to read RDBUFF instead of the AP register.
                            if transfer_idx < ntransfers - 1 {
                                match swd.read_ap(self.swd_wait_retries, a).check(resp.mut_at(3)) {
                                    Some(v) => v,
                                    None => break,
                                }
                            } else {
                                let rdbuff = swd::DPRegister::RDBUFF.into();
                                match swd
                                    .read_dp(self.swd_wait_retries, rdbuff)
                                    .check(resp.mut_at(3))
                                {
                                    Some(v) => v,
                                    None => break,
                                }
                            }
                        } else {
                            // For DP reads, no special care required
                            match swd.read_dp(self.swd_wait_retries, a).check(resp.mut_at(3)) {
                                Some(v) => v,
                                None => break,
                            }
                        };

                        // Save read register value to response
                        resp.write_u32(read_value);
                    } else {
                        // Handle repeated register writes
                        let write_value = req.next_u32();
                        let result = swd.write(self.swd_wait_retries, apndp, a, write_value);
                        if result.check(resp.mut_at(3)).is_none() {
                            break;
                        }
                    }
                }

                // Write number of transfers to response
                resp.write_u16_at(1, transfers + 1);
            }
            _ => return,
        }
    }

    fn process_transfer_abort(&mut self) {
        // We'll only ever receive an abort request when we're not already
        // processing anything else, since processing blocks checking for
        // new requests. Therefore there's nothing to do here.
    }

    fn process_execute_commands(&self, _req: Request, _resp: &mut ResponseWriter) {
        // TODO: Implement one day.
    }

    fn process_queue_commands(&self, _req: Request, _resp: &mut ResponseWriter) {
        // TODO: Implement one day.
    }
}

trait CheckResult<T> {
    /// Check result of an SWD transfer, updating the response status byte.
    ///
    /// Returns Some(T) on successful transfer, None on error.
    fn check(self, resp: &mut u8) -> Option<T>;
}

impl<T> CheckResult<T> for swd::Result<T> {
    fn check(self, resp: &mut u8) -> Option<T> {
        match self {
            Ok(v) => {
                *resp = 1;
                Some(v)
            }
            Err(swd::Error::AckWait) => {
                *resp = 2;
                None
            }
            Err(swd::Error::AckFault) => {
                *resp = 4;
                None
            }
            Err(_) => {
                *resp = (1 << 3) | 7;
                None
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::mock_device::*;
    use mockall::predicate::*;

    struct FakeLEDs {}
    impl DapLeds for FakeLEDs {
        fn react_to_host_status(&mut self, _host_status: HostStatus) {}
    }

    struct StdDelayUs {}
    impl DelayNs for StdDelayUs {
        fn delay_ns(&mut self, ns: u32) {
            std::thread::sleep(std::time::Duration::from_nanos(ns as u64));
        }
    }

    type TestDap<'a> = Dap<
        'a,
        MockSwdJtagDevice,
        FakeLEDs,
        StdDelayUs,
        MockSwdJtagDevice,
        MockSwdJtagDevice,
        swo::MockSwo,
    >;

    #[test]
    fn test_swd_output_reset() {
        let mut dap = TestDap::new(
            MockSwdJtagDevice::new(),
            FakeLEDs {},
            StdDelayUs {},
            None,
            "test_dap",
        );

        let report = [0x1Du8, 52, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0];
        let mut rbuf = [0u8; 64];
        dap.state.to_swd();
        match &mut dap.state {
            State::Swd(swd) => {
                swd.expect_write_sequence()
                    .once()
                    .with(eq(52), eq([0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0]))
                    .return_const(Ok(()));
            }
            _ => assert!(false, "can't switch to swd"),
        }
        let rsize = dap.process_command(&report, &mut rbuf, DapVersion::V2);
        assert_eq!(rsize, 2);
        assert_eq!(&rbuf[..2], &[0x1Du8, 0x00])
    }

    #[test]
    fn test_swd_output_max_size() {
        let mut dap = TestDap::new(
            MockSwdJtagDevice::new(),
            FakeLEDs {},
            StdDelayUs {},
            None,
            "test_dap",
        );

        let report = [0x1Du8, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00];
        let mut rbuf = [0u8; 64];
        dap.state.to_swd();
        match &mut dap.state {
            State::Swd(swd) => {
                swd.expect_write_sequence()
                    .once()
                    .with(
                        eq(64),
                        eq([0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00]),
                    )
                    .return_const(Ok(()));
            }
            _ => assert!(false, "can't switch to swd"),
        }
        let rsize = dap.process_command(&report, &mut rbuf, DapVersion::V2);
        assert_eq!(rsize, 2);
        assert_eq!(&rbuf[..2], &[0x1Du8, 0x00])
    }

    #[test]
    fn test_swd_input() {
        let mut dap = TestDap::new(
            MockSwdJtagDevice::new(),
            FakeLEDs {},
            StdDelayUs {},
            None,
            "test_dap",
        );

        let report = [0x1Du8, 0x80 | 52];
        let mut rbuf = [0u8; 64];
        dap.state.to_swd();
        match &mut dap.state {
            State::Swd(swd) => {
                swd.expect_read_sequence()
                    .once()
                    .withf(|nbits, buf| buf.len() >= 7 && *nbits == 52)
                    .returning(|_, buf| {
                        buf[..7].clone_from_slice(&[0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0]);
                        Ok(())
                    });
            }
            _ => assert!(false, "can't switch to swd"),
        }
        let rsize = dap.process_command(&report, &mut rbuf, DapVersion::V2);
        assert_eq!(rsize, 9);
        assert_eq!(
            &rbuf[..9],
            &[0x1Du8, 0x00, 0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0]
        )
    }

    #[test]
    fn test_swd_input_max_size() {
        let mut dap = TestDap::new(
            MockSwdJtagDevice::new(),
            FakeLEDs {},
            StdDelayUs {},
            None,
            "test_dap",
        );

        let report = [0x1Du8, 0x80];
        let mut rbuf = [0u8; 64];
        dap.state.to_swd();
        match &mut dap.state {
            State::Swd(swd) => {
                swd.expect_read_sequence()
                    .once()
                    .withf(|nbits, buf| buf.len() >= 8 && *nbits == 64)
                    .returning(|_, buf| {
                        buf[..8]
                            .clone_from_slice(&[0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00]);
                        Ok(())
                    });
            }
            _ => assert!(false, "can't switch to swd"),
        }
        let rsize = dap.process_command(&report, &mut rbuf, DapVersion::V2);
        assert_eq!(rsize, 10);
        assert_eq!(
            &rbuf[..10],
            &[0x1Du8, 0x00, 0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00]
        )
    }
}
