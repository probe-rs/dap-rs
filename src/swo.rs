use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SwoTransport {
    None = 0,
    DAPCommand = 1,
    USBEndpoint = 2,
}

#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SwoMode {
    Off = 0,
    UART = 1,
    Manchester = 2,
}

#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SwoControl {
    Stop = 0,
    Start = 1,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SwoSupport {
    /// `true` if the UART encoding is supported.
    pub uart: bool,
    /// `true` if the Manchester encoding is supported.
    pub manchester: bool,
    /// `true` if the USB interface has a trace endpoint.
    pub streaming: bool,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SwoStatus {
    pub active: bool,
    pub trace_error: bool,
    pub trace_overrun: bool,
    pub bytes_available: u32,
}

pub trait Swo {
    fn set_transport(&mut self, transport: SwoTransport) -> bool;
    fn set_mode(&mut self, mode: SwoMode) -> bool;
    fn set_baudrate(&mut self, baudrate: u32) -> u32;
    fn set_control(&mut self, control: SwoControl) -> bool;
    fn polling_data(&mut self, buf: &mut [u8]) -> u32;
    fn streaming_data(&mut self); //  -> SomeBufferFromStreaming; // TODO: What is a good interface?
    fn is_active(&self) -> bool;
    fn bytes_available(&self) -> u32;
    fn buffer_size(&self) -> u32;
    fn support(&self) -> SwoSupport;
    fn status(&mut self) -> SwoStatus;
}

/// Marker struct for no SWO support.
pub struct NoSwo;

impl Swo for NoSwo {
    fn set_transport(&mut self, _transport: SwoTransport) -> bool {
        false
    }
    fn set_mode(&mut self, _mode: SwoMode) -> bool {
        false
    }
    fn set_baudrate(&mut self, _baudrate: u32) -> u32 {
        0
    }
    fn set_control(&mut self, _control: SwoControl) -> bool {
        false
    }
    fn polling_data(&mut self, _buf: &mut [u8]) -> u32 {
        0
    }
    fn streaming_data(&mut self) {}
    fn is_active(&self) -> bool {
        false
    }
    fn bytes_available(&self) -> u32 {
        0
    }
    fn buffer_size(&self) -> u32 {
        0
    }
    fn support(&self) -> SwoSupport {
        SwoSupport {
            uart: false,
            manchester: false,
            streaming: false,
        }
    }
    fn status(&mut self) -> SwoStatus {
        SwoStatus {
            active: false,
            trace_error: false,
            trace_overrun: false,
            bytes_available: 0,
        }
    }
}
