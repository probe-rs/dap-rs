use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
enum SWOTransport {
    None = 0,
    DAPCommand = 1,
    USBEndpoint = 2,
}

#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
enum SWOMode {
    Off = 0,
    UART = 1,
    Manchester = 2,
}

#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
enum SWOControl {
    Stop = 0,
    Start = 1,
}

pub trait Swo {
    fn configure(&mut self);
    fn set_transport(&mut self);
    fn set_mode(&mut self);
    fn set_baudrate(&mut self);
    fn control(&mut self);
    fn polling_data(&mut self);
    fn streaming_data(&mut self);
}
