pub const DAP1_PACKET_SIZE: u16 = 64;

#[cfg(feature = "usb-hs")]
pub const DAP2_PACKET_SIZE: u16 = 512;
#[cfg(not(feature = "usb-hs"))]
pub const DAP2_PACKET_SIZE: u16 = 64;

pub mod dap_v1;
pub mod dap_v2;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::large_enum_variant)]
pub enum Request {
    Suspend,
    DAP1Command(([u8; DAP1_PACKET_SIZE as usize], usize)),
    DAP2Command(([u8; DAP2_PACKET_SIZE as usize], usize)),
}
