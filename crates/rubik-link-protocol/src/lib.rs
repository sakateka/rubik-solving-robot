#![no_std]

//! Shared wire protocol for the phone, ESP32-C6, and Milk-V Duo.
//!
//! The inner packet is transport-neutral. UART wraps it with COBS and appends
//! a zero delimiter; BLE transports the same inner packet using its own message
//! or fragmentation boundaries.

mod cobs;
mod crc;
mod wire;

pub use wire::{
    decode_packet, encode_packet, frame_uart, parse_uart_frame, MessageKind, Packet, WireError,
    CRC_LEN, HEADER_LEN, MAX_PACKET_LEN, MAX_PAYLOAD_LEN, MAX_UART_FRAME_LEN, PROTOCOL_VERSION,
    UART_DELIMITER,
};
