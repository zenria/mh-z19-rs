//! Winsen Infrared CO2 Module MH-Z19 / MH-Z19B / MH-Z14 serial "api" implementation.
//!
//! [MH-Z19 Datasheet](https://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf)
//!
//! [MH-Z19B Datasheet](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf)
//!
//! [MH-Z14 Dahasheet](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf)
//!
//! This crates proposes two kinds of functions:
//! - functions for parsing response read from the uart
//! - functions to create command payload to send to the sensor though the uart.
//!

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use core::fmt;
#[cfg(feature = "std")]
use std::fmt;

/// MH-Z12 Commands
enum Command {
    /// Read the gas concentration
    ReadGasConcentration,
    /// Execute a zero point calibration.
    ///
    /// The sensor must be in a stable gas environment for 20 minutes with a co2 concentration of 400ppm.
    CalibrateZero,
    /// Execute a span point calibration
    CalibrateSpan,
    /// Enable or disable Automatic Baseline Correction (MH-Z19B only)
    SetAutomaticBaselineCorrection,
    /// Set the sensor range detection (2000 or 5000 MH-Z19B only)
    SetSensorDetectionRange,
}

impl Command {
    fn get_command_value(&self) -> u8 {
        use Command::*;
        match self {
            ReadGasConcentration => 0x86,
            CalibrateZero => 0x87,
            CalibrateSpan => 0x88,
            SetAutomaticBaselineCorrection => 0x79,
            SetSensorDetectionRange => 0x99,
        }
    }
}

/// Both input and output packets are 9 bytes long
pub type Packet = [u8; 9];

/// Get the command packet with proper header and checksum.
fn get_command_with_bytes34(command: Command, device_number: u8, byte3: u8, byte4: u8) -> Packet {
    let mut ret: Packet = [
        0xFF,
        device_number,
        command.get_command_value(),
        byte3,
        byte4,
        0x00,
        0x00,
        0x00,
        0x00,
    ];
    ret[8] = checksum(&ret[1..8]);
    ret
}

/// Create a command to read the gas concentration of the sensor.
pub fn read_gas_concentration(device_number: u8) -> Packet {
    get_command_with_bytes34(Command::ReadGasConcentration, device_number, 0x00, 0x00)
}

/// Create a command to enable or disable Automatic Baseline Correction (ABC)
pub fn set_automatic_baseline_correction(device_number: u8, enabled: bool) -> Packet {
    get_command_with_bytes34(
        Command::SetAutomaticBaselineCorrection,
        device_number,
        if enabled { 0xA0 } else { 0x00 },
        0x00,
    )
}

/// Create a command to calibrate the span point.
///
/// Quoting the datasheet: "Note: Pls do ZERO calibration before span calibration
/// Please make sure the sensor worked under a certain level co2 for over 20 minutes.
///
/// Suggest using 2000ppm as span, at least 1000ppm"
pub fn calibrate_span_point(device_number: u8, value: u16) -> Packet {
    get_command_with_bytes34(
        Command::CalibrateSpan,
        device_number,
        ((value & 0xff00) >> 8) as u8,
        (value & 0xff) as u8,
    )
}

/// Create a command to set the sensor detection range (MH-Z19B only).
///
/// Quoting the datasheet: "Detection range is 2000 or 5000ppm"
pub fn set_detection_range(device_number: u8, value: u16) -> Packet {
    get_command_with_bytes34(
        Command::SetSensorDetectionRange,
        device_number,
        ((value & 0xff00) >> 8) as u8,
        (value & 0xff) as u8,
    )
}

/// Create a command to calibrate the zero point.
///
/// Quoting the datasheet: "Note：Zero point is 400ppm, please make sure the sensor has
/// been worked under 400ppm for over 20 minutes"
pub fn calibrate_zero_point(device_number: u8) -> Packet {
    get_command_with_bytes34(Command::CalibrateZero, device_number, 0x00, 0x00)
}

/// Implementation of the checksum as defined in https://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf
fn checksum(payload: &[u8]) -> u8 {
    1u8.wrapping_add(0xff - payload.iter().fold(0u8, |sum, c| sum.wrapping_add(*c)))
}

/// Extract the payload from a packet, validating packet length, checksum & header.
pub fn parse_payload(packet: &[u8]) -> Result<&[u8], MHZ19Error> {
    use MHZ19Error::*;
    if packet.len() != 9 {
        return Err(WrongPacketLength(packet.len()));
    }
    let header = packet[0];
    if header != 0xFF {
        return Err(WrongStartByte(header));
    }
    let payload = &packet[1..8];
    let found_checksum = packet[8];
    let payload_checksum = checksum(payload);
    if found_checksum != payload_checksum {
        return Err(WrongChecksum(payload_checksum, found_checksum));
    }

    Ok(payload)
}

/// Get the CO2 gas concentration in ppm from a response packet.
///
/// Will return an error if the packet is not a "read gas concentration packet"
pub fn parse_gas_contentration_ppm(packet: &[u8]) -> Result<u32, MHZ19Error> {
    let payload = parse_payload(packet)?;
    if payload[0] != Command::ReadGasConcentration.get_command_value() {
        Err(MHZ19Error::WrongPacketType(
            Command::ReadGasConcentration.get_command_value(),
            payload[0],
        ))
    } else {
        Ok(256 * (payload[1] as u32) + (payload[2] as u32))
    }
}

#[derive(Debug, PartialEq)]
pub enum MHZ19Error {
    /// Packet of bytes has the wrong size
    WrongPacketLength(usize),
    /// Packet of bytes has the wrong checksum
    WrongChecksum(u8, u8),
    /// Wrong start byte (must be 0xFF)
    WrongStartByte(u8),
    /// The packet type is not the one excepting (eg must be 0x86 when reading gas concentration)
    WrongPacketType(u8, u8),
}

#[cfg(feature = "std")]
impl std::error::Error for MHZ19Error {}

impl fmt::Display for MHZ19Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use MHZ19Error::*;
        match self {
            WrongChecksum(expected, found) => write!(
                f,
                "Invalid checksum, expected {:X}, found {:X}",
                expected, found
            ),
            WrongPacketLength(found) => {
                write!(f, "Wrong packet length, expected 9, found {}", found)
            }
            WrongStartByte(found) => {
                write!(f, "Wrong start byte, expected 0xFF, found {:X}", found)
            }
            WrongPacketType(expected, found) => write!(
                f,
                "Wrong packet type, expected {}, found {:X}",
                expected, found
            ),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    /// The command payload used to query the co2 cas concentration
    /// from the device number 1 (the default device number)
    ///
    /// It is the same value as the result of
    /// get_command_packet(Command::ReadGasConcentration, 1).
    static READ_GAS_CONCENTRATION_COMMAND_ON_DEV1_PACKET: &'static [u8] =
        &[0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];

    #[test]
    fn test_checksum() {
        assert_eq!(0x79, checksum(&[0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00]));
        assert_eq!(0xA0, checksum(&[0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00]));
        assert_eq!(0xD1, checksum(&[0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00]));
    }

    #[test]
    fn test_get_payload() {
        assert_eq!(Err(MHZ19Error::WrongPacketLength(0)), parse_payload(&[]));
        assert_eq!(Err(MHZ19Error::WrongPacketLength(1)), parse_payload(&[12]));
        assert_eq!(
            Err(MHZ19Error::WrongPacketLength(12)),
            parse_payload(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        );
        assert_eq!(
            Err(MHZ19Error::WrongStartByte(10)),
            parse_payload(&[10, 2, 3, 4, 5, 6, 7, 8, 9])
        );
        assert_eq!(
            Err(MHZ19Error::WrongChecksum(221, 9)),
            parse_payload(&[0xFF, 2, 3, 4, 5, 6, 7, 8, 9])
        );
        assert_eq!(
            Err(MHZ19Error::WrongChecksum(0xD1, 0x10)),
            parse_payload(&[0xFF, 0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00, 0x10])
        );
        assert_eq!(
            Ok(&[0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00][..]),
            parse_payload(&[0xFF, 0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00, 0xD1])
        );
    }

    #[test]
    fn test_get_command_packet() {
        assert_eq!(
            [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79],
            get_command_with_bytes34(Command::ReadGasConcentration, 1, 0, 0)
        );
        assert_eq!(
            Ok(&[0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00][..]),
            parse_payload(&get_command_with_bytes34(
                Command::ReadGasConcentration,
                1,
                0,
                0
            ))
        );
        assert_eq!(
            READ_GAS_CONCENTRATION_COMMAND_ON_DEV1_PACKET,
            get_command_with_bytes34(Command::ReadGasConcentration, 1, 0, 0)
        );
        assert_eq!(
            READ_GAS_CONCENTRATION_COMMAND_ON_DEV1_PACKET,
            read_gas_concentration(1)
        );

        // Check command values
        assert_eq!(
            super::Command::SetSensorDetectionRange.get_command_value(),
            set_detection_range(1, 1)[2]
        );
        assert_eq!(
            super::Command::CalibrateZero.get_command_value(),
            calibrate_zero_point(1)[2]
        );
        assert_eq!(
            super::Command::CalibrateSpan.get_command_value(),
            calibrate_span_point(1, 1)[2]
        );
        assert_eq!(
            super::Command::ReadGasConcentration.get_command_value(),
            read_gas_concentration(1)[2]
        );
    }

    #[test]
    fn issue_3_op_precedence() {
        let p = set_detection_range(1, 0x07D0);
        assert_eq!(0x07, p[3]);
        assert_eq!(0xD0, p[4]);

        let p = calibrate_span_point(1, 0x07D0);
        assert_eq!(0x07, p[3]);
        assert_eq!(0xD0, p[4]);
    }
}
