use std::fmt;

/// MH-Z12 Commands
pub enum Command {
    /// Read the gas concentration
    ReadGasConcentration,
    /// Execute a zero point calibration.
    ///
    /// The sensor must be in a stable gas environment for 20 minutes with a co2 concentration of 400ppm.
    CalibrateZero,
    /// Execute a span point calibration
    CalibrateSpan,
}

impl Command {
    fn get_command_value(&self) -> u8 {
        use Command::*;
        match self {
            ReadGasConcentration => 0x86,
            CalibrateZero => 0x87,
            CalibrateSpan => 0x88,
        }
    }
}

/// The command payload used to query the co2 cas concentration from the device number 1 (the default device
/// number)
///
/// It is the same value as the result of get_command_packet(Command::ReadGasConcentration, 1).
pub static READ_GAS_CONCENTRATION_COMMAND_ON_DEV1_PACKET: &'static [u8] =
    &[0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];

/// Get the command packet with proper header and checksum.
pub fn get_command_packet(command: Command, device_number: u8) -> Vec<u8> {
    let mut ret = vec![
        0xFF,
        device_number,
        command.get_command_value(),
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    ];
    ret.push(checksum(&ret[1..]));
    ret
}

/// Implementation of the checksum as defined in https://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf
fn checksum(payload: &[u8]) -> u8 {
    1u8.wrapping_add(0xff - payload.iter().fold(0u8, |sum, c| sum.wrapping_add(*c)))
}

/// Extract the payload from a packet, validating packet length, checksum & header.
pub fn get_payload(packet: &[u8]) -> Result<&[u8], MHZ19Error> {
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
pub fn get_gas_contentration_ppm(packet: &[u8]) -> Result<u32, MHZ19Error> {
    let payload = get_payload(packet)?;
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

    #[test]
    fn test_checksum() {
        assert_eq!(
            0x79,
            checksum(&vec![0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00])
        );
        assert_eq!(
            0xA0,
            checksum(&vec![0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00])
        );
        assert_eq!(
            0xD1,
            checksum(&vec![0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00])
        );
    }

    #[test]
    fn test_get_payload() {
        assert_eq!(Err(MHZ19Error::WrongPacketLength(0)), get_payload(&vec![]));
        assert_eq!(
            Err(MHZ19Error::WrongPacketLength(1)),
            get_payload(&vec![12])
        );
        assert_eq!(
            Err(MHZ19Error::WrongPacketLength(12)),
            get_payload(&vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        );
        assert_eq!(
            Err(MHZ19Error::WrongStartByte(10)),
            get_payload(&vec![10, 2, 3, 4, 5, 6, 7, 8, 9])
        );
        assert_eq!(
            Err(MHZ19Error::WrongChecksum(221, 9)),
            get_payload(&vec![0xFF, 2, 3, 4, 5, 6, 7, 8, 9])
        );
        assert_eq!(
            Err(MHZ19Error::WrongChecksum(0xD1, 0x10)),
            get_payload(&vec![0xFF, 0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00, 0x10])
        );
        assert_eq!(
            Ok(vec![0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00].as_slice()),
            get_payload(&vec![0xFF, 0x86, 0x02, 0x60, 0x47, 0x00, 0x00, 0x00, 0xD1])
        );
    }

    #[test]
    fn test_get_command_packet() {
        assert_eq!(
            vec![0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79],
            get_command_packet(Command::ReadGasConcentration, 1)
        );
        assert_eq!(
            Ok(vec![0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00].as_slice()),
            get_payload(&get_command_packet(Command::ReadGasConcentration, 1))
        );
        assert_eq!(
            Vec::from(READ_GAS_CONCENTRATION_COMMAND_ON_DEV1_PACKET),
            get_command_packet(Command::ReadGasConcentration, 1)
        );
    }
}
