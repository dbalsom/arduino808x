#![allow(dead_code, unused_variables)]
use std::io::{Read, Write};
use std::{cell::RefCell, error::Error, fmt::Display, rc::Rc, str};

use log;
use serialport::{ClearBuffer, SerialPort};

pub const ARDUINO_BAUD: u32 = 1000000;

/// [ServerCpuType] maps to the CPU types that can be detected by the Arduino808X server.
#[derive(Copy, Clone, Debug, Default)]
pub enum ServerCpuType {
    #[default]
    Intel8088,
    Intel8086,
    NecV20,
    NecV30,
    Intel80188(bool),
    Intel80186(bool),
}

impl ServerCpuType {
    /// Returns whether the CPU type is an Intel CPU.
    pub fn is_intel(&self) -> bool {
        match self {
            ServerCpuType::Intel8088
            | ServerCpuType::Intel8086
            | ServerCpuType::Intel80188(_)
            | ServerCpuType::Intel80186(_) => true,
            _ => false,
        }
    }
    /// Returns whether we can prefetch the user program for this CPU type.
    /// Currently, all CPU types support prefetching.
    pub fn can_prefetch(&self) -> bool {
        true
    }
    /// Returns whether this CPU type supports 8080 emulation. Only the NEC V20 and V30
    /// support this.
    pub fn has_8080_emulation(&self) -> bool {
        match self {
            ServerCpuType::NecV20 | ServerCpuType::NecV30 => true,
            _ => false,
        }
    }
}

/// Derive the [CpuWidth] from a [ServerCpuType].
impl From<ServerCpuType> for CpuWidth {
    fn from(cpu_type: ServerCpuType) -> Self {
        match cpu_type {
            ServerCpuType::Intel8088 | ServerCpuType::Intel80188(_) => CpuWidth::Eight,
            _ => CpuWidth::Sixteen,
        }
    }
}

/// Convert a raw u8 value received from the Arduino808X server to a [ServerCpuType].
impl TryFrom<u8> for ServerCpuType {
    type Error = CpuClientError;
    fn try_from(value: u8) -> Result<ServerCpuType, CpuClientError> {
        match value & 0x7F {
            0x00 => Ok(ServerCpuType::Intel8088),
            0x01 => Ok(ServerCpuType::Intel8086),
            0x02 => Ok(ServerCpuType::NecV20),
            0x03 => Ok(ServerCpuType::NecV30),
            0x04 => Ok(ServerCpuType::Intel80188((value & 0x80) != 0)),
            0x05 => Ok(ServerCpuType::Intel80186((value & 0x80) != 0)),
            _ => Err(CpuClientError::BadValue),
        }
    }
}

/// [DataWidth] represents the current width of the data bus.
#[derive(Copy, Clone, Debug, Default)]
pub enum DataWidth {
    #[default]
    Invalid,
    /// The entire data bus is being driven.
    Sixteen,
    /// The low half of the data bus is being driven, A0 is even.
    EightLow,
    /// The low half of the data bus is being driven, A0 is odd.
    EightHigh,
}

/// Convert the BHE and A0 signals to a [DataWidth].
impl From<(bool, bool)> for DataWidth {
    fn from(signals: (bool, bool)) -> DataWidth {
        match signals {
            (true, true) => {
                // BHE is enabled, A0 is odd. Eight bit read high half of bus is active.
                DataWidth::EightHigh
            }
            (true, false) => {
                // BHE is enabled, A0 is even. Sixteen bit read, full bus active.
                DataWidth::Sixteen
            }
            (false, true) => {
                // BHE is disabled, A0 is odd. This is an invalid condition -
                // neither half of the data bus is driven
                DataWidth::Invalid
            }
            (false, false) => {
                // BHE is disabled, A0 is even. Eight bit low half of bus is active.
                DataWidth::EightLow
            }
        }
    }
}

/// [CpuWidth] represents the width of the detected CPU's data bus.
#[derive(Copy, Clone, Debug, Default)]
pub enum CpuWidth {
    #[default]
    Eight,
    Sixteen,
}

/// Returns the size of the instruction queue for the CPU width.
impl CpuWidth {
    pub fn queue_size(&self) -> usize {
        match self {
            CpuWidth::Eight => 4,
            CpuWidth::Sixteen => 6,
        }
    }
}

/// Convert a raw u8 value received from the Arduino808X server to a [CpuWidth].
impl From<u8> for CpuWidth {
    fn from(value: u8) -> Self {
        match value {
            0 => CpuWidth::Eight,
            _ => CpuWidth::Sixteen,
        }
    }
}

/// Convert a [CpuWidth] to a usize value representing the number of bytes.
impl From<CpuWidth> for usize {
    fn from(value: CpuWidth) -> usize {
        match value {
            CpuWidth::Eight => 1,
            CpuWidth::Sixteen => 2,
        }
    }
}

/// [ServerCommand] represents the commands that can be sent to the Arduino808X server.
#[derive(Copy, Clone, Debug)]
pub enum ServerCommand {
    CmdNull = 0x00,
    CmdVersion = 0x01,
    CmdReset = 0x02,
    CmdLoad = 0x03,
    CmdCycle = 0x04,
    CmdReadAddressLatch = 0x05,
    CmdReadStatus = 0x06,
    CmdRead8288Command = 0x07,
    CmdRead8288Control = 0x08,
    CmdReadDataBus = 0x09,
    CmdWriteDataBus = 0x0A,
    CmdFinalize = 0x0B,
    CmdBeginStore = 0x0C,
    CmdStore = 0x0D,
    CmdQueueLen = 0x0E,
    CmdQueueBytes = 0x0F,
    CmdWritePin = 0x10,
    CmdReadPin = 0x11,
    CmdGetProgramState = 0x12,
    CmdGetLastError = 0x13,
    CmdGetCycleState = 0x14,
    CmdCGetCycleState = 0x15,
    CmdPrefetchStore = 0x16,
    CmdReadAddressU = 0x17,
    CmdCpuType = 0x18,
    CmdEmulate8080 = 0x19,
    CmdPrefetch = 0x1A,
    CmdInvalid,
}

/// [ProgramState] represents the current state of the Arduino808X server.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ProgramState {
    Reset = 0,
    CpuId,
    CpuSetup,
    JumpVector,
    Load,
    LoadDone,
    EmuEnter,
    Prefetch,
    Execute,
    ExecuteFinalize,
    ExecuteDone,
    EmuExit,
    Store,
    StoreDone,
    Done,
}

/// Convert a raw u8 value received from the Arduino808X server to a [ProgramState].
impl TryFrom<u8> for ProgramState {
    type Error = CpuClientError;
    fn try_from(value: u8) -> Result<ProgramState, CpuClientError> {
        match value {
            0x00 => Ok(ProgramState::Reset),
            0x01 => Ok(ProgramState::CpuId),
            0x02 => Ok(ProgramState::CpuSetup),
            0x03 => Ok(ProgramState::JumpVector),
            0x04 => Ok(ProgramState::Load),
            0x05 => Ok(ProgramState::LoadDone),
            0x06 => Ok(ProgramState::EmuEnter),
            0x07 => Ok(ProgramState::Prefetch),
            0x08 => Ok(ProgramState::Execute),
            0x09 => Ok(ProgramState::ExecuteFinalize),
            0x0A => Ok(ProgramState::ExecuteDone),
            0x0B => Ok(ProgramState::EmuExit),
            0x0C => Ok(ProgramState::Store),
            0x0D => Ok(ProgramState::StoreDone),
            0x0E => Ok(ProgramState::Done),
            _ => Err(CpuClientError::BadValue),
        }
    }
}

/// [Segment] represents the segment registers in the CPU.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Segment {
    ES = 0,
    SS,
    CS,
    DS,
}

/// [QueueOp] represents the operation performed on the instruction queue on the last cycle.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum QueueOp {
    Idle = 0,
    First,
    Flush,
    Subsequent,
}

/// [BusState] represents the current state of the bus as decoded by the CPU S0-S2 status lines.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BusState {
    INTA = 0, // IRQ Acknowledge
    IOR = 1,  // IO Read
    IOW = 2,  // IO Write
    HALT = 3, // Halt
    CODE = 4, // Code
    MEMR = 5, // Memory Read
    MEMW = 6, // Memory Write
    PASV = 7, // Passive
}

/// [CpuPin] represents the miscellaneous CPU pins that can be read or written to.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CpuPin {
    READY = 0,
    TEST,
    INTR,
    NMI,
}

pub const REQUIRED_PROTOCOL_VER: u8 = 3;

pub const CONTROL_ALE_BIT: u8 = 0b0000_0001;

pub const COMMAND_MRDC_BIT: u8 = 0b0000_0001;
pub const COMMAND_AMWC_BIT: u8 = 0b0000_0010;
pub const COMMAND_MWTC_BIT: u8 = 0b0000_0100;
pub const COMMAND_IORC_BIT: u8 = 0b0000_1000;
pub const COMMAND_AIOWC_BIT: u8 = 0b0001_0000;
pub const COMMAND_IOWC_BIT: u8 = 0b0010_0000;
pub const COMMAND_INTA_BIT: u8 = 0b0100_0000;
pub const COMMAND_BHE_BIT: u8 = 0b1000_0000;

pub const STATUS_SEG_BITS: u8 = 0b0001_1000;

#[macro_export]
macro_rules! get_segment {
    ($s:expr) => {
        match (($s >> 3) & 0x03) {
            0b00 => Segment::ES,
            0b01 => Segment::SS,
            0b10 => Segment::CS,
            _ => Segment::DS,
        }
    };
}

#[macro_export]
macro_rules! get_bus_state {
    ($s:expr) => {
        match ($s & 0x07) {
            0 => BusState::INTA,
            1 => BusState::IOR,
            2 => BusState::IOW,
            3 => BusState::HALT,
            4 => BusState::CODE,
            5 => BusState::MEMR,
            6 => BusState::MEMW,
            _ => BusState::PASV,
        }
    };
}

#[macro_export]
macro_rules! get_queue_op {
    ($s:expr) => {
        match (($s >> 6) & 0x03) {
            0b00 => QueueOp::Idle,
            0b01 => QueueOp::First,
            0b10 => QueueOp::Flush,
            _ => QueueOp::Subsequent,
        }
    };
}

#[macro_export]
macro_rules! is_reading {
    ($s:expr) => {
        match ((!($s) & 0b0000_1001) != 0) {
            true => true,
            false => false,
        }
    };
}

#[macro_export]
macro_rules! is_writing {
    ($s:expr) => {
        match ((!($s) & 0b0011_0110) != 0) {
            true => true,
            false => false,
        }
    };
}

/// [CpuClientError] represents the errors that can occur when communicating with the Arduino808X
/// server.
#[derive(Debug)]
pub enum CpuClientError {
    ReadFailure,
    WriteFailure,
    BadValue,
    ReadTimeout,
    EnumerationError,
    DiscoveryError,
    CommandFailed,
}

impl Error for CpuClientError {}
impl Display for CpuClientError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            CpuClientError::ReadFailure => {
                write!(f, "Failed to read from serial port.")
            }
            CpuClientError::WriteFailure => {
                write!(f, "Failed to write to serial port.")
            }
            CpuClientError::BadValue => {
                write!(f, "Received invalid value from command.")
            }
            CpuClientError::ReadTimeout => {
                write!(f, "Response timeout.")
            }
            CpuClientError::EnumerationError => {
                write!(f, "Failed to find a valid serial port.")
            }
            CpuClientError::DiscoveryError => {
                write!(f, "Failed to find a listening ArduinoX86 server.")
            }
            CpuClientError::CommandFailed => {
                write!(f, "Server command returned failure code.")
            }
        }
    }
}

/// A [CpuClient] represents a connection to an `ArduinoX86` server over a serial port.
pub struct CpuClient {
    port: Rc<RefCell<Box<dyn serialport::SerialPort>>>,
}

impl CpuClient {
    pub fn init(com_port: Option<String>) -> Result<CpuClient, CpuClientError> {
        let mut matched_port = false;
        match serialport::available_ports() {
            Ok(ports) => {
                for port in ports {
                    if let Some(ref p) = com_port {
                        if port.port_name != *p {
                            continue; // Skip ports that don't match the specified port
                        }
                        matched_port = true;
                    }
                    println!("Trying port: {}", port.port_name);
                    if let Some(rtk_port) = CpuClient::try_port(port, 1000) {
                        return Ok(CpuClient {
                            port: Rc::new(RefCell::new(rtk_port)),
                        });
                    }
                }

                if let Some(ref p) = com_port {
                    return if !matched_port {
                        log::warn!("Did not find specified port: {}", p);
                        Err(CpuClientError::DiscoveryError)
                    } else {
                        log::warn!("Did not find Arduino808X server at specified port: {}", p);
                        Err(CpuClientError::DiscoveryError)
                    };
                }
                Err(CpuClientError::DiscoveryError)
            }
            Err(e) => {
                log::warn!("Didn't find any serial ports: {:?}", e);
                Err(CpuClientError::EnumerationError)
            }
        }
    }

    /// Try to open the specified serial port and query it for an Arduino808X server.
    pub fn try_port(
        port_info: serialport::SerialPortInfo,
        timeout: u64,
    ) -> Option<Box<dyn SerialPort>> {
        let port_result = serialport::new(port_info.port_name.clone(), 0)
            .baud_rate(0)
            .timeout(std::time::Duration::from_millis(timeout))
            .stop_bits(serialport::StopBits::One)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .open();

        match port_result {
            Ok(mut new_port) => {
                //log::trace!("Successfully opened host port {}", port_info.port_name);

                // Flush
                new_port.clear(ClearBuffer::Input).unwrap();
                new_port.clear(ClearBuffer::Output).unwrap();

                let cmd: [u8; 1] = [1];
                let mut buf: [u8; 100] = [0; 100];

                _ = new_port.flush();
                log::trace!("Sending version query to {}...", port_info.port_name);

                match new_port.write(&cmd) {
                    Ok(_) => {
                        log::trace!("Sent version query to {}...", port_info.port_name);
                    }
                    Err(e) => {
                        log::error!("try_port: Write error to {}: {:?}", port_info.port_name, e);
                        return None;
                    }
                }
                match new_port.flush() {
                    Ok(_) => {
                        log::trace!("Flushed output to {}...", port_info.port_name);
                    }
                    Err(e) => {
                        log::error!(
                            "try_port: flush error from {}: {:?}",
                            port_info.port_name,
                            e
                        );
                        return None;
                    }
                }

                let bytes_read = match new_port.read(&mut buf) {
                    Ok(bytes) => bytes,
                    Err(e) => {
                        log::error!("try_port: Read error from {}: {:?}", port_info.port_name, e);
                        return None;
                    }
                };

                new_port.clear(serialport::ClearBuffer::Input).unwrap();
                if bytes_read == 9 {
                    let ver_text = str::from_utf8(&buf).unwrap();
                    if ver_text.contains("ardX86 ") {
                        let proto_ver = buf[7];
                        log::trace!(
                            "Found an ArduinoX86 server, protocol verison: {} on port {}",
                            proto_ver,
                            port_info.port_name
                        );

                        if proto_ver != REQUIRED_PROTOCOL_VER {
                            log::error!("Unsupported protocol version.");
                            return None;
                        }
                    }
                    return Some(new_port);
                } else {
                    log::trace!(
                        "Invalid response from discovery command. Read {} bytes (Expected 9).",
                        bytes_read
                    );
                    let ver_text = str::from_utf8(&buf).unwrap();
                    log::trace!("First 9 bytes of response: {:?}", ver_text);
                }
                None
            }
            Err(e) => {
                log::error!(
                    "try_port: Error opening host port {}: {}",
                    port_info.port_name,
                    e
                );
                None
            }
        }
    }

    pub fn send_command_byte(&mut self, cmd: ServerCommand) -> Result<(), CpuClientError> {
        let cmd: [u8; 1] = [cmd as u8];

        self.port.borrow_mut().clear(ClearBuffer::Input).unwrap();
        match self.port.borrow_mut().write(&cmd) {
            Ok(_) => Ok(()),
            Err(_) => Err(CpuClientError::WriteFailure),
        }
    }

    pub fn read_result_code(&mut self) -> Result<bool, CpuClientError> {
        let mut buf: [u8; 1] = [0; 1];

        match self.port.borrow_mut().read(&mut buf) {
            Ok(bytes) => {
                if bytes == 0 {
                    log::error!("read_result_code: 0 bytes read");
                    Err(CpuClientError::ReadFailure)
                } else if (buf[0] & 0x01) != 0 {
                    // LSB set in return code == success
                    Ok(true)
                } else {
                    log::error!("read_result_code: command returned failure");
                    Err(CpuClientError::CommandFailed)
                }
            }
            Err(e) => {
                log::error!("read_result_code: read operation failed: {}", e);
                Err(CpuClientError::ReadFailure)
            }
        }
    }

    pub fn send_buf(&mut self, buf: &[u8]) -> Result<bool, CpuClientError> {
        match self.port.borrow_mut().write(&buf) {
            Ok(bytes) => {
                if bytes != buf.len() {
                    Err(CpuClientError::WriteFailure)
                } else {
                    Ok(true)
                }
            }
            Err(_) => Err(CpuClientError::WriteFailure),
        }
    }

    pub fn recv_buf(&mut self, buf: &mut [u8]) -> Result<bool, CpuClientError> {
        match self.port.borrow_mut().read(buf) {
            Ok(bytes) => {
                if bytes != buf.len() {
                    // We didn't read entire buffer worth of data, fail
                    log::error!("recv_buf: Only read {} bytes of {}.", bytes, buf.len());
                    Err(CpuClientError::ReadFailure)
                } else {
                    Ok(true)
                }
            }
            Err(e) => {
                log::error!("recv_buf: read operation failed: {}", e);
                Err(CpuClientError::ReadFailure)
            }
        }
    }

    /// Receive a buffer of dynamic size (don't expect the entire buffer read like recv_buf does)
    /// Returns the number of bytes read.
    /// Primarily used for get_last_error
    pub fn recv_dyn_buf(&mut self, buf: &mut [u8]) -> Result<usize, CpuClientError> {
        match self.port.borrow_mut().read(buf) {
            Ok(bytes) => Ok(bytes),
            Err(_) => Err(CpuClientError::ReadFailure),
        }
    }

    /// Server command - Load
    /// Load the specified register state into the CPU.
    /// This command takes 28 bytes, which correspond to the word values of each of the 14
    /// CPU registers.
    /// Registers should be loaded in the following order, little-endian:
    ///
    /// AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
    pub fn load_registers_from_buf(&mut self, reg_data: &[u8]) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdLoad)?;
        self.send_buf(reg_data)?;
        self.read_result_code()
    }

    pub fn begin_store(&mut self) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdBeginStore)?;
        self.read_result_code()
    }

    pub fn store_registers_to_buf(&mut self, reg_data: &mut [u8]) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdStore)?;
        self.recv_buf(reg_data)?;
        self.read_result_code()
    }

    pub fn cycle(&mut self) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdCycle)?;
        self.read_result_code()
    }

    pub fn cpu_type(&mut self) -> Result<ServerCpuType, CpuClientError> {
        let mut buf: [u8; 1] = [0; 1];
        self.send_command_byte(ServerCommand::CmdCpuType)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        ServerCpuType::try_from(buf[0])
    }

    pub fn read_address_latch(&mut self) -> Result<u32, CpuClientError> {
        let mut buf: [u8; 3] = [0; 3];
        self.send_command_byte(ServerCommand::CmdReadAddressLatch)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        let address = buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16;

        Ok(address)
    }

    pub fn read_address(&mut self) -> Result<u32, CpuClientError> {
        let mut buf: [u8; 3] = [0; 3];
        self.send_command_byte(ServerCommand::CmdReadAddressU)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        let address = buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16;

        Ok(address)
    }

    pub fn read_status(&mut self) -> Result<u8, CpuClientError> {
        let mut buf: [u8; 1] = [0; 1];
        self.send_command_byte(ServerCommand::CmdReadStatus)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        Ok(buf[0])
    }

    pub fn read_8288_command(&mut self) -> Result<u8, CpuClientError> {
        let mut buf: [u8; 1] = [0; 1];
        self.send_command_byte(ServerCommand::CmdRead8288Command)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        Ok(buf[0])
    }

    pub fn read_8288_control(&mut self) -> Result<u8, CpuClientError> {
        let mut buf: [u8; 1] = [0; 1];
        self.send_command_byte(ServerCommand::CmdRead8288Control)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        Ok(buf[0])
    }

    pub fn read_data_bus(&mut self) -> Result<u16, CpuClientError> {
        let mut buf: [u8; 2] = [0; 2];
        self.send_command_byte(ServerCommand::CmdReadDataBus)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        let word = u16::from_le_bytes([buf[0], buf[1]]);
        Ok(word)
    }

    pub fn write_data_bus(&mut self, data: u16) -> Result<bool, CpuClientError> {
        let mut buf: [u8; 2] = [0; 2];
        self.send_command_byte(ServerCommand::CmdWriteDataBus)?;
        buf.copy_from_slice(&data.to_le_bytes());

        self.send_buf(&mut buf)?;
        self.read_result_code()?;

        Ok(true)
    }

    pub fn prefetch_store(&mut self) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdPrefetchStore)?;
        self.read_result_code()
    }

    pub fn finalize(&mut self) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdFinalize)?;
        self.read_result_code()
    }

    pub fn get_program_state(&mut self) -> Result<ProgramState, CpuClientError> {
        let mut buf: [u8; 1] = [0; 1];
        self.send_command_byte(ServerCommand::CmdGetProgramState)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        ProgramState::try_from(buf[0])
    }

    pub fn get_last_error(&mut self) -> Result<String, CpuClientError> {
        let mut errbuf: [u8; 50] = [0; 50];
        self.send_command_byte(ServerCommand::CmdGetLastError)?;
        let bytes = self.recv_dyn_buf(&mut errbuf)?;
        let err_string = str::from_utf8(&errbuf[..bytes - 1]).unwrap();

        Ok(err_string.to_string())
    }

    pub fn write_pin(&mut self, pin_no: CpuPin, val: bool) -> Result<bool, CpuClientError> {
        let mut buf: [u8; 2] = [0; 2];
        buf[0] = pin_no as u8;
        buf[1] = val as u8;
        self.send_command_byte(ServerCommand::CmdWritePin)?;
        self.send_buf(&mut buf)?;
        self.read_result_code()
    }

    /// Get the per-cycle state of the CPU
    pub fn get_cycle_state(&mut self) -> Result<(ProgramState, u8, u8, u8, u16), CpuClientError> {
        let mut buf: [u8; 5] = [0; 5];
        self.send_command_byte(ServerCommand::CmdGetCycleState)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        let state_bits: u8 = buf[0] >> 4;
        let state = ProgramState::try_from(state_bits)?;

        let control_bits = buf[0] & 0x0F;

        let data_bus = u16::from_le_bytes([buf[3], buf[4]]);
        Ok((state, control_bits, buf[1], buf[2], data_bus))
    }

    /// Like `get_cycle_state`, but also cycles the CPU. Saves a command.
    pub fn cycle_get_cycle_state(
        &mut self,
    ) -> Result<(ProgramState, u8, u8, u8, u16), CpuClientError> {
        let mut buf: [u8; 5] = [0; 5];
        self.send_command_byte(ServerCommand::CmdCGetCycleState)?;
        self.recv_buf(&mut buf)?;
        self.read_result_code()?;

        let state_bits: u8 = buf[0] >> 4;
        let state = ProgramState::try_from(state_bits)?;

        let control_bits = buf[0] & 0x0F;

        let data_bus = u16::from_le_bytes([buf[3], buf[4]]);
        Ok((state, control_bits, buf[1], buf[2], data_bus))
    }

    pub fn emu8080(&mut self) -> Result<bool, CpuClientError> {
        self.send_command_byte(ServerCommand::CmdEmulate8080)?;
        self.read_result_code()
    }
}
