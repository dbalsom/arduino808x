/*
    ArduinoX86 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduinoX86

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

#![allow(dead_code, unused_variables)]

mod queue;
#[macro_use]
pub(crate) mod opcodes;
mod code_stream;
mod remote_program;

use std::str::FromStr;

// Re-export the client module for convenience
pub use ard808x_client;
use ard808x_client::*;

use code_stream::CodeStream;
use opcodes::*;
use queue::*;
use remote_program::RemoteProgram;

pub use queue::QueueDataType;

pub const WAIT_STATES: u32 = 0;

//pub const CYCLE_LIMIT: u32 = 100_000;
pub const CYCLE_LIMIT: u32 = u32::MAX;

pub const HALT_CYCLE_LIMIT: u32 = 52;

pub const CPU_FLAG_CARRY: u16 = 0b0000_0000_0000_0001;
pub const CPU_FLAG_RESERVED1: u16 = 0b0000_0000_0000_0010;
pub const CPU_FLAG_PARITY: u16 = 0b0000_0000_0000_0100;
pub const CPU_FLAG_RESERVED3: u16 = 0b0000_0000_0000_1000;
pub const CPU_FLAG_AUX_CARRY: u16 = 0b0000_0000_0001_0000;
pub const CPU_FLAG_RESERVED5: u16 = 0b0000_0000_0010_0000;
pub const CPU_FLAG_ZERO: u16 = 0b0000_0000_0100_0000;
pub const CPU_FLAG_SIGN: u16 = 0b0000_0000_1000_0000;
pub const CPU_FLAG_TRAP: u16 = 0b0000_0001_0000_0000;
pub const CPU_FLAG_INT_ENABLE: u16 = 0b0000_0010_0000_0000;
pub const CPU_FLAG_DIRECTION: u16 = 0b0000_0100_0000_0000;
pub const CPU_FLAG_OVERFLOW: u16 = 0b0000_1000_0000_0000;
// Emulation mode bit on V20
pub const CPU_FLAG_MODE: u16 = 0b1000_0000_0000_0000;

const ADDRESS_SPACE: usize = 0x10_0000;
const ADDRESS_SPACE_MASK: usize = 0x0F_FFFF;

const IO_FINALIZE_ADDR: u32 = 0x00FF;
const ISR_SEGMENT: u16 = 0xF800;

const I8080_EMULATION_SEGMENT: u16 = 0x1000;
const BRKEM_INT: u8 = 0xFF;

static NULL_PRELOAD_PGM: [u8; 0] = [];
static INTEL808X_PRELOAD_PGM: [u8; 4] = [0xAA, 0xAA, 0xAA, 0xAA]; // (4x stosb)
static NECVX0_PRELOAD_PGM: [u8; 2] = [0x63, 0xC0]; // (undefined, no side effects)

static INTEL_PREFIXES: [u8; 8] = [0x26, 0x2E, 0x36, 0x3E, 0xF0, 0xF1, 0xF2, 0xF3];
static NEC_PREFIXES: [u8; 10] = [0x26, 0x2E, 0x36, 0x3E, 0xF0, 0xF1, 0xF2, 0xF3, 0x64, 0x65];

macro_rules! cycle_comment {
    ($self:ident, $($t:tt)*) => {{
        $self.cycle_comment = Some(format!($($t)*));
    }};
}

#[derive(Copy, Clone, Debug)]
pub enum CpuType {
    Intel8088,
    NecV20,
}

impl FromStr for CpuType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String>
    where
        Self: Sized,
    {
        match s.to_lowercase().as_str() {
            "8088" => Ok(CpuType::Intel8088),
            "v20" => Ok(CpuType::NecV20),
            _ => Err("Bad value for CpuType".to_string()),
        }
    }
}

impl CpuType {
    // Return whether this CPU has a defined prefetch program.
    pub fn can_prefetch(&self) -> bool {
        // We might have another CPU someday that we can't prefetch
        #[allow(unreachable_patterns)]
        match self {
            CpuType::Intel8088 | CpuType::NecV20 => true,
            _ => false,
        }
    }

    // Return whether this CPU has an 8080 emulation mode.
    pub fn has_8080_emulation(&self) -> bool {
        matches!(self, CpuType::NecV20)
    }
}

#[derive(Default, Debug)]
pub struct RemoteCpuRegisters {
    pub ax: u16,
    pub bx: u16,
    pub cx: u16,
    pub dx: u16,
    pub ss: u16,
    pub ds: u16,
    pub es: u16,
    pub sp: u16,
    pub bp: u16,
    pub si: u16,
    pub di: u16,
    pub cs: u16,
    pub ip: u16,
    pub flags: u16,
}

impl RemoteCpuRegisters {
    pub fn rewind_ip(&mut self, adjust: u16) {
        self.ip = self.ip.wrapping_sub(adjust);
    }

    pub fn write_buf(&self, buf: &mut [u8]) {
        // AX, BX, CX, DX, SS, SP, FLAGS, IP, CS, DS, ES, BP, SI, DI
        buf[0] = (self.ax & 0xFF) as u8;
        buf[1] = ((self.ax >> 8) & 0xFF) as u8;

        buf[2] = (self.bx & 0xFF) as u8;
        buf[3] = ((self.bx >> 8) & 0xFF) as u8;

        buf[4] = (self.cx & 0xFF) as u8;
        buf[5] = ((self.cx >> 8) & 0xFF) as u8;

        buf[6] = (self.dx & 0xFF) as u8;
        buf[7] = ((self.dx >> 8) & 0xFF) as u8;

        buf[8] = (self.ip & 0xFF) as u8;
        buf[9] = ((self.ip >> 8) & 0xFF) as u8;

        buf[10] = (self.cs & 0xFF) as u8;
        buf[11] = ((self.cs >> 8) & 0xFF) as u8;

        buf[12] = (self.flags & 0xFF) as u8;
        buf[13] = ((self.flags >> 8) & 0xFF) as u8;

        buf[14] = (self.ss & 0xFF) as u8;
        buf[15] = ((self.ss >> 8) & 0xFF) as u8;

        buf[16] = (self.sp & 0xFF) as u8;
        buf[17] = ((self.sp >> 8) & 0xFF) as u8;

        buf[18] = (self.ds & 0xFF) as u8;
        buf[19] = ((self.ds >> 8) & 0xFF) as u8;

        buf[20] = (self.es & 0xFF) as u8;
        buf[21] = ((self.es >> 8) & 0xFF) as u8;

        buf[22] = (self.bp & 0xFF) as u8;
        buf[23] = ((self.bp >> 8) & 0xFF) as u8;

        buf[24] = (self.si & 0xFF) as u8;
        buf[25] = ((self.si >> 8) & 0xFF) as u8;

        buf[26] = (self.di & 0xFF) as u8;
        buf[27] = ((self.di >> 8) & 0xFF) as u8;
    }
}

impl From<&[u8; 28]> for RemoteCpuRegisters {
    fn from(buf: &[u8; 28]) -> Self {
        RemoteCpuRegisters {
            ax: buf[0] as u16 | ((buf[1] as u16) << 8),
            bx: buf[2] as u16 | ((buf[3] as u16) << 8),
            cx: buf[4] as u16 | ((buf[5] as u16) << 8),
            dx: buf[6] as u16 | ((buf[7] as u16) << 8),
            ip: buf[8] as u16 | ((buf[9] as u16) << 8),
            cs: buf[10] as u16 | ((buf[11] as u16) << 8),
            flags: buf[12] as u16 | ((buf[13] as u16) << 8),
            ss: buf[14] as u16 | ((buf[15] as u16) << 8),
            sp: buf[16] as u16 | ((buf[17] as u16) << 8),
            ds: buf[18] as u16 | ((buf[19] as u16) << 8),
            es: buf[20] as u16 | ((buf[21] as u16) << 8),
            bp: buf[22] as u16 | ((buf[23] as u16) << 8),
            si: buf[24] as u16 | ((buf[25] as u16) << 8),
            di: buf[26] as u16 | ((buf[27] as u16) << 8),
        }
    }
}
impl From<&[u8]> for RemoteCpuRegisters {
    fn from(buf: &[u8]) -> Self {
        RemoteCpuRegisters {
            ax: buf[0] as u16 | ((buf[1] as u16) << 8),
            bx: buf[2] as u16 | ((buf[3] as u16) << 8),
            cx: buf[4] as u16 | ((buf[5] as u16) << 8),
            dx: buf[6] as u16 | ((buf[7] as u16) << 8),
            ip: buf[8] as u16 | ((buf[9] as u16) << 8),
            cs: buf[10] as u16 | ((buf[11] as u16) << 8),
            flags: buf[12] as u16 | ((buf[13] as u16) << 8),
            ss: buf[14] as u16 | ((buf[15] as u16) << 8),
            sp: buf[16] as u16 | ((buf[17] as u16) << 8),
            ds: buf[18] as u16 | ((buf[19] as u16) << 8),
            es: buf[20] as u16 | ((buf[21] as u16) << 8),
            bp: buf[22] as u16 | ((buf[23] as u16) << 8),
            si: buf[24] as u16 | ((buf[25] as u16) << 8),
            di: buf[26] as u16 | ((buf[27] as u16) << 8),
        }
    }
}

#[derive(PartialEq)]
pub enum BusCycle {
    T1,
    T2,
    T3,
    T4,
    Tw,
}

#[derive(Copy, Clone, Debug)]
pub struct PrintOptions {
    pub print_pgm: bool,
    pub print_preload: bool,
    pub print_finalize: bool,
}

impl Default for PrintOptions {
    fn default() -> Self {
        Self {
            print_pgm: true,
            print_preload: false,
            print_finalize: false,
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub enum RunState {
    #[default]
    Init,
    Preload,
    Program,
    Finalize,
}

pub struct RemoteCpu<'a> {
    cpu_type: ServerCpuType,
    width: CpuWidth,
    client: CpuClient,
    regs: RemoteCpuRegisters,
    memory: Vec<u8>,
    pc: usize,
    start_addr: usize,
    end_addr: usize,
    program_state: ProgramState,
    run_state: RunState,

    do_prefetch: bool,
    do_emu8080: bool,

    active_pgm: Option<&'a RemoteProgram>,
    preload_pgm: Option<RemoteProgram>,
    code_stream: CodeStream,
    program_end_offset: u16,

    address_bus: u32,
    address_latch: u32,
    status: u8,
    command_status: u8,
    control_status: u8,
    data_bus: u16,
    data_width: DataWidth,
    data_type: QueueDataType,

    cycle_num: u32,
    cycle_comment: Option<String>,
    instruction_num: u32,
    mcycle_state: BusState,
    bus_cycle: BusCycle,

    nready_states: u32,

    have_queue_status: bool,
    queue: InstructionQueue,
    queue_byte: u8,
    queue_type: QueueDataType,
    queue_first_fetch: bool,
    queue_fetch_n: u8,
    queue_fetch_addr: u32,
    queue_len_at_finalize: u8,
    opcode: u8,
    finalize: bool,

    do_nmi: bool,
    intr: bool,
    nmi: bool,

    halted: bool,
    halt_ct: u32,

    wait_state_opt: u32,
    intr_on_cycle: u32,
    intr_after: u32,
    nmi_on_cycle: u32,
}

impl RemoteCpu<'_> {
    pub fn new(
        mut client: CpuClient,
        do_prefetch: bool,
        do_emu8080: bool,
        wait_state_opt: u32,
        intr_on: u32,
        intr_after: u32,
        nmi_on: u32,
    ) -> RemoteCpu<'static> {
        // Determine CPU type/width

        let (server_cpu_type, width) = match client.cpu_type() {
            Ok(server_cpu_type) => {
                let width = CpuWidth::from(server_cpu_type);

                (server_cpu_type, width)
            }
            Err(e) => {
                log::warn!("Failed to get CPU width!");
                (ServerCpuType::default(), CpuWidth::default())
            }
        };

        log::debug!("Detected CPU type: {:?}", server_cpu_type);

        let have_queue_status = match server_cpu_type {
            ServerCpuType::Intel8088 | ServerCpuType::Intel8086 => true,
            ServerCpuType::NecV20 | ServerCpuType::NecV30 => true,
            ServerCpuType::Intel80188(status) => status,
            ServerCpuType::Intel80186(status) => status,
        };

        if !have_queue_status {
            log::warn!("Detected CPU does not provide queue status!");
        }

        if do_emu8080 {
            if !server_cpu_type.has_8080_emulation() {
                log::error!("Emulation mode requested but detected CPU type does not support it.");
                std::process::exit(1);
            } else {
                match client.emu8080() {
                    Ok(_) => {
                        log::debug!("Emulation mode enabled for CPU type: {:?}", server_cpu_type);
                    }
                    Err(e) => {
                        log::error!("Failed to enable emulation mode: {}", e);
                        std::process::exit(1);
                    }
                }
            }
        }

        let mut preload_pgm = None;

        if do_prefetch {
            if server_cpu_type.can_prefetch() {
                log::trace!("Using prefetch program for {:?}", server_cpu_type);
                preload_pgm = match server_cpu_type {
                    ServerCpuType::Intel8088 | ServerCpuType::Intel8086 => Some(
                        RemoteProgram::new(&INTEL808X_PRELOAD_PGM, OPCODE_NOP, width),
                    ),
                    ServerCpuType::NecV20 | ServerCpuType::NecV30 => {
                        Some(RemoteProgram::new(&NECVX0_PRELOAD_PGM, OPCODE_NOP, width))
                    }
                    _ => {
                        log::error!("Unsupported CPU type for prefetch: {:?}", server_cpu_type);
                        None
                    }
                };

                if let Some(ref program) = preload_pgm {
                    log::trace!("Size of prefetch program: {}", program.len());
                }
            } else {
                log::error!("Prefetch option chosen but no prefetch program for specified CPU.");
                std::process::exit(1);
            }
        }

        if do_emu8080 && server_cpu_type.has_8080_emulation() {
            match client.emu8080() {
                Ok(result_code) if result_code == true => {
                    log::debug!("8080 Emulation flag successfully set.");
                }
                Err(e) => {
                    log::error!("Failed to set emulation mode flag!");
                    std::process::exit(1);
                }
                _ => {
                    log::error!("Failed to set emulation mode flag!");
                    std::process::exit(1);
                }
            };
        }

        RemoteCpu {
            cpu_type: server_cpu_type,
            width,
            client,
            regs: Default::default(),
            memory: vec![0; ADDRESS_SPACE],
            pc: 0,
            start_addr: 0,
            end_addr: 0,
            program_state: ProgramState::Reset,
            run_state: RunState::Init,

            do_prefetch,
            do_emu8080,

            active_pgm: None,
            preload_pgm,
            code_stream: CodeStream::new(width),
            program_end_offset: 0,

            address_bus: 0,
            address_latch: 0,
            status: 0,
            command_status: 0,
            control_status: 0,
            data_bus: 0,
            data_width: Default::default(),
            data_type: QueueDataType::Program,
            cycle_num: 0,
            cycle_comment: None,
            instruction_num: 0,
            mcycle_state: BusState::PASV,
            bus_cycle: BusCycle::T1,
            nready_states: 0,

            have_queue_status,
            queue: InstructionQueue::new(width, !have_queue_status),
            queue_byte: 0,
            queue_type: QueueDataType::Program,
            queue_first_fetch: true,
            queue_fetch_n: 0,
            queue_fetch_addr: 0,
            queue_len_at_finalize: 0,
            opcode: 0,
            finalize: false,
            do_nmi: false,
            intr: false,
            nmi: false,
            halted: false,
            halt_ct: 0,
            wait_state_opt,
            intr_on_cycle: intr_on,
            intr_after,
            nmi_on_cycle: nmi_on,
        }
    }

    pub fn reset(&mut self) {
        log::trace!("Resetting!");
        self.program_state = ProgramState::Reset;
        self.run_state = RunState::default();

        self.preload_pgm.as_mut().map(|p| p.reset());
        self.code_stream = CodeStream::new(self.width);
        self.program_end_offset = 0;
        self.address_bus = 0;
        self.address_latch = 0;
        self.status = 0;
        self.command_status = 0;
        self.control_status = 0;
        self.data_bus = 0;
        self.data_width = Default::default();
        self.data_type = QueueDataType::Program;
        self.cycle_num = 0;
        self.instruction_num = 0;
        self.mcycle_state = BusState::PASV;
        self.bus_cycle = BusCycle::T1;
        self.queue = InstructionQueue::new(self.width, !self.have_queue_status);
        self.queue_byte = 0;
        self.queue_type = QueueDataType::Program;
        self.queue_first_fetch = true;
        self.queue_fetch_n = 0;
        self.queue_fetch_addr = 0;
        self.queue_len_at_finalize = 0;
        self.opcode = 0;
        self.finalize = false;
        self.do_nmi = false;
    }

    pub fn set_pc(&mut self, cs: u16, ip: u16) {
        self.regs.cs = cs;
        self.regs.ip = ip;
        self.pc = ((cs as usize) << 4) + ip as usize;
    }

    pub fn cpu_type(&self) -> ServerCpuType {
        self.cpu_type
    }

    pub fn pc(&self) -> usize {
        self.pc
    }

    pub fn mount_bin(&mut self, data: &[u8], location: usize) -> bool {
        let src_size = data.len();

        if location + src_size > self.memory.len() {
            // copy request goes out of bounds
            return false;
        }

        let mem_slice: &mut [u8] = &mut self.memory[location..location + src_size];
        for (dst, src) in mem_slice.iter_mut().zip(data) {
            *dst = *src;
        }

        // Update end address past sizeof program
        self.start_addr = location;
        self.end_addr = location + src_size;

        log::debug!(
            "Program mounted! Start addr: [{:05X}] end addr: [{:05X}]",
            self.start_addr,
            self.end_addr
        );
        true
    }

    pub fn set_program_bounds(&mut self, start: usize, end: usize) {
        self.start_addr = start;
        self.end_addr = end;
    }

    /// Set up the virtual memory space's Interrupt Vector Table
    pub fn setup_ivt(&mut self) {
        // Populate the IVR with pointers to two-byte ISRs that simply contain an IRET and a NOP for alignment.
        for i in 0..256 {
            // Calculate address of ISR for each IVT entry
            let table_offset: usize = i * 4;

            // Write offset first
            self.write_u16(table_offset, (table_offset / 2) as u16);
            // Write segment next
            self.write_u16(table_offset + 2, ISR_SEGMENT);

            // Write ISR routine
            let isr_address =
                RemoteCpu::calc_linear_address(ISR_SEGMENT, (table_offset / 2) as u16);

            self.memory[isr_address as usize] = OPCODE_IRET;
            self.memory[(isr_address + 1) as usize] = OPCODE_NOP;
        }

        if self.do_emu8080 && self.cpu_type.has_8080_emulation() {
            self.setup_emulation_ivt();
        }
    }

    /// Set up the IVT entry for i8080 emulation mode.
    pub fn setup_emulation_ivt(&mut self) {
        let table_offset = BRKEM_INT as usize * 4;

        // Write offset first
        self.write_u16(table_offset, 0 as u16);
        // Write segment next
        self.write_u16(table_offset + 2, I8080_EMULATION_SEGMENT);
    }

    /// Return true if this address is an ISR
    pub fn is_isr_address(&self, address: u32) -> bool {
        let isr_start = RemoteCpu::calc_linear_address(ISR_SEGMENT, 0);
        let isr_end = RemoteCpu::calc_linear_address(ISR_SEGMENT, 256 * 4);

        if address >= isr_start && address < isr_end {
            true
        } else {
            false
        }
    }

    pub fn write_u8(&mut self, address: usize, byte: u8) {
        if address < self.memory.len() {
            self.memory[address] = byte;
        }
    }

    pub fn write_u16(&mut self, address: usize, word: u16) {
        if address < self.memory.len() - 1 {
            self.memory[address] = (word & 0xFF) as u8;
            self.memory[address + 1] = (word >> 8) as u8;
        }
    }

    pub fn calc_linear_address(segment: u16, offset: u16) -> u32 {
        ((segment as u32) << 4) + offset as u32 & 0xFFFFFu32
    }

    pub fn load_registers_from_buf(&mut self, reg_data: &[u8]) -> bool {
        self.reset(); // CPU is reset on register load

        // Adjust registers as needed for CPU prefetch.
        let mut regs = RemoteCpuRegisters::from(reg_data);

        if let Some(preload_pgm) = &self.preload_pgm {
            // Adjust IP by size of preload program.
            regs.ip = regs
                .ip
                .wrapping_sub((preload_pgm.len() + preload_pgm.get_fill_ct()) as u16);

            if preload_pgm.len() > 0 {
                if self.cpu_type.is_intel() {
                    log::trace!("Adjusting registers for 8088 prefetch...");
                    // (4x stosb)

                    // Adjust DI. This depends on the state of the Direction flag.
                    if regs.flags & CPU_FLAG_DIRECTION == 0 {
                        // Direction forward. Decrement DI.
                        regs.di = regs.di.wrapping_sub(4);
                    } else {
                        // Direction backwards. Increment DI.
                        regs.di = regs.di.wrapping_add(4);
                    }
                }
            }
        }

        let mut new_reg_data = reg_data.to_vec();
        regs.write_buf(&mut new_reg_data);

        match self.client.load_registers_from_buf(&new_reg_data) {
            Ok(_) => {
                self.regs = regs;
                true
            }
            Err(_) => false,
        }
    }

    pub fn load_registers_from_struct(&mut self, regs: &RemoteCpuRegisters) -> bool {
        self.reset(); // CPU is reset on register load

        let mut reg_data: [u8; 28] = [0; 28];
        reg_data[0] = (regs.ax & 0xFF) as u8;
        reg_data[1] = (regs.ax >> 8) as u8;
        reg_data[2] = (regs.bx & 0xFF) as u8;
        reg_data[3] = (regs.bx >> 8) as u8;
        reg_data[4] = (regs.cx & 0xFF) as u8;
        reg_data[5] = (regs.cx >> 8) as u8;
        reg_data[6] = (regs.dx & 0xFF) as u8;
        reg_data[7] = (regs.dx >> 8) as u8;
        reg_data[8] = (regs.ss & 0xFF) as u8;
        reg_data[9] = (regs.ss >> 8) as u8;
        reg_data[10] = (regs.sp & 0xFF) as u8;
        reg_data[11] = (regs.sp >> 8) as u8;
        reg_data[12] = (regs.flags & 0xFF) as u8;
        reg_data[13] = (regs.flags >> 8) as u8;
        reg_data[14] = (regs.ip & 0xFF) as u8;
        reg_data[15] = (regs.ip >> 8) as u8;
        reg_data[16] = (regs.cs & 0xFF) as u8;
        reg_data[17] = (regs.cs >> 8) as u8;
        reg_data[18] = (regs.ds & 0xFF) as u8;
        reg_data[19] = (regs.ds >> 8) as u8;
        reg_data[20] = (regs.es & 0xFF) as u8;
        reg_data[21] = (regs.es >> 8) as u8;
        reg_data[22] = (regs.bp & 0xFF) as u8;
        reg_data[23] = (regs.bp >> 8) as u8;
        reg_data[24] = (regs.si & 0xFF) as u8;
        reg_data[25] = (regs.si >> 8) as u8;
        reg_data[26] = (regs.di & 0xFF) as u8;
        reg_data[27] = (regs.di >> 8) as u8;

        match self.client.load_registers_from_buf(&reg_data) {
            Ok(_) => true,
            Err(_) => false,
        }
    }

    pub fn update_state(&mut self) -> bool {
        (
            self.program_state,
            self.control_status,
            self.status,
            self.command_status,
            self.data_bus,
        ) = self
            .client
            .get_cycle_state()
            .expect("Failed to get cycle state!");

        // BHE pin is packed into 8288 command status byte. Use it to set the
        // data bus width now.
        self.data_width = DataWidth::from((self.bhe(), self.a0()));
        true
    }

    pub fn get_last_error(&mut self) -> String {
        let error_msg = self
            .client
            .get_last_error()
            .unwrap_or_else(|err| format!("Couldn't get error string: {err}"));

        error_msg
    }

    #[inline]
    pub fn data_bus_str(&self) -> String {
        match self.data_width {
            DataWidth::Invalid => "----".to_string(),
            DataWidth::Sixteen => format!("{:04X}", self.data_bus),
            DataWidth::EightLow => format!("{:>4}", format!("{:02X}", self.data_bus as u8)),
            DataWidth::EightHigh => format!("{:<4}", format!("{:02X}", (self.data_bus >> 8) as u8)),
        }
    }

    /// Return true if the current address latch is within execution bounds.
    pub fn address_in_bounds(&self) -> bool {
        let addr = self.address_latch as usize;
        self.is_isr_address(self.address_latch)
            || ((addr >= self.start_addr) && (addr < self.end_addr))
    }

    pub fn in_preload(&self) -> bool {
        matches!(self.run_state, RunState::Preload)
    }

    pub fn fetch_from_memory(&mut self, address: u32, end_address: u32) -> u16 {
        let data = self.read_memory(address);

        match self.data_width {
            DataWidth::EightLow | DataWidth::EightHigh => data,
            DataWidth::Sixteen => {
                // Did we read past end_address?
                if address >= (end_address.wrapping_sub(1)) {
                    // Replace high byte with NOP.
                    self.program_end_offset += 1;
                    (data & 0xFF) | ((self.nop() as u16) << 8)
                } else {
                    data
                }
            }
            _ => data,
        }
    }

    /// Return a NOP instruction for the current emulation mode.
    #[inline]
    pub fn nop(&self) -> u8 {
        if self.do_emu8080 {
            OPCODE_NOP80
        } else {
            OPCODE_NOP
        }
    }

    // Produce a data bus value from a memory read.
    // This function is size-aware. For an 8-bit read, the upper byte will be 00.
    pub fn read_memory(&self, address: u32) -> u16 {
        log::trace!("read_memory(): data_width is {:?}", self.data_width);
        match self.data_width {
            DataWidth::EightLow => self.memory[self.address_latch as usize] as u16,
            DataWidth::EightHigh => (self.memory[self.address_latch as usize] as u16) << 8,
            DataWidth::Sixteen => u16::from_le_bytes([
                self.memory[self.address_latch as usize],
                self.memory[((self.address_latch + 1) as usize) & ADDRESS_SPACE_MASK],
            ]),
            _ => {
                log::error!("read_memory(): Invalid data width!");
                0
            }
        }
    }

    // Write a data bus value to memory
    // This function is size-aware. For an 8-bit write, the upper byte is ignored.
    pub fn write_memory(&mut self, address: u32, data: u16) {
        let mem_idx = address as usize & ADDRESS_SPACE_MASK;
        match self.data_width {
            DataWidth::EightLow => {
                self.memory[mem_idx] = self.data_bus as u8;
            }
            DataWidth::EightHigh => {
                self.memory[mem_idx] = (self.data_bus >> 8) as u8;
            }
            DataWidth::Sixteen => {
                let bytes = self.data_bus.to_le_bytes();
                self.memory[mem_idx] = bytes[0];
                self.memory[(mem_idx + 1) & ADDRESS_SPACE_MASK] = bytes[1];
            }
            _ => {
                log::error!("write_memory(): Invalid data width!");
            }
        }
    }

    pub fn cycle(&mut self) -> bool {
        match self.client.cycle() {
            Ok(_) => {}
            Err(e) => {
                log::warn!("Ignoring cycle timeout");
            }
        }

        self.bus_cycle = match self.bus_cycle {
            BusCycle::T1 => {
                // Capture the state of the bus transfer in T1, as the state will go PASV in t3-t4
                self.mcycle_state = get_bus_state!(self.status);

                // Only exit T1 state if bus transfer state indicates a bus transfer
                match get_bus_state!(self.status) {
                    BusState::PASV => BusCycle::T1,
                    BusState::HALT => BusCycle::T1,
                    _ => BusCycle::T2,
                }
            }
            BusCycle::T2 => {
                // If wait states are configured, deassert READY line now
                if self.wait_state_opt > 0 {
                    self.nready_states = self.wait_state_opt;
                    //log::debug!("Deasserting READY to emulate wait states...");
                    self.client
                        .write_pin(CpuPin::READY, false)
                        .expect("Failed to write READY pin!");
                }
                BusCycle::T3
            }
            BusCycle::T3 => {
                if self.nready_states > 0 {
                    self.nready_states -= 1;

                    if self.nready_states == 0 {
                        // Reassert READY line
                        self.client
                            .write_pin(CpuPin::READY, true)
                            .expect("Failed to write READY pin!");
                    }
                    BusCycle::Tw
                } else {
                    BusCycle::T4
                }
            }
            BusCycle::Tw => {
                if self.nready_states > 0 {
                    self.nready_states -= 1;

                    if self.nready_states == 0 {
                        // Reassert READY line
                        self.client
                            .write_pin(CpuPin::READY, true)
                            .expect("Failed to write READY pin!");
                    }
                    BusCycle::Tw
                } else {
                    BusCycle::T4
                }
            }
            BusCycle::T4 => {
                if self.mcycle_state == BusState::CODE {
                    // We completed a code fetch, so add to prefetch queue

                    match self.run_state {
                        RunState::Preload => {
                            if self.have_preload_pgm() {
                                // Preload program is being fetched.
                                self.queue.push(
                                    self.data_bus,
                                    self.data_width,
                                    QueueDataType::Preload,
                                    self.address_latch,
                                );
                            } else if self.address_in_bounds() {
                                log::trace!(
                                    "Preload: program pushed to queue: {}",
                                    self.data_bus_str()
                                );
                                // We are in preloading state, but have exhausted preload program. Mark the next byte to be put
                                // in queue to signal start of main program.
                                self.queue.push(
                                    self.data_bus,
                                    self.data_width,
                                    QueueDataType::Program,
                                    self.address_latch,
                                );
                            } else {
                                log::trace!(
                                    "Preload: out of instruction bounds data pushed to queue: {} @ [{:05X}]",
                                    self.data_bus_str(), self.address_latch
                                );
                                // We are in preloading state, but have exhausted preload program. Mark the next byte to be put
                                // in queue to signal start of main program.
                                self.queue.push(
                                    self.data_bus,
                                    self.data_width,
                                    QueueDataType::Finalize,
                                    self.address_latch,
                                );
                            }
                        }
                        _ if self.address_in_bounds() => {
                            // Normal fetch within program boundaries
                            self.queue.push(
                                self.data_bus,
                                self.data_width,
                                self.data_type,
                                self.address_latch,
                            );
                        }
                        _ => {
                            // We have fetched past the end of the current program, so push a flagged NOP into the queue.
                            // When a byte flagged with Finalize is read we will enter the Finalize state.
                            log::trace!("Fetch out of bounds: [{:05X}], in native mode. Tagging fetch as [Finalize].", self.address_latch);
                            // If we did not enter emulation, then we can immediately move to finalize.
                            self.queue.push(
                                self.data_bus,
                                self.data_width,
                                QueueDataType::Finalize,
                                self.address_latch,
                            );
                        }
                    }
                }
                BusCycle::T1
            }
        };

        self.update_state();
        if self.program_state == ProgramState::ExecuteDone {
            self.cycle_num += 1;
            return true;
        }

        if self.ale() {
            if self.bus_cycle != BusCycle::T1 {
                log::warn!("ALE on non-T1 cycle state! CPU desynchronized.");
                self.bus_cycle = BusCycle::T1;
            }

            let addr = self
                .client
                .read_address()
                .expect("Failed to get address bus!");
            self.address_bus = addr;
            self.address_latch = addr;
        } else {
            self.address_bus = self
                .client
                .read_address()
                .expect("Failed to get address bus!");
        }
        //log::trace!("state: {:?}", self.program_state);

        // Do reads & writes if we are in execute state.
        if self.program_state == ProgramState::Execute {
            if let BusState::HALT = get_bus_state!(self.status) {
                cycle_comment!(self, "CPU halted!");
                self.halted = true;
            }

            // MRDC status is active-low.
            if ((self.command_status & COMMAND_MRDC_BIT) == 0) && (self.bus_cycle == BusCycle::T2) {
                let mut write_store = false;
                let a0 = self.a0();

                match self.mcycle_state {
                    BusState::MEMR => {
                        // CPU is reading data from bus. Provide value from memory.
                        self.data_bus = self.read_memory(self.address_latch);
                    }
                    BusState::CODE => {
                        // CPU is reading code from bus. Provide value from memory if we are not past the
                        // end of the program area.
                        let bus_written = match self.run_state {
                            RunState::Preload if self.have_preload_pgm() => {
                                // Feed the CPU the preload program instead of memory.
                                let program = self.preload_pgm.as_mut().unwrap();

                                program.read_program(
                                    a0,
                                    &mut self.code_stream,
                                    QueueDataType::Program,
                                );
                                if !self.code_stream.is_empty() {
                                    let value = self.code_stream.pop_data_bus();
                                    log::trace!(
                                        "Writing [Preload] program: [{:0X}]",
                                        value.bus_value()
                                    );
                                    self.data_bus = value.bus_value();
                                    true
                                } else {
                                    false
                                }
                            }
                            _ => false,
                        };

                        if !bus_written {
                            if self.address_in_bounds() {
                                // Within program range.
                                let value = self
                                    .fetch_from_memory(self.address_latch, self.end_addr as u32);
                                log::trace!(
                                    "Reading [User] program: [{:0X}] end_addr: [{:05X}]",
                                    value,
                                    self.end_addr
                                );
                                self.data_bus = value;
                            } else {
                                log::trace!("Out of program bounds!");
                                // Prefetching out of bounds. This terminates execution; so we should start
                                // feeding the CPU server the store program.
                                write_store = true;
                            }
                        }

                        if write_store {
                            // Execute prefetch_store command instead of writing to the data bus ourselves.
                            log::trace!("Writing cpu_server store program byte to bus");
                            self.client
                                .prefetch_store()
                                .expect("Failed to execute CmdPrefetchStore");
                        } else {
                            if !self.address_in_bounds() {
                                log::warn!(
                                    "Writing user program out of bounds. CPU desynchronized."
                                );
                            }
                            log::trace!(
                                "Writing [User] program word to bus: [{:04X}]",
                                self.data_bus
                            );
                            self.client
                                .write_data_bus(self.data_bus)
                                .expect("Failed to write data bus.");
                        }
                    }
                    _ => {
                        // Handle other states?
                        log::warn!("Unhandled bus state!");
                    }
                }
            }

            // MWTC status is active-low.
            if (self.command_status & COMMAND_MWTC_BIT) == 0 {
                // CPU is writing to memory. Get data bus from CPU and write to host memory.
                self.data_bus = self
                    .client
                    .read_data_bus()
                    .expect("Failed to read data bus.");

                self.write_memory(self.address_latch, self.data_bus);
            }

            // IOWC status is active-low.
            if (self.command_status & COMMAND_IOWC_BIT) == 0 {
                // CPU is writing to IO address.

                self.data_bus = self
                    .client
                    .read_data_bus()
                    .expect("Failed to read data bus.");

                // Check if this is our special port address
                if self.address_latch == 0x000FF {
                    cycle_comment!(self, "IO write to INTR trigger!");

                    // Set INTR line high
                    self.client
                        .write_pin(CpuPin::INTR, true)
                        .expect("Failed to set INTR line high.");
                    self.intr = true;
                }
            }
        }

        // Handle queue activity
        if self.have_queue_status {
            let q_op = get_queue_op!(self.status);

            match q_op {
                QueueOp::First | QueueOp::Subsequent => {
                    // We fetched a byte from the queue last cycle
                    (self.queue_byte, self.queue_type, self.queue_fetch_addr) = self.queue.pop();
                    if q_op == QueueOp::First {
                        // First byte of instruction fetched.
                        self.queue_first_fetch = true;
                        self.queue_fetch_n = 0;
                        self.opcode = self.queue_byte;

                        // Was NMI triggered?
                        if self.do_nmi {
                            cycle_comment!(self, "Setting NMI pin high...");
                            self.client
                                .write_pin(CpuPin::NMI, true)
                                .expect("Failed to write NMI pin!");
                            self.do_nmi = false;
                        }

                        // Is this opcode an NMI trigger?
                        if self.opcode == OPCODE_NMI_TRIGGER {
                            // set flag to enable NMI on next instruction
                            //self.do_nmi = true;
                        }

                        // Does this opcode mark the end of a preload program?
                        self.advance_run_state_on_queue_read();

                        // Finalize execution if this queue byte was flagged as final
                        if self.queue_type == QueueDataType::Finalize {
                            self.finalize();
                        }

                        // Handle INTR instruction trigger
                        if !is_group_op(self.queue_byte) {
                            self.instruction_num += 1;

                            if self.instruction_num == self.intr_after {
                                cycle_comment!(
                                    self,
                                    "Setting INTR high after instruction #{}",
                                    self.intr_after
                                );

                                // Set INTR line high
                                self.client
                                    .write_pin(CpuPin::INTR, true)
                                    .expect("Failed to set INTR line high.");
                                self.intr = true;
                            }
                        }
                    } else {
                        // Subsequent byte of instruction fetched
                        self.queue_fetch_n += 1;
                    }
                }
                QueueOp::Flush => {
                    // Queue was flushed last cycle
                    self.queue.flush();
                }
                _ => {}
            }
        }

        // if self.halted {
        //     self.halt_ct += 1;
        //     if self.halt_ct == HALT_CYCLE_LIMIT {
        //         // cycle_comment!(self, "Setting INTR high to recover from halt...");
        //         // self.client
        //         //     .write_pin(CpuPin::INTR, true)
        //         //     .expect("Failed to write INTR pin!");
        //         // self.intr = true;
        //
        //         cycle_comment!(self, "Setting NMI high to recover from halt...");
        //         self.client
        //             .write_pin(CpuPin::NMI, true)
        //             .expect("Failed to write NMI pin!");
        //         self.nmi = true;
        //         self.do_nmi = false;
        //     }
        // }
        self.cycle_num += 1;

        // Do cycle-based INTR trigger
        if self.cycle_num == self.intr_on_cycle {
            cycle_comment!(
                self,
                "Setting INTR high after cycle #{}",
                self.intr_on_cycle
            );

            // Set INTR line high
            self.client
                .write_pin(CpuPin::INTR, true)
                .expect("Failed to set INTR line high.");
            self.intr = true;
        }

        // Do cycle-based NMI trigger
        if self.cycle_num == self.nmi_on_cycle {
            cycle_comment!(self, "Setting NMI high after cycle #{}", self.intr_on_cycle);

            // Set INTR line high
            self.client
                .write_pin(CpuPin::NMI, true)
                .expect("Failed to set NMI line high.");
            self.nmi = true;
        }

        if self.cycle_num > CYCLE_LIMIT {
            log::warn!("Hit cycle limit!");
            match self.client.finalize() {
                Ok(_) => {
                    log::trace!("Finalized execution!");
                }
                Err(_) => {
                    log::trace!(
                        "Failed to finalize: {}",
                        self.client.get_last_error().unwrap()
                    );
                }
            }
        }
        true
    }

    pub fn finalize(&mut self) {
        // Save the current queue length - we have to rewind the IP returned by store by this much.
        self.queue_len_at_finalize = self.queue.len() as u8;
        self.run_state = RunState::Finalize;
        log::trace!(
            "Finalizing execution with {} bytes in queue.",
            self.queue.len()
        );
        cycle_comment!(self, "Finalizing execution!");
        self.client.finalize().expect("Failed to finalize!");
    }

    pub fn advance_run_state_on_queue_read(&mut self) {
        match self.run_state {
            RunState::Preload => {
                if self.queue_type == QueueDataType::Program {
                    log::trace!("Ending preload, entering main Program!");
                    self.run_state = RunState::Program;
                }
                if self.queue_type == QueueDataType::EmuEnter {
                    panic!("Can't preload into emulation mode!");
                }
            }
            _ => {}
        }
    }

    pub fn print_cpu_state(&self) {
        println!("{}", self.get_cpu_state_str())
    }

    pub fn ale(&self) -> bool {
        self.control_status & CONTROL_ALE_BIT == 1
    }

    /// Return whether the BHE signal is asserted (active-low)
    pub fn bhe(&self) -> bool {
        self.command_status & 0x80 == 0
    }

    /// Return a bool representing the state of address pin 0.
    pub fn a0(&self) -> bool {
        self.address_latch & 0x1 != 0
    }

    pub fn get_cpu_state_str(&self) -> String {
        let ale_str = match self.ale() {
            true => "A:",
            false => "  ",
        };

        let mut seg_str = "  ";
        if self.bus_cycle != BusCycle::T1 {
            // Segment status only valid in T2+
            seg_str = match get_segment!(self.status) {
                Segment::ES => "ES",
                Segment::SS => "SS",
                Segment::CS => "CS",
                Segment::DS => "DS",
            };
        }

        let q_op = get_queue_op!(self.status);
        let q_op_chr = match q_op {
            QueueOp::Idle => ' ',
            QueueOp::First => 'F',
            QueueOp::Flush => 'E',
            QueueOp::Subsequent => 'S',
        };

        // All read/write signals are active/low
        let rs_chr = match self.command_status & 0b0000_0001 == 0 {
            true => 'R',
            false => '.',
        };
        let aws_chr = match self.command_status & 0b0000_0010 == 0 {
            true => 'A',
            false => '.',
        };
        let ws_chr = match self.command_status & 0b0000_0100 == 0 {
            true => 'W',
            false => '.',
        };
        let ior_chr = match self.command_status & 0b0000_1000 == 0 {
            true => 'R',
            false => '.',
        };
        let aiow_chr = match self.command_status & 0b0001_0000 == 0 {
            true => 'A',
            false => '.',
        };
        let iow_chr = match self.command_status & 0b0010_0000 == 0 {
            true => 'W',
            false => '.',
        };

        let bhe_chr = match self.command_status & 0b1000_0000 == 0 {
            true => 'B',
            false => '.',
        };

        let intr_chr = if self.intr { 'R' } else { '.' };
        let inta_chr = if self.command_status & COMMAND_INTA_BIT == 0 {
            'A'
        } else {
            '.'
        };

        let bus_state = get_bus_state!(self.status);
        let bus_str = match bus_state {
            BusState::INTA => "INTA",
            BusState::IOR => "IOR ",
            BusState::IOW => "IOW ",
            BusState::HALT => "HALT",
            BusState::CODE => "CODE",
            BusState::MEMR => "MEMR",
            BusState::MEMW => "MEMW",
            BusState::PASV => "PASV",
        };

        let t_str = match self.bus_cycle {
            BusCycle::T1 => "T1",
            BusCycle::T2 => "T2",
            BusCycle::T3 => "T3",
            BusCycle::T4 => "T4",
            BusCycle::Tw => "Tw",
        };

        let is_reading = is_reading!(self.command_status);
        let is_writing = is_writing!(self.command_status);

        let mut xfer_str = "        ".to_string();
        if let BusState::PASV = bus_state {
            let value = self.data_bus_str();
            if is_reading {
                xfer_str = format!("r-> {}", value);
            } else if is_writing {
                xfer_str = format!("<-w {}", value);
            }
        }

        // Handle queue activity
        let mut q_read_str = "       |".to_string();

        let decode_arch = if self.cpu_type.is_intel() {
            DecodeArch::Intel8088
        } else {
            match self.run_state {
                RunState::Program if self.do_emu8080 => DecodeArch::Intel8080,
                _ => DecodeArch::Intel8088,
            }
        };

        if q_op == QueueOp::First {
            // First byte of opcode read from queue. Decode it to opcode or group specifier
            if self.queue_byte == OPCODE_IRET {
                let iret_addr = self.queue_fetch_addr;
                let isr_base_addr = RemoteCpu::calc_linear_address(ISR_SEGMENT, 0);
                let isr_number = (iret_addr.wrapping_sub(isr_base_addr)) / 2;
                q_read_str = format!(
                    "q-> {:02X} | {} @ [{:05X}] ISR:{:02X}",
                    self.queue_byte,
                    opcodes::get_opcode_str(self.opcode, 0, false, decode_arch),
                    self.queue_fetch_addr,
                    isr_number
                );
            } else {
                q_read_str = format!(
                    "q-> {:02X} | {} @ [{:05X}]",
                    self.queue_byte,
                    opcodes::get_opcode_str(self.opcode, 0, false, decode_arch),
                    self.queue_fetch_addr
                );
            }
        } else if q_op == QueueOp::Subsequent {
            if is_group_op(self.opcode) && self.queue_fetch_n == 1 {
                // Modrm was just fetched for a group opcode, so display the mnemonic now
                q_read_str = format!(
                    "q-> {:02X} | {}",
                    self.queue_byte,
                    opcodes::get_opcode_str(self.opcode, self.queue_byte, true, decode_arch)
                );
            } else {
                // Not modrm byte
                q_read_str = format!("q-> {:02X} |", self.queue_byte);
            }
        }

        let c_comment = if let Some(comment) = self.cycle_comment.clone() {
            comment
        } else {
            "".to_string()
        };

        let rs_str = format!("{:?}", self.run_state);
        // format!(
        //     "[{:8}] {:08} {:02}[{:05X}:{:05X}] {:02} M:{}{}{} I:{}{}{} P:{}{} {:04} {:02} {:06} {:1}[{:width$}] {} {}",
        //     rs_str,
        //     self.cycle_num,
        //     ale_str,
        //     self.address_latch,
        //     self.address_bus,
        //     seg_str,
        //     rs_chr, aws_chr, ws_chr, ior_chr, aiow_chr, iow_chr,
        //     intr_chr, inta_chr,
        //     bus_str,
        //     t_str,
        //     xfer_str,
        //     q_op_chr,
        //     self.queue.to_string(),
        //     q_read_str,
        //     ccomment
        // )
        format!(
            "[{rs_str:8}] {cycle_num:08} {ale_str:02}[{addr_latch:05X}:{addr_bus:05X}] \
            {seg_str:02} M:{rs_chr}{aws_chr}{ws_chr} I:{ior_chr}{aiow_chr}{iow_chr} \
            P:{intr_chr}{inta_chr}{bhe_chr} {bus_str:04} {t_str:02} {xfer_str:06} {q_op_chr:1}[{q_str:width$}] {q_read_str} {c_comment}",
            rs_str = rs_str,
            cycle_num = self.cycle_num,
            ale_str = ale_str,
            addr_latch = self.address_latch,
            addr_bus = self.address_bus,
            seg_str = seg_str,
            rs_chr = rs_chr,
            aws_chr = aws_chr,
            ws_chr = ws_chr,
            ior_chr = ior_chr,
            aiow_chr = aiow_chr,
            iow_chr = iow_chr,
            intr_chr = intr_chr,
            inta_chr = inta_chr,
            bhe_chr = bhe_chr,
            bus_str = bus_str,
            t_str = t_str,
            xfer_str = xfer_str,
            q_op_chr = q_op_chr,
            q_str = self.queue.to_string(),
            width = self.queue.size() * 2,
            q_read_str = q_read_str,
            c_comment = c_comment
        )
    }

    /// Return whether we are inside the preload program.
    pub fn have_preload_pgm(&self) -> bool {
        if let Some(program) = &self.preload_pgm {
            !program.is_finished()
        } else {
            false
        }
    }

    pub fn print_run_state(&self, print_opts: &PrintOptions) {
        //log::trace!("print_run_state: {:?}", self.run_state);
        match self.run_state {
            RunState::Preload if print_opts.print_preload => {
                self.print_cpu_state();
            }
            RunState::Program if print_opts.print_pgm => {
                self.print_cpu_state();
            }
            RunState::Finalize if print_opts.print_finalize => {
                self.print_cpu_state();
            }
            _ => {}
        }
    }

    /// Run the CPU for the specified number of cycles.
    pub fn run(
        &mut self,
        cycle_limit: Option<u32>,
        print_opts: &PrintOptions,
    ) -> Result<RemoteCpuRegisters, bool> {
        if let Some(preload_pgm) = &mut self.preload_pgm {
            preload_pgm.reset();
            log::trace!("Entering [Preload] run state");
            self.run_state = RunState::Preload;
        } else {
            log::trace!("Entering [Program] run state");
            self.run_state = RunState::Program;
        }

        self.address_latch = self
            .client
            .read_address_latch()
            .expect("Failed to get address latch!");
        self.update_state();

        // ALE should be active at start of execution
        // TODO: We moved the ALE bit...
        if !self.ale() {
            log::warn!("Execution is not starting on T1.");
        }

        self.print_run_state(&print_opts);

        while self.program_state != ProgramState::ExecuteDone {
            match self.program_state {
                ProgramState::Execute => {
                    self.cycle();
                    self.print_run_state(&print_opts);
                    self.cycle_comment = None;
                }
                ProgramState::ExecuteFinalize => {
                    self.cycle();
                }
                _ => {
                    log::error!("Invalid program state: {:?}!", self.program_state);
                    panic!("Invalid program state!");
                }
            }

            //log::trace!("Program state: {:?}", self.program_state);
        }

        // Program finalized!
        log::trace!("Program finalized! Run store now.");
        let mut regs = self.store();

        match regs {
            Ok(mut regs) => {
                regs.rewind_ip(self.program_end_offset);
                Ok(regs)
            }
            Err(e) => {
                log::error!("Failed to store registers: {}", e);
                Err(false)
            }
        }
    }

    /// Command the CPU server to store registers, and return them as a RemoteCpuRegisters struct.
    pub fn store(&mut self) -> Result<RemoteCpuRegisters, CpuClientError> {
        let mut buf: [u8; 28] = [0; 28];
        self.client.store_registers_to_buf(&mut buf)?;
        let regs = RemoteCpuRegisters::from(&buf);
        Ok(regs)
    }

    /// Perform a sanity check that we have a valid address latch
    /// (Debugging use)
    pub fn test(&mut self) -> bool {
        let al = match self.client.read_address_latch() {
            Ok(address) => address,
            Err(_) => return false,
        };

        log::trace!("Initial Address Latch: {:05X}", al);
        true
    }

    pub fn print_reg_buf(reg_buf: &[u8], cpu_type: ServerCpuType) {
        Self::print_regs(&RemoteCpuRegisters::from(reg_buf), cpu_type);
    }

    pub fn print_regs(regs: &RemoteCpuRegisters, cpu_type: ServerCpuType) {
        let reg_str = format!(
            "AX: {:04X} BX: {:04X} CX: {:04X} DX: {:04X}\n\
          SP: {:04X} BP: {:04X} SI: {:04X} DI: {:04X}\n\
          CS: {:04X} DS: {:04X} ES: {:04X} SS: {:04X}\n\
          IP: {:04X}\n\
          FLAGS: {:04X}",
            regs.ax,
            regs.bx,
            regs.cx,
            regs.dx,
            regs.sp,
            regs.bp,
            regs.si,
            regs.di,
            regs.cs,
            regs.ds,
            regs.es,
            regs.ss,
            regs.ip,
            regs.flags
        );

        print!("{} ", reg_str);

        // Expand flag info
        let f = regs.flags;
        let c_chr = if CPU_FLAG_CARRY & f != 0 { 'C' } else { 'c' };
        let p_chr = if CPU_FLAG_PARITY & f != 0 { 'P' } else { 'p' };
        let a_chr = if CPU_FLAG_AUX_CARRY & f != 0 {
            'A'
        } else {
            'a'
        };
        let z_chr = if CPU_FLAG_ZERO & f != 0 { 'Z' } else { 'z' };
        let s_chr = if CPU_FLAG_SIGN & f != 0 { 'S' } else { 's' };
        let t_chr = if CPU_FLAG_TRAP & f != 0 { 'T' } else { 't' };
        let i_chr = if CPU_FLAG_INT_ENABLE & f != 0 {
            'I'
        } else {
            'i'
        };
        let d_chr = if CPU_FLAG_DIRECTION & f != 0 {
            'D'
        } else {
            'd'
        };
        let o_chr = if CPU_FLAG_OVERFLOW & f != 0 { 'O' } else { 'o' };
        let m_chr = if cpu_type.is_intel() {
            '1'
        } else {
            if f & CPU_FLAG_MODE != 0 {
                'M'
            } else {
                'm'
            }
        };

        println!(
            "{}111{}{}{}{}{}{}0{}0{}1{}",
            m_chr, o_chr, d_chr, i_chr, t_chr, s_chr, z_chr, a_chr, p_chr, c_chr
        );
    }

    pub fn print_regs_delta(
        initial: &RemoteCpuRegisters,
        regs: &RemoteCpuRegisters,
        cpu_type: ServerCpuType,
    ) {
        let a_diff = initial.ax != regs.ax;
        let b_diff = initial.bx != regs.bx;
        let c_diff = initial.cx != regs.cx;
        let d_diff = initial.dx != regs.dx;
        let sp_diff = initial.sp != regs.sp;
        let bp_diff = initial.bp != regs.bp;
        let si_diff = initial.si != regs.si;
        let di_diff = initial.di != regs.di;
        let cs_diff = initial.cs != regs.cs;
        let ds_diff = initial.ds != regs.ds;
        let es_diff = initial.es != regs.es;
        let ss_diff = initial.ss != regs.ss;
        let ip_diff = initial.ip != regs.ip;
        let f_diff = initial.flags != regs.flags;

        let reg_str = format!(
            "AX:{}{:04X} BX:{}{:04X} CX:{}{:04X} DX:{}{:04X}\n\
             SP:{}{:04X} BP:{}{:04X} SI:{}{:04X} DI:{}{:04X}\n\
             CS:{}{:04X} DS:{}{:04X} ES:{}{:04X} SS:{}{:04X}\n\
             IP: {:04X}\n\
             FLAGS:{}{:04X}",
            if a_diff { "*" } else { " " },
            regs.ax,
            if b_diff { "*" } else { " " },
            regs.bx,
            if c_diff { "*" } else { " " },
            regs.cx,
            if d_diff { "*" } else { " " },
            regs.dx,
            if sp_diff { "*" } else { " " },
            regs.sp,
            if bp_diff { "*" } else { " " },
            regs.bp,
            if si_diff { "*" } else { " " },
            regs.si,
            if di_diff { "*" } else { " " },
            regs.di,
            if cs_diff { "*" } else { " " },
            regs.cs,
            if ds_diff { "*" } else { " " },
            regs.ds,
            if es_diff { "*" } else { " " },
            regs.es,
            if ss_diff { "*" } else { " " },
            regs.ss,
            regs.ip,
            if f_diff { "*" } else { " " },
            regs.flags
        );

        print!("{} ", reg_str);

        // Expand flag info
        let f = regs.flags;
        let c_chr = if CPU_FLAG_CARRY & f != 0 { 'C' } else { 'c' };
        let p_chr = if CPU_FLAG_PARITY & f != 0 { 'P' } else { 'p' };
        let a_chr = if CPU_FLAG_AUX_CARRY & f != 0 {
            'A'
        } else {
            'a'
        };
        let z_chr = if CPU_FLAG_ZERO & f != 0 { 'Z' } else { 'z' };
        let s_chr = if CPU_FLAG_SIGN & f != 0 { 'S' } else { 's' };
        let t_chr = if CPU_FLAG_TRAP & f != 0 { 'T' } else { 't' };
        let i_chr = if CPU_FLAG_INT_ENABLE & f != 0 {
            'I'
        } else {
            'i'
        };
        let d_chr = if CPU_FLAG_DIRECTION & f != 0 {
            'D'
        } else {
            'd'
        };
        let o_chr = if CPU_FLAG_OVERFLOW & f != 0 { 'O' } else { 'o' };
        let m_chr = if cpu_type.is_intel() {
            '1'
        } else {
            if f & CPU_FLAG_MODE != 0 {
                'M'
            } else {
                'm'
            }
        };

        println!(
            "{}111{}{}{}{}{}{}0{}0{}1{}",
            m_chr, o_chr, d_chr, i_chr, t_chr, s_chr, z_chr, a_chr, p_chr, c_chr
        );
    }
}
