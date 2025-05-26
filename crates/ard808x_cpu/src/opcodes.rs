pub const OPCODE_IRET: u8 = 0xCF;
pub const OPCODE_NOP: u8 = 0x90;
pub const OPCODE_NOPS: u16 = 0x9090;
pub const OPCODE_NOP80: u8 = 0x00; // NOP for 8080
pub const OPCODE_NOPS80: u16 = 0x0000; // NOP for 8080
pub const OPCODE_NMI_TRIGGER: u8 = 0xF1; // Undefined opcode to use as NMI trigger

/*
#define MODRM_OP(M) (((M & 0b00111000) >> 3) & 0x07)
#define IS_GRP_OP(O) ((OPCODE_REFS[O] >= GRP1) && (OPCODE_REFS[O] <= GRP2B))
*/

pub enum DecodeArch {
    Intel8088,
    Intel8080,
}

macro_rules! modrm_op {
    ($m:expr) => {
        ((($m >> 3) & 0x07) as usize)
    };
}

pub fn is_prefix(op1: u8) -> bool {
    match op1 {
        0x26 | 0x2E | 0x36 | 0x3E | 0xF0..=0xF3 => true,
        _ => false,
    }
}

pub fn is_group_op(op1: u8) -> bool {
    (OPCODE_REFS[op1 as usize] >= 105) && (OPCODE_REFS[op1 as usize] <= 110)
}

// Return the mnemonic string for the specified opcode. If the opcode is a group
// opcode, op2 should be specified and modrm set to true.
pub fn get_opcode_str(op1: u8, op2: u8, modrm: bool, decode_arch: DecodeArch) -> &'static str {
    let op_idx: usize = match decode_arch {
        DecodeArch::Intel8088 => OPCODE_REFS[op1 as usize],
        DecodeArch::Intel8080 => OPCODE_8080_REFS[op1 as usize],
    };

    if !modrm {
        // Just return primary opcode
        match decode_arch {
            DecodeArch::Intel8088 => {
                return OPCODE_STRS[op_idx];
            }
            DecodeArch::Intel8080 => {
                return OPCODE_8080_STRS[op_idx];
            }
        }
    } else {
        // modrm is in use, check if this is a group instruction...
        if is_group_op(op1) {
            // Lookup opcode group
            let grp_idx: usize = modrm_op!(op2);

            match OPCODE_REFS[op1 as usize] {
                GRP1 => OPCODE_STRS_GRP1[grp_idx],
                GRP2A => OPCODE_STRS_GRP2A[grp_idx],
                GRP2B => OPCODE_STRS_GRP2B[grp_idx],
                GRP3 => OPCODE_STRS_GRP3[grp_idx],
                GRP4 => OPCODE_STRS_GRP4[grp_idx],
                GRP5 => OPCODE_STRS_GRP5[grp_idx],
                _ => "ERROR",
            }
        } else {
            // Not a group instruction, just return as normal
            OPCODE_STRS[op_idx]
        }
    }
}

const GRP1: usize = 105;
const GRP2A: usize = 106;
const GRP2B: usize = 110;
const GRP3: usize = 107;
const GRP4: usize = 108;
const GRP5: usize = 109;

// LUT of primary opcode to Mnemonic (Or Group name)
const OPCODE_REFS: [usize; 256] = [
    0, 0, 0, 0, 0, 0, 1, 2, 3, 3, 3, 3, 3, 3, 1, 2, 4, 4, 4, 4, 4, 4, 1, 2, 5, 5, 5, 5, 5, 5, 1, 2,
    6, 6, 6, 6, 6, 6, 7, 8, 9, 9, 9, 9, 9, 9, 10, 11, 12, 12, 12, 12, 12, 12, 13, 14, 15, 15, 15,
    15, 15, 15, 16, 17, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 1, 1, 1, 1,
    1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
    35, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 105, 105, 105, 105, 36, 36,
    37, 37, 38, 38, 38, 38, 38, 39, 38, 2, 111, 37, 37, 37, 37, 37, 37, 37, 40, 41, 42, 103, 43,
    44, 45, 46, 38, 38, 38, 38, 47, 48, 49, 50, 36, 36, 51, 52, 53, 54, 55, 56, 38, 38, 38, 38, 38,
    38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 57, 57, 57, 57, 58, 59, 38, 38, 60, 60, 60, 60, 61,
    61, 62, 63, 106, 106, 110, 110, 71, 73, 104, 75, 104, 104, 104, 104, 104, 104, 104, 104, 76,
    77, 78, 79, 80, 80, 81, 81, 82, 83, 84, 83, 80, 80, 81, 81, 85, 104, 86, 87, 89, 90, 107, 107,
    97, 98, 99, 100, 101, 102, 108, 109,
];

const OPCODE_8080_REFS: [usize; 256] = [
    0, 1, 2, 3, 4, 5, 6, 7, 80, 8, 9, 10, 4, 5, 6, 11, 80, 1, 2, 3, 4, 5, 6, 12, 80, 8, 9, 10, 4,
    5, 6, 13, 80, 1, 14, 3, 4, 5, 6, 15, 80, 8, 16, 10, 4, 5, 6, 17, 80, 1, 18, 3, 4, 5, 6, 19, 80,
    8, 20, 10, 4, 5, 6, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
    22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
    22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 22, 22, 22, 22, 22, 22, 22, 22, 22, 24, 24,
    24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27,
    27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30,
    30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
    42, 80, 43, 44, 45, 39, 46, 33, 47, 48, 49, 37, 50, 39, 51, 80, 52, 53, 54, 80, 55, 39, 56, 33,
    57, 58, 59, 37, 60, 39, 61, 62, 63, 64, 65, 81, 68, 39, 69, 33, 70, 71, 72, 37, 73, 39, 74, 75,
    76, 77, 78, 80, 79, 39,
];

const OPCODE_STRS: &[&str] = &[
    "ADD", "PUSH", "POP", "OR", "ADC", "SBB", "AND", "ES", "DAA", "SUB", "CS", "DAS", "XOR", "SS",
    "AAA", "CMP", "DS", "AAS", "INC", "DEC", "JO", "JNO", "JB", "JNB", "JZ", "JNZ", "JBE", "JNBE",
    "JS", "JNS", "JP", "JNP", "JL", "JNL", "JLE", "JNLE", "TEST", "XCHG", "MOV", "LEA", "CBW",
    "CWD", "CALLF", "PUSHF", "POPF", "SAHF", "LAHF", "MOVSB", "MOVSW", "CMPSB", "CMPSW", "STOSB",
    "STOSW", "LODSB", "LODSW", "SCASB", "SCASW", "RETN", "LES", "LDS", "RETF", "INT", "INTO",
    "IRET", "ROL", "ROR", "RCL", "RCR", "SHL", "SHR", "SAR", "AAM", "AMX", "AAD", "ADX", "XLAT",
    "LOOPNE", "LOOPE", "LOOP", "JCXZ", "IN", "OUT", "CALL", "JMP", "JMPF", "LOCK", "REPNZ", "REP",
    "REPZ", "HLT", "CMC", "NOT", "NEG", "MUL", "IMUL", "DIV", "IDIV", "CLC", "STC", "CLI", "STI",
    "CLD", "STD", "WAIT", "INVAL", "GRP1", "GRP2A", "GRP3", "GRP4", "GRP5", "GRP2B", "NOP",
];

// 0x80 - 0x81
const OPCODE_STRS_GRP1: &[&str] = &["ADD", "OR", "ADC", "SBB", "AND", "SUB", "XOR", "CMP"];

// 0xD0 - 0xD1
const OPCODE_STRS_GRP2A: &[&str] = &["ROL", "ROR", "RCL", "RCR", "SHL", "SHR", "SETMO", "SAR"];

// 0xD2 - 0xD3
const OPCODE_STRS_GRP2B: &[&str] = &["ROL", "ROR", "RCL", "RCR", "SHL", "SHR", "SETMOC", "SAR"];

// 0xF6 - 0xF7
const OPCODE_STRS_GRP3: &[&str] = &["TEST", "TEST", "NOT", "NEG", "MUL", "IMUL", "DIV", "IDIV"];

// 0xFE
const OPCODE_STRS_GRP4: &[&str] = &[
    "INC", "DEC", "INVAL", "INVAL", "INVAL", "INVAL", "INVAL", "INVAL",
];

// 0xFF
const OPCODE_STRS_GRP5: &[&str] = &[
    "INC", "DEC", "CALL", "CALLF", "JMP", "JMPF", "PUSH", "INVAL",
];

const OPCODE_8080_STRS: &[&str] = &[
    "NOP", "LXI", "STAX", "INX", "INR", "DCR", "MVI", "RLC", "DAD", "LDAX", "DCX", "RRC", "RAL",
    "RAR", "SHLD", "DAA", "LHLD", "CMA", "STA", "STC", "LDA", "CMC", "MOV", "HLT", "ADD", "ADC",
    "SUB", "SBB", "ANA", "XRA", "ORA", "CMP", "RNZ", "POP", "JNZ", "JMP", "CNZ", "PUSH", "ADI",
    "RST", "RZ", "RET", "JZ", "CZ", "CALL", "ACI", "RNC", "JNC", "OUT", "CNC", "SUI", "RC", "JC",
    "IN", "CC", "SBI", "RPO", "JPO", "XTHL", "CPO", "ANI", "RPE", "PCHL", "JPE", "XCHG", "CPE",
    "CALLN", "RETEM", "XRI", "RP", "JP", "DI", "CP", "ORI", "RM", "SPHL", "JM", "EI", "CM", "CPI",
    "INVAL", "SPECIAL",
];
