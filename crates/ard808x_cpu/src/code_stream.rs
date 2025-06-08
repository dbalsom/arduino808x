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

use std::collections::VecDeque;

use crate::opcodes::OPCODE_NOP;
use crate::queue::QueueDataType;
use ard808x_client::CpuWidth;

pub struct CodeStream {
    width: CpuWidth,
    bytes: VecDeque<(u8, QueueDataType)>,
}

pub enum CodeStreamValue {
    Byte(u16, QueueDataType),
    Word(u16, QueueDataType, QueueDataType),
}

impl CodeStreamValue {
    pub fn bus_value(&self) -> u16 {
        match &self {
            CodeStreamValue::Byte(val, _) => *val,
            CodeStreamValue::Word(val, _, _) => *val,
        }
    }
}

impl CodeStream {
    pub fn new(width: CpuWidth) -> Self {
        Self {
            width,
            bytes: Default::default(),
        }
    }

    pub fn push_byte(&mut self, data: u8, data_type: QueueDataType) {
        self.bytes.push_back((data, data_type))
    }

    pub fn push_word(&mut self, data: u16, data_type: QueueDataType) {
        let bytes = data.to_le_bytes();
        self.bytes.push_back((bytes[0], data_type));
        self.bytes.push_back((bytes[1], data_type));
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn have_complete_data(&self) -> bool {
        match self.width {
            CpuWidth::Eight => self.bytes.len() > 0,
            CpuWidth::Sixteen => self.bytes.len() > 1,
        }
    }

    /// Pop a value of the appropriate width from the code stream deque and return it, along with
    /// a tuple of data types. Overflows are filled with NOPs set to type Fill.
    pub fn pop_data_bus(&mut self) -> CodeStreamValue {
        match self.width {
            CpuWidth::Eight => {
                let byte0_val = self
                    .bytes
                    .pop_front()
                    .unwrap_or((OPCODE_NOP, QueueDataType::Fill));
                let bus_value = byte0_val.0 as u16;

                CodeStreamValue::Byte(bus_value, byte0_val.1)
            }
            CpuWidth::Sixteen => {
                let byte0_val = self
                    .bytes
                    .pop_front()
                    .unwrap_or((OPCODE_NOP, QueueDataType::Fill));
                let byte1_val = self
                    .bytes
                    .pop_front()
                    .unwrap_or((OPCODE_NOP, QueueDataType::Fill));
                let bus_value = (byte0_val.0 as u16) | ((byte1_val.0 as u16) << 8);

                CodeStreamValue::Word(bus_value, byte0_val.1, byte1_val.1)
            }
        }
    }
}
