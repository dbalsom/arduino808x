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

use ard808x_client::{CpuWidth, DataWidth};

#[derive(Copy, Clone, PartialEq)]
pub enum QueueDataType {
    Preload,
    EmuEnter,
    Program,
    Finalize,
    Fill,
}

#[derive(Copy, Clone)]
pub struct QueueEntry {
    opcode: u8,
    dtype: QueueDataType,
    addr: u32,
}

pub struct InstructionQueue {
    width: CpuWidth,
    size: usize,
    len: usize,
    back: usize,
    front: usize,
    q: Vec<QueueEntry>,
    silent: bool,
}

impl InstructionQueue {
    pub fn new(width: CpuWidth, silent: bool) -> Self {
        Self {
            width,
            size: width.queue_size(),
            len: 0,
            back: 0,
            front: 0,
            q: vec![
                QueueEntry {
                    opcode: 0,
                    dtype: QueueDataType::Program,
                    addr: 0,
                };
                width.queue_size()
            ],
            silent,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn has_room(&self) -> bool {
        self.len() + usize::from(self.width) <= self.size
    }

    pub fn push(&mut self, data: u16, width: DataWidth, dtype: QueueDataType, addr: u32) {
        if self.has_room() {
            match width {
                DataWidth::EightHigh => {
                    self.q[self.front] = QueueEntry {
                        opcode: (data >> 8) as u8,
                        dtype,
                        addr,
                    };
                    self.front = (self.front + 1) % self.size;
                    self.len += 1;
                }
                DataWidth::Sixteen => {
                    self.q[self.front] = QueueEntry {
                        opcode: data as u8,
                        dtype,
                        addr,
                    };
                    self.front = (self.front + 1) % self.size;
                    self.q[self.front] = QueueEntry {
                        opcode: (data >> 8) as u8,
                        dtype,
                        addr,
                    };
                    self.front = (self.front + 1) % self.size;
                    self.len += 2;
                }
                _ => {
                    log::error!("Bad DataWidth for queue push: {:?}", width);
                }
            }
        } else {
            if !self.silent {
                log::error!("Queue overrun!");
            }
        }
    }

    pub fn pop(&mut self) -> (u8, QueueDataType, u32) {
        if self.len > 0 {
            let q_entry = self.q[self.back];
            //let dt = self.dt[self.back];

            self.back = (self.back + 1) % self.size;
            self.len -= 1;

            (q_entry.opcode, q_entry.dtype, q_entry.addr)
        } else {
            if !self.silent {
                log::error!("Queue underrun!");
            }
            (0, QueueDataType::Program, 0)
        }
    }

    pub fn flush(&mut self) {
        self.len = 0;
        self.back = 0;
        self.front = 0;
    }

    pub fn to_string(&self) -> String {
        let mut base_str = "".to_string();

        for i in 0..self.len {
            base_str.push_str(&format!(
                "{:02X}",
                self.q[(self.back + i) % self.size].opcode
            ));
        }
        base_str
    }
}
