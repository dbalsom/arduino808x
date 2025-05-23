use ard808x_client::{CpuWidth, DataWidth};

#[derive (Copy, Clone, PartialEq)]
pub enum QueueDataType {
    Preload,
    EmuEnter,
    Program,
    EmuExit,
    Finalize,
    Fill,
}

#[derive (Copy, Clone)]
pub struct QueueEntry {
    opcode: u8,
    dtype: QueueDataType,
    addr: u32
}

pub struct InstructionQueue {
    width: CpuWidth,
    size: usize,
    len: usize,
    back: usize,
    front: usize,
    q: Vec<QueueEntry>,
}

impl InstructionQueue {
    pub fn new(width: CpuWidth) -> Self {
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
                }; width.queue_size()
            ],
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
                        addr
                    };
                    self.front = (self.front + 1) % self.size;
                    self.len += 1;
                }
                DataWidth::Sixteen => {
                    self.q[self.front] = QueueEntry {
                        opcode: data as u8,
                        dtype,
                        addr
                    };
                    self.front = (self.front + 1) % self.size;
                    self.q[self.front] = QueueEntry {
                        opcode: (data >> 8) as u8,
                        dtype,
                        addr
                    };
                    self.front = (self.front + 1) % self.size;
                    self.len += 2;
                }
                _ => {
                    log::error!("Bad DataWidth for queue push: {:?}", width);
                }
            }
        }
        else {
            //panic!("Queue overrun!");
            log::error!("Queue overrun!");
        }
    }

    pub fn pop(&mut self) -> (u8, QueueDataType, u32) {
        if self.len > 0 {
            let q_entry = self.q[self.back];
            //let dt = self.dt[self.back];

            self.back = (self.back + 1) % self.size;
            self.len -= 1;

            return (q_entry.opcode, q_entry.dtype, q_entry.addr)
        }

        panic!("Queue underrun!");
    }

    pub fn flush(&mut self) {
        self.len = 0;
        self.back = 0;
        self.front = 0;
    }

    pub fn to_string(&self) -> String {

        let mut base_str = "".to_string();

        for i in 0..self.len {
            base_str.push_str(&format!("{:02X}", self.q[(self.back + i) % self.size].opcode));
        }
        base_str
    }


}