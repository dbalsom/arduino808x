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

// Functions to emulate the processor instruction queue.
// This is used for server state control on CPUs that have queue status lines
// and enables debug display of queue state contents and reporting of same to
// the client.

void init_queue() {
  CPU.queue.len = 0;
  CPU.queue.back = 0;
  CPU.queue.front = 0;
}

void push_queue(uint16_t data, uint8_t dtype, data_width_t width) {
  if (width == EightLow) {
    // 8-bit low byte fetch (8088/V20)
    //Serial1.println("### 8-bit queue push ###");
    if(queue_has_room(CPU.data_width)) {
      CPU.queue.queue[CPU.queue.front] = (uint8_t)data;
      CPU.queue.types[CPU.queue.front] = dtype;
      CPU.queue.front = (CPU.queue.front + 1) % CPU.queue.size;
      CPU.queue.len++;
    }
  }
  else if (width == EightHigh) {
    // 8-bit high byte fetch (8086/V30 odd address)
    if(queue_has_room(CPU.data_width)) {
      CPU.queue.queue[CPU.queue.front] = (uint8_t)(data >> 8);
      CPU.queue.types[CPU.queue.front] = dtype;
      CPU.queue.front = (CPU.queue.front + 1) % CPU.queue.size;
      CPU.queue.len++;
    }
  }
  else {
    // 16-bit fetch
    //Serial1.println("### 16-bit queue push ###");
    if(queue_has_room(CPU.data_width)) {
      CPU.queue.queue[CPU.queue.front] = (uint8_t)data;
      CPU.queue.types[CPU.queue.front] = dtype;
      CPU.queue.front = (CPU.queue.front + 1) % CPU.queue.size;
      CPU.queue.queue[CPU.queue.front] = (uint8_t)(data >> 8);
      CPU.queue.types[CPU.queue.front] = dtype;
      CPU.queue.front = (CPU.queue.front + 1) % CPU.queue.size;
      CPU.queue.len += 2;
    }
  }
}

bool pop_queue(uint8_t *byte, uint8_t *dtype) {
  if(CPU.queue.len > 0) {
    *byte = CPU.queue.queue[CPU.queue.back];
    *dtype = CPU.queue.types[CPU.queue.back];
    CPU.queue.back = (CPU.queue.back + 1) % CPU.queue.size;
    CPU.queue.len--;
    return true;
  }
  else {
    return false;
  }
}

// Return true if we have room in the queue for a
bool queue_has_room(data_width_t width) {
  bool has_room = false;

  if((width == EightLow) || (width == EightHigh)) {
    return CPU.queue.len + 1 <= CPU.queue.size;
  }
  else {
    return CPU.queue.len + 2 <= CPU.queue.size;
  }
}

void empty_queue() {
  // Need to rewind the program counter by length of queue on flush.
  // Otherwise we would lose opcodes already fetched.
  CPU.v_pc -= CPU.queue.len;
  init_queue();
}

void print_queue() {
  char buf[(6 * 2) + 1] = {0};
  char hex[3] = {0};
  uint8_t i;
  uint8_t byte;
  for(i = 0; i < CPU.queue.len; i++ ) {
    byte = CPU.queue.queue[(CPU.queue.back + i) % CPU.queue.size];
    sprintf(hex, "%02X", byte);
    strcat(buf, hex);
  }
  Serial.println(buf);
}

uint8_t read_queue(size_t idx) {
  if(idx < CPU.queue.len) {
    return CPU.queue.queue[(CPU.queue.back + idx) % CPU.queue.size];
  }
  else {
    return 0;
  }
}

const char *queue_to_string() {
  const size_t buf_len = (6 * 2) + 1;
  static char buf[buf_len];
  char *buf_p = buf;
  *buf_p = 0;
  uint8_t byte;
  for(uint8_t i = 0; i < CPU.queue.len; i++ ) {
    byte = CPU.queue.queue[(CPU.queue.back + i) % CPU.queue.size];
    snprintf(buf_p, buf_len - (i * 2), "%02X", byte);
    buf_p += 2;
  }  

  return buf;
}