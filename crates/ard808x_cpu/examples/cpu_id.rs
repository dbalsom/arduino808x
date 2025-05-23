/*
    Arduino8088 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduino_8088

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
use ard808x_client::*;
use ard808x_cpu::*;

fn main() {
    env_logger::init();

    // Create a cpu_client connection to cpu_server.
    let mut cpu_client = match CpuClient::init() {
        Ok(ard_client) => {
            println!("Opened connection to Arduino_808X server!");
            ard_client
        }
        Err(e) => {
            eprintln!("Error connecting to Arduino_808X server: {e}");
            std::process::exit(1);
        }
    };

    match cpu_client.cpu_type() {
        Ok(cpu_id) => {
            println!("Detected CPU: {:?}", cpu_id);
        }
        Err(e) => {
            eprintln!("Error detecting CPU: {e}");
            std::process::exit(1);
        }
    }
}
