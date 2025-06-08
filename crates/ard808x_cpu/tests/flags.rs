use ard808x_client::*;
use ard808x_cpu::*;

#[test]
fn test_flag_init() {
    // Create a cpu_client connection to cpu_server.
    let cpu_client = match CpuClient::init(None) {
        Ok(ard_client) => {
            println!("Opened connection to Arduino_8088 server!");
            ard_client
        }
        Err(e) => {
            eprintln!("Error connecting to Arduino_8088 server: {e}");
            std::process::exit(1);
        }
    };

    // Create a remote cpu instance using the cpu_client which should now be connected.
    let mut cpu = RemoteCpu::new(cpu_client, false, false, 0, 0, 0, 0);

    let regs = RemoteCpuRegisters {
        ax: 0,
        bx: 0,
        cx: 0,
        dx: 0,
        ss: 0,
        ds: 0,
        es: 0,
        sp: 0xFFFF,
        bp: 0,
        si: 0,
        di: 0,
        cs: 0xF000,
        ip: 0x0000,
        flags: 0xF002,
    };

    // Load the registers from struct
    let result = cpu.load_registers_from_struct(&regs);
    if result {
        log::trace!("Successfully set up registers!");

        // Load opcode into memory at cs:ip
        let pc = (regs.cs as usize) << 4 + regs.ip as usize;
        cpu.write_u8(pc, 0x90); // NOP
        cpu.set_program_bounds(pc, pc + 1);

        cpu.test();
        match cpu.run(Some(100), &PrintOptions::default()) {
            Ok(regs) => {
                println!("Flags: {:04X}", regs.flags);
            }
            Err(_) => {
                log::error!("Program execution failed!");
            }
        }
    } else {
        log::error!("Register setup failed: {}", cpu.get_last_error());
    }
}
