use ard808x_client::*;
use ard808x_cpu::*;

#[derive(Copy, Clone)]
pub struct AaaResult {
    ax: u16,
    flags: u16,
    of: bool,
    sf: bool,
    zf: bool,
    pf: bool,
}

#[test]
fn test_aaa() {
    // Create a cpu_client connection to cpu_server.

    let mut results = [AaaResult {
        ax: 0,
        flags: 0,
        of: false,
        sf: false,
        zf: false,
        pf: false,
    }; 512];

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

    let cf = true;

    for af in 0..2 {
        for i in 0..256 {
            //println!("i:{}", i);
            let mut regs = RemoteCpuRegisters {
                ax: i as u16,
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
                flags: 0,
            };

            regs.flags &= !CPU_FLAG_AUX_CARRY;

            if cf {
                regs.flags |= CPU_FLAG_CARRY;
            }

            if af == 1 {
                regs.flags |= CPU_FLAG_AUX_CARRY;
            }

            // Load the registers from struct
            let result = cpu.load_registers_from_struct(&regs);
            if result {
                log::trace!("Successfully set up registers!");

                // Load opcode into memory at cs:ip
                let pc = (regs.cs as usize) << 4 + regs.ip as usize;
                cpu.write_u8(pc, 0x37); // AAA
                cpu.set_program_bounds(pc, pc + 1);

                cpu.test();
                match cpu.run(Some(100), &PrintOptions::default()) {
                    Ok(regs) => {
                        let idx = i + (256 * af);
                        println!("idx: {}", idx);
                        results[idx].ax = regs.ax;
                        results[idx].flags = regs.flags;
                        results[idx].of = regs.flags & CPU_FLAG_OVERFLOW != 0;
                        results[idx].sf = regs.flags & CPU_FLAG_SIGN != 0;
                        results[idx].zf = regs.flags & CPU_FLAG_ZERO != 0;
                        results[idx].pf = regs.flags & CPU_FLAG_PARITY != 0;
                        //RemoteCpu::print_regs(&regs);
                    }
                    Err(_) => {
                        log::error!("Program execution failed!");
                    }
                }
            } else {
                log::error!("Register setup failed: {}", cpu.get_last_error());
            }
        }
    }

    for i in 0..256 {
        println!(
            "{:04X} (af==0): ax: {:04X} flags: {:04X} of: {} sf: {} zf: {} pf: {}",
            i & 0xFF,
            results[i].ax,
            results[i].flags,
            results[i].of,
            results[i].sf,
            results[i].zf,
            results[i].pf
        );
    }
    for i in 256..512 {
        println!(
            "{:04X} (af==1): ax: {:04X} flags: {:04X} of: {} sf: {} zf: {} pf: {}",
            i & 0xFF,
            results[i].ax,
            results[i].flags,
            results[i].of,
            results[i].sf,
            results[i].zf,
            results[i].pf
        );
    }
}
