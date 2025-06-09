use std::path::PathBuf;

use ard808x_client::*;
use ard808x_cpu::ard808x_client;
use ard808x_cpu::*;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(long)]
    com_port: Option<String>,

    // The binary file containing the register data. Produced from an assembly
    // file 'program_regs.asm'
    #[arg(long, required(true))]
    reg_file: PathBuf,

    // The binary file containing the code to execute.
    #[arg(long, required(true))]
    bin_file: PathBuf,

    // Specify the address in memory to mount the bin file. This should typically
    // match the address specified by CS:IP, but doesn't have to...
    #[arg(long, required(true))]
    mount_addr: String,

    // Specify the number of wait states for every bus transfer.
    // TODO: Currently no division between memory and IO, should change...
    #[arg(long, default_value_t = 0)]
    wait_states: u32,

    // Enter 8080 emulation mode. Must have a compatible CPU such as a V20/V30.
    #[arg(long, default_value_t = false)]
    emu8080: bool,

    // Fill the prefetch queue before executing code.
    #[arg(long, default_value_t = false)]
    prefetch: bool,

    // Raise the INTR line on N cycles after HLT.
    #[arg(long, default_value_t = 0)]
    intr_after: u32,

    // Raise the INTR line on the specified cycle #.
    #[arg(long, default_value_t = 0)]
    intr_on: u32,

    // Raise the NMI line on the specified cycle #.
    #[arg(long)]
    nmi_on: Option<u32>,

    // Run the CPU for a single instruction.
    #[arg(long, default_value_t = false)]
    single_step: bool,
}

fn main() {
    env_logger::init();

    let args = Args::parse();

    // Parse commandline arguments
    let reg_bytes = std::fs::read(args.reg_file.clone()).unwrap_or_else(|e| {
        eprintln!("Couldn't read register file {:?}: {}", args.reg_file, e);
        std::process::exit(1);
    });

    let bin_bytes = std::fs::read(args.bin_file.clone()).unwrap_or_else(|e| {
        eprintln!("Couldn't read binary file {:?}: {}", args.bin_file, e);
        std::process::exit(1);
    });

    let mount_addr = u32::from_str_radix(&args.mount_addr, 16).unwrap_or_else(|e| {
        eprintln!(
            "Couldn't parse code mount address '{}': {}",
            args.mount_addr, e
        );
        std::process::exit(1);
    });

    if (mount_addr as usize) > (0xFFFFFusize - bin_bytes.len()) {
        eprintln!("Specified mount point out of range.");
        std::process::exit(1);
    }

    // Create a cpu_client connection to cpu_server.
    let cpu_client = match CpuClient::init(args.com_port.clone()) {
        Ok(ard_client) => {
            println!("Opened connection to Arduino_8088 server!");
            ard_client
        }
        Err(e) => {
            eprintln!("Error connecting to Arduino_8088 server: {e}");
            std::process::exit(1);
        }
    };

    if args.nmi_on.is_some() && args.single_step {
        eprintln!("Cannot use NMI with single step mode!");
        std::process::exit(1);
    }

    let nmi_on = if let Some(nmi_cycle) = args.nmi_on {
        nmi_cycle
    } else {
        if args.single_step {
            // If single step mode is enabled, we can use NMI on cycle 1.
            1
        } else {
            // Otherwise, we don't use NMI.
            0
        }
    };

    // Create a remote cpu instance using the cpu_client which should now be connected.
    let mut cpu = RemoteCpu::new(
        cpu_client,
        args.prefetch,
        args.emu8080,
        args.wait_states,
        args.intr_on,
        args.intr_after,
        nmi_on,
    );

    let cpu_type = cpu.cpu_type();
    log::debug!("Detected CPU type: {:?}", cpu_type);

    // Capture initial regs before adjustment.
    let initial_regs = RemoteCpuRegisters::from(reg_bytes.as_slice());

    // Copy the binary to memory
    log::debug!("Mounting program code at: {:05X}", mount_addr);
    cpu.mount_bin(&bin_bytes, mount_addr as usize);

    // Set up IVR table
    cpu.setup_ivt();

    // Load the registers from binary file
    let result = cpu.load_registers_from_buf(&reg_bytes);
    if result {
        log::trace!("Successfully set up registers!");

        println!("Initial register state:");

        RemoteCpu::print_regs(&initial_regs, cpu_type);

        let print_opts = PrintOptions {
            print_pgm: true,
            print_preload: false,
            print_finalize: false,
        };

        //cpu.test();
        match cpu.run(Some(10_000), &print_opts) {
            Ok(regs) => {
                println!("Final register state:");
                RemoteCpu::print_regs_delta(&initial_regs, &regs, cpu_type);
            }
            Err(_) => {
                log::error!("Program execution failed!");
            }
        }
    } else {
        log::error!("Register setup failed: {}", cpu.get_last_error());
    }
}
