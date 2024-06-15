#![no_std]
#![no_main]

mod system_config;

use ad983x::FrequencyRegister;
use bsp::entry;
use defmt::info;
use defmt_rtt as _;
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use panic_probe as _;
use rp_pico as bsp;
use system_config::{Dds, SystemConfig, Uart1Reader, Uart1Writer};
use ufmt::uwrite;

#[derive(Command)]
enum Base<'a> {
    /// Set frequency for channel 0 or 1. Defaults to 100Hz on channel 0
    SetFrequency {
        frequency: Option<u32>,
        channel: Option<u8>,
    },
    /// Say hello to World or someone else
    Hello {
        /// To whom to say hello (World by default)
        name: Option<&'a str>,
    },

    /// Stop CLI and exit
    Exit,
}

#[entry]
fn main() -> ! {
    info!("Program starting ...");
    let mut system_config = SystemConfig::take().unwrap();

    info!(
        "external_xtal_freq_hz is {0} MHz",
        system_config.external_xtal_freq_hz.to_MHz()
    );

    let spi0_bus = system_config.spi0_bus.configure();
    // let mut pot = spi0_bus.pot;
    let mut dds: Dds = spi0_bus.dds;

    let writer: Uart1Writer = system_config.writer;
    let reader: Uart1Reader = system_config.reader;

    writer.write_full_blocking(b"Hello World - via serial!\r\n");

    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 32] = [0; 32];
        static mut HISTORY_BUFFER: [u8; 32] = [0; 32];
        (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
    };

    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build()
        .unwrap();

    info!("Program running ...");
    loop {
        let mut uart_byte: [u8; 1] = ['#' as u8; 1];
        reader.read_full_blocking(&mut uart_byte).unwrap();
        let _ = cli.process_byte::<Base, _>(
            uart_byte[0],
            &mut Base::processor(|cli, command| {
                match command {
                    Base::SetFrequency { frequency, channel } => {
                        let requested_frequency = frequency.unwrap_or(100);
                        let requested_channel = channel.unwrap_or(0);

                        let freqreg_val: u32 = (requested_frequency as u64 * 2u64.pow(28)
                            / (system_config.bmc_mckl.to_Hz() as u64))
                            .try_into()
                            .unwrap();
                        let freq_register = match requested_channel {
                            0 => FrequencyRegister::F0,
                            _ => FrequencyRegister::F1,
                        };

                        let s = match freq_register {
                            FrequencyRegister::F0 => "F0",
                            _ => "F1",
                        };

                        uwrite!(cli.writer(), "Writing {} to register {}", freqreg_val, s)?;
                        dds.set_frequency(freq_register, freqreg_val).unwrap();
                    }
                    Base::Hello { name } => {
                        // last write in command callback may or may not
                        // end with newline. so both uwrite!() and uwriteln!()
                        // will give identical results
                        uwrite!(cli.writer(), "Hello, {}", name.unwrap_or("World"))?;
                    }
                    Base::Exit => {
                        // We can write via normal function if formatting not needed
                        cli.writer().write_str("Cli can't shutdown now")?;
                    }
                }
                Ok(())
            }),
        );
    }
}
