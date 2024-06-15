#![no_std]
#![no_main]

mod dds;
mod system_config;

use bsp::entry;
use dds::Dds;
use defmt::info;
use defmt_rtt as _;
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use panic_probe as _;
use rp_pico as bsp;
use system_config::{SystemConfig, Uart1Reader, Uart1Writer};
use ufmt::uwrite;

#[derive(Command)]
enum Base<'a> {
    /// Set or read frequency. Defaults to 100Hz. It preserves the last set
    /// signal function.
    Frequency {
        frequency: Option<u32>,
    },
    /// Set or read volume. Defaults to 50%.
    Volume {
        volume: Option<u8>,
    },
    //// Set output to sine wave with given frequency or keep existing frequency.
    Sinusoidal {
        frequency: Option<u32>,
    },
    //// Set output to a triangle wave with given frequency or keep existing frequency.
    Triangle {
        frequency: Option<u32>,
    },
    //// Set output to a square wave with given frequency or keep existing frequency.
    Square {
        frequency: Option<u32>,
    },
    //// Set output to a square wave with half of the given frequency or half of
    ///the existing frequency.
    SquareHalf {
        frequency: Option<u32>,
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
                    Base::Frequency { frequency } => match frequency {
                        Some(frequency) => {
                            dds.set_frequency(frequency);
                        }
                        None => {
                            uwrite!(
                                cli.writer(),
                                "Current frequency is {} Hz",
                                dds.current_frequency_hz()
                            )?;
                        }
                    },
                    Base::Hello { name } => {
                        uwrite!(cli.writer(), "Hello, {}", name.unwrap_or("World"))?;
                    }
                    Base::Exit => {
                        cli.writer().write_str("Cli can't shutdown now")?;
                    }
                    Base::Sinusoidal { frequency } => match frequency {
                        Some(frequency) => {
                            dds.set_frequency(frequency);
                            dds.sine();
                        }
                        None => {
                            dds.sine();
                        }
                    },
                    Base::Triangle { frequency } => match frequency {
                        Some(frequency) => {
                            dds.set_frequency(frequency);
                            dds.triangle();
                        }
                        None => {
                            dds.triangle();
                        }
                    },
                    Base::Square { frequency } => match frequency {
                        Some(frequency) => {
                            dds.set_frequency(frequency);
                            dds.square();
                        }
                        None => {
                            dds.square();
                        }
                    },
                    Base::SquareHalf { frequency } => match frequency {
                        Some(frequency) => {
                            dds.set_frequency(frequency);
                            dds.half_square();
                        }
                        None => {
                            dds.half_square();
                        }
                    },
                    Base::Volume { volume } => match volume {
                        Some(volume) => {
                            dds.set_volume(volume);
                        }
                        None => {
                            dds.set_volume(50);
                        }
                    },
                }
                Ok(())
            }),
        );
    }
}
