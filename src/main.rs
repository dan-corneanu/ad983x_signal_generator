#![no_std]
#![no_main]

mod system_config;

use ad983x::{Ad983x, FrequencyRegister};
use bsp::entry;
use defmt::info;
use defmt_rtt as _;
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use embedded_hal_bus::spi::RefCellDevice;
use panic_probe as _;
use rp_pico as bsp;
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
    let system_config = system_config::SystemConfig::take().unwrap();

    info!(
        "external_xtal_freq_hz is {0} MHz",
        system_config.external_xtal_freq_hz.to_MHz()
    );

    let shared_spi_bus = system_config.shared_spi_bus;
    let dds_cs_pin = system_config.dds_cs_pin;
    let timer = system_config.timer;

    let dds_spi_device = RefCellDevice::new(&shared_spi_bus, dds_cs_pin, timer).unwrap();
    let mut dds = Ad983x::new_ad9833(dds_spi_device);
    dds.reset().unwrap();

    let writer = system_config.writer;
    let reader = system_config.reader;

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

// use ad983x::{Ad983x, FrequencyRegister};
// use bsp::{
//     entry,
//     hal::{
//         uart::{DataBits, StopBits, UartConfig, UartPeripheral},
//         Timer,
//     },
// };
// use core::cell::RefCell;
// use defmt::info;
// use defmt_rtt as _;
// use embedded_cli::cli::CliBuilder;
// use fugit::RateExtU32;
// use ufmt::uwrite;
// // use mcp4x::{Channel, Mcp4x};
// use panic_probe as _;
// use rp_pico as bsp;

// use bsp::hal::{
//     clocks::{init_clocks_and_plls, Clock},
//     gpio::FunctionSpi,
//     pac,
//     sio::Sio,
//     Spi, Watchdog,
// };
// use embedded_hal::spi::MODE_2;
// use embedded_hal_bus::spi::RefCellDevice;

// use embedded_cli::Command;

// #[derive(Command)]
// enum Base<'a> {
//     /// Set frequency for channel 0 or 1. Defaults to 100Hz on channel 0
//     SetFrequency {
//         frequency: Option<u32>,
//         channel: Option<u8>,
//     },
//     /// Say hello to World or someone else
//     Hello {
//         /// To whom to say hello (World by default)
//         name: Option<&'a str>,
//     },

//     /// Stop CLI and exit
//     Exit,
// }

// #[entry]
// fn main() -> ! {
//     let sys_mckl: fugit::Rate<u32, 1, 1> = 125_000_000u32.Hz();
//     let bmc_mckl: fugit::Rate<u32, 1, 1> = 25_000_000u32.Hz();
//     let spi_baud_rate: fugit::Rate<u32, 1, 1> = 32_000u32.Hz();

//     info!("Program start");
//     let mut pac = pac::Peripherals::take().unwrap();

//     let mut watchdog = Watchdog::new(pac.WATCHDOG);
//     // SIO(single cycle IO)
//     let sio = Sio::new(pac.SIO);

//     // External high-speed crystal on the pico board is 12Mhz
//     let external_xtal_freq_hz = 12_000_000u32;
//     let clocks = init_clocks_and_plls(
//         external_xtal_freq_hz,
//         pac.XOSC,
//         pac.CLOCKS,
//         pac.PLL_SYS,
//         pac.PLL_USB,
//         &mut pac.RESETS,
//         &mut watchdog,
//     )
//     .ok()
//     .unwrap();

//     let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

//     let pins = bsp::Pins::new(
//         pac.IO_BANK0,
//         pac.PADS_BANK0,
//         sio.gpio_bank0,
//         &mut pac.RESETS,
//     );

//     let sclk = pins.gpio2.into_function::<FunctionSpi>();
//     let mosi = pins.gpio3.into_function::<FunctionSpi>();
//     // let pot_cs_pin = pins.gpio17.into_push_pull_output();
//     let dds_cs_pin = pins.gpio15.into_push_pull_output();

//     let spi_device = pac.SPI0;
//     let spi_pin_layout = (mosi, sclk);

//     let spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
//         &mut pac.RESETS,
//         sys_mckl,
//         spi_baud_rate,
//         MODE_2,
//     );
//     let shared_spi_bus = RefCell::new(spi);

//     let dds_spi_device = RefCellDevice::new(&shared_spi_bus, dds_cs_pin, timer).unwrap();
//     let mut dds = Ad983x::new_ad9833(dds_spi_device);

//     // mcp41x.set_position(Channel::Ch0, 50).unwrap();

//     // FREQREG = f * 2^28 / fMCLK
//     info!(
//         "The calculated frequency is: {}",
//         440 * 2u64.pow(28) / (bmc_mckl.to_Hz() as u64)
//     );
//     let f_out: fugit::Rate<u32, 1, 1> = 440u32.Hz();
//     let freqreg0: u32 = (f_out.to_Hz() as u64 * 2u64.pow(28) / (bmc_mckl.to_Hz() as u64))
//         .try_into()
//         .unwrap();
//     let freqreg1: u32 = (f_out.to_Hz() as u64 * 2u64.pow(28) / (bmc_mckl.to_Hz() as u64))
//         .try_into()
//         .unwrap();

//     dds.reset().unwrap();

//     dds.set_frequency(FrequencyRegister::F0, freqreg0).unwrap();
//     dds.set_frequency(FrequencyRegister::F1, freqreg1).unwrap();

//     dds.select_frequency(FrequencyRegister::F0).unwrap();
//     dds.set_output_waveform(ad983x::OutputWaveform::Sinusoidal)
//         .unwrap();

//     dds.enable().unwrap();

//     // Given a 25 MHz clock, this now outputs a sine wave
//     // with a frequency of 440 Hz, which is a standard
//     // A4 tone.

//     // Get device back
//     // let _dev = dds.destroy();

//     // Set up UART0
//     // Set up UART on GP0 and GP1 (Pico pins 1 and 2)
//     let uart_pins = (pins.gpio4.into_function(), pins.gpio5.into_function());

//     // Need to perform clock init before using UART or it will freeze.
//     let uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
//         .enable(
//             UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
//             clocks.peripheral_clock.freq(),
//         )
//         .unwrap();

//     let (reader, writer) = uart.split();
//     writer.write_full_blocking(b"Hello World!\r\n");

//     let (command_buffer, history_buffer) = unsafe {
//         static mut COMMAND_BUFFER: [u8; 32] = [0; 32];
//         static mut HISTORY_BUFFER: [u8; 32] = [0; 32];
//         (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
//     };

//     let mut cli = CliBuilder::default()
//         .writer(writer)
//         .command_buffer(command_buffer)
//         .history_buffer(history_buffer)
//         .build()
//         .unwrap();

//     loop {
//         let mut uart_byte: [u8; 1] = ['#' as u8; 1];
//         reader.read_full_blocking(&mut uart_byte).unwrap();
//         let _ = cli.process_byte::<Base, _>(
//             uart_byte[0],
//             &mut Base::processor(|cli, command| {
//                 match command {
//                     Base::SetFrequency { frequency, channel } => {
//                         let requested_frequency = frequency.unwrap_or(100);
//                         let requested_channel = channel.unwrap_or(0);

//                         let freqreg_val: u32 = (requested_frequency as u64 * 2u64.pow(28)
//                             / (bmc_mckl.to_Hz() as u64))
//                             .try_into()
//                             .unwrap();
//                         let freq_register = match requested_channel {
//                             0 => FrequencyRegister::F0,
//                             _ => FrequencyRegister::F1,
//                         };
//                         uwrite!(
//                             cli.writer(),
//                             "Writing {} Hz, to channel {}",
//                             requested_frequency,
//                             requested_channel
//                         )?;
//                         dds.set_frequency(freq_register, freqreg_val).unwrap();
//                     }
//                     Base::Hello { name } => {
//                         // last write in command callback may or may not
//                         // end with newline. so both uwrite!() and uwriteln!()
//                         // will give identical results
//                         uwrite!(cli.writer(), "Hello, {}", name.unwrap_or("World"))?;
//                     }
//                     Base::Exit => {
//                         // We can write via normal function if formatting not needed
//                         cli.writer().write_str("Cli can't shutdown now")?;
//                     }
//                 }
//                 Ok(())
//             }),
//         );
//     }
// }

// // End of file
