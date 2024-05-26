#![no_std]
#![no_main]

use ad983x::{Ad983x, FrequencyRegister};
use bsp::{
    entry,
    hal::{
        gpio::{
            bank0::{Gpio0, Gpio1},
            FunctionUart, Pin, PullDown,
        },
        uart::{DataBits, Enabled, StopBits, UartConfig, UartPeripheral},
        Timer,
    },
    pac::UART0,
};
use core::{cell::RefCell, fmt::Write};
use defmt::*;
use defmt_rtt as _;
use fugit::RateExtU32;
use ushell::{autocomplete::StaticAutocomplete, history::LRUHistory, Input, ShellError, UShell};
// use mcp4x::{Channel, Mcp4x};
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::FunctionSpi,
    pac,
    sio::Sio,
    Spi, Watchdog,
};
use embedded_hal::spi::MODE_2;
use embedded_hal_bus::spi::RefCellDevice;

type Serial = UartPeripheral<
    Enabled,
    UART0,
    (
        Pin<Gpio0, FunctionUart, PullDown>,
        Pin<Gpio1, FunctionUart, PullDown>,
    ),
>;
type Shell = UShell<Serial, StaticAutocomplete<6>, LRUHistory<32, 4>, 32>;

#[entry]
fn main() -> ! {
    let sys_mckl: fugit::Rate<u32, 1, 1> = 125_000_000u32.Hz();
    let bmc_mckl: fugit::Rate<u32, 1, 1> = 25_000_000u32.Hz();
    let spi_baud_rate: fugit::Rate<u32, 1, 1> = 32_000u32.Hz();

    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();

    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    // SIO(single cycle IO)
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sclk = pins.gpio2.into_function::<FunctionSpi>();
    let mosi = pins.gpio3.into_function::<FunctionSpi>();
    // let pot_cs_pin = pins.gpio17.into_push_pull_output();
    let dds_cs_pin = pins.gpio15.into_push_pull_output();

    let spi_device = pac.SPI0;
    let spi_pin_layout = (mosi, sclk);

    let spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
        &mut pac.RESETS,
        sys_mckl,
        spi_baud_rate,
        MODE_2,
    );
    let shared_spi_bus = RefCell::new(spi);

    let dds_spi_device = RefCellDevice::new(&shared_spi_bus, dds_cs_pin, timer).unwrap();
    let mut dds = Ad983x::new_ad9833(dds_spi_device);

    // mcp41x.set_position(Channel::Ch0, 50).unwrap();

    // FREQREG = f * 2^28 / fMCLK
    info!(
        "The calculated frequency is: {}",
        440 * 2u64.pow(28) / (bmc_mckl.to_Hz() as u64)
    );
    let f_out: fugit::Rate<u32, 1, 1> = 440u32.Hz();
    let freqreg0: u32 = (f_out.to_Hz() as u64 * 2u64.pow(28) / (bmc_mckl.to_Hz() as u64))
        .try_into()
        .unwrap();
    let freqreg1: u32 = (f_out.to_Hz() as u64 * 2u64.pow(28) / (bmc_mckl.to_Hz() as u64))
        .try_into()
        .unwrap();

    dds.reset().unwrap();

    dds.set_frequency(FrequencyRegister::F0, freqreg0).unwrap();
    dds.set_frequency(FrequencyRegister::F1, freqreg1).unwrap();

    dds.select_frequency(FrequencyRegister::F0).unwrap();
    dds.set_output_waveform(ad983x::OutputWaveform::Sinusoidal)
        .unwrap();

    dds.enable().unwrap();

    // Given a 25 MHz clock, this now outputs a sine wave
    // with a frequency of 440 Hz, which is a standard
    // A4 tone.

    // Get device back
    // let _dev = dds.destroy();

    // Set up UART0
    // Set up UART on GP0 and GP1 (Pico pins 1 and 2)
    let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());

    // Need to perform clock init before using UART or it will freeze.
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    // uart.write_full_blocking(b"Hello World!\r\n");

    let autocomplete = StaticAutocomplete(["clear", "help", "off", "on", "set ", "status"]);
    let history: LRUHistory<32, 4> = LRUHistory::default();
    let mut shell: Shell = UShell::new(uart, autocomplete, history);

    shell.clear().ok();

    loop {
        let poll_value = shell.poll();
        match poll_value {
            Ok(Some(Input::Command((cmd, args)))) => {
                shell.write_str(cmd).ok();
            }
            //Err(ShellError::WouldBlock) => break
            _ => {}
        }
        delay.delay_ms(500);
    }
}

// End of file
