#![no_std]
#![no_main]

use ad983x::{Ad983x, FrequencyRegister};
use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::spi::MODE_0;
use fugit::RateExtU32;
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::FunctionSpi,
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Spi,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sclk = pins.gpio2.into_function::<FunctionSpi>();
    let mosi = pins.gpio3.into_function::<FunctionSpi>();
    let cs_pin = pins.gpio15.into_push_pull_output();

    let spi_device = pac.SPI0;
    let spi_pin_layout = (mosi, sclk);

    let spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        32_000u32.Hz(),
        MODE_0,
    );

    let mut dds = Ad983x::new_ad9833(spi, cs_pin);

    dds.reset().unwrap();
    dds.set_frequency(FrequencyRegister::F0, 4724).unwrap();
    dds.set_frequency(FrequencyRegister::F1, 4724).unwrap();

    dds.set_output_waveform(ad983x::OutputWaveform::Sinusoidal)
        .unwrap();

    dds.enable().unwrap();

    dds.select_frequency(FrequencyRegister::F0).unwrap();

    // Given a 25 MHz clock, this now outputs a sine wave
    // with a frequency of 440 Hz, which is a standard
    // A4 tone.

    // Get device back
    // let _dev = dds.destroy();

    loop {
        delay.delay_ms(500);
    }
}

// End of file
