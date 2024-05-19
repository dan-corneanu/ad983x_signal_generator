#![no_std]
#![no_main]

// use ad983x::{Ad983x, FrequencyRegister};
// use bsp::{entry, hal::Timer};
// use core::cell::RefCell;
use defmt::{info, panic, unwrap};
use defmt_rtt as _;
// use fugit::RateExtU32;
// // use mcp4x::{Channel, Mcp4x};
use panic_probe as _;
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

use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn blinker(mut led: Output<'static, embassy_rp::peripherals::PIN_22>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let led: Output<embassy_rp::peripherals::PIN_22> = Output::new(p.PIN_22, Level::Low);
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(500))));

    // https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_serial.rs
    let usb_driver = Driver::new(p.USB, Irqs);
    // Create embassy-usb Config
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-serial example");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            usb_driver,
            config,
            DEVICE_DESCRIPTOR.init([0; 256]),
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    // Create classes on the builder.
    let mut class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    unwrap!(spawner.spawn(usb_task(usb)));

    // Do stuff with the class!
    loop {
        class.wait_connection().await;
        info!("Connected");
        let _ = echo(&mut class).await;
        info!("Disconnected");
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

// #[entry]
// fn main() -> ! {
//     let sys_mckl: fugit::Rate<u32, 1, 1> = 125_000_000u32.Hz();
//     let bmc_mckl: fugit::Rate<u32, 1, 1> = 25_000_000u32.Hz();
//     let spi_baud_rate: fugit::Rate<u32, 1, 1> = 32_000u32.Hz();

//     info!("Program start");
//     let mut pac = pac::Peripherals::take().unwrap();

//     let core = pac::CorePeripherals::take().unwrap();
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

//     let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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

//     loop {
//         delay.delay_ms(500);
//     }
// }

// End of file
