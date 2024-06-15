use core::cell::RefCell;

use ad983x::{marker::Ad9833Ad9837, Ad983x};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::FunctionSpi,
    pac,
    sio::Sio,
    uart::{DataBits, Reader, StopBits, UartConfig, UartPeripheral, Writer},
    Spi, Timer, Watchdog,
};
use embedded_hal::spi::MODE_2;
use embedded_hal_bus::spi::RefCellDevice;
use fugit::{Rate, RateExtU32};
use rp_pico as bsp;

use crate::dds::Dds;

type Uart1RxPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio4,
    bsp::hal::gpio::FunctionUart,
    bsp::hal::gpio::PullDown,
>;
type Uart1TxPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio5,
    bsp::hal::gpio::FunctionUart,
    bsp::hal::gpio::PullDown,
>;

type Uart1 = UartPeripheral<bsp::hal::uart::Enabled, pac::UART1, (Uart1RxPin, Uart1TxPin)>;
pub type Uart1Reader = Reader<pac::UART1, (Uart1RxPin, Uart1TxPin)>;
pub type Uart1Writer = Writer<pac::UART1, (Uart1RxPin, Uart1TxPin)>;

type Spi0MosiPin =
    bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio3, FunctionSpi, bsp::hal::gpio::PullDown>;

type Spi0sclkPin =
    bsp::hal::gpio::Pin<bsp::hal::gpio::bank0::Gpio2, FunctionSpi, bsp::hal::gpio::PullDown>;

type Spi0 = Spi<bsp::hal::spi::Enabled, pac::SPI0, (Spi0MosiPin, Spi0sclkPin)>;

type SharedSpi0Bus = RefCell<Spi0>;

type PotCsPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio17,
    bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioOutput>,
    bsp::hal::gpio::PullDown,
>;

type DdsCsPin = bsp::hal::gpio::Pin<
    bsp::hal::gpio::bank0::Gpio15,
    bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioOutput>,
    bsp::hal::gpio::PullDown,
>;

pub type DdsDevice<'a> = Ad983x<
    RefCellDevice<
        'a,
        Spi<
            rp_pico::hal::spi::Enabled,
            pac::SPI0,
            (
                rp_pico::hal::gpio::Pin<
                    rp_pico::hal::gpio::bank0::Gpio3,
                    FunctionSpi,
                    rp_pico::hal::gpio::PullDown,
                >,
                rp_pico::hal::gpio::Pin<
                    rp_pico::hal::gpio::bank0::Gpio2,
                    FunctionSpi,
                    rp_pico::hal::gpio::PullDown,
                >,
            ),
        >,
        rp_pico::hal::gpio::Pin<
            rp_pico::hal::gpio::bank0::Gpio15,
            rp_pico::hal::gpio::FunctionSio<rp_pico::hal::gpio::SioOutput>,
            rp_pico::hal::gpio::PullDown,
        >,
        Timer,
    >,
    Ad9833Ad9837,
>;

pub struct SystemConfig {
    pub sys_mckl: fugit::Rate<u32, 1, 1>,
    pub bmc_mckl: fugit::Rate<u32, 1, 1>,
    pub spi_baud_rate: fugit::Rate<u32, 1, 1>,
    pub external_xtal_freq_hz: fugit::Rate<u32, 1, 1>,
    pub reader: Uart1Reader,
    pub writer: Uart1Writer,
    pub spi0_bus: Spi0Bus,
}

pub struct Spi0Bus {
    bmc_mckl: Rate<u32, 1, 1>,
    shared_spi_bus: SharedSpi0Bus,
    timer: Option<Timer>,
    pot_cs_pin: Option<PotCsPin>,
    dds_cs_pin: Option<DdsCsPin>,
} //

impl SystemConfig {
    pub fn take() -> Option<SystemConfig> {
        let sys_mckl: fugit::Rate<u32, 1, 1> = 125u32.MHz();
        let bmc_mckl: fugit::Rate<u32, 1, 1> = 25u32.MHz();
        let spi_baud_rate: fugit::Rate<u32, 1, 1> = 32u32.kHz();
        // External high-speed crystal on the pico board is 12Mhz
        let external_xtal_freq_hz = 12u32.MHz();

        let mut pac = pac::Peripherals::take().unwrap();
        let sio = Sio::new(pac.SIO);
        let pins = bsp::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let clocks = init_clocks_and_plls(
            external_xtal_freq_hz.to_Hz(),
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let uart_pins = (pins.gpio4.into_function(), pins.gpio5.into_function());
        let uart: Uart1 = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        let (reader, writer) = uart.split();

        // ------------- Timer --------------
        let timer: Timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

        // ------------- SPI ----------------

        let sclk = pins.gpio2.into_function::<FunctionSpi>();
        let mosi = pins.gpio3.into_function::<FunctionSpi>();

        let spi_device = pac.SPI0;
        let spi_pin_layout = (mosi, sclk);

        let spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
            &mut pac.RESETS,
            sys_mckl,
            spi_baud_rate,
            MODE_2,
        );
        let shared_spi_bus: RefCell<Spi0> = RefCell::new(spi);

        let pot_cs_pin: PotCsPin = pins.gpio17.into_push_pull_output();
        let dds_cs_pin: DdsCsPin = pins.gpio15.into_push_pull_output();
        let spi0_bus: Spi0Bus = Spi0Bus {
            bmc_mckl,
            shared_spi_bus,
            timer: Some(timer),
            pot_cs_pin: Some(pot_cs_pin),
            dds_cs_pin: Some(dds_cs_pin),
        };

        let instance = SystemConfig {
            sys_mckl,
            bmc_mckl,
            spi_baud_rate,
            external_xtal_freq_hz,
            reader,
            writer,
            spi0_bus,
        };
        Some(instance)
    }
}

pub struct ConfiguredSpi0Bus<'a> {
    pub dds: Dds<'a>,
    // pub dds: DdsDevice<'a>,
}

impl<'a> Spi0Bus {
    pub fn configure(&'a mut self) -> ConfiguredSpi0Bus<'a> {
        let dds_cs_pin = self.dds_cs_pin.take().unwrap();
        let timer = self.timer.take().unwrap();

        let dds_spi_device = RefCellDevice::new(&self.shared_spi_bus, dds_cs_pin, timer).unwrap();
        let dds_device: DdsDevice<'a> = Ad983x::new_ad9833(dds_spi_device);
        let dds = Dds::new(dds_device, self.bmc_mckl);

        ConfiguredSpi0Bus { dds }
    }
}
