use ad983x::FrequencyRegister;
use fugit::Rate;

use crate::system_config::DdsDevice;

pub struct Dds<'a> {
    dds_device: DdsDevice<'a>,
    bmc_mckl: Rate<u32, 1, 1>,
    current_internal_frequency: u32,
    current_register: FrequencyRegister,
    current_output_waveform: ad983x::OutputWaveform,
}

impl<'a> Dds<'a> {
    pub fn new(dds_device: DdsDevice<'a>, bmc_mckl: Rate<u32, 1, 1>) -> Dds<'a> {
        let mut dds = Dds {
            dds_device,
            bmc_mckl,
            current_internal_frequency: 0u32,
            current_register: FrequencyRegister::F0,
            current_output_waveform: ad983x::OutputWaveform::Sinusoidal,
        };
        dds.reset();
        dds
    }

    fn reset(&mut self) -> () {
        self.current_internal_frequency = self.to_internal_frequency(100u32);
        self.dds_device.reset().unwrap();

        self.dds_device
            .set_frequency(self.current_register, self.current_internal_frequency)
            .unwrap();

        self.dds_device
            .set_frequency(FrequencyRegister::F0, self.current_internal_frequency)
            .unwrap();
        self.dds_device
            .set_frequency(FrequencyRegister::F1, self.current_internal_frequency)
            .unwrap();

        self.dds_device
            .select_frequency(self.current_register)
            .unwrap();
        self.dds_device
            .set_output_waveform(self.current_output_waveform)
            .unwrap();
        self.dds_device.enable().unwrap();
    }

    fn to_internal_frequency(&self, frequency_hz: u32) -> u32 {
        (frequency_hz as u64 * 2u64.pow(28) / (self.bmc_mckl.to_Hz() as u64))
            .try_into()
            .unwrap()
    }

    // fn swap_freq_registers(&mut self) -> () {
    //     self.current_register = match self.current_register {
    //         FrequencyRegister::F0 => FrequencyRegister::F1,
    //         FrequencyRegister::F1 => FrequencyRegister::F0,
    //     };
    // }

    pub fn current_frequency_hz(&self) -> u32 {
        (((self.current_internal_frequency as u64) * (self.bmc_mckl.to_Hz() as u64)) / 2u64.pow(28))
            as u32
    }

    pub fn set_frequency(&mut self, requested_frequency_hz: u32) -> () {
        self.current_internal_frequency = self.to_internal_frequency(requested_frequency_hz);
        self.dds_device
            .set_frequency(FrequencyRegister::F0, self.current_internal_frequency)
            .unwrap();
    }

    pub fn sine(&mut self) -> () {
        self.current_output_waveform = ad983x::OutputWaveform::Sinusoidal;
        self.dds_device
            .set_output_waveform(self.current_output_waveform)
            .unwrap();
    }

    pub fn triangle(&mut self) -> () {
        self.current_output_waveform = ad983x::OutputWaveform::Triangle;
        self.dds_device
            .set_output_waveform(self.current_output_waveform)
            .unwrap();
    }

    pub fn square(&mut self) -> () {
        self.current_output_waveform = ad983x::OutputWaveform::SquareMsbOfDac;
        self.dds_device
            .set_output_waveform(self.current_output_waveform)
            .unwrap();
    }

    pub fn half_square(&mut self) -> () {
        self.current_output_waveform = ad983x::OutputWaveform::SquareMsbOfDacDiv2;
        self.dds_device
            .set_output_waveform(self.current_output_waveform)
            .unwrap();
    }
}
