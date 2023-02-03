//! A platform agnostic Rust driver for the MCP346[1/2/4], based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!
//! ## Usage
//!
//! ```no_run
//! use embedded_hal::spi::MODE_0;
//! use stm32f3_discovery::stm32f3xx_hal as hal;
//! use hal::spi::Spi;
//! use mcp346x::MCP346x;
//!
//! # fn main() {
//! 
//! // Initialize gpio pins, clocks, etc.
//! 
//! let spi_pins = (sck, miso, mosi);
//! let spi = Spi::new(p.SPI0, spi_pins, Frequency::K500, MODE_0);
//! 
//! let address = 0b01; // This is the default address for most chips
//! let mut adc = MCP346x::new(spi, address).into_continuous_mode().unwrap();
//! 
//! adc.set_clock_source(mcp346x::ClockSource::Internal).unwrap();
//! adc.set_irq_internal_pullup(true).unwrap();
//! 
//! let voltage = adc.measure().unwrap();
//! # }
//! ```

#![no_std]

use bitflags::bitflags;
use embedded_hal as hal;
use hal::spi::blocking::Transfer;
use hal::digital::blocking::OutputPin;
use measurements::Voltage;

#[derive(Debug)]
pub enum Error<E, PE> {
    Spi(E),
    Pin(PE),
    VoltageTooHigh,
    VoltageTooLow,
    NotInitialized,
    NotReady,
}

pub enum ClockSource {
    Internal = 1,
    External = 0
}

pub enum Input {
    VInPlus = 4,
    VInMinus = 0
}

pub enum Channel {
    CH0 = 0b0000,
    CH1 = 0b0001,
    CH2 = 0b0010,
    CH3 = 0b0011,
    CH4 = 0b0100,
    CH5 = 0b0101,
    CH6 = 0b0110,
    CH7 = 0b0111,
    AGND = 0b1000,
    AVDD = 0b1001,
    REFINPlus = 0b1011,
    REFINMinus = 0b1100,
    TEMPDiodeP = 0b1101,
    TEMPDiodeM = 0b1110,
    VCM = 0b1111
}

bitflags! {
    pub struct StatusRegister: u8 {
        const DR = 0b00000100;
        const CRCCFG = 0b00000010;
        const POR = 0b00000001;
    }
}

pub struct MCP346x<SPI, CS, MODE> {
    spi: SPI,
    cs: CS,
    _mode: MODE,
    address: u8,
}

fn generate_fast_command_byte(device_address: u8, command: u8) -> [u8; 1] {
    return [(device_address << 6) | (command << 2) | 0b00000000; 1];
}

fn generate_register_command_byte(
    device_address: u8,
    register_address: u8,
    command_type: u8,
) -> [u8; 1] {
    return [(device_address << 6) | (register_address << 2) | command_type; 1];
}

impl<SPI, CS, E> MCP346x<SPI, CS, Unconfigured>
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS, address: u8) -> Self {
        Self {
            spi,
            cs,
            _mode: Unconfigured,
            address,
        }
    }
}

impl<SPI, CS, MODE, E, PE> MCP346x<SPI, CS, MODE>
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin<Error = PE>,
{
    fn spi_write(&mut self, write_buffer: &[u8]) -> Result<(), Error<E, PE>> {
        self.cs.set_low().map_err(Error::Pin)?;
        let mut transfer_buffer = [0; 40];
        let mut transfer_slice = &mut transfer_buffer[0..write_buffer.len()];
        transfer_slice.copy_from_slice(write_buffer);
        self.spi.transfer(&mut transfer_slice).map_err( Error::Spi)?;
        self.cs.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    fn spi_transfer(&mut self, read_buffer: &mut [u8], write_buffer: &[u8]) -> Result<(), Error<E, PE>> {
        self.cs.set_low().map_err(Error::Pin)?;
        let mut transfer_buffer = [0; 40];
        let transfer_slice = &mut transfer_buffer[0..write_buffer.len()];
        transfer_slice.copy_from_slice(write_buffer);
        self.spi
            .transfer(&mut transfer_buffer[0..read_buffer.len().max(write_buffer.len())]).map_err(Error::Spi)?;
        read_buffer.copy_from_slice(&transfer_buffer[0..read_buffer.len()]);
        self.cs.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    fn tranfer_with_status_register(
        &mut self,
        read_buf: &mut [u8],
        command: &[u8],
    ) -> Result<StatusRegister, Error<E, PE>> {
        let mut buf = [0; 40];
        self.spi_transfer(&mut buf[0..read_buf.len() + 1], command)?;
        let mut status_register_raw = [0; 1];
        status_register_raw.clone_from_slice(&buf[0..1]);
        let status_register =
            StatusRegister::from_bits_truncate(u8::from_be_bytes(status_register_raw));
        read_buf.copy_from_slice(&buf[1..read_buf.len() + 1]);
        Ok(status_register)
    }

    fn set_mode(&mut self, mode: u8) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = [mode; 1];
        self.spi_write(&command)?;
        Ok(())
    }

    fn static_read(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_register_command_byte(self.address, register, 0b01);
        let _ = self.tranfer_with_status_register(buf, &command);
        Ok(())
    }

    fn incremental_read(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_register_command_byte(self.address, register, 0b11);
        self.spi_transfer(buf, &command)?;
        Ok(())
    }

    fn incremental_write(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_register_command_byte(self.address, register, 0b10);
        let mut buffer: [u8; 10] = [0; 10];
        let _ = &mut buffer[0..1].copy_from_slice(&command);
        let _ = &mut buffer[1..buf.len()+1].copy_from_slice(buf);
        self.spi_write(&buffer[0..buf.len()+1])?;
        Ok(())
    }

    fn read_data_register_16bit(&mut self) -> Result<i16, Error<E, PE>> {
        let mut buf: [u8; 2] = [0; 2];
        self.static_read(0x00, &mut buf)?;
        let measurement = i16::from_be_bytes(buf);
        Ok(measurement)
    }

    pub fn into_continuous_mode(mut self) -> Result<MCP346x<SPI, CS, ContinuousMode>, Error<E, PE>> {
        self.set_mode(0b00)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            _mode: ContinuousMode,
            address: self.address,
        })
    }

    pub fn into_scan_mode(mut self) -> Result<MCP346x<SPI, CS, ScanMode>, Error<E, PE>> {
        self.set_mode(0b10)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            _mode: ScanMode,
            address: self.address,
        })
    }

    pub fn into_oneshot_mode(mut self) -> Result<MCP346x<SPI, CS, OneShotMode>, Error<E, PE>> {
        self.set_mode(0b11)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            _mode: OneShotMode,
            address: self.address,
        })
    }


    /// Sets the clock source.
    /// MCP346x can be used with an internal clock or an external
    /// one connected to Pin 18.
    pub fn set_clock_source(&mut self, clock_source: ClockSource) -> Result<(), Error<E, PE>> {
        let mut buf: [u8; 1] = [0; 1];
        self.static_read(0x01, &mut buf)?;
        buf[0] = ((clock_source as u8) << 4) | (!(0b11 << 4) & buf[0]);
        self.incremental_write(0x01, &mut buf)?;
        Ok(())
    }

    /// Sets the irq internal pullup.
    /// This is nessecary if the irq pin (Pin 17) is not 
    /// connected and no external pullup is used for this pin.
    pub fn set_irq_internal_pullup(&mut self, pullup: bool) -> Result<(), Error<E, PE>> {
        let mut buf = [0; 1];
        self.static_read(0x05, &mut buf)?;
        buf[0] = ((pullup as u8) << 3) | (!(0b1 << 3) & buf[0]);
        self.incremental_write(0x5, &mut buf)?;
        Ok(())
    }

    
    /// Sets the mux input for one of the two Inputs.
    /// 
    /// # Arguments
    /// * `input` one of the two Inputs VIn+ or VIn-
    /// * `channel` the channel to be connected to the Input
    ///
    pub fn set_mux_input(&mut self, input: Input, channel: Channel) -> Result<(), Error<E, PE>> {
        let mut buf = [0; 1];
        self.static_read(0x06, &mut buf)?;
        let shift = input as u8;
        buf[0] = ((channel as u8) << shift) | ((0b1111 << shift) & buf[0]);
        self.incremental_write(0x06, &mut buf)?;
        Ok(())
    }

    fn int_measure(&mut self) -> Result<Voltage, Error<E, PE>> {
        let measurement = self.read_data_register_16bit()?;
        Ok(Voltage::from_volts(measurement as f64))
    }

    fn int_start_conversion(&mut self) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1010);
        self.spi_write(&command)?;
        Ok(())
    }

    pub fn standby(&mut self) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1011);
        self.spi_write(&command)?;
        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1100);
        self.spi_write(&command)?;
        Ok(())
    }

    pub fn full_shutdown(mut self) -> Result<(), Error<E, PE>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1101);
        self.spi_write(&command)?;
        Ok(())
    }

    pub fn reset(mut self) -> Result<MCP346x<SPI, CS, Unconfigured>, Error<E, PE>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1110);
        self.spi_write(&command)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            _mode: Unconfigured,
            address: self.address,
        })
    }
}

impl<SPI, CS, E, PE> MCP346x<SPI, CS, ContinuousMode> 
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin<Error = PE>,
{

    pub fn measure(&mut self) -> Result<Voltage, Error<E, PE>> {
        self.int_measure()
    }

    pub fn start_conversion(&mut self) -> Result<(), Error<E, PE>> {
        self.int_start_conversion()
    }

}

impl<SPI, CS, E, PE> MCP346x<SPI, CS, OneShotMode> 
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin<Error = PE>,
{

    pub fn measure(&mut self) -> Result<Voltage, Error<E, PE>> {
        self.start_conversion()?;
        self.int_measure()
    }

    pub fn start_conversion(&mut self) -> Result<(), Error<E, PE>> {
        self.int_start_conversion()
    }

}

impl<SPI, CS, E, PE> MCP346x<SPI, CS, ScanMode> 
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin<Error = PE>,
{

    pub fn measure(&mut self) -> Result<Voltage, Error<E, PE>> {
        self.int_measure()
    }

    pub fn start_conversion(&mut self) -> Result<(), Error<E, PE>> {
        self.int_start_conversion()
    }

}


pub struct ContinuousMode;
pub struct OneShotMode;
pub struct ScanMode;
pub struct Unconfigured;
