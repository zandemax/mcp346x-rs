#![no_std]

use bitflags::bitflags;
use embedded_hal as hal;
use hal::blocking::spi::Transfer;
use hal::digital::v2::OutputPin;
use hal::spi::{Mode, Phase, Polarity};
use measurements::Voltage;

#[derive(Debug)]
pub enum Error<E> {
    Spi(E),
    VoltageTooHigh,
    VoltageTooLow,
    NotInitialized,
    NotReady,
}

bitflags! {
    pub struct StatusRegister: u8 {
        const DR_STATUS = 0b00000100;
        const CRCCFG_STATUS = 0b00000010;
        const POR_STATUS = 0b00000001;
    }
}

pub struct MCP346x<SPI, CS, MODE> {
    spi: SPI,
    cs: CS,
    mode: MODE,
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
            mode: Unconfigured,
            address,
        }
    }
}

impl<SPI, CS, MODE, E> MCP346x<SPI, CS, MODE>
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    fn spi_write(&mut self, write_buffer: &[u8]) -> Result<(), E> {
        self.cs.set_low();
        let mut transfer_buffer = [0; 40];
        let mut transfer_slice = &mut transfer_buffer[0..write_buffer.len()];
        transfer_slice.copy_from_slice(write_buffer);
        self.spi.transfer(&mut transfer_slice)?;
        self.cs.set_high();
        Ok(())
    }

    fn spi_transfer(&mut self, read_buffer: &mut [u8], write_buffer: &[u8]) -> Result<(), E> {
        self.cs.set_low();
        let mut transfer_buffer = [0; 40];
        let mut transfer_slice = &mut transfer_buffer[0..write_buffer.len()];
        transfer_slice.copy_from_slice(write_buffer);
        self.spi
            .transfer(&mut transfer_buffer[0..read_buffer.len().max(write_buffer.len())])?;
        read_buffer.copy_from_slice(&transfer_buffer[0..read_buffer.len()]);
        self.cs.set_high();
        Ok(())
    }

    fn tranfer_with_status_register(
        &mut self,
        read_buf: &mut [u8],
        command: &[u8],
    ) -> Result<StatusRegister, Error<E>> {
        let mut buf = [0; 40];
        self.spi_transfer(&mut buf[0..read_buf.len() + 1], command)
            .map_err(|e| Error::Spi(e))?;
        let mut status_register_raw = [0; 1];
        status_register_raw.clone_from_slice(&buf[0..1]);
        let status_register =
            StatusRegister::from_bits_truncate(u8::from_be_bytes(status_register_raw));
        read_buf.copy_from_slice(&buf[1..read_buf.len() + 1]);
        Ok(status_register)
    }

    fn set_mode(&mut self, mode: u8) -> Result<(), Error<E>> {
        let command: [u8; 1] = [0; 1];
        self.spi_write(&command).map_err(Error::Spi)?;
        Ok(())
    }

    fn static_read(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_register_command_byte(self.address, register, 0b01);
        let _ = self.tranfer_with_status_register(buf, &command);
        Ok(())
    }

    fn incremental_read(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_register_command_byte(self.address, register, 0b11);
        self.spi_transfer(buf, &command)
            .map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    fn incremental_write(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_register_command_byte(self.address, register, 0b10);
        self.spi_transfer(buf, &command)
            .map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    fn read_data_register_16bit(&mut self) -> Result<i16, Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.static_read(0x00, &mut buf)?;
        let measurement = i16::from_be_bytes(buf);
        Ok(measurement)
    }

    pub fn into_continuous_mode(mut self) -> Result<MCP346x<SPI, CS, ContinuousMode>, Error<E>> {
        self.set_mode(0b00)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            mode: ContinuousMode,
            address: self.address,
        })
    }

    pub fn into_scan_mode(mut self) -> Result<MCP346x<SPI, CS, ScanMode>, Error<E>> {
        self.set_mode(0b10)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            mode: ScanMode,
            address: self.address,
        })
    }

    pub fn into_oneshot_mode(mut self) -> Result<MCP346x<SPI, CS, OneShotMode>, Error<E>> {
        self.set_mode(0b11)?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            mode: OneShotMode,
            address: self.address,
        })
    }

    pub fn measure(&mut self) -> Result<Voltage, Error<E>> {
        self.start_conversion()?;
        let measurement = self.read_data_register_16bit()?;
        Ok(Voltage::from_volts(measurement as f64))
    }

    pub fn start_conversion(&mut self) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1010);
        self.spi_write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    pub fn standby(&mut self) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1011);
        self.spi_write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1100);
        self.spi_write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    pub fn full_shutdown(mut self) -> Result<(), Error<E>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1101);
        self.spi_write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    pub fn reset(mut self) -> Result<MCP346x<SPI, CS, Unconfigured>, Error<E>> {
        let command: [u8; 1] = generate_fast_command_byte(self.address, 0b1110);
        self.spi_write(&command).map_err(|e| Error::Spi(e))?;
        Ok(MCP346x {
            spi: self.spi,
            cs: self.cs,
            mode: Unconfigured,
            address: self.address,
        })
    }
}

pub struct ContinuousMode;
pub struct OneShotMode;
pub struct ScanMode;
pub struct Unconfigured;
