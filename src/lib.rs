#![no_std]

use embedded_hal::spi::{SpiDevice, SpiBus};
use measurements::Voltage;
use bitflags::bitflags;

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


pub struct MCP346x<SPI, MODE> {
    spi: SPI,
    mode: MODE,
    address: u8
}

fn generate_fast_command_byte(device_address: u8, command: u8) -> [u8;1] {
    return [(device_address << 6) | (command << 2) | 0b00000000;1];
}

fn generate_register_command_byte(device_address: u8, register_address: u8, command_type: u8) -> [u8;1] {
    return [(device_address << 6) | (register_address << 2) | command_type;1];
}


impl<SPI> MCP346x<SPI, Unconfigured>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus
{
    pub fn new(spi: SPI, address: u8) -> Self {
        Self { spi, mode: Unconfigured, address }
    }
}

impl<SPI, MODE, E> MCP346x<SPI, MODE> 
where
    SPI: SpiDevice<Error = E>,
    SPI::Bus: SpiBus,
{

    fn tranfer_with_status_register(&mut self, read_buf: &mut [u8], command: &[u8]) -> Result<StatusRegister, Error<E>> {
        let mut buf = [0;40];
        self.spi.transfer(&mut buf[0..read_buf.len()+8], command).map_err(|e| Error::Spi(e))?;
        let mut status_register_raw = [0;1];
        status_register_raw.clone_from_slice(&buf[0..8]);
        let status_register = StatusRegister::from_bits_truncate(u8::from_be_bytes(status_register_raw));
        read_buf.copy_from_slice(&buf[8..read_buf.len()]);
        Ok(status_register)
    } 

    fn set_mode(&mut self, mode: u8) -> Result<(), Error<E>> {
        let command: [u8;1] = [0;1];
        self.spi.write(&command).map_err(Error::Spi)?;
        Ok(())
    }

    fn static_read(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_register_command_byte(self.address, register, 0b01);
        let _ = self.tranfer_with_status_register(buf, &command);
        Ok(())
    }

    fn incremental_read(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_register_command_byte(self.address, register, 0b11);
        self.spi.transfer(buf, &command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    fn incremental_write(&mut self, register: u8, buf: &mut[u8]) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_register_command_byte(self.address, register, 0b10);
        self.spi.transfer(buf, &command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    fn read_data_register_16bit(&mut self) -> Result<i16, Error<E>> {
        let mut buf: [u8;2] = [0;2];
        self.static_read(0x00, &mut buf)?;
        let measurement = i16::from_be_bytes(buf);
        Ok(measurement)
    }

    pub fn into_continuous_mode(mut self) -> Result<MCP346x<SPI, ContinuousMode>, Error<E>> {
        self.set_mode(0b00)?;
        Ok(MCP346x {
            spi: self.spi,
            mode: ContinuousMode,
            address: self.address
        })
    }

    pub fn into_scan_mode(mut self) -> Result<MCP346x<SPI, ScanMode>, Error<E>> {
        self.set_mode(0b10)?;
        Ok(MCP346x {
            spi: self.spi,
            mode: ScanMode,
            address: self.address
        })
    }

    pub fn into_oneshot_mode(mut self) -> Result<MCP346x<SPI, OneShotMode>, Error<E>> {
        self.set_mode(0b11)?;
        Ok(MCP346x {
            spi: self.spi,
            mode: OneShotMode,
            address: self.address
        })
    }
    
    pub fn measure(&mut self) -> Result<Voltage, Error<E>> {
        self.start_conversion()?;
        let measurement = self.read_data_register_16bit()?;
        Ok(Voltage::from_volts(measurement as f64))
    }

    pub fn start_conversion(&mut self) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_fast_command_byte(self.address, 0b1010);
        self.spi.write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }
    
    pub fn standby(&mut self) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_fast_command_byte(self.address, 0b1011);
        self.spi.write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_fast_command_byte(self.address, 0b1100);
        self.spi.write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }
    
    pub fn full_shutdown(mut self) -> Result<(), Error<E>> {
        let command: [u8;1] = generate_fast_command_byte(self.address, 0b1101);
        self.spi.write(&command).map_err(|e| Error::Spi(e))?;
        Ok(())
    }
    
    pub fn reset(mut self) -> Result<MCP346x<SPI, Unconfigured>, Error<E>> {
        let command: [u8;1] = generate_fast_command_byte(self.address, 0b1110);
        self.spi.write(&command).map_err(|e| Error::Spi(e))?;
        Ok(MCP346x { spi: self.spi, mode: Unconfigured, address: self.address })
    }
}


pub struct ContinuousMode;
pub struct OneShotMode;
pub struct ScanMode;
pub struct Unconfigured;
