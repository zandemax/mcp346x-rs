#![no_std]

use embedded_hal::SPI;
use embedded_hal::spi::{SpiDevice, SpiBus};
use measurements::Voltage;

#[derive(Debug)]
pub enum Error<E> {
    Spi(E),
    VoltageTooHigh,
    VoltageTooLow,
    NotInitialized,
    NotReady,
}


pub struct MCP346x<SPI, MODE> {
    spi: SPI,
    mode: MODE,
    address: u8
}

fn generate_fast_command_byte(device_address: u8, command: u8) -> u8 {
    return (device_address << 6) || (command << 2) || 0b00000000;
}

fn generate_register_command_byte(device_address: u8, register_address: u8, command_type: u8) -> u8 {
    return (device_address << 6) || (register_address << 2) || command_type;
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

    fn set_mode(&self, mode: u8) -> Result<(), Error<E>> {
        let command: [u8;1] = [0;1];
        self.spi.write(&command).map_err(Error::Spi)?;
        Ok(())
    }

    fn static_read(&self, register: u8) -> Result<[u32;16], Error<E>> {
        let command: u8 = generate_register_command_byte(self.address, register, 0b01);
        let mut buf: [u32;16] = [0;16];
        self.spi.transfer(&mut buf, &command)?;
        Ok(buf)
    }

    fn incremental_read(&self, register: u8) -> Result<[u32;16], Error<E>> {
        let command: u8 = generate_register_command_byte(self.address, register, 0b01);
        let mut buf: [u32;16] = [0;16];
        self.spi.transfer(&mut buf, &command)?;
        Ok(buf)
    }

    fn static_write(&self, register: u8) -> Result<[u32;16], Error<E>> {
        let command: u8 = generate_register_command_byte(self.address, register, 0b01);
        let mut buf: [u32;16] = [0;16];
        self.spi.transfer(&mut buf, &command)?;
        Ok(buf)
    }

    pub fn into_continuous_mode(self) -> Result<MCP346x<SPI, ContinuousMode>, Error<E>> {
        self.set_mode(0b00)?;
        Ok(MCP346x {
            spi: self.spi,
            mode: ContinuousMode,
            address: self.address
        })
    }

    pub fn into_scan_mode(self) -> Result<MCP346x<SPI, ScanMode>, Error<E>> {
        self.set_mode(0b10)?;
        Ok(MCP346x {
            spi: self.spi,
            mode: ScanMode,
            address: self.address
        })
    }

    pub fn into_oneshot_mode(self) -> Result<MCP346x<SPI, OneShotMode>, Error<E>> {
        self.set_mode(0b11)?;
        Ok(MCP346x {
            spi: self.spi,
            mode: OneShotMode,
            address: self.address
        })
    }
    
    pub fn measure(&self) -> Result<Voltage, Error<E>> {
        let buf: [u8;4] = [0;4];
        let command: u8 = generate_fast_command_byte(, command);
        self.spi.transfer(&mut buf, &command);
        Ok(Voltage::from_volts(u32::from_be_bytes(buf) as f64))
    }

    pub fn start_conversion(&self) -> Result<Ok(()), Error<E>> {
        let command: u8 = generate_fast_command_byte(self.address, 0b1010);
        self.spi.write(&command)?;
        Ok(())
    }
    
    pub fn standby(&self) -> Result<Ok(()), Error<E>> {
        let command: u8 = generate_fast_command_byte(self.address, 0b1011);
        self.spi.write(&command)?;
        Ok(())
    }

    pub fn shutdown(&self) -> Result<Ok(()), Error<E>> {
        let command: u8 = generate_fast_command_byte(self.address, 0b1100);
        self.spi.write(&command)?;
        Ok(())
    }
    
    pub fn full_shutdown(self) -> Result<Ok(()), Error<E>> {
        let command: u8 = generate_fast_command_byte(self.address, 0b1101);
        self.spi.write(&command)?;
        Ok(())
    }
    
    pub fn reset(&self) -> Result<MCP346x<SPI, Unconfigured>, Error<E>> {
        let command: u8 = generate_fast_command_byte(self.address, 0b1110);
        self.spi.write(&command)?;
        Ok(MCP346x { spi: self.spi, mode: Unconfigured, address: self.address })
    }
}


pub struct ContinuousMode;
pub struct OneShotMode;
pub struct ScanMode;
pub struct Unconfigured;
