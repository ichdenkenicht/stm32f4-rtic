//! todo

use enc28j60::{Enc28j60};
use smoltcp::{
    phy::{self, Device, DeviceCapabilities, Medium},
    time::Instant,
};
use stm32f4xx_hal::hal::spi::SpiDevice;
use stm32f4xx_hal::hal::spi::ErrorType;


pub struct EncPhy<SPI, INT, RESET> {
    phy: Enc28j60<SPI, INT, RESET>,
    rx_buffer: [u8; 1536],
    tx_buffer: [u8; 1536],
}

impl<'a, SPI, INT, RESET> EncPhy<SPI, INT, RESET> {
    pub fn new(phy: Enc28j60<SPI, INT, RESET>) -> Self {
        EncPhy {
            phy: phy,
            rx_buffer: [0; 1536],
            tx_buffer: [0; 1536],
        }
    }
}

impl<SPI, INT, RESET> phy::Device for EncPhy<SPI, INT, RESET> 
where 
    SPI: stm32f4xx_hal::hal::spi::ErrorType + SpiDevice,
    INT: enc28j60::IntPin,
    RESET: enc28j60::ResetPin,
{
    type RxToken<'a> = EncPhyRxToken<'a> where Self: 'a;
    type TxToken<'a> = EncPhyTxToken<'a, SPI, INT, RESET> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {


        self.phy.receive(&mut self.rx_buffer).unwrap();

        let rxt = EncPhyRxToken(&mut self.rx_buffer[..]);
        let txt = EncPhyTxToken{buf: &mut self.tx_buffer[..], phy: &mut self.phy};

        


        Some((rxt,txt))
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(EncPhyTxToken{buf: &mut self.tx_buffer[..], phy: &mut self.phy})
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1536;
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps
    }
}

pub struct EncPhyRxToken<'a>(&'a mut [u8]);

impl<'a> phy::RxToken for EncPhyRxToken<'a> {
    fn consume<R, F>(mut self, f: F) -> R
        where F: FnOnce(&mut [u8]) -> R
    {
        // TODO: receive packet into buffer
        let result = f(&mut self.0);
        
        result
    }
}

pub struct EncPhyTxToken<'a, SPI, INT , RESET>{
    buf: &'a mut [u8],
    phy: &'a mut Enc28j60<SPI, INT, RESET>,
}

impl<'a, SPI, INT, RESET> phy::TxToken for EncPhyTxToken<'a, SPI, INT, RESET> 
where 
    SPI: stm32f4xx_hal::hal::spi::ErrorType + SpiDevice,
    INT: enc28j60::IntPin,
    RESET: enc28j60::ResetPin,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
        where F: FnOnce(&mut [u8]) -> R
    {
        let result = f(&mut self.buf[..len]);

        // TODO: send packet out - needs testing
        self.phy.transmit(&self.buf[..len]).unwrap();
        //println!("tx called {}", len);
        result
    }
}



// does not work because of double &mut on phy when token are symetric :(
/*

pub struct EncPhyRxToken<'a, SPI, INT , RESET>{
    buf: &'a mut [u8],
    phy: &'a mut Enc28j60<SPI, INT, RESET>,
}

impl<'a, SPI, INT , RESET> phy::RxToken for EncPhyRxToken<'a, SPI, INT , RESET> 
where 
    SPI: stm32f4xx_hal::hal::spi::ErrorType + SpiDevice,
    INT: enc28j60::IntPin,
    RESET: enc28j60::ResetPin,
{
    fn consume<R, F>(mut self, f: F) -> R
        where F: FnOnce(&mut [u8]) -> R
    {
        // TODO: receive packet into buffer
        let result = f(&mut self.buf);

        self.phy.receive(&mut self.buf).unwrap();
        //println!("rx called");
        result
    }
}

*/




//################################################
/*
struct EncPhy<'a, SPI, INT, RESET> {
    phy: Enc28j60<SPI, INT, RESET>,
    rx_buf: &'a mut [u8],
    tx_buf: &'a mut [u8],
}

impl<'a, SPI, INT, RESET> EncPhy<'a, SPI, INT, RESET> {
    ///Create a new ethernet interface
    pub fn new(phy: Enc28j60<SPI, INT, RESET>, rx_buf: &'a mut [u8], tx_buf: &'a mut [u8]) -> Self {
        EncPhy { phy, rx_buf, tx_buf }
    }
}


impl<'a, SPI, INT, RESET> phy::Device for EncPhy<'a, SPI, INT, RESET> 
where 
    SPI: stm32f4xx_hal::hal::spi::ErrorType + SpiDevice,
    INT: enc28j60::IntPin,
    RESET: enc28j60::ResetPin,
{

    type RxToken = RxToken<'a, SPI, INT, RESET>
        where Self: 'a;
    type TxToken = TxToken<'a, SPI, INT, RESET> 
        where Self: 'a;

    fn receive(&'a mut self, timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        
    }
    
    fn transmit(&'a mut self, timestamp: Instant) -> Option<Self::TxToken<'_>> {
        
    }


    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps
    }
    

}


/// A token to receive a single network packet
pub struct RxToken<'a, SPI, INT, RESET> {
    phy: &'a mut Enc28j60<SPI, INT, RESET>,
    buf: &'a mut [u8],
}



/// A Token to transmit a single network packet
pub struct TxToken<'a, SPI, INT, RESET> {
    phy: &'a mut Enc28j60<SPI, INT, RESET>,
    buf: &'a mut [u8],
}
*/