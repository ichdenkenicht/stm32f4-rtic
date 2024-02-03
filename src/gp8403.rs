use embedded_hal::i2c::{I2c, Error};

pub enum OutputRange{
    V5,
    V10,
}

pub enum Addr {
    A58 = 0x58,
}

pub enum Channel {
    Channel0,
    Channel1,
}

pub struct GP8403Driver<I2C> {
    i2c: I2C,
    addr: u8,
}


impl<I2C: I2c> GP8403Driver<I2C> {
    pub fn new(i2c: I2C, addr: Addr) -> Self {
        Self { i2c, addr: addr as u8}
    }
    /*
    pub fn read_temperature(&mut self) -> Result<u8, I2C::Error> {
        let mut temp = [0];
        self.i2c.write_read(ADDR, &[TEMP_REGISTER], &mut temp)?;
        Ok(temp[0])
    }
    */


    /// Set DAC Output Range
    pub fn setOutputRange(&mut self, range: OutputRange) -> Result<(), I2C::Error> {
        match range {
            OutputRange::V5 => {
                self.i2c.write(self.addr, &[0x01, 0x00])?;
                Ok(())
            },
            OutputRange::V10 => {
                self.i2c.write(self.addr, &[0x01, 0x11])?;
                Ok(())
            },
        }
    }

    pub fn setOutput(&mut self,ch: Channel, v: u16) -> Result<(), I2C::Error> {

        let t: [u8; 2] = (v << 4).to_be_bytes();


        match ch {
            Channel::Channel0 => {
                self.i2c.write(self.addr, &[0x2, t[1], t[0]])?;
                Ok(())
            },
            Channel::Channel1 => {
                self.i2c.write(self.addr, &[0x4, t[1], t[0]])?;
                Ok(())
            },
        }
    }


}