use stm32h7::stm32h743 as pac;
use super::i2c;

const I2C_ADDR: u8 = 0xa0;

pub fn read_eui48<'a>(i2c: &pac::I2C2) -> Result<[u8; 6], i2c::Error> {
    let mut buffer = [0u8; 6];
    i2c::write_read(i2c, I2C_ADDR, &[0xFAu8], &mut buffer)?;
    Ok(buffer)
}
