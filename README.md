# STTS22H
Arduino library to support the STTS22H digital temperature sensor

## API

This sensor uses I2C to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    dev_i2c = new TwoWire(I2C_SDA, I2C_SCL);  
    dev_i2c->begin();

An instance can be created and enbaled when the I2C bus is used following the procedure below:  

    Temp = new STTS22HSensor(dev_i2c);  
    Temp->Enable();

The access to the sensor values is done as explained below:  

  Read temperature.  

    Temp->GetTemperature(&temperature);

## Documentation

You can find the source files at  
https://github.com/stm32duino/STTS22H

The STTS22H datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/temperature-sensors/stts22h.html
