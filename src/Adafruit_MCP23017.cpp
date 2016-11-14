/*************************************************** 
  This is a library for the MCP23017 i2c port expander

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include "Adafruit_MCP23017.h"

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <uStepper.h>

////////////////////////////////////////////////////////////////////////////////

void Adafruit_MCP23017::begin(uint8_t addr) {
  uint8_t data = 0xFF;

  if (addr > 7) {
    addr = 7;
  }
  i2caddr = addr;
  
  // set defaults!
  // 
  I2C.write(MCP23017_ADDRESS | i2caddr, MCP23017_IODIRA, 1, &data);
  I2C.write(MCP23017_ADDRESS | i2caddr, MCP23017_IODIRB, 1, &data); 
  
  /*WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(MCP23017_IODIRA);
  wiresend(0xFF);  // all inputs on port A
  WIRE.endTransmission();

  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(MCP23017_IODIRB);
  wiresend(0xFF);  // all inputs on port B
  WIRE.endTransmission();*/
}


void Adafruit_MCP23017::begin(void) {
  begin(0);
}

void Adafruit_MCP23017::pinMode(uint8_t p, uint8_t d) {
  uint8_t iodir;
  uint8_t iodiraddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8)
    iodiraddr = MCP23017_IODIRA;
  else {
    iodiraddr = MCP23017_IODIRB;
    p -= 8;
  }

  // read the current IODIR
  I2C.read(MCP23017_ADDRESS | i2caddr, iodiraddr, 1, &iodir); 
  
  /*WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(iodiraddr);	
  WIRE.endTransmission();
  
  WIRE.requestFrom(MCP23017_ADDRESS | i2caddr, 1);
  iodir = wirerecv();
  */
  

  // set the pin and direction
  if (d == INPUT) {
    iodir |= 1 << p; 
  } else {
    iodir &= ~(1 << p);
  }

  // write the new IODIR
  I2C.write(MCP23017_ADDRESS | i2caddr, iodiraddr, 1, &iodir);
  
  /*WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(iodiraddr);
  wiresend(iodir);	
  WIRE.endTransmission();*/
}

uint16_t Adafruit_MCP23017::readGPIOAB() {
  uint8_t data[2];
  uint16_t ba = 0;
  uint8_t a;

  // read the current GPIO output latches
  I2C.read(MCP23017_ADDRESS | i2caddr, MCP23017_GPIOA, 2, data); 
  /*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(MCP23017_GPIOA);	
  WIRE.endTransmission();
  
  WIRE.requestFrom(MCP23017_ADDRESS | i2caddr, 2);
  */
  a = data[0];
  ba = data[1];
  ba <<= 8;
  ba |= a;

  return ba;
}

void Adafruit_MCP23017::writeGPIOAB(uint16_t ba) {
  uint8_t data[2];

  data[0] = (uint8_t)(ba & 0xFF);
  data[1] = (uint8_t)(ba >> 8);

/*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(MCP23017_GPIOA);	
  wiresend(ba & 0xFF);
  wiresend(ba >> 8);
  WIRE.endTransmission();*/

  I2C.write(MCP23017_ADDRESS | i2caddr, MCP23017_GPIOA, 2, data);
}

void Adafruit_MCP23017::digitalWrite(uint8_t p, uint8_t d) {
  uint8_t gpio;
  uint8_t gpioaddr, olataddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8) {
    olataddr = MCP23017_OLATA;
    gpioaddr = MCP23017_GPIOA;
  } else {
    olataddr = MCP23017_OLATB;
    gpioaddr = MCP23017_GPIOB;
    p -= 8;
  }


  // read the current GPIO output latches
  I2C.read(MCP23017_ADDRESS | i2caddr, olataddr, 1, &gpio); 
  /*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(olataddr);	
  WIRE.endTransmission();
  
  WIRE.requestFrom(MCP23017_ADDRESS | i2caddr, 1);
   gpio = wirerecv();
*/
  // set the pin and direction
  if (d == HIGH) {
    gpio |= 1 << p; 
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  
  I2C.write(MCP23017_ADDRESS | i2caddr, gpioaddr, 1, &gpio);

  /*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(gpioaddr);
  wiresend(gpio);	
  WIRE.endTransmission();
*/
}

void Adafruit_MCP23017::pullUp(uint8_t p, uint8_t d) {
  uint8_t gppu;
  uint8_t gppuaddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8)
    gppuaddr = MCP23017_GPPUA;
  else {
    gppuaddr = MCP23017_GPPUB;
    p -= 8;
  }


  // read the current pullup resistor set
  
  I2C.read(MCP23017_ADDRESS | i2caddr, gppuaddr, 1, &gppu); 
/*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(gppuaddr);	
  WIRE.endTransmission();
  
  WIRE.requestFrom(MCP23017_ADDRESS | i2caddr, 1);
  gppu = wirerecv();
*/
  // set the pin and direction
  if (d == HIGH) {
    gppu |= 1 << p; 
  } else {
    gppu &= ~(1 << p);
  }

  // write the new GPIO
  I2C.write(MCP23017_ADDRESS | i2caddr, gppuaddr, 1, &gppu);
  /*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(gppuaddr);
  wiresend(gppu);	
  WIRE.endTransmission();
*/
}

uint8_t Adafruit_MCP23017::digitalRead(uint8_t p) {
  uint8_t gpioaddr;
  uint8_t gpio;

  // only 16 bits!
  if (p > 15)
    return 0;

  if (p < 8)
    gpioaddr = MCP23017_GPIOA;
  else {
    gpioaddr = MCP23017_GPIOB;
    p -= 8;
  }

  // read the current GPIO
  I2C.read(MCP23017_ADDRESS | i2caddr, gpioaddr, 1, &gpio); 
  /*
  WIRE.beginTransmission(MCP23017_ADDRESS | i2caddr);
  wiresend(gpioaddr);	
  WIRE.endTransmission();
  
  WIRE.requestFrom(MCP23017_ADDRESS | i2caddr, 1);
  */
 
  return (gpio >> p) & 0x1;
}
