#include "IO_AD7150.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>

// requires Arduino Modbus library (and dependecies, install with library manager) and modified AD7150 library (included in tar file, place in libraries folder)

IO_AD7150 ad7150;
AD7150_Values result;

#include <SPI.h>
#include <NativeEthernet.h>

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
//#define DEBUG1

byte mac[6];

IPAddress ip(172, 21, 103, 30);
IPAddress netmask(255,255,254,0);
IPAddress gateway(172,21,102,1);

EthernetServer ethServer(502);
ModbusTCPServer modbusTCPServer;


void setup()
{
  Serial.begin(115200);

  teensyMAC(mac);
  // uncomment this for DHCP
  // Ethernet.begin(mac);

  // uncomment this for static IP (set above)
  Ethernet.begin(mac,ip,gateway,gateway,netmask);
    


  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());

  // start the server
  ethServer.begin();

  // start the Modbus TCP server
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }

  // configure input registers at address 0x00

  modbusTCPServer.configureInputRegisters(0x00,6);


  // configure capacitance sensor
   
  ad7150.begin();
   // ad7150.setOffset( AD7150_OFFSET_AUTO );
   ad7150.setOffset(128+45);
  ad7150.setRange(AD7150_RANGE_0_4);
 ad7150.setup();
  
//  uint8_t capdac1 = 0x85;
//  uint8_t capdac2 = 0x97;
//  calibrate(capdac1, capdac2);



  
}

void loop()
{
  ad7150.configure(AD7150_MODE_CONT_CONV);
  delay(10);
  // ad7150.configure(AD7150_MODE_POWER_DOWN); 
  result = ad7150.getValue();
  EthernetClient client = ethServer.available();
  
  if (client) {
    // a new client connected
    // Serial.println("new client");

    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);
    if (client.connected()) {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
    }
  }

 
#ifdef DEBUG1
  Serial.print("Status :");
  Serial.print(result.status, BIN);
  Serial.print("\t");
  Serial.print("Value :");
  Serial.print(result.value, DEC);
  Serial.print(" ");
  Serial.print(result.capdac,DEC);
#endif 

 float cap = 0;
 float offset = 0;

 if (result.value >= 0x3000 and result.value < 0xFFF0 ) {
   cap = ((((float) result.value) - 0x3000) / 0x9ff0) * 4;
   offset = (((float) result.capdac) / 63) * 12.62;
 } 



  Serial.print("Value :");
  Serial.print(cap+offset);
  Serial.print(" ");
  Serial.print(cap);
  Serial.println();

  
  uint8_t reg = 0;

  modbusTCPServer.inputRegisterWrite(reg++, (uint16_t) ((cap+offset)*1000));
  modbusTCPServer.inputRegisterWrite(reg++, (uint16_t) (cap*1000));
  modbusTCPServer.inputRegisterWrite(reg++, (uint16_t) (offset*1000));
  
  // raw values
  modbusTCPServer.inputRegisterWrite(reg++, (uint16_t) result.value);
  modbusTCPServer.inputRegisterWrite(reg++, (uint16_t) result.capdac);
  modbusTCPServer.inputRegisterWrite(reg++, (uint16_t) result.status);

  
#ifdef DEBUG1   
  Serial.print(" ");
  Serial.print(cap);
  Serial.print(" ");
  Serial.print(offset);
  Serial.print(" ");
  Serial.print(cap+offset); 
  delay(100);
  Serial.println();
#endif 

 
delay(10);
}


void teensyMAC(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


float calibrate(uint8_t capdac1, uint8_t capdac2) {

 Serial.print("Calibrating");
 Serial.println();

 ad7150.setOffset(capdac1); 
 ad7150.setup();
 ad7150.configure(AD7150_MODE_SING_CONV);
  delay(10);
  AD7150_Values result1 = ad7150.getValue();
  Serial.print("Status :");
  Serial.print(result1.status, BIN);
  Serial.print("\t");
  Serial.print("Value :");
  Serial.print(result1.value, DEC);
  Serial.print(" ");
  Serial.print(result1.capdac,DEC);

  Serial.println();

  ad7150.setOffset(capdac2); 
  ad7150.setup();
  ad7150.configure(AD7150_MODE_SING_CONV);
  delay(10);
  AD7150_Values result2 = ad7150.getValue();
  Serial.print("Status :");
  Serial.print(result2.status, BIN);
  Serial.print("\t");
  Serial.print("Value :");
  Serial.print(result2.value, DEC);
  Serial.print(" ");
  Serial.print(result2.capdac,DEC);
  Serial.println();

  float cap1 = ((((float) result1.value) - 0x3000) / 0x9ff0) * 4;
  float cap2 = ((((float) result2.value) - 0x3000) / 0x9ff0) * 4;

  
  
  float slope = (cap1 - cap2) / (result2.capdac - result1.capdac);

  Serial.print(slope * 63); 
  Serial.println();

  return slope;
  
  
}

 
