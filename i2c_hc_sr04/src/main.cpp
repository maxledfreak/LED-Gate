//-------------------------- I2C (SDA, SCL) -------------------------------------
/*
module：           RCWL-9610,RCWL-9600,RCWL-9620 16MM Split Type Open Type Ultrasonic Ranging Module
Version：          V2.0
date：             20220710
Master chip：      RCWL-9610
Function：         RCWL-9610 ultrasonic distance measuring module distance measurement and serial port display in IIC mode
Notice：           Need to set the module in IIC mode
test board：       RCWL-3310
write：            Wuxi Richen IoT Technology Co., Ltd.
Program customization：  13915288564
I2C data format：        The IIC of RCWL-9610 outputs three 8BIT data, and the distance MM 
                         value = 24-bit data is converted into decimal/10000.
connection ：
   -VCC               = 3.3V/5.5V
   -Trig_RX_SCL_I/O   = A5
   -Echo_TX_SDA       = A4
   -GND               = GND
*/

#include "SoftwareSerial.h"
#include "Wire.h" 
#include <Adafruit_Sensor.h>

float    distance = 0;           // Distance data decimal value
float    ds[3];                  // [3] 8BIT distance data

#define I2C_SDA 8
#define I2C_SCL 9
TwoWire I2CSensors = TwoWire(0);

void setup()
{
  Serial.begin(115200);           //Define serial port baud rate 9600 Factory default baud rate 9600
  delay(10);
  I2CSensors.begin(I2C_SDA, I2C_SCL);
  Serial.println("RCWL-9610 ranging start："); 
}
 
void loop() 
{
   char i = 0;
    ds[0]=0;
    ds[1]=0;
    ds[2]=0;                       // Initialize three 8BIT distance data as 0
    
   I2CSensors.beginTransmission(0x57);   // The address is 0X57, write 8-bit data as AE, and read 8-bit data as AF
   I2CSensors.write(0X01);                  // Write command 0X01, 0X01 is the start measurement command
   I2CSensors.endTransmission();         // I2C end command
           
   delay(150);                     //Measurement cycle delay, one cycle is 120mS, set 150MS, leave a margin   
   
   I2CSensors.requestFrom(0x57,3);       //The address is 0X57 to read three 8-bit distance data     
    while (I2CSensors.available())
    {
     ds[i++] = I2CSensors.read();
    }        
    
   distance=(ds[0]*65536+ds[1]*256+ds[2])/10000;  //Calculated as CM value  
   Serial.print("distance："); 
   
   if ((1<=distance)&&(distance<=900))  // Numerical display between 1CM-9M
    {
     #if 0
     Serial.println();    
     Serial.print(ds[0]);
     Serial.println();    
     Serial.print(ds[1]);
     Serial.println();    
     Serial.print(ds[2]);    
     Serial.println();      
     #endif                                      //#if 1, output 3 distance data of I2C
     
    Serial.print(distance);
    Serial.print(" CM ");  
    }
   else 
    {
     Serial.println();    
     Serial.print(ds[0]);
     Serial.println();    
     Serial.print(ds[1]);
     Serial.println();    
     Serial.print(ds[2]);    
     Serial.println();  
      
    Serial.print(" - - - - ");     //Invalid value Numerical display - - - -
    }
  
    Serial.println();              //new line
    delay(20);                     // After a single measurement is completed, 
                                   // add a 30mS delay before the next measurement. 
                                   // To prevent the aftermath of the last measurement 
                                   // when measuring at close range, resulting in inaccurate measurement.
    delay(100);                    // Delay 200mS to measure again, delay is not necessary
}