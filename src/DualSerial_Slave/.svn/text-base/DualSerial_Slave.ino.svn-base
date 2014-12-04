/*
Dual Serial: Slave
Echoes the inputs received at Software Serial ports 
to hardware Serial

 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 */
#include <SoftwareSerial.h>

SoftwareSerial masterSerial(10, 11); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port
  masterSerial.begin(4800);
}

void loop() // run over and over
{
  if (masterSerial.available())
    Serial.write(masterSerial.read());
  if (Serial.available())
    masterSerial.write(Serial.read());
}