/**
 * Example code for using a microchip mrf24j40 module to send simple packets
 *
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011

 * 12/10/14 :-
 * Ditching all the serial debug output
 */
 
#include <SPI.h>
#include <mrf24j.h>
#include "EmonLib.h"  // Include Emon Library

const int pin_reset     = 9;
const int pin_cs        = 8;
const int pin_interrupt = 2;
Mrf24j mrf(pin_reset, pin_cs, pin_interrupt); // Set up the MRF24J Module
EnergyMonitor emon1;                          // Create EnergyMonitor instance

long last_time;
long tx_interval = 1000;
boolean contact_int = false;

void setup() {
  mrf.reset();  // Reset and Init the Radio Module
  mrf.init();
  emon1.current(1, 111.1);             // Current: input pin, calibration.
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  pinMode(9, INPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(8,HIGH);
  
  // The personal network address
  mrf.set_pan(0xcafe);

  // This board's address
  mrf.address16_write(0x6003); 
  
  // uncomment if you want to enable PA/LNA external control
  //mrf.set_palna(true);

  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();
}

void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
    String output_string = "";
    mrf.check_flags(&handle_rx, &handle_tx);
    double Irms = emon1.calcIrms(1480);  // Calculate Irms only
//    //getting the voltage reading from the temperature sensor
//     int reading = analogRead(0);
// //
// //    // converting that reading to voltage, for 3.3v arduino use 3.3
//     float icvcc = readVcc();
//     icvcc = (icvcc / 1000) + 0.025;
//     float voltage = reading * icvcc / 1024;
//     float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
// //                                                  //to degrees ((volatge - 500mV) times 100);


    Serial.print(Irms*230.0);        // Apparent power
    Serial.print(" ");
    Serial.println(Irms);          // Irms
//     // Convert the float into a char array
    char tbuf[8];
    dtostrf(Irms, 2, 3, tbuf);
    // Add the Temp to the output string
    output_string += "PA:" + String(tbuf) + ";";

    // ;PI:"+digitalRead(9)+";";


    // Add some fake data so I can work on the bridge app better
    // output_string +="D0:1;D1:0;D2:1;";

    // Debug print the string
    // Serial.println(output_string);

    // All the stuff below isn't really needed atm

//    sprintf(buf, "%f", temperatureC);
//    
//    byte* array = (byte*) &temperatureC;

    // Value A0 is hardcoded, I would like to select which pins are enabled by using flags stored in eeprom
    // and can set be set from the raspberry pi.
    // but one step at the time
    // char cbuf[sizeof(temperatureC)+5] = "A0:";
    // strncat(cbuf,tbuf,sizeof(temperatureC)); // BUG.. Why am I readin tbug and the size of temperatureC?
    // // I'll look into it tomorrrow
    // strncat(cbuf,";",1);

    // //lets create a fake Digital pin 1 and 2
    // // I only say fake cause I can't be arsed to type digitalRead(blahh);
    // char digitalpins[16];
    // strncat(digitalpins,"D0:1;D1:0;D2:1;",15);
    
    // char tbuffer[sizeof(cbuf)+sizeof(digitalpins)];
    // strncat(tbuffer,cbuf,sizeof(cbuf));
    // strncat(tbuffer,digitalpins,sizeof(digitalpins));

    // for (int i = 0; i < sizeof(tbuffer); i++){
    //   Serial.write(tbuffer[i]);
    // }
    //   Serial.println();

    // string.toCharArray(buf, len) 

    // Copy the output string into a char array
    char cbuf[output_string.length() + 1];
    output_string.toCharArray(cbuf,output_string.length() + 1);

    // Debug Output
    for (int i = 0; i < sizeof(cbuf); i++){
      Serial.write(cbuf[i]);
    }
      Serial.println();
    
    // I want to store the address to send this to in the epprom.
    mrf.send16(0x6001, (char *) cbuf, strlen((char *)cbuf));
    delay(10000);
}

void handle_rx() {
    // data to receive, nothing to do
}

void handle_tx() {
    if (!mrf.get_txinfo()->tx_ok) {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    } else {
      Serial.println("TX Successful");
      digitalWrite(8,!digitalRead(8));
    }
    
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}