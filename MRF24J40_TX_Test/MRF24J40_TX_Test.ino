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

const int pin_reset = 4;
const int pin_cs = 10; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328
Mrf24j mrf(pin_reset, pin_cs, pin_interrupt); // Set up the MRF24J Module
EnergyMonitor emon1;                          // Create EnergyMonitor instance

char tbuf[16];

long last_time;
long tx_interval = 1000;
boolean contact_int = false;

void setup() {
  Serial.begin(9600);
  Serial.print("hello ");
  for (int i=0; i<9; i++,delay(1000))
    Serial.print(".");
  Serial.println();
  emon1.current(1, 111.1);             // Current: input pin, calibration.
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only


  delay(10);
  mrf.reset();  // Reset and Init the Radio Module
  mrf.init();
  
  // The personal network address
  mrf.set_pan(0xcafe);

  // This board's address
  mrf.address16_write(0x6003); 

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

    Serial.print(Irms*230.0);        // Apparent power
    Serial.print(" ");
    Serial.println(Irms);          // Irms


    // Clear the buffer
    for (int i = 0; i<16; i++) {
      tbuf[i]=0x00;
    }
    
    //     // Convert the float into a char array
    dtostrf(Irms, 5, 3, tbuf);
    // I need to make sure that tbuf is termated but for know I am cheating and just expanding the size of the char array

    // Add the Temp to the output string
    output_string += "PA:" + String(tbuf) + ";";

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
    delay(30000);
}

void handle_rx() {
    // data to receive, nothing to do
}

void handle_tx() {
    if (!mrf.get_txinfo()->tx_ok) {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    } else {
      Serial.println("TX Successful");
    }
    
}