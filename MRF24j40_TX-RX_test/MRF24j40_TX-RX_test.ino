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
#include <LiquidCrystal.h>
#include <util/crc16.h>

const int pin_reset = 4;
const int pin_cs = 10; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328
Mrf24j mrf(pin_reset, pin_cs, pin_interrupt); // Set up the MRF24J Module
EnergyMonitor emon1;                          // Create EnergyMonitor instance
LiquidCrystal lcd(5, 6, 7, 8, 9, A5);

char tbuf[16];
byte incoming_data_buf[255];

long last_time;
long tx_interval = 1000;
boolean contact_int = false;

int lcdchar = 0;
int lcdline = 0;
bool lcd_overflow = false;

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);
  for (int i=0; i<9; i++,delay(1000)) {
    lcdwrite(0x2E);
  }
  lcdclear();
  emon1.current(1, 111.1);             // Current: input pin, calibration.
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only


  delay(10);
  mrf.reset();  // Reset and Init the Radio Module
  mrf.init();
  
  // The personal network address
  mrf.set_pan(0xcafe);

  // This board's address
  mrf.address16_write(0x6006); 

  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();
}

void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
    // String output_string = "";
    // double Irms = emon1.calcIrms(1480);  

    // Serial.print(Irms*230.0);        // Apparent power
    // Serial.print(" ");
    // Serial.println(Irms);          // Irms


    // // Clear the buffer
    // for (int i = 0; i<16; i++) {
    //   tbuf[i]=0x00;
    // }
    
    // //     // Convert the float into a char array
    // dtostrf(Irms, 5, 3, tbuf);
    // // I need to make sure that tbuf is termated but for know I am cheating and just expanding the size of the char array

    // // Add the Temp to the output string
    // output_string += "PA:" + String(tbuf) + ";";

    // // Copy the output string into a char array
    // char cbuf[output_string.length() + 1];
    // output_string.toCharArray(cbuf,output_string.length() + 1);

    // // Debug Output
    // for (int i = 0; i < sizeof(cbuf); i++){
    //   Serial.write(cbuf[i]);
    // }
    //   Serial.println();
    
    // I want to store the address to send this to in the epprom.
    //mrf.send16(0x6001, (char *) cbuf, strlen((char *)cbuf));
    mrf.check_flags(&handle_rx, &handle_tx);
}

void handle_rx() {

  if (mrf.get_rxinfo()->src_addr16 != 0x6001) { // Check if the data came from "openHab"
    return;
  }

  if (mrf.rx_datalength() > 255) { // Check the data length
    Serial.println(mrf.rx_datalength());
    return;
  }

  // Wipe the buffer
  for (int i = 0; i < 255; i++) {
    incoming_data_buf[i] = 0x00;
  }

  // Copy the data (apart from the last 2 bytes) into the incoming data buffer
  for (int i = 0; i<mrf.rx_datalength()-2; i++) {
    incoming_data_buf[i] = mrf.get_rxinfo()->rx_data[i];
  }

  word crcchecksum = word(mrf.get_rxinfo()->rx_data[mrf.rx_datalength()-2],mrf.get_rxinfo()->rx_data[mrf.rx_datalength()-1]);
  word crcchecksumcal = CRC16_checksum((char *)incoming_data_buf);
  Serial.print("CRC: ");
  Serial.println(crcchecksum,HEX);
  Serial.println(crcchecksumcal,HEX);

  if (crcchecksumcal==crcchecksum) {
    Serial.println("CRC MATCH!!!");
    lcdclear();
    for (int i = 0; i<strlen((char *)incoming_data_buf); i++) {
      lcdwrite(incoming_data_buf[i]);
    }
    Serial.println();
    // Tell the pi we were successfull
    mrf.send16(mrf.get_rxinfo()->src_addr16, (char *)"TXOK", 4);
  } else {
    Serial.println("CRC MISMATCH!!");
    // Tell The pi to try again
    mrf.send16(mrf.get_rxinfo()->src_addr16, (char *)"TXFAIL", 6);
  }

  if(incoming_data_buf[0] == '\0') { // First char null Just return
    return;
  }
}

void lcdclear() {
  lcdchar = 0;
  lcdline = 0;
  lcd_overflow = false;
  lcd.clear();
  lcd.setCursor(0,0);
  Serial.println();
}

void lcdwrite(byte b) {
  if (lcd_overflow == false) {
    // Check for Line Break and other special chars
    if (b == 0x0A) { // Line Break
      lcdchar = 19; // Jump to "the end of the line" and let the code below handle the line break
    } else if (lcdchar == 0 && b == 0x20) { // If we are on a new line and the char is a space, skip it
      return;
    } else {
      lcd.write(b);
    }
    
    Serial.write(b);
    
    if (++lcdchar > 19) {
      // Need a new line
      if (++lcdline > 3) {
        // Need to return to 0,0
        // I decided not to return to 0,0 but set a flag and not over write the data at the start of the screen
        lcd_overflow = true;
        return;
      }
      lcd.setCursor(0,lcdline);
      lcdchar = 0;
    }
  }
}



  // Check the data and send it to the LCD
  // I need to decide on which i/o pins I will be using for the LCD
  
    
    // incoming_data_buf[sender_address_low]   = lowByte(mrf.get_rxinfo()->src_addr16);
    // incoming_data_buf[sender_address_high]  = highByte(mrf.get_rxinfo()->src_addr16);
    // incoming_data_buf[lqi_low]              = lowByte(mrf.get_rxinfo()->lqi);
    // incoming_data_buf[lqi_high]             = highByte(mrf.get_rxinfo()->lqi);
    // incoming_data_buf[rssi_low]             = lowByte(mrf.get_rxinfo()->rssi);
    // incoming_data_buf[rssi_high]            = highByte(mrf.get_rxinfo()->rssi);
    // incoming_data_buf[data_length]          = mrf.rx_datalength();

    // // data to receive, nothing to do
    // for (int i = 0; i <= 255; i++) {
    //   incoming_data_buf[i] = 0x00;
    // }
    
    // for (int i = 0; i <= mrf.rx_datalength(); i++) {
    //   incoming_data_buf[i]      = mrf.get_rxinfo()->rx_data[i];
    // }

    // delay(50);
// }

void handle_tx() {
    if (!mrf.get_txinfo()->tx_ok) {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    } else {
      Serial.println("TX Successful");
    }
    
}

uint16_t CRC16_checksum (char *string)
{
 size_t i;
 uint16_t crc;
 uint8_t c;
 
crc = 0xFFFF;
 
// Calculate checksum ignoring the first two $s
 for (i = 2; i < strlen(string); i++)
 {
 c = string[i];
 crc = _crc_xmodem_update (crc, c);
 }
 
return crc;
}