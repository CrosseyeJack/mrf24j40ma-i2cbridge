/*
 * MFR24j40 <-> I2C Bridge
 * Hooking the MFRF24j40 up to the rpi was giving headaches, so I decided to cheat an use a ATMega328 as a bridge
 * It is still a WIP but atleast I am now getting data from another transceiver dumpped into the rpi with i2cdump
*/ 

/*
 * TODO:-
 * Create a byte array for the stored data and call that from within the i2c functions to save a bucket load of if
 * Statements. - DONE
 * Disable the radio once we have received the data and send a pin high so the Raspberry Pi has a chance to pull
 * the data. Once data has been collected reenable the radio, interrupts and send the pin low
 * Tidy up the code, this was more of an experiment to see if I could use i2c as a bride for the MFR24j40 so it
 * contains sloppy code. - KINDA DONE
 * Store the PAN and Radio Addresses in EEPROM so they don't need to be set every time
 * First Sweep at Cleanup

 Yeah I know that incoming/outgoing buffers are kind labeled the wrong way around. Think of it as incoming and outgoing
 of the micro and not in the sense of incoming/going in the sense of the raspberry pi. I might refactor this later.
 */

#define pan_address_low     0x10
#define pan_address_high    0x11
#define board_address_low   0x12
#define board_address_high  0x13
#define sender_address_low  0x14
#define sender_address_high 0x15
#define lqi_low             0x16
#define lqi_high            0x17
#define rssi_low            0x18
#define rssi_high           0x19
#define data_length         0x1F  // Even though we get an int back atm we are only storing
                                  // a max of (256 - 32) bytes, I could use pages to store
                                  // more but 1) I'm being lazy B) 224 bytes is enough
#define data_offset         0x20   

#include <SPI.h>
#include <mrf24j.h>
#include <Wire.h>
#include <util/crc16.h>

const int pin_reset = 4;
const int pin_cs = 10;
const int pin_interrupt = 2;
#define led_blink 8

#define pin_i2c_int 6

word pan_address   = 0xcafe;
word board_address = 0x6001;

byte outgoing_data_buf[255];	// Data from the Micro to the Pi
byte incoming_data_buf[255];	// Data from the Pi to the Micro
byte data_tx[255];

int loop_count = 0;
bool led_state = false;

int errorcheck_count = 0;
int tx_error_check=0;

int tx_loop = 0;
bool tx_flag = false;

// test counter
int counter = 0;

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

void setup() {
  Serial.begin(9600);

  // Setup the Interrupt Pin
  pinMode(pin_i2c_int,OUTPUT);
  digitalWrite(pin_i2c_int,LOW);
  pinMode(led_blink, OUTPUT);
  
  // Make sure the buffers are clear (0xFF)
  for (int i = 0; i < 255; i++) {
    outgoing_data_buf[i] = 0x00;
    incoming_data_buf[i] = 0x00;
  }

  // For a test make put "header" in the top of the output data
  // Going to scrap this later so just being lazy with the code right now
  outgoing_data_buf[0x00] = 0xDE;
  outgoing_data_buf[0x01] = 0xAD;
  outgoing_data_buf[0x02] = 0xBE;
  outgoing_data_buf[0x03] = 0xEF;
  outgoing_data_buf[0x04] = 0x42;
  outgoing_data_buf[0x05] = 0x48;
  outgoing_data_buf[0x06] = 0x48;
  outgoing_data_buf[0x07] = 0x47;
  outgoing_data_buf[0x08] = 0x54;
  outgoing_data_buf[0x09] = 0x54;
  outgoing_data_buf[0x0A] = 0x47;
  outgoing_data_buf[0x0B] = 0x42;
  outgoing_data_buf[0x0C] = 0xDE;
  outgoing_data_buf[0x0D] = 0xAD;
  outgoing_data_buf[0x0E] = 0xBE;
  outgoing_data_buf[0x0F] = 0xEF;
  
  // Setup i2c
  Wire.begin(0x42);
  Wire.onReceive(i2creceiveEvent);
  Wire.onRequest(i2crequestEvent); // register wire.request interrupt event
  

  delay(2500);

  // Setup mrf24j40ma
  mrf.reset();
  delay(25);
  mrf.init();
  delay(25);
  mrf.set_pan(pan_address); // Pan Address
  delay(25);
  mrf.address16_write(board_address); // Board Address
  outgoing_data_buf[pan_address_low]=lowByte(pan_address);
  outgoing_data_buf[pan_address_high]=highByte(pan_address);
  outgoing_data_buf[board_address_low]=lowByte(board_address);
  outgoing_data_buf[board_address_high]=highByte(board_address);
  attachInterrupt(0, interrupt_routine, FALLING); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  interrupts();
}


void loop() {  
  // Toggle a I/O so I can see that the micro is still running
  if (++loop_count == 5) {
    loop_count = 0;
    digitalWrite(led_blink, !digitalRead(led_blink));
  }

  // Check for time out on reseting the int pin
  if (digitalRead(pin_i2c_int)) {
    if (++errorcheck_count == 10) {
      errorcheck_count = 0;
      digitalWrite(pin_i2c_int, LOW);
    }
  }

  if (tx_flag == true) {
    if (tx_error_check>3) {
      tx_error_check = 0;
      Serial.println("Failed CRC Check 3 times");
      memset(incoming_data_buf, 0, sizeof(incoming_data_buf));
      memset(data_tx, 0, sizeof(data_tx));
      return;
    }
    word crccheck = CRC16_checksum((char *)incoming_data_buf);
    Serial.print("CRC: ");
    Serial.println(crccheck,HEX);
    
    for (int i=0; i<strlen((char *)incoming_data_buf); i++) {
      Serial.print(incoming_data_buf[i],HEX);
    }
    Serial.println();

    int copycount = 0;
    for (copycount=0; copycount<strlen((char *)incoming_data_buf); copycount++) {
      data_tx[copycount] = incoming_data_buf[copycount];
    }
    data_tx[copycount++] = highByte(crccheck);
    data_tx[copycount++] = lowByte(crccheck);

    for(int i=0; i<strlen((char *)data_tx); i++) {
      Serial.print(data_tx[i],HEX);  
    }

    Serial.println();
    mrf.send16(0x6006, (char *) data_tx, strlen((char *)data_tx));

    // Reset the flag
    // memset(incoming_data_buf, 0, sizeof(incoming_data_buf));
    // memset(data_tx, 0, sizeof(data_tx));
    tx_flag = false;
  }

  // Check the flags
  mrf.check_flags(&handle_rx, &handle_tx);

  // Delay the loop
  delay(100);
}

void i2crequestEvent(void) { // Request for data from buffer
  while(Wire.available() > 0) {
    // Send the data requested from the outgoing_data_buf
    byte c = Wire.read();
    if ((c>=0x00)&&(c<=0xFF)) {
        Wire.write(outgoing_data_buf[c]);
    } else {
        Wire.write(0xFF);
    }
  }
}

void i2creceiveEvent(int numBytes) { // Write to Micro Command
  // Need to do some checks here, I think this is whats is fucking things up.
	if (numBytes == 2) {
		byte address = Wire.read();
		byte data = Wire.read();

		if (address == 0xFF && data == 0xFF) { // Flush the outgoing data buffer
			for (int i = 16; i < 255; i++) {
				outgoing_data_buf[i] = 0x00;
			}
        digitalWrite(pin_i2c_int, LOW);
        errorcheck_count = 0;
		} else if (address == 0xFF && data == 0xFB) { // Copy the data out of the buffer and TX it.
      // As this is an interupt I'm going to just set a flag and check for the flag in the main code.
      tx_flag = true;
    } else if (address == 0xFF && data == 0xFD) {
          // Wipe the TX Buffer(s)
      for (int i = 0; i < 255; i++) {
        incoming_data_buf[i] = 0x00;
      }
      Serial.println("Buffer Cleared");
    } else {
      // Copy the data into the incomping data buffer
      incoming_data_buf[address] = data;
      // Serial.print("Addy: 0x");Serial.print(address,HEX);Serial.print(" Data: 0x");Serial.print(data,HEX);
      // Serial.print(" Stored: 0x");Serial.print(incoming_data_buf[address],HEX);Serial.print(" ASCII: ");
      // Serial.write(incoming_data_buf[address]);Serial.println();
    }
	} 
}

void interrupt_routine() {
  mrf.interrupt_handler(); // mrf24 object interrupt routine
}
  
void handle_rx() {
  // If this is the issue I will not be a happy bunny.
  // Bad coding Dan. Very Bad Coding.
  // Set the flags AFTER you have copied the data into the buffers.
  // No wonder the first read was always blank.
  // data_buff_flag = true;
  // digitalWrite(pin_i2c_int,HIGH);
  // Lets wipe the buffer first and then but the new data on top of that

  
  // Check to see payload reads "TXOK"
  if (mrf.rx_datalength()==4 && mrf.get_rxinfo()->rx_data[0]=='T' && mrf.get_rxinfo()->rx_data[1]=='X' && mrf.get_rxinfo()->rx_data[2]=='O' && mrf.get_rxinfo()->rx_data[3]=='K') {
    Serial.println("TX was CRC checked on other end...");
    tx_error_check = 0;
    memset(incoming_data_buf, 0, sizeof(incoming_data_buf));
    memset(data_tx, 0, sizeof(data_tx));
    return;
  }
  // Check to see payload reads "TXFAIL"
  if (mrf.rx_datalength()==6 && mrf.get_rxinfo()->rx_data[0]=='T' && mrf.get_rxinfo()->rx_data[1]=='X' && mrf.get_rxinfo()->rx_data[2]=='F' && mrf.get_rxinfo()->rx_data[3]=='A' && mrf.get_rxinfo()->rx_data[4]=='I' && mrf.get_rxinfo()->rx_data[5]=='L') {
    Serial.println("TX failed CRC checked on other end...");
    tx_error_check++;
    tx_flag = true;
    return;
  }

  for (int i = 16; i < 255; i++) {
    outgoing_data_buf[i] = 0x00;
  }
  
  outgoing_data_buf[sender_address_low]   = lowByte(mrf.get_rxinfo()->src_addr16);
  outgoing_data_buf[sender_address_high]  = highByte(mrf.get_rxinfo()->src_addr16);
  outgoing_data_buf[lqi_low]              = lowByte(mrf.get_rxinfo()->lqi);
  outgoing_data_buf[lqi_high]             = highByte(mrf.get_rxinfo()->lqi);
  outgoing_data_buf[rssi_low]             = lowByte(mrf.get_rxinfo()->rssi);
  outgoing_data_buf[rssi_high]            = highByte(mrf.get_rxinfo()->rssi);
  outgoing_data_buf[data_length]          = mrf.rx_datalength();

  for (int i = 0; i < mrf.rx_datalength(); i++) {
    outgoing_data_buf[data_offset+i]      = mrf.get_rxinfo()->rx_data[i];
  }

  delay(50);

  errorcheck_count = 0;
  digitalWrite(pin_i2c_int,HIGH);

}

void handle_tx() {
  if (!mrf.get_txinfo()->tx_ok) {
    Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
  } else {
    Serial.println("TX Successful");
  }

  // I want to check if the message was correctly recieved before clearing the buffer but for now.
  // Just wipe it here
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