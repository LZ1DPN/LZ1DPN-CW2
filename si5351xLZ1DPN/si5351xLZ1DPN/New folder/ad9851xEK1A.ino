/*
Revision 1.0 - Main code by Richard Visokey AD7C - www.ad7c.com
Revision 2.0 - November 6th, 2013...  ever so slight revision by  VK8BN for AD9851 chip Feb 24 2014
Revision 3.0 - April, 2016 - AD9851 + ARDUINO PRO NANO + integrate cw decoder (by LZ1DPN) (uncontinued version)
Revision 4.0 - May 31, 2016  - deintegrate cw decoder and add button for band change (by LZ1DPN)
Revision 5.0 - July 20, 2016  - change LCD with OLED display + IF --> ready to control transceiver RFT SEG-100 (by LZ1DPN)
Revision 6.0 - August 16, 2016  - serial control buttons from computer with USB serial (by LZ1DPN) (1 up freq, 2 down freq, 3 step increment change, 4 print state)
									for no_display work with DDS generator
Revision 7.0 - November 30, 2016  - added some things from Ashhar Farhan's Minima TRX sketch to control transceiver, keyer, relays and other ... (LZ1DPN mod)								
Revision 8.0 - December 12, 2016  - EK1A trx end revision. Setup last hardware changes ... (LZ1DPN mod)
Revision 9.0 - January 07, 2017  - EK1A trx last revision. Remove not worked bands ... trx work well on 3.5, 5, 7, 10, 14 MHz (LZ1DPN mod)
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Include the library code
//#include <LiquidCrystal.h>
#include <rotary.h>
#include <EEPROM.h>
#include <avr/io.h>
//Setup some items
#define CW_TIMEOUT (600l) // in milliseconds, this is the parameter that determines how long the tx will hold between cw key downs
unsigned long cwTimeout = 0;     //keyer var - dead operator control

#define TX_RX (5)   //mute + (+12V) relay - antenna switch relay TX/RX, and +V in TX for PA - RF Amplifier (2 sided 2 possition relay)
#define TX_ON (7)   // this is for microfone PTT in SSB transceivers (not need for EK1A)
#define CW_KEY (4)   // KEY output pin - in Q7 transistor colector (+5V when keyer down for RF signal modulation) (in Minima to enable sidetone generator on)
#define BAND_HI (6)  // relay for RF output LPF  - (0) < 10 MHz , (1) > 10 MHz (see LPF in EK1A schematic)  
#define FBUTTON (A3)  // tuning step freq CHANGE from 1Hz to 1MHz step for single rotary encoder possition
#define ANALOG_KEYER (A1)  // KEYER input - for analog straight key
char inTx = 0;     // trx in transmit mode temp var
char keyDown = 0;   // keyer down temp vat

//AD9851 control
#define W_CLK 8   // Pin 8 - connect to AD9851 module word load clock pin (CLK)
#define FQ_UD 9   // Pin 9 - connect to freq update pin (FQ)
#define DATA 10   // Pin 10 - connect to serial data load pin (DATA)
#define RESET 11  // Pin 11 - connect to reset pin (RST) 

#define BTNDEC (A2)  // BAND CHANGE BUTTON from 1,8 to 29 MHz - 11 bands
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
Rotary r = Rotary(2,3); // sets the pins for rotary encoder uses.  Must be interrupt pins.
//LiquidCrystal lcd(12, 13, 7, 6, 5, 4); // I used an odd pin combination because I need pin 2 and 3 for the interrupts. for LCD 16x2 - not used now
  
int_fast32_t rit=600; // RIT +600 Hz
int_fast32_t rx=(7000000 - rit); // Starting frequency of VFO
int_fast32_t rx2=1; // temp variable to hold the updated frequency
int_fast32_t rxif=6000000; // IF freq, will be summed with vfo freq - rx variable, my xtal filter now is made from 6 MHz xtals

int_fast32_t increment = 50; // starting VFO update increment in HZ. tuning step
int buttonstate = 0;   // temp var
String hertz = "50 Hz";
int  hertzPosition = 0;

byte ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
String freq; // string to hold the frequency
int_fast32_t timepassed = millis(); // int to hold the arduino miilis since startup
int memstatus = 1;  // value to notify if memory is current or old. 0=old, 1=current.
int ForceFreq = 1;  // Change this to 0 after you upload and run a working sketch to activate the EEPROM memory.  YOU MUST PUT THIS BACK TO 0 AND UPLOAD THE SKETCH AGAIN AFTER STARTING FREQUENCY IS SET!
int byteRead = 0;
const int colums = 10; /// have to be 16 or 20 - in LCD 16x2 - 16, or other , see LCD spec.
const int rows = 2;  /// have to be 2 or 4 - in LCD 16x2 - 2, or other , see LCD spec.
int lcdindex = 0;
int line1[colums];
int line2[colums];

// buttons temp var
int BTNdecodeON = 0;   
int BTNlaststate = 0;
int BTNcheck = 0;
int BTNcheck2 = 0;
int BTNinc = 2; // set number of default band minus 1 ---> (for 7MHz = 2)

void checkCW(){
  pinMode(TX_RX, OUTPUT);
  if (keyDown == 0 && analogRead(ANALOG_KEYER) < 50){
    //switch to transmit mode if we are not already in it
    if (inTx == 0){
      //put the TX_RX line to transmit
      digitalWrite(TX_RX, 1);
      //give the relays a few ms to settle the T/R relays
    }
    inTx = 1;
    keyDown = 1;
    rxif = rit;  // in tx freq +600Hz 
    sendFrequency(rx);
    digitalWrite(CW_KEY, 1); //start the side-tone
  }

  //reset the timer as long as the key is down
  if (keyDown == 1){
     cwTimeout = CW_TIMEOUT + millis();
  }

  //if we have a keyup
  if (keyDown == 1 && analogRead(ANALOG_KEYER) > 150){
    keyDown = 0;
	inTx = 0;    /// NEW
	rxif = 6000000;  /// NEW
	sendFrequency(rx);  /// NEW
    digitalWrite(CW_KEY, 0);  // stop the side-tone
    digitalWrite(TX_RX, 0);
    cwTimeout = millis() + CW_TIMEOUT;
  }

  //if we have keyuup for a longish time while in cw rx mode
  if (inTx == 1 && cwTimeout < millis()){
    //move the radio back to receive
    digitalWrite(TX_RX, 0);
	digitalWrite(CW_KEY, 0);
    inTx = 0;
    rxif = 6000000;
    sendFrequency(rx);
    cwTimeout = 0;
  }
}

// start variable setup

void setup() {

//set up the pins in/out and logic levels
pinMode(TX_RX, OUTPUT);
digitalWrite(TX_RX, LOW);
  
pinMode(FBUTTON, INPUT);  
digitalWrite(FBUTTON, 1);
  
pinMode(TX_ON, INPUT);    // need pullup resistor see Minima schematic
digitalWrite(TX_ON, LOW);
  
pinMode(CW_KEY, OUTPUT);
digitalWrite(CW_KEY, LOW);
  

// Initialize the Serial port so that we can use it for debugging
  Serial.begin(115200);
  Serial.println("Start VFO ver 8.0");

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C address 0x3C (for oled 128x32)
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();

  // Clear the buffer.
  display.clearDisplay();
  
  // text display tests
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(rx+rit);
  display.display();
  
  pinMode(BTNDEC,INPUT);		// band change button
  digitalWrite(BTNDEC,HIGH);    // level
  pinMode(A0,INPUT); // Connect to a button that goes to GND on push - rotary encoder push button - for FREQ STEP change
  digitalWrite(A0,HIGH);  //level
//  lcd.begin(16, 2);  // for LCD
/// next AD9851 communication settings
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT); 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode on the AD9851 - see datasheet
//  lcd.setCursor(hertzPosition,1);    
//  lcd.print(hertz);

  display.clearDisplay();	
  display.setCursor(0,0);
  display.println(rx+rit);
  display.setCursor(0,18);
  display.println(hertz);
  display.display();
  
   // Load the stored frequency  
  if (ForceFreq == 0) {
    freq = String(EEPROM.read(0))+String(EEPROM.read(1))+String(EEPROM.read(2))+String(EEPROM.read(3))+String(EEPROM.read(4))+String(EEPROM.read(5))+String(EEPROM.read(6));
    rx = freq.toInt();  
  }
  
 for (int index = 0; index < colums; index++){
    line1[index] = 32;
	line2[index] = 32;
 }
}

///// START LOOP - MAIN LOOP

void loop() {
	checkCW();   // when pres keyer
	checkBTNdecode();  // BAND change
	
// freq change 
  if (rx != rx2){
		BTNcheck = 0;   
		if (BTNcheck == 0) {
			showFreq();
            display.clearDisplay();	
			display.setCursor(0,0);
			display.println(rx+rit);
			display.setCursor(0,18);
			display.println(hertz);
			display.display();
		}
        sendFrequency(rx);
        rx2 = rx;
      }

//  step freq change     
  buttonstate = digitalRead(A0);
  if(buttonstate == LOW) {
        setincrement();        
    };

  // Write the frequency to memory if not stored and 2 seconds have passed since the last frequency change.
    if(memstatus == 0){   
      if(timepassed+2000 < millis()){
        storeMEM();
        }
      }   

// LPF band switch relay	  // stopped now, need to calcualte new LPF for upper bands
	  
//	if(rx < 10000000){
		digitalWrite(BAND_HI, 0);
//	    }
//	if(rx > 10000000){
//		digitalWrite(BAND_HI, 1);
//		}
		  
///	  SERIAL COMMUNICATION - remote computer control for DDS - worked but not finishet yet - 1, 2, 3, 4 - worked 
   /*  check if data has been sent from the computer: */
  if (Serial.available()) {
    /* read the most recent byte */
    byteRead = Serial.read();
	if(byteRead == 49){     // 1 - up freq
		rx = rx + increment;
		Serial.println(rx+rit);
		}
	if(byteRead == 50){		// 2 - down freq
		rx = rx - increment;
		Serial.println(rx+rit);
		}
	if(byteRead == 51){		// 3 - up increment
		setincrement();
		Serial.println(increment);
		}
	if(byteRead == 52){		// 4 - print VFO state in serial console
		Serial.println("VFO_VERSION 6.0");
		Serial.println(rx+rit);
		Serial.println(rxif);
		Serial.println(increment);
		Serial.println(hertz);
		}
        if(byteRead == 53){		// 5 - scan freq from 7000 to 7050 and back to 7000
             for (int i=0; i=500; (i=i+100))
                rx = rx + i;
                sendFrequency(rx);
                Serial.println(rx+rit);
                showFreq();
                display.clearDisplay();	
				display.setCursor(0,0);
				display.println(rx);
				display.setCursor(0,18);
				display.println(hertz);
				display.display();
                delay(250);
                }
	}
}	  
/// END of main loop ///
/// ===================================================== END ============================================


/// START EXTERNAL FUNCTIONS

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result) {    
    if (result == DIR_CW){rx=rx+increment;}
    else {rx=rx-increment;};       
      if (rx >=70000000){rx=rx2;}; // UPPER VFO LIMIT 
      if (rx <=100000){rx=rx2;}; // LOWER VFO LIMIT
  }
}

// frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {  
  int32_t freq = (frequency + rxif) * 4294967296./180000000;  // note 180 MHz clock on 9851. also note slight adjustment of this can be made to correct for frequency error of onboard crystal
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x001);   // Final control byte, LSB 1 to enable 6 x xtal multiplier on 9851 set to 0x000 for 9850
  pulseHigh(FQ_UD);  // Done!  Should see output
  
//    Serial.println(frequency);   // for serial console debuging
//    Serial.println(frequency + rxif);
}

// transfers a byte, a bit at a time, LSB first to the 9851 via serial DATA line
void tfr_byte(byte data){
  for (int i=0; i<8; i++, data>>=1){
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

// step increments for rotary encoder button
void setincrement(){
  if(increment == 1){increment = 10; hertz = "10 Hz"; hertzPosition=0;} 
  else if(increment == 10){increment = 50; hertz = "50 Hz"; hertzPosition=0;}
  else if (increment == 50){increment = 100;  hertz = "100 Hz"; hertzPosition=0;}
  else if (increment == 100){increment = 500; hertz="500 Hz"; hertzPosition=0;}
  else if (increment == 500){increment = 1000; hertz="1 Khz"; hertzPosition=0;}
  else if (increment == 1000){increment = 2500; hertz="2.5 Khz"; hertzPosition=0;}
  else if (increment == 2500){increment = 5000; hertz="5 Khz"; hertzPosition=0;}
  else if (increment == 5000){increment = 10000; hertz="10 Khz"; hertzPosition=0;}
  else if (increment == 10000){increment = 100000; hertz="100 Khz"; hertzPosition=0;}
  else if (increment == 100000){increment = 1000000; hertz="1 Mhz"; hertzPosition=0;} 
  else{increment = 1; hertz = "1 Hz"; hertzPosition=0;};  
//   lcd.setCursor(0,1);
//   lcd.print("           ");
//   lcd.setCursor(hertzPosition,1); 
//   lcd.print(hertz);
  display.clearDisplay();	
  display.setCursor(0,0);
  display.println(rx+rit);
  display.setCursor(0,18);
  display.println(hertz);
  display.display();
  delay(250); // Adjust this delay to speed up/slow down the button menu scroll speed.
};

// oled display functions
void showFreq(){
    millions = int(rx/1000000);
    hundredthousands = ((rx/100000)%10);
    tenthousands = ((rx/10000)%10);
    thousands = ((rx/1000)%10);
    hundreds = ((rx/100)%10);
    tens = ((rx/10)%10);
    ones = ((rx/1)%10);

	display.clearDisplay();	
	display.setCursor(0,0);
	display.println(rx+rit);
	display.setCursor(0,18);
	display.println(hertz);
	display.display();

	timepassed = millis();
    memstatus = 0; // Trigger memory write
};

void storeMEM(){
   //Write each frequency section to a EPROM slot.  Yes, it's cheating but it works!
   EEPROM.write(0,millions);
   EEPROM.write(1,hundredthousands);
   EEPROM.write(2,tenthousands);
   EEPROM.write(3,thousands);
   EEPROM.write(4,hundreds);       
   EEPROM.write(5,tens);
   EEPROM.write(6,ones);   
   memstatus = 1;  // Let program know memory has been written
};


void checkBTNdecode(){

//  BAND CHANGE !!! band plan - change if need
  
BTNdecodeON = digitalRead(BTNDEC);
if(BTNdecodeON != BTNlaststate){
    if(BTNdecodeON == HIGH){
    delay(200);
    BTNcheck2 = 1;
    BTNinc = BTNinc + 1;
switch (BTNinc) {
    case 1:
      rx=3500000;
      break;
    case 2:
      rx=5250000;
      break;
    case 3:
      rx=7000000;
      break;
    case 4:
      rx=10100000;
      break;
    case 5:
      rx=14000000;
      break;
    default:
      if(BTNinc > 5){
         BTNinc = 0;
      }
    break;
  }
//  lcd.clear(); 	//for lcd only - CHECK IF NEED uncoment 
}

if(BTNdecodeON == LOW){
    BTNcheck2 = 0;
//  lcd.clear();   //for lcd only - CHECK IF NEED uncoment 
	}
    BTNlaststate = BTNcheck2;
  }
}

//// OK END OF PROGRAM
