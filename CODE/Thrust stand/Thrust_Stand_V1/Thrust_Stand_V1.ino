
/*********************************************************************
This code is writen by JENKINS ROBOTICS. Subscibe to us on Youtube!
This code is design to control our thrust stand for static rocket fire. 

Key Information:
  - Revision 1.0
  - Board : Arduino
  - Screen : Monochrome OLEDs based on SSD1306
  - Screen : 128x32 size display using I2C 

  Referenced Examples
    - button cycler/ Neopixel/ adafruit library
    - HX711_ADC

 *  if you are sharing this code, you must keep this copyright note.
*********************************************************************/


 // **************** INCLUDE NECESSARY LIBRARIES *****************
 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif


// **************** OLED LIBRARY *****************
// OLED SCREEN  SETUP
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// this is the Width and Height of Display which is 128 xy 32 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define XPOS 0
#define YPOS 1
#define DELTAY 2
double count=0;

// **************** NEOPIXEL LIBRARY *****************

// Simple demonstration on using an input device to trigger changes on your
// NeoPixels. Wire a momentary push button to connect from ground to a
// digital IO pin. When the button is pressed it will change to a new pixel
// animation. Initial state has all pixels off -- press the button once to
// start the first animation. As written, the button does not interrupt an
// animation in-progress, it works only when idle.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.
#define BUTTON_PIN   2
#define PIXEL_PIN    6  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 16  // Number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
int  newState = 0;
int  oldState = 0;
int  mode     = 0;    // Currently-active animation mode, 0-9



// **************** HX711 SCALE LIBRARY *****************

//pins:
const int HX711_dout = 9; //mcu > HX711 dout pin
const int HX711_sck = 8; //mcu > HX711 sck pin
//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;



// **************** VARIABLES *****************

// INITIATE CONSTANT INTEGERS
// constants won't change. They're used here to set pin numbers:
  const int armpin = 2; // input pullup
  const int testpin = 3; // input pullup
  const int zeropin = 4; // input pullup
  const int mosfetpin = 5; // output
  const int pixelpin = 6; // output
  const int strainDTpin = 9; // Strain gage DT
  const int strainSCKpin = 8; // Strain gage SCK
  
 // INITIATE VARIABLES (GLOBAL)
 // variables will change:
  int pushbutton1 = digitalRead(armpin); // PUSH BUTTON 1 --- ARM 
  int pushbutton2 = digitalRead(testpin); // PUSH BUTTON 2 --- TEST
  int pushbutton3 = digitalRead(zeropin); // PUSH BUTTON 3 --- ZERO
  int armfunction = LOW; // initiate arm change
  int testfunction = LOW; // initiate Static Thrust Test
  int zerofunction = LOW; // initiate Zeroing of gage
  int armlimit = LOW; //  arm button limit
  int testlimit = LOW; // test button limit
  int zerolimit = LOW; // zero button limit
  int straingage = 445.77; // strain gage value
  int armstate = 0; // arm control 
  int systemstate = 0; // system control 
  int armtime = 0; // arm delay timer
  int testtime = 0; // test program delay timer
        unsigned long int now = 0; 
        unsigned long int starttime = now;
        unsigned long int laptime = now-starttime;
  

// ****************SETUP *****************

 

void setup() {
  Serial.begin(9600); delay(10);
  Serial.println();
  Serial.println("Starting...");

//OLED SETUP
  // OLED by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // drawing commands to make them visible on screen!
   // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000);
   // Clear the buffer.
  display.clearDisplay(); 

//NEOPIXEL SETUP
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.
 // pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'


//Scale SETUP
  float calibrationValue; // calibration value
  calibrationValue = 40771.77; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch this value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch this value from eeprom

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  

   if (LoadCell.getTareTimeoutFlag()) {
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
  }
  while (!LoadCell.update());
 

// initialize Digital Input and Outputs
   pinMode(mosfetpin, OUTPUT);
   pinMode(armpin,INPUT_PULLUP); 
   pinMode(testpin,INPUT_PULLUP);
   pinMode(zeropin,INPUT_PULLUP);
   digitalWrite(mosfetpin,LOW);
   Serial.println("READY");
   delay(100);
   systemstate=1;
  
}




// ****************  Main Loop *****************

void loop() {
  UI();         // Function that controls Oled Screen
  neopixel();   // function that controls Neopixel state
    arm();   // function that controls changing arm state   
    test();   // function that controls the static firing test    
    zero();   // function that controls gage zeroing    
   scale();   // function that controls gage zeroing   
  delay(20);
  
}

// **************** End Main Loop *****************



void UI() {
  // UPDATE UI
    String gage =  String(straingage);// using a float and the
    display.clearDisplay();
    
   //  ARM STATUS;
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(4,2);
    switch (armstate) {
        case 0 :
                  display.println("DISARM"); //do something when var equals 1
                  break;
        case 1 :
                 display.println("ARM"); //do something when var equals 2
                  break;
        default :
                   display.println("ERROR");   // if nothing else matches, do the default
                  break;
    }
    
   //  GAGE STATUS;
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(70,2);
    display.print(gage);  
    display.print(' '); 
    display.print('N'); 
     
   //  SYSTEM STATUS;
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(4,14);
    switch (systemstate) {
      case 0 :
                display.println("BOOTING"); //do something when var equals 1
                break;
      case 1 :
                 display.println("STANDBY"); //do something when var equals 2
                break;
      case 2 :
                display.println("TEST FIRE"); 
                break;
      case 3 :
                display.println("IGNIITION"); 
                break;
      case 4 :
                display.println("COOLDOWN"); 
                break;
      case 5 :
                display.println("COMPLETED"); 
                break;
      case 6 :
                display.println("ARM ERROR"); 
                break;
    }
    display.display();
    
 }




void arm(){

pushbutton1 = digitalRead(armpin); // UPDATE CURRENT STATE OF BUTTON 1  

  if(pushbutton1 == LOW && armlimit == LOW){                  // PRESS INITIATED
      armlimit = HIGH;                                        // Set button limit to high. function can only run once per press:
      armfunction = HIGH;                                     // turn Armfunction high:  arm fuction will run once
   } 
  if(pushbutton1 == HIGH && armlimit == HIGH){                // WAIT TILL BUTTON IS RELEASED
        armlimit = LOW;                                        // reset button limit. 
  } 

  if(armfunction == HIGH && armlimit == LOW){                 // BUTTON IS RELEASED INITIATE TASK
     armstate = !armstate; // toggle Arm State 0/1
     armfunction = LOW; // function complete set to low
  }

    
}




void test(){

   pushbutton2 = digitalRead(testpin); // UPDATE CURRENT STATE OF BUTTON 2  

  if(pushbutton2 == LOW && testlimit == LOW){                  // PRESS INITIATED
      testlimit = HIGH;                                        // Set button limit to high. function can only run once per press:
      testfunction = HIGH;                                     // turn Armfunction high:  arm fuction will run once
      now = millis(); 
      starttime = now; 
   } 
  if(pushbutton2 == HIGH && testlimit == HIGH){                // WAIT TILL BUTTON IS RELEASED
        testlimit = LOW;                                        // reset button limit. 
        now = millis(); 
        laptime = now-starttime;
  } 

  if(testfunction == HIGH && testlimit == LOW && armstate == 1){                 // BUTTON IS RELEASED INITIATE TASK
        now = millis();
        laptime= now-starttime;
     
        
        if((laptime >= 0) && (laptime <= 4000)){
          systemstate = 2;// START DELAY 
    
        }
        if((laptime >= 4000) && (laptime <= 8000)){ // IGNITION
          digitalWrite(mosfetpin,HIGH);
          systemstate = 3; // IGNITION
        }
         if((laptime >= 8000) && (laptime <= 25000)){   // COOLDOWN
          digitalWrite(mosfetpin,LOW); 
          systemstate = 4; //COOL DOWN
        }
         if((laptime >= 25000) && (laptime <= 30000)){ // COMPLETED
          systemstate = 5; // COMPLETED
        }
         if(laptime >= 30000){ // Back to standby
          testfunction = LOW; // function complete set to low
          systemstate = 1;  // Standby 
          now = 0;
          starttime = 0;
          laptime = now-starttime;
        }            
  }


  if(testfunction == HIGH && testlimit == LOW && armstate == 0){                 // BUTTON IS RELEASED INITIATE TASK    
   now = millis();
   laptime = now-starttime;
    systemstate = 6;//  Set system into Error ... Please ARM 
       if(laptime>=3000){ // Back to standby
              testfunction = LOW; // function complete set to low
              systemstate = 1;  // Standby 
              now = 0;
              starttime = 0;
              laptime = now-starttime;
          }
  }
}



void zero(){
  
    pushbutton3 = digitalRead(zeropin); // PUSH BUTTON 3 --- ZERO

   if(pushbutton3 == LOW && zerolimit == LOW){
      zerolimit = HIGH; // Set button limit to high. function can only run once per press:
      zerofunction = HIGH;   // test fuction will run once
   } 
  if(pushbutton3 == HIGH && zerolimit == HIGH){
      zerolimit = LOW;         // reset button limit. 
  } 

  if(zerofunction == HIGH && zerolimit == LOW){
    LoadCell.tareNoDelay();
    zerofunction = LOW; // function complete set to low
  }

  
}






void neopixel() {

  // Get current button state.
   newState = systemstate;

  // Check if state changed from high to low (button press).
  if((oldState != newState) ) {
      
      switch (systemstate) {
    case 0 :  // SYSTEM STATE = BOOTING
               colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off          
              break;
    case 1 :  // SYSTEM STATE = standby
              colorWipe(strip.Color(  0, 255,   0), 50);    // Green       
              break;
    case 2 :  // SYSTEM STATE = BEGINING TEST
              colorWipe(strip.Color(  0,   0, 255), 50);    // Blue    
              break;
    case 3 :  // SYSTEM STATE =  IGNITION
              colorWipe(strip.Color(  255,   255, 51), 50);    // YELLOW    
              break;
    case 4 :  // SYSTEM STATE = COOL DOWN
              colorWipe(strip.Color(  0,   0, 127), 50);    // Blue    
              break;
    case 5 :  // SYSTEM STATE = TEST COMPLETED
              colorWipe(strip.Color(  0, 255,   0), 50);    // Green       
              break;
    case 6 :  // SYSTEM STATE = ERROR
              colorWipe(strip.Color(255,   0,   0), 50);    // Red
              break;
    }

  // Set the last-read button state to the old state.
  oldState = newState;
    }

}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}




void scale(){
  
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell:");
      Serial.println(i);
     straingage = i; // strain gage value

      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  
}
