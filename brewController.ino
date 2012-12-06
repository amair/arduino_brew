/*********************

**********************/

// include the library code:
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <MenuBackend.h>

const char VERSION[6] ="0.1";

// This defines the addresses of the 2Wire devices in use so we can identify them
#define NUM_PROBES 2
const byte PROBE[NUM_PROBES][8] = {{0x28, 0xE8, 0xE9, 0x18, 0x04, 0x00, 0x00, 0x05},{0x28, 0x1A, 0xCE, 0x0C, 0x04, 0x00, 0x00, 0x86}};
const bool parasitic_mode = false; // Define if we are using parasitic power or not

Adafruit_RGBLCDShield lcd;
// RGB Shield needs 5 wires
// VSS and VDD
// i2c clock and data connect to A4 and A5 on Arduino
// i2c interrupt connects to digital pin 3 on arduino

//Arduino pins
OneWire  ds(7);  // on pin 7


volatile boolean buttonPress=false;  
boolean liquor_preheat = false;
float HLT_temp, HERMS_out_temp, mash_temp; // in C as this is default for DS1820

//this controls the menu backend and the event generation
MenuBackend menu = MenuBackend(menuEventUse,menuEventChange);
MenuItem root = MenuItem("Root");
MenuItem preheat_liquor = MenuItem("Preheat");
MenuItem mash = MenuItem("Mash");
MenuItem settings = MenuItem("Settings");

void menuSetup()
{
  Serial.println("Setting up menu...");

  menu.getRoot().add(root); 
  root.addRight(preheat_liquor);
  //root.addLeft(settings);
  preheat_liquor.addRight(mash);
  //preheat_liquor.addLeft(root);
  mash.addRight(settings);
  //mash.addLeft(preheat_liquor);
  settings.addRight(root);
}

/*
	This is an important function
	Here all use events are handled
	
	This is where you define a behaviour for a menu item
*/
void menuEventUse(MenuUseEvent used)
{
 // Serial.print("Menu use ");
 // Serial.println(used.item.getName());
  
  if (used.item == preheat_liquor) {
    toggle_preheat_liquor();
  }
}

/*
	This is an important function
	Here we get a notification whenever the user changes the menu
	That is, when the menu is navigated
*/
void menuEventChange(MenuChangeEvent changed)
{
	Serial.print("Menu change ");
	Serial.print(changed.from.getName());
	Serial.print(" ");
	Serial.println(changed.to.getName());
  lcdtopRow(changed.to.getName());
}


void Button_Pressed() {
  // Keep this as short as possible - because you are holding up the proc
  // So we will handle this by just marking that an interrupt has happened,
  // The rest of the code will need to monitor for this and abort long running 
  // processes if they find out there is an interrupt.
  buttonPress=true;
}

void setup() {
  // Debugging output
  Serial.begin(9600);
  
  // initialise the LCD 
  lcd = Adafruit_RGBLCDShield();
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  int time = millis();
  
  //Catch the external interrupt from the button input
  attachInterrupt(1,Button_Pressed,FALLING);
  //This signal is active low, so HIGH-to-LOW when interrupt

  //Tell the MCP23017 to start making interrupts
  lcd.enableButtonInterrupt();
  
  //Clear the interrupt if it was present
  while (!digitalRead(3)) {
    lcd.readButtons();
  }
  
  menuSetup();
  
  startBrewConroller();
}

uint8_t i=0;
void loop() {
  

  //lcd.setCursor(0, 1);
  //lcd.print(millis()/1000);
  
  if (buttonPress) {
    uint8_t buttons = lcd.readButtons();
    
    if (buttons & BUTTON_UP) { up_pressed(); }
    if (buttons & BUTTON_DOWN) { down_pressed(); }
    if (buttons & BUTTON_RIGHT) { right_pressed(); }
    if (buttons & BUTTON_LEFT) { left_pressed(); }
    if (buttons & BUTTON_SELECT) { enter_pressed(); }
    
    buttonPress=false;
  } else 
  {
      updateTemperatures();
  }

}


void startBrewConroller()
{
  lcd.clear();
  lcdtopRow("Brew Controller");
  lcdBottomRow(VERSION);
}

void left_pressed() {
  Serial.println("Left"); 
  menu.moveLeft();
}

void right_pressed() {
  Serial.println("Right"); 
  menu.moveRight();
}
void up_pressed() {
  Serial.println("Up"); 
  menu.moveUp();
}
void down_pressed() {
  Serial.println("Down"); 
  menu.moveDown();
}
void enter_pressed() {
  Serial.println("Enter"); 
  menu.use();
}

void update_display()
{
}

void lcdtopRow( const char message[20])
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(message);
}

void lcdBottomRow( const char message[20] )
{
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(message);
}

void toggle_preheat_liquor() 
{
  liquor_preheat = !liquor_preheat;
  
//  if (liquor_preheat) {
//    lcdBottomRow("ON");
//  } else {
//    lcdBottomRow("OFF");
//  }
}

// Function to test the address against the PROBE array to identify which one we are talking to
// Requires PROBE to be a 2Dimensional byte Array 1st Dimension has NUM_PROBES entries (<254), 2nd dimension is 8
// addr needs to be a byte array of length 8
// Return value is 255 if the address couldn't be found or the index of the 1st Dimension.

unsigned char identifyProbe (byte * addr)
{
  byte i,probe;
  byte found_probe = 255;
  
  for (probe=0; probe < NUM_PROBES & found_probe==255; probe++) {
    for( i = 0; i < 8; i++) {
      if (addr[i] != PROBE[probe][i])  {break;} // It's not this probe so bail
      if (i==7) {found_probe = probe;} // If we got this far then we have a match
    }
  }
    return found_probe;
}

void updateTemperatures()
{
    byte probe;
  byte present = 0;
  byte data[12];
  byte addr[8];
  float celsius;
  
  while ( ds.search(addr)) {
    
    if (OneWire::crc8(addr, 7) == addr[7]) {
      probe = identifyProbe(addr);
      
      Serial.print("Probe:"); Serial.print(probe); Serial.print(" ");
    
      ds.reset();
      ds.select(addr);
      ds.write(0x44,1);  // Issue the convert command
      
      if (parasitic_mode) {
        delay(1000);     // maybe 750ms is enough, maybe not
      } else
      { // Poll for completion
        while (ds.read() == 0) { 
          if (buttonPress)
          {
            // An interrupt has occurred, so abort and bail
            ds.reset_search();
            return;
          }
        } 
      }
      
      // we might do a ds.depower() here, but the reset will take care of it.
      
      present = ds.reset();
      ds.select(addr);    
      ds.write(0xBE);         // Read Scratchpad
    
       for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
      }
  
    // Verify the CRC matches and that the data isn't corrupt     
      if (OneWire::crc8(data, 8) == data[8]) {
        // convert the data to actual temperature
        unsigned int raw = (data[1] << 8) | data[0];
      
        byte cfg = (data[4] & 0x60);
        if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
        // default is 12 bit resolution, 750 ms conversion time
      
        celsius = (float)raw / 16.0;
        Serial.print("  Temperature = ");
        Serial.print(celsius);
        Serial.println(" Celsius, ");
        
        switch(probe) {
          case 0:
            HLT_temp = celsius; break;
          case 1: 
            HERMS_out_temp = celsius; break;
          case 2: 
            mash_temp = celsius; break;
        }
        
      } else {
       Serial.println("CRC mismatch - ignoring reading");
      }
    } else
    {
      Serial.println("CRC mismatch - ignoring reading");
    }
  }
  
  ds.reset_search();

}


