/*********************

**********************/

// include the library code:
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <MenuBackend.h>

const char VERSION[6] ="0.1";
const byte HLT_TEM_ADDR addr[8] = {0x28, 0xE8, 0xE9, 0x18, 0x04, 0x00, 0x00, 0x05};


Adafruit_RGBLCDShield lcd;
// RGB Shield needs 5 wires
// VSS and VDD
// i2c clock and data connect to A4 and A5 on Arduino
// i2c interrupt connects to digital pin 3 on arduino
OneWire  ds(7);  // on pin 10


volatile boolean buttonPress=false;
boolean liquor_preheat = false;
float HLT_temp, HERMS_out_temp, mash_temp; // in C as this is default for DS1820

//this controls the menu backend and the event generation
MenuBackend menu = MenuBackend(menuUseEvent,menuChangeEvent);
  MenuItem root = MenuItem("Root");
  MenuItem preheat_liquor = MenuItem("Preheat");
  MenuItem mash = MenuItem("Mash");
  MenuItem settings = MenuItem("Settings");

	
//this function builds the menu and connects the correct items together
void menuSetup()
{
  Serial.println("Setting up menu...");

  menu.getRoot().add(root); 
  root.addRight(preheat_liquor);
  root.addLeft(settings);
  preheat_liquor.addRight(mash);
  preheat_liquor.addLeft(root);
  mash.addRight(settings);
  mash.addLeft(preheat_liquor);
}

/*
	This is an important function
	Here all use events are handled
	
	This is where you define a behaviour for a menu item
*/
void menuUseEvent(MenuUseEvent used)
{
  Serial.print("Menu use ");
  Serial.println(used.item.getName());
  
  if (used.item == preheat_liquor) {
    toggle_preheat_liquor();
  }
}

/*
	This is an important function
	Here we get a notification whenever the user changes the menu
	That is, when the menu is navigated
*/
void menuChangeEvent(MenuChangeEvent changed)
{
	Serial.print("Menu change ");
	Serial.print(changed.from.getName());
	Serial.print(" ");
	Serial.println(changed.to.getName());
  lcdtopRow(changed.to.getName());
}


void Button_Pressed() {
// Keep this as short as possible - because you are holding up the proc  
  
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
   
    //Serial.println("INTERRUPT");
   
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
  //menu.moveUp();
}
void down_pressed() {
  Serial.println("Down"); 
  //menu.moveDown();
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
  lcd.setCursor(0,0);
  lcd.print(message);
}

void lcdBottomRow( const char message[20] )
{
  lcd.setCursor(0,1);
  lcd.print(message);
}

void toggle_preheat_liquor() 
{
  liquor_preheat = !liquor_preheat;
  
  if (liquor_preheat) {
    lcdBottomRow("ON");
  } else {
    lcdBottomRow("OFF");
  }
}

void updateTemperatures()
{
    byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  float celsius;
  
  while ( ds.search(addr)) {
    
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
  
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();
   
  
    ds.reset();
    ds.select(addr);
    ds.write(0x44,1);         // start conversion, with parasite power on at the end
    
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad
  
    Serial.print("  Data = ");
    Serial.print(present,HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    //Serial.print(" CRC=");
    //Serial.print(OneWire::crc8(data, 8), HEX);
    //Serial.println();
    byte calculated_crc = OneWire::crc8(data, 8);
    
    if (calculated_crc == data[9]) {
     Serial.print("CRC match");
    }
    
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
    Serial.print(" Celsius, ");
    
  }
  
  ds.reset_search();

}
