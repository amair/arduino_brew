/*********************

**********************/

// include the library code:
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <MenuBackend.h>

const char VERSION[6] ="0.3";

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
OneWire  ds(7);  // One wire thermometers on pin 7
const uint8_t HLT_HEATER_1=12; // Heater coil 1 in HLT
const uint8_t HLT_HEATER_2=11; // Heater coil 2 in HLT
const uint8_t HERMS_HEATER=10; // Heater coil in HERMS recirculator

enum DISPLAY_SCREEN{ROOT, PREHEAT, MASH, SETTINGS};  // Each screen must be defined here

volatile boolean buttonPress=false;// Indicates if we have received an
                                   // Interupt to tell us a button needs read
boolean liquor_preheat = false; // Are we in preheat mode for the HLT
boolean herms_recirc = false; // Are we running the HERMS recirculation
float HLT_temp; 
float HERMS_out_temp;
float mash_temp; // in Celsius as this is default for DS1820

float Strike_temp=18.0;//80.0; //Target temperature for HLT water
float Mash_target=18.0;//65.5; // Target mash temp inside mash tun

boolean HLT_heater_ON=false; //True when the HLT heater(s) are on
boolean HERMS_heater_ON=false; //True when the HERMS heat source is on

DISPLAY_SCREEN display=ROOT; // The currently displayed screen
boolean display_refresh=true; // Set to true when the display should be refreshed

//this controls the menu backend and the event generation
MenuBackend menu = MenuBackend(menuEventUse,menuEventChange);
MenuItem root = MenuItem("Root");
MenuItem preheat_liquor = MenuItem("Preheat");
MenuItem mash = MenuItem("Mash");
MenuItem settings = MenuItem("Settings");
char prev_item[5];
char next_item[5];

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

// Called whenever the user presses the select/enter key
void menuEventUse(MenuUseEvent used)
{
  if (used.item == preheat_liquor) {
    toggle_preheat_liquor();
  } else if (used.item == mash ) {
    toggle_herms_recirc();
  } else
  {
    Serial.println("Nothing to do here");
  }
}

// Called whenever the menu changes
void menuEventChange(MenuChangeEvent changed)
{
  const char * name = changed.to.getName();
  uint8_t len = strlen(name);
  uint8_t start = (20-len)/2;
  
  lcdClearTopRow();
  
  lcd.setCursor(start,0);
  lcd.print(name);

  if (changed.to==preheat_liquor)
  {
    display=PREHEAT;
  } else if (changed.to==mash) {
    display=MASH;
  } else if (changed.to==root) {
    display=ROOT;
  } else if (changed.to==settings){
    display=SETTINGS;
  }

  display_refresh = true;
  
  strncpy(next_item, changed.to.getRight()->getName(), 4);
  next_item[4]='\0';
  strncpy(prev_item, changed.to.getLeft()->getName(), 4);
  prev_item[4]='\0';
}

void display_temp_menu(float current, float target) {
    lcd.setCursor(0,1);
    lcd.print("Current Temp = ");
    lcd.setCursor(15,1);
    lcd.print(current,2);
    
    lcd.setCursor(0,2);
    lcd.print("Target Temp =");
    lcd.setCursor(15,2);
    lcd.print(target,2);

    lcd.setCursor(9,3);

    if((liquor_preheat && display==PREHEAT)||(herms_recirc && display==MASH)) {
        lcd.print("ON ");
    } else {
        lcd.print("OFF");
    }
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

  //Setup the pins
  pinMode(HLT_HEATER_1, OUTPUT);
  pinMode(HLT_HEATER_2, OUTPUT);
  pinMode(HERMS_HEATER, OUTPUT);

  // initialise the LCD
  lcd = Adafruit_RGBLCDShield();
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);

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
}

void display_root()
{
  lcd.clear();
  lcdtopRow("Brew Controller");
  lcd.setCursor(9,3);
  lcd.print(VERSION);
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

void lcdtopRow( const char message[20])
{
  lcdClearTopRow();
  lcd.setCursor(0,0);
  lcd.print(message);
}

void lcdClearTopRow( void )
{
    lcd.setCursor(0,0);
    lcd.print("                    ");
}

void lcdBottomRow( const char message[20] )
{
  lcd.setCursor(0,3);
  lcd.print(message);
}

void lcdClearBottomRow(void)
{
    lcd.setCursor(0,3);
    lcd.print("                    ");
}

// Allow a manual override of whether heaters are used or not
void toggle_preheat_liquor()
{
  liquor_preheat = !liquor_preheat;
  Serial.println("Toggle preheat mode");
  toggle_HLT_Heater();
  display_refresh=true;
}

// Allow a manual override of whether heaters are used or not
void toggle_herms_recirc()
{
  herms_recirc=!herms_recirc;
  Serial.println("Toggle HERMS mode");
  toggle_HERMS_Heater();
  display_refresh=true;
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

      for ( uint8_t i = 0; i < 9; i++) {           // we need 9 bytes
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
            if (celsius!=HLT_temp && display==PREHEAT) display_refresh=true;
            HLT_temp = celsius; break;
          case 1:
            if (celsius!=HERMS_out_temp && display==MASH) display_refresh=true;
            HERMS_out_temp = celsius; break;
          case 2:
            if (celsius!=mash_temp && display==MASH) display_refresh=true;
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

void cycle_HLT()
{
    if (HLT_heater_ON && HLT_temp>=Strike_temp) {
        toggle_HLT_Heater();
    } else if (!HLT_heater_ON && Strike_temp>HLT_temp) {
        toggle_HLT_Heater();
    }

}

void cycle_HERMS()
{
    if (HERMS_heater_ON && HERMS_out_temp>=Mash_target)
    {
        toggle_HERMS_Heater();
    } else if (!HERMS_heater_ON && Mash_target>HERMS_out_temp) {
        toggle_HERMS_Heater();
    }
}

void toggle_HLT_Heater()
{
    HLT_heater_ON = ! HLT_heater_ON;
    Serial.print("Toggle HLT: ");
    Serial.println(HLT_heater_ON);
    digitalWrite(HLT_HEATER_1, HLT_heater_ON ? HIGH : LOW );
    digitalWrite(HLT_HEATER_2, HLT_heater_ON ? HIGH : LOW);
}

void toggle_HERMS_Heater()
{
    HERMS_heater_ON = ! HERMS_heater_ON;
    Serial.print("Toggle Herms: ");
    Serial.println(HERMS_heater_ON);
    digitalWrite(HERMS_HEATER,HERMS_heater_ON ? HIGH : LOW);
}

void updateDisplay ()
{
  lcd.setCursor(0,3);
  lcd.print("<");
  lcd.print(prev_item);
  lcd.setCursor(15,3);
  lcd.print(next_item);
  lcd.print(">");

  switch (display) {
    case ROOT:
      display_root();
    break;
    case PREHEAT:
       display_temp_menu(HLT_temp,Strike_temp);
    break;
    case MASH:
       display_temp_menu(HERMS_out_temp, Mash_target);
    break;
    case SETTINGS:
    break;
  }
  
 display_refresh=false;
}

void loop() {

  if (buttonPress) {
    uint8_t buttons = lcd.readButtons(); // NB This allows for >1 button to be depressed

    if (buttons & BUTTON_UP) { up_pressed(); }
    if (buttons & BUTTON_DOWN) { down_pressed(); }
    if (buttons & BUTTON_RIGHT) { right_pressed(); }
    if (buttons & BUTTON_LEFT) { left_pressed(); }
    if (buttons & BUTTON_SELECT) { enter_pressed(); }

    buttonPress=false;
  } else
  {
      updateTemperatures();

      if (liquor_preheat) {
        cycle_HLT();
      }
      
      if (herms_recirc) {
        cycle_HERMS();
      }
      
      if (display_refresh) {
        updateDisplay();
      } 
  }

}

