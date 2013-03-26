/*********************
NB This requires Arduino 1.0.2 or later for the Interrupt PIN to work on Leonardo
**********************/

// include the library code:
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <MenuBackend.h>
#include <PID_v1.h>

const char VERSION[6] ="0.81";

const boolean serial_output_temp = false;
const boolean mash_output_as_csv=true;

// This defines the addresses of the 2Wire devices in use so we can identify them
#define NUM_PROBES 4

//Chinese Probe 
const byte PROBE[NUM_PROBES][8] = {{0x28, 0xE8, 0xE9, 0x18, 0x04, 0x00, 0x00, 0x05},{0x28, 0xF3, 0xDC, 0x0C, 0x04, 0x00, 0x00, 0xC6},{0x28, 0x1A, 0xCE, 0x0C, 0x04, 0x00, 0x00, 0x86},{0x28, 0x50, 0xDE, 0x71, 0x04, 0x00, 0x00, 0x75}};

const bool parasitic_mode = false; // Define if we are using parasitic power or not

Adafruit_RGBLCDShield lcd;
// RGB Shield needs 5 wires
// VSS and VDD
// i2c clock and data connect to A4 and A5 on Duo, Pin 2&3 on Leonardo
// i2c interrupt connects to digital pin 3 on Duo, pin 1 on Leonardo

//Arduino pins
OneWire  ds(7);  // One wire thermometers on pin 7
const uint8_t HLT_HEATER_1=12; // Heater coil 1 in HLT
const uint8_t HLT_HEATER_2=11; // Heater coil 2 in HLT
const uint8_t HERMS_HEATER=10; // Heater coil in HERMS recirculator
const uint8_t ATX_POWER=4; //Controls whether the ATX power supply is active

//PID Settings
const int PIDWindowSize = 10;
int mash_pid_window = 0;
//Define Variables we'll be connecting to
double Strike_temp=87.0; //Target temperature for HLT water
double Mash_target=65.5; // Target mash temp inside mash tun
double HLT_temp; 
double HERMS_out_temp;
double mash_temp, mash_temp2; // in Celsius as this is default for DS1820
double mash_pid_output; // Hardwire at 50% for test
//Direct Kettle values
//double pid_kd = 1;
//double pid_ki = 0.25;
//double pid_kp = 0.25;
// HERMS Setup
double pid_kd = 1;
double pid_ki = 1;
double pid_kp = 100;

//Specify the links and initial tuning parameters
PID mashPID(&HERMS_out_temp, &mash_pid_output, &Mash_target,pid_kd,pid_ki,pid_kd, DIRECT);

enum DISPLAY_SCREEN{ROOT, PREHEAT, MASH, TEMP, SETTINGS};  // Each screen must be defined here

volatile boolean buttonPress=false;// Indicates if we have received an
                                   // Interupt to tell us a button needs read
boolean liquor_preheat = false; // Are we in preheat mode for the HLT
boolean herms_recirc = false; // Are we running the HERMS recirculation

boolean HLT_heater_ON=false; //True when the HLT heater(s) are on
boolean herms_active = false; // Track whether the herms heater is ON

unsigned long mash_start_time = 0; // Number of seconds when mash started

DISPLAY_SCREEN display=ROOT; // The currently displayed screen
boolean display_refresh=true; // Set to true when the display should be refreshed

boolean change_setting=false; // defines whether we are navigating the settings menu or changing values
uint8_t current_setting = 0; // which setting are we changing or highlighting 
const uint8_t NUMBER_SETTINGS=2; // Number of values which can be set

//this controls the menu backend and the event generation
MenuBackend menu = MenuBackend(menuEventUse,menuEventChange);
MenuItem root = MenuItem("Root");
MenuItem preheat_liquor = MenuItem("Preheat");
MenuItem mash = MenuItem("Mash");
MenuItem temperatures = MenuItem("Temp");
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
  mash.addRight(temperatures);
  temperatures.addRight(settings);
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
  } else if (used.item == settings ) {
    change_setting = !change_setting; //Toggle the settings mode
  }else{
    Serial.print("Enter pressed on unknown ");
  }
  // Whenever we press a button we want a refresh
  display_refresh=true;
}

// Called whenever the menu changes
void menuEventChange(MenuChangeEvent changed)
{
  const char * name = changed.to.getName();
  uint8_t len = strlen(name);
  uint8_t start = (20-len)/2;
  
  //Stop any cursor blink or underline
  lcd.noBlink();
  lcd.noCursor();
  
  lcdClearRow(0);
  
  lcd.setCursor(start,0);
  lcd.print(name);

  if (changed.to==preheat_liquor)
  {
    display=PREHEAT;
  } else if (changed.to==mash) {
    display=MASH;
  } else if (changed.to==root) {
    display=ROOT;
  } else if (changed.to==temperatures){
    display=TEMP;
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
    lcd.print(current,2);
    
    lcd.setCursor(0,2);
    lcd.print("Target Temp  = ");
    lcd.print(target,2);

    lcd.setCursor(7,3);

    if(liquor_preheat && display==PREHEAT) {
        lcd.print("  ON  ");
    } else if (herms_recirc && display==MASH){
        display_mash_timer();
    } else {
        lcd.print("  OFF  ");
    }
}

void display_mash_timer()
{
  uint8_t mins = mash_mins();
  uint8_t secs = mash_secs();
  
  if (mins>99)
    lcd.setCursor(7,3);
  else if (mins>9)
    lcd.setCursor(8,3);
  else
    lcd.setCursor(9,3);
    
  lcd.print(mins);
  lcd.print(":");
  
  if (secs<10)
    lcd.print("0");
    
  lcd.print(secs);
}

uint8_t mash_mins() {
  unsigned long current_time = millis();
  
  return ((current_time-mash_start_time)/60000);
}

uint8_t mash_secs() {
  unsigned long current_time = millis();
  unsigned long elapsed_secs = ((current_time-mash_start_time)/1000);
    
  return (elapsed_secs%60);
}

void display_settings() {
  lcd.noBlink();
  lcd.noCursor();
  
  lcdClearRow(1);
  lcd.setCursor(0,1);
  lcd.print("HLT target =   ");
  lcd.print(Strike_temp);
  
  lcdClearRow(2);
  lcd.setCursor(0,2);
  lcd.print("Mash temp  =   ");
  lcd.print(Mash_target);
  
  lcd.setCursor(14,current_setting+1);
  
  if (change_setting)
    lcd.blink();
  else
    lcd.cursor();
}

void display_all_temperatures() {
  lcdClearRow(1);
  lcd.setCursor(0,1);
  lcd.print("T1= ");
  lcd.print(HLT_temp);
  
  lcd.setCursor(10,1);
  lcd.print("T2= ");
  lcd.print(HERMS_out_temp);
  
  lcdClearRow(2);
  lcd.setCursor(0,2);
  lcd.print("T3= ");
  lcd.print(mash_temp);
  
  lcd.setCursor(10,2);
  lcd.print("T4= ");
  lcd.print(mash_temp2);
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
  
  output("Beginning setup",0);

  //Setup the pins
  pinMode(HLT_HEATER_1, OUTPUT);
  pinMode(HLT_HEATER_2, OUTPUT);
  pinMode(HERMS_HEATER, OUTPUT);
  pinMode(ATX_POWER, OUTPUT);
  
  digitalWrite(HLT_HEATER_1, LOW);
  digitalWrite(HLT_HEATER_2, LOW);
  digitalWrite(HERMS_HEATER, LOW);
  digitalWrite(ATX_POWER, HIGH);

  // initialise the LCD
  lcd = Adafruit_RGBLCDShield();
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);

  //Catch the external interrupt from the button input
  // 1 for Pin3 on Duo, 2 for Pin0 on Leonardo
  attachInterrupt(2,Button_Pressed,FALLING);
  //This signal is active low, so HIGH-to-LOW when interrupt

  //Tell the MCP23017 to start making interrupts
  lcd.enableButtonInterrupt();

  //Clear the interrupt if it was present
  while (!digitalRead(0)) {
    lcd.readButtons();
  }
  
  //tell the PID to range between 0 and the full window size
  mashPID.SetOutputLimits(0, PIDWindowSize);

  //turn the PID on
  mashPID.SetMode(AUTOMATIC);

  menuSetup();
  
  output("Setup Complete",0);
}

void display_root()
{
  lcdClearRow(1);
  lcdClearRow(2);
  
  lcd.setCursor(2,1);
  lcd.print("Brew Controller");
  lcd.setCursor(9,2);
  lcd.print(VERSION);
}

void left_pressed() {
  menu.moveLeft();
}

void right_pressed() {
  menu.moveRight();
}
void up_pressed() {
  
  if (menu.getCurrent() == settings)
  {
    if (change_setting) {
      if (current_setting==0) Strike_temp += 0.25;
      else if (current_setting==1) Mash_target += 0.25;
    } else {
      current_setting = (current_setting + 1) % NUMBER_SETTINGS;
    }
    display_refresh = true;
  } else {
    menu.moveUp();
    Serial.println("Menu UP");
  }
}
void down_pressed() {
  if (menu.getCurrent() == settings )
  {
    if (change_setting) {
      if (current_setting==0) {
        if (Strike_temp>0.25) Strike_temp -= 0.25;
      } else if (current_setting==1) {
        if (Mash_target > 0.25 ) Mash_target -= 0.25;
      }
    } else
    {
      if (current_setting ==0) {
        current_setting=NUMBER_SETTINGS-1;
      } else {
        current_setting -=1;
      }      
      Serial.println(current_setting);
    }
    display_refresh = true;
  } else {
    menu.moveDown();
    Serial.println("Menu DOWN");
  }
}
void enter_pressed() {
  Serial.println("Enter");
  menu.use();
}

void lcdClearRow( const uint8_t row )
{
    lcd.setCursor(0,row);
    lcd.print("                    ");
}

// Allow a manual override of whether heaters are used or not
void toggle_preheat_liquor()
{
  liquor_preheat = !liquor_preheat;
  Serial.println("Toggle preheat mode");
  toggle_HLT_Heater();
}

// Allow a manual override of whether heaters are used or not
void toggle_herms_recirc()
{
  herms_recirc=!herms_recirc;
  digitalWrite(ATX_POWER, herms_recirc ? LOW : HIGH);

  Serial.println("Toggle HERMS mode");
  
  if (herms_recirc) mash_start_time = millis();
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
      //Serial.println(addr[i]);
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
  byte num_probes_found = 0;
    
  while ( ds.search(addr)) {
    num_probes_found++;

    if (OneWire::crc8(addr, 7) == addr[7]) {
      probe = identifyProbe(addr);
      
      if (serial_output_temp) {
        Serial.print("Probe:"); Serial.print(probe); Serial.print(" ");
      }

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
        
        if (serial_output_temp) {
          Serial.print("  Temperature = ");
          Serial.print(celsius);
          Serial.println(" Celsius, ");
        }

        switch(probe) {
          case 0:
            if (celsius!=HLT_temp && (display==PREHEAT || display==TEMP)) display_refresh=true;
            HLT_temp = celsius; break;
          case 1:
            if (celsius!=HERMS_out_temp && (display==MASH || display==TEMP)) display_refresh=true;
            HERMS_out_temp = celsius; 
            mashPID.Compute();
            serial_mash_output();
            break;
          case 2:
            if (celsius!=mash_temp && (display==MASH|| display==TEMP)) display_refresh=true;
            mash_temp = celsius; 
            break;
          case 3:
            if (celsius!=mash_temp2 && (display==MASH|| display==TEMP)) display_refresh=true;
            mash_temp2 = celsius; 
            break;        
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
  if (serial_output_temp){
    output("Found probes",num_probes_found);  
  }
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
  mash_pid_window ++;
  
  if (mash_pid_window > PIDWindowSize) mash_pid_window = 0;
  
    if(mash_pid_output > mash_pid_window) {
      if (HLT_heater_ON) {
        //Disable 2nd boil element to prevent overload
        digitalWrite (HLT_HEATER_2, LOW);
      }
      herms_active=true;
      digitalWrite(HERMS_HEATER,HIGH);
    } else { 
      digitalWrite(HERMS_HEATER,LOW);
      herms_active=false;
      if (HLT_heater_ON) {
         //Re-enable 2nd boil element in case it was disabled
        digitalWrite (HLT_HEATER_2, HIGH);     
      }
    }
    
    if (!mash_output_as_csv){ 
      output("PID Window", mash_pid_window);
      output("PID out", mash_pid_output);
    }
}


void output(const char * title, double value)
{
  Serial.print(title);Serial.print(" : ");Serial.println(value);
}

void toggle_HLT_Heater()
{
    HLT_heater_ON = ! HLT_heater_ON;
    Serial.print("Toggle HLT: ");
    Serial.println(HLT_heater_ON);
    digitalWrite(HLT_HEATER_1, HLT_heater_ON ? HIGH : LOW );
    if (!herms_active) {
      digitalWrite(HLT_HEATER_2, HLT_heater_ON ? HIGH : LOW);
    }
}

void updateDisplay ()
{
  lcd.setCursor(0,3);
  lcd.print("<");
  lcd.print(prev_item);
  
  if (menu.getCurrent() == settings || menu.getCurrent() == root )
  {
    lcd.setCursor(5,3);
    lcd.print("          ");
  }
  
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
    case TEMP:
      display_all_temperatures();
    break;
    case SETTINGS:
      display_settings();
    break;
  }
  
 display_refresh=false;
}

void serial_mash_output() {
  double time=(millis()-mash_start_time);
  time=time/1000;
  
  if (mash_output_as_csv)
  {
    Serial.print(time);
    Serial.print(",");
    Serial.println(HERMS_out_temp);
  } else {
    Serial.print("Time :" );
    Serial.println(time);
    
    Serial.print("Temp : ");
    Serial.println(HERMS_out_temp);
    
    Serial.print("Target : ");
    Serial.println(Mash_target);
    
    Serial.println("");
  }
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
      
      if(herms_recirc && display==MASH) {
        display_mash_timer();
      }
  }    

}

