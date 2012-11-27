/*********************

**********************/

// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

#define VERSION 0.1

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// RGB Shield needs 5 wires
// VSS and VDD
// i2c clock and data connect to A4 and A5 on Arduino
// i2c interrupt connects to digital pin 3 on arduino

volatile boolean buttonPress=false;

void setup() {
  // Debugging output
  Serial.begin(9600);
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
  
  startBrewConroller();
}

uint8_t i=0;
void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);
  
  if (buttonPress) {
    uint8_t buttons = lcd.readButtons();
    if (buttons) {
      lcd.clear();
      lcd.setCursor(0,0);
      if (buttons & BUTTON_UP) {
        lcd.print("UP ");
      }
      if (buttons & BUTTON_DOWN) {
        lcd.print("DOWN ");
      }
  
      if (buttons & BUTTON_LEFT) {
        lcd.print("LEFT ");
      }
      if (buttons & BUTTON_RIGHT) {
        lcd.print("RIGHT ");
      }
      if (buttons & BUTTON_SELECT) {
        lcd.print("SELECT ");
       }
    }
    
    buttonPress=false;
  }

}

void Button_Pressed() {
// Keep this as short as possible - because you are holding up the proc  
  Serial.println("INTERRUPT");
  
  buttonPress=true;

}

void startBrewConroller()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Brew control");
  lcd.setCursor(1,0);
  lcd.print(VERSION);
}
