/*********************

**********************/

// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <MenuBackend.h>

const char VERSION[6] ="0.1";

Adafruit_RGBLCDShield lcd;
// RGB Shield needs 5 wires
// VSS and VDD
// i2c clock and data connect to A4 and A5 on Arduino
// i2c interrupt connects to digital pin 3 on arduino

volatile boolean buttonPress=false;
boolean liquor_preheat = false;

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
  liquor_preheat = !liqour_preheat
  
  if (liqour_preheat) {
    lcdBottomRow("ON");
  } else {
    lcdBottomRow("OFF");
  }
}
