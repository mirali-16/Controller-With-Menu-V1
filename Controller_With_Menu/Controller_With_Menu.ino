/*
 * Project Name => Controller With Menu
 * 
 * Designed by Mirali Tiryaki
 * instagram => https://www.instagram.com/mirali_tiryaki_/
 * Linkedin  => https://tr.linkedin.com/in/mirali-tiryaki-35ba27224
 * Mail => miralitiryaki1@gmail.com
 * 
 * 
 */
#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <ezButton.h>
#include <EEPROM.h>

ezButton button(5); 
LiquidCrystal_I2C lcd(0x27,20,4); 

String directionData = "Right";
String power = "OFF";
const int LONG_PRESS_TIME  = 1000;
unsigned long pressedTime  = 0;
int counter = 0;
int buttonState = 0;      
int yon;       
int clk_son;   
int clk_simdi; 
int clk = 3;   
int dt = 4;    
int buton = 5;
int speedCount = 10;
int lastState = LOW;
int btnState;
int currentState;
bool menuFlag = true;
bool isPressing = false;
bool isFirstPress = true;
bool isLongDetected = false;
bool longpress = false;
bool yonBilgileri = false;

  byte rightOk[8] = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};
void updateEncoder()
{
  clk_simdi = digitalRead(clk);        
  if (clk_simdi != clk_son) {           
    if (digitalRead(dt) == clk_simdi) {
      counter++;                          
      if(counter >= 3)
      {
        counter = 3;
      }
      yon = true;                      
    }
    else {                              
      counter--; 
       if(counter <= 0)
      {
        counter = 0;
      }
      yon = false;                      
    }
  }
  clk_son = clk_simdi;                   
}
int resetPin = 6;
void setup() {
  lcd.init();
  pinMode(buton, INPUT_PULLUP);
  pinMode(clk, INPUT_PULLUP);           
  pinMode(dt, INPUT_PULLUP);            
  pinMode(buton, INPUT_PULLUP);
  digitalWrite(resetPin, HIGH); 
  pinMode(resetPin, OUTPUT);     
  lcd.backlight();
  clk_son = digitalRead(clk);          
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(clk), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dt), updateEncoder, CHANGE);
  button.setDebounceTime(40);
  
    lcd.setCursor(0,0);
    lcd.print("    Designed By    ");
    lcd.setCursor(0,1);
    lcd.print("   Mirali Tiryaki");
    lcd.setCursor(0,2);
    lcd.print("  Motor Controller");
    lcd.setCursor(0,3);
    lcd.print("      Starting");
    delay(1000);
    lcd.setCursor(14,3);
    lcd.print(".");
    delay(1000);
    lcd.setCursor(15,3);
    lcd.print(".");
    delay(1000);
    lcd.setCursor(16,3);
    lcd.print(".");
    delay(4000);
    lcd.clear();
  
}
void longPressRead()
{
 currentState = digitalRead(buton);

  if(lastState == HIGH && currentState == LOW) {       
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  } else if(lastState == LOW && currentState == HIGH) {
    isPressing = false;
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      Serial.println("A long press is detected");
       if(counter == 0 and menuFlag == false)
      {
      lcd.clear();
      lcd.setCursor(3,1);
      lcd.print("KAYDEDILDI");
      delay(750);
      lcd.clear();
      menuFlag = true;
      }
      isLongDetected = true;
      longpress = true;
      delay(1000);
    }
  }
  lastState = currentState;
}

void loop()
{
    longPressRead();
    Serial.println(counter); 
    if(menuFlag == true)
    {
    lcd.setCursor(2,0);
    lcd.print("Demo Test");
    lcd.setCursor(2,1);
    lcd.print("Motor Control");
    lcd.setCursor(2,2);
    lcd.print("System Reboot");
    lcd.setCursor(2,3);
    lcd.print("Lcd Backlight");
    }
    
  button.loop(); 
  btnState = button.getState();
  
if(menuFlag == false)
{

      lcd.setCursor(14,2);
      lcd.print(directionData);
      lcd.setCursor(14,3);
      lcd.print(power);        
      lcd.setCursor(14,1);
      lcd.print(speedCount); 
}

  switch(counter){

case 0:
   
    lcd.setCursor(0,1);
    lcd.print(" ");
    lcd.setCursor(0,2);
    lcd.print(" ");
    lcd.setCursor(0,3);
    lcd.print(" ");
    lcd.createChar(0, rightOk);       
    lcd.setCursor(0,0);
    lcd.write(byte(0));
   
    break;    
  
case 1:
    lcd.setCursor(0,0);
    lcd.print(" ");
    lcd.setCursor(0,2);
    lcd.print(" ");
    lcd.setCursor(0,3);
    lcd.print(" ");
    lcd.setCursor(0,1);
    lcd.write(byte(0));
    if(button.isPressed())
    {
      menuFlag = false;
      lcd.setCursor(1,0);
      lcd.print("Motor Control Menu");
      lcd.setCursor(1,1);
      lcd.print("Motor speedCount :");
      lcd.setCursor(1,2);
      lcd.print("Direction   :");
      lcd.setCursor(1,3);
      lcd.print("Motor Power :");   
    
      if(digitalRead(buton) == LOW and counter == 1 and menuFlag == false )
      {  
        if( speedCount < 100)
        {
        speedCount = speedCount+10;     
        }
       
        else if (speedCount == 100)
        {
          speedCount = 0;
          lcd.setCursor(16,1);
          lcd.print(" ");
        }
        
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        lcd.setCursor(14,1);
        lcd.print(speedCount);
      }
      }
    break;      
       
case 2:
    lcd.setCursor(0,0);
    lcd.print(" ");
    lcd.setCursor(0,1);
    lcd.print(" ");
    lcd.setCursor(0,3);
    lcd.print(" ");
    lcd.setCursor(0,2);
    lcd.write(byte(0));
    buttonState = digitalRead(buton);
    if(buttonState == LOW and menuFlag == true)
    {
      digitalWrite(resetPin, LOW);
    }
  
if(menuFlag == false)
{
  if (buttonState == LOW) { 
    if (isFirstPress) {
     isFirstPress = false;
     lcd.setCursor(14,2);
     directionData = "Right";
     lcd.print(directionData);
    } else {
      lcd.setCursor(14,2);
      directionData = "Left ";
      lcd.print(directionData);
      isFirstPress = true;
    }
    delay(500); 
  }
}
 
    break;

case 3:
    lcd.setCursor(0,0);
    lcd.print(" ");
    lcd.setCursor(0,1);
    lcd.print(" ");
    lcd.setCursor(0,2);
    lcd.print(" ");
    lcd.setCursor(0,3);
    lcd.write(byte(0));
    
    buttonState = digitalRead(buton);
    if(counter == 3 and buttonState== LOW)
    {
      
    if (isFirstPress) {
      lcd.noBacklight(); 
      isFirstPress = false;
    } else {
      lcd.backlight(); 
      isFirstPress = true;
    }
    delay(500);  
  }

    
    if(menuFlag == false)
{

  if (buttonState == LOW) {  
    if (isFirstPress) {
      power = "ON ";
      isFirstPress = false;
    } else {
      power = "OFF";
      isFirstPress = true;
    }
    delay(500); 
  }
}
    break;
  }
}
