#define MOTOR_X_STEP_PIN 45
#define MOTOR_Y_STEP_PIN 44
#define MOTOR_X_DIR_PIN 52
#define MOTOR_Y_DIR_PIN 46
#define ENCODER_X_BUTTON 40
#define ENCODER_Y_BUTTON 38
#define SPEED_BUTTON 36
#define MOVETO_BUTTON 42
#define ENCODER_X_PIN_1 3
#define ENCODER_X_PIN_2 5
#define ENCODER_Y_PIN_1 2
#define ENCODER_Y_PIN_2 4
#define LCD_ADDRESS 0x27

#include <Arduino.h>
#include <FlexyStepper.h>
#include "EEPROMStore.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <math.h>
#include <coordinates.h>

//###################################################################
                    String VERSION = "1.5.0";
//###################################################################

struct timer{
  long startTime;
  long stopTime;
};
struct position{
  double posX;
  double posY;
};
struct movement{
  double speed;
  double acceleration;
};
struct positionSteps{
  long stepsX;
  long stepsY;
};
struct movementSteps{
  double stepsSpeed;
  double stepsAccerleration;
};
//EEPROM Storage
struct Stettings{
  int INVERT_X;
  int INVERT_Y;
  int MICROSTEPPING;
  int STEPS_PER_ROTATION;
  double MM_PER_ROTATION;
  double STANDARDACCELERATION;
  double STANDARDSPEED;
  double BACKLASH_X;
  double BACKLASH_Y;
  position posLib[10];

  void Reset(){
    /////////DEFAULT VALUES//////////
    //set to -1 to invert axis
    INVERT_X = -1;
    INVERT_Y = -1;
    //Steppingfactors
    MICROSTEPPING = 16;
    STEPS_PER_ROTATION = 200;
    MM_PER_ROTATION = 1;
    //Acceleration in mm^2/s
    STANDARDACCELERATION = 2;
    //speed in mm/s
    STANDARDSPEED = 2.0;
    //backlash
    BACKLASH_X = 0.07;
    BACKLASH_Y = 0.01;
    //array of previousPositions posLib[0] = current position
    for(int i = 0; i < 10; i++){
      posLib[i].posX = 0.0;
      posLib[i].posY = 0.0;
    }
  }
};
EEPROMStore<Stettings> config;
FlexyStepper stepperX;
FlexyStepper stepperY;
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);
Encoder xEnc(ENCODER_X_PIN_1, ENCODER_X_PIN_2);
Encoder yEnc(ENCODER_Y_PIN_1, ENCODER_Y_PIN_2);

int ENABLEDISPLAY = 1;

int INVERT_X;
int INVERT_Y;
int MICROSTEPPING;
int STEPS_PER_ROTATION;
int HOLDTIME = 750;
double MM_PER_ROTATION;
double STANDARDACCELERATION;
double STANDARDSPEED;
double BACKLASH_X;
double BACKLASH_Y;
position posLib[10];
position targetPosition; //position currently being set
movement currentMovement; //current movement
timer speedTimer;
timer moveTimer;
timer xencTimer;
timer yencTimer;
int speedFlag;
int moveFlag;
int xencFlag;
int yencFlag;
int oldXEncoderPos;
int oldYEncoderPos;

void initialize();
void initSteppers();
void initIO();
void initVars();
void initDisplay();
position addBacklash();
void moveToPosition(position pos, movement mov);
int managePosLib(position newPosition);
positionSteps convertPositionToSteps(double differenceX, double differenceY);
movementSteps convertMovementToSteps(movement mov);
void initConfig();
void setConfig();
void restoreDefaultValues();
void moveXYWithCoordination(long stepsX, long stepsY, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond);
void processInputs();
void zeroXValues();
void zeroYValues();
void setTargetPositionX();
void setTargetPositionY();
void moveToTargetPosition();
void moveToPreviousPosition();
void highlightSpeedChar(long value);
void highlightChar(String axis, long value);
String generatePositionDisplayString(String axis, double displayVar);
String generateSpeedDisplayString(double displayVar);
void updateDisplay();
void setMoveSpeed();
int openAdvancedMenu();
void startMode(int mode);
int selectSteps();
double selectCutWidth();
int selectOverlap();
void startOsz();
void startPlane();
void startCut();
void back();
void about();
void editEEPROM();
void startCircle();
void moveCircle(int direction, position anchor, position target, movement mov, double degrees);

void setup() {
  restoreDefaultValues();
  initialize();
}

void loop() {
  processInputs();
  updateDisplay();
}

String menuItems[] = {"  Oszillieren   ", "     Planen     ", "   Ausschnitt   ","      Kreis     ", " Zum Hauptmenue ", "     EEPROM     ", "      Ueber     "};
int numberOfMenuItems = 7;
String eepromItems[] = {"    INVERT_X    ", "    INVERT_Y    ", "   BACKLASH_X   ", "   BACKLASH_Y   ", "  ACCELERATION  ","     SPEED      ", "    HOLDTIME    ", " MICROSTEPPING  ", "STEPSPERROTATION", " MM_PER_ROTATION"};
int numberOfeepromItems = 10;

int openAdvancedMenu(){
  int currentItem = 0;
  lcd.setCursor(0, 0);
  lcd.print("Erweitertes Menu");
  lcd.setCursor(0, 1);
  lcd.print(menuItems[currentItem]);
  while(digitalRead(MOVETO_BUTTON) != 1){
    if(digitalRead(ENCODER_X_BUTTON) == 1){
      if(currentItem == numberOfMenuItems - 1){
        currentItem = 0;
      }else{
        currentItem++;
      }
      lcd.setCursor(0, 1);
      lcd.print(menuItems[currentItem]);
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delay(50);
      }
    }
  }
  return currentItem;
}
void startMode(int mode){
  if(mode == 0){
    startOsz();
  }else if(mode == 1){
    startPlane();
  }else if(mode == 2){
    startCut();
  }else if(mode == 3){
    startCircle();
  }else if(mode == 4){
    back();
  }else if(mode == 5){
    editEEPROM();
  }else if(mode == 6){
    about();
  }
}
int selectSteps(){
  lcd.setCursor(0, 1);
  lcd.print("Anzahl:      0  ");
  int xEncoderStartPos = oldXEncoderPos;
  int Steps = 0;
  while(digitalRead(MOVETO_BUTTON) == 1){
    delay(50);
  }
  while(digitalRead(MOVETO_BUTTON) != 1){
    if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
      oldXEncoderPos = xEnc.read() / 4;
      Steps = oldXEncoderPos - xEncoderStartPos;
      if(Steps < 0){
        xEncoderStartPos += 1;
        Steps = oldXEncoderPos - xEncoderStartPos;
      }
      lcd.setCursor(7, 1);
      if(Steps > 99){
        lcd.print("    " + String(Steps) + "  ");
      }else if(Steps > 9){
        lcd.print("     " + String(Steps) + "  ");
      }else if(Steps > 0){
        lcd.print("      " + String(Steps) + "  ");
      }
    }
  }
  return Steps;
}
double selectCutWidth(){
  double cutWidth = 0;
  lcd.setCursor(0, 1);
  lcd.print("Fraser:     0.0  ");
  int xEncoderStartPos = oldXEncoderPos;
  int Steps = 0;
  while(digitalRead(MOVETO_BUTTON) == 1){
    delay(50);
  }
  while(digitalRead(MOVETO_BUTTON) != 1){
    if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
      oldXEncoderPos = xEnc.read() / 4;
      Steps = oldXEncoderPos - xEncoderStartPos;
      if(Steps < 0){
        xEncoderStartPos += 1;
        Steps = oldXEncoderPos - xEncoderStartPos;
      }
      lcd.setCursor(10, 1);
      cutWidth = Steps / 10.0;
      if(Steps > 99){
        lcd.print(" " + String(cutWidth, 1) + "  ");
      }else if(Steps > 9){
        lcd.print("  " + String(cutWidth, 1) + "  ");
      }else if(Steps > 0){
        lcd.print("  " + String(cutWidth, 1) + "  ");
      }
    }
  }
  return cutWidth;}
int selectOverlap(){
  lcd.setCursor(0, 1);
  lcd.print("Uberlappen:  0% ");
  int xEncoderStartPos = oldXEncoderPos;
  int Steps = 0;
  while(digitalRead(MOVETO_BUTTON) == 1){
    delay(50);
  }
  while(digitalRead(MOVETO_BUTTON) != 1){
    if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
      oldXEncoderPos = xEnc.read() / 4;
      Steps = oldXEncoderPos - xEncoderStartPos;
      if(Steps < 0){
        xEncoderStartPos += 1;
        Steps = oldXEncoderPos - xEncoderStartPos;
      }else if(Steps >= 100){
        xEncoderStartPos -= 1;
        Steps = oldXEncoderPos - xEncoderStartPos;
      }
      lcd.setCursor(11, 1);
      if(Steps > 99){
        lcd.print("" + String(Steps) + "% ");
      }else if(Steps > 9){
        lcd.print(" " + String(Steps) + "% ");
      }else if(Steps > 0){
        lcd.print("  " + String(Steps) + "% ");
      }
    }
  }
  return Steps;}

void startOsz(){
  lcd.setCursor(0, 0);
  lcd.print("  Oszillieren:   ");
  int Steps = selectSteps();
  for(int i = Steps; i > 0; i--){
    lcd.setCursor(0, 0);
    lcd.print("  Verbleibend:  ");
    lcd.setCursor(0, 1);
    if(i > 99){
      lcd.print("      " + String(i) + "      ");
    }else if(i > 9){
      lcd.print("       " + String(i) + "      ");
    }else if(i > 0){
      lcd.print("        " + String(i) + "      ");
    }
    delay(500);
    moveToPreviousPosition();
  }
}
void startCut(){
  double startPosX;
  double startPosY;
  double endPosX;
  double endPosY;
  if(posLib[0].posX < posLib[1].posX){
    startPosX = posLib[0].posX;
    endPosX = posLib[1].posX;
  }else{
    startPosX = posLib[1].posX;
    endPosX = posLib[0].posX;
  }
  if(posLib[0].posY < posLib[1].posY){
    startPosY = posLib[0].posY;
    endPosY = posLib[1].posY;
  }else{
    startPosY = posLib[1].posY;
    endPosY = posLib[0].posY;
  }
  lcd.setCursor(0, 0);
  lcd.print("  Ausschnitt:   ");
  int Steps = selectSteps();
  position drivePosition;
  drivePosition.posX = startPosX;
  drivePosition.posY = startPosY;
  moveToPosition(drivePosition, currentMovement);
  for(int i = Steps; i > 0; i--){
    lcd.setCursor(0, 0);
    lcd.print("  Verbleibend:  ");
    lcd.setCursor(0, 1);
    if(i > 99){
      lcd.print("      " + String(i) + "      ");
    }else if(i > 9){
      lcd.print("       " + String(i) + "      ");
    }else if(i > 0){
      lcd.print("        " + String(i) + "      ");
    }
    delay(500);
    drivePosition.posX = startPosX;
    drivePosition.posY = endPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = endPosX;
    drivePosition.posY = endPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = endPosX;
    drivePosition.posY = startPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = startPosX;
    drivePosition.posY = startPosY;
    moveToPosition(drivePosition, currentMovement);
  }
}
void startPlane(){
  double startPosX;
  double startPosY;
  double endPosX;
  double endPosY;
  if(posLib[0].posX < posLib[1].posX){
    startPosX = posLib[0].posX;
    endPosX = posLib[1].posX;
  }else{
    startPosX = posLib[1].posX;
    endPosX = posLib[0].posX;
  }
  if(posLib[0].posY < posLib[1].posY){
    startPosY = posLib[0].posY;
    endPosY = posLib[1].posY;
  }else{
    startPosY = posLib[1].posY;
    endPosY = posLib[0].posY;
  }
  lcd.setCursor(0, 0);
  lcd.print("     Planen:    ");
  int Steps = selectSteps();
  double drillwidth = selectCutWidth();
  int overlap = selectOverlap();
  position drivePosition;
  for(int i = Steps; i > 0; i--){
    lcd.setCursor(0, 0);
    lcd.print("  Verbleibend:  ");
    lcd.setCursor(0, 1);
    if(i > 99){
      lcd.print("      " + String(i) + "      ");
    }else if(i > 9){
      lcd.print("       " + String(i) + "      ");
    }else if(i > 0){
      lcd.print("        " + String(i) + "      ");
    }
    delay(500);
    drivePosition.posX = startPosX;
    drivePosition.posY = startPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = startPosX;
    drivePosition.posY = endPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = endPosX;
    drivePosition.posY = endPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = endPosX;
    drivePosition.posY = startPosY;
    moveToPosition(drivePosition, currentMovement);
    delay(200);
    drivePosition.posX = startPosX;
    drivePosition.posY = startPosY;
    moveToPosition(drivePosition, currentMovement);

    /////////////////////////////////////////////////////////////////
    //run from startpos - (cutwidth - (overlap / 100))
    double upperBounds = startPosY + (drillwidth / 2);
    double lowerBounds = endPosY - (drillwidth / 2);
    double rightBounds = startPosX + (drillwidth / 2);
    drivePosition.posX = rightBounds;
    drivePosition.posY = startPosY;
    moveToPosition(drivePosition, currentMovement);
    double realCutWidth = drillwidth - (2 * (overlap / 100.0));
    double travelDistanceX = endPosX - startPosX - drillwidth;
    int cuts = (travelDistanceX / realCutWidth) + 1;
    for(int i = 0; i < cuts; i++){
      ////// MOVE UP / DOWN //////
      // X = i * realCutWidth + rightBounds;
      // if cuts % 2 = 0 =>  Y = lowerBounds;
      // else => Y = upperBounds;
      ////// MOVE LEFT //////
      // if( i < cuts - 1) => X = (i + 1) * realCutWidth + rightBounds;
      // if cuts % 2 = 0 =>  Y = lowerBounds;
      // else => Y = upperBounds;
      drivePosition.posX = i * realCutWidth + rightBounds;
      if(i % 2 == 0){
        drivePosition.posY = lowerBounds;
      }else{
        drivePosition.posY = upperBounds;
      }
      moveToPosition(drivePosition, currentMovement);
      if(i < (cuts - 1)){
        drivePosition.posX = (i + 1) * realCutWidth + rightBounds;
        if(i % 2 == 0){
          drivePosition.posY = lowerBounds;
        }else{
          drivePosition.posY = upperBounds;
        }
        moveToPosition(drivePosition, currentMovement);
      }
    }
  }
}
void startCircle(){
  lcd.setCursor(0, 0);
  lcd.print("    Richtung:   ");
  int value = 1;
  lcd.setCursor(0, 1);
  lcd.print("     rechts     ");
  delay(HOLDTIME);
  while(digitalRead(MOVETO_BUTTON) != 1){
    if(digitalRead(ENCODER_X_BUTTON) == 1){
      lcd.setCursor(0, 1);
      if(value == 0){
        value = 1;
        lcd.print("     rechts     ");
      }else{
        value = 0;
        lcd.print("      links     ");
      }
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delay(50);
      }
    }
  }
  lcd.setCursor(0, 0);
  lcd.print("   Auswahl des  ");
  lcd.setCursor(0, 1);
  lcd.print("     Ankers     ");
  delay(HOLDTIME);
  position target = targetPosition;
  setTargetPositionX();
  setTargetPositionY();
  position anchor;
  anchor.posX = targetPosition.posX;
  anchor.posY = targetPosition.posY;
  targetPosition.posY = target.posY;
  lcd.setCursor(0, 0);
  lcd.print("   Auswahl des  ");
  lcd.setCursor(0, 1);
  lcd.print("   Zielwinkels  ");
  delay(HOLDTIME);
  targetPosition.posX = 0;
  setTargetPositionX();
  double degrees = targetPosition.posX * M_PI / 180;
  targetPosition.posX = target.posX;
  moveCircle(value, anchor, {0,0}, currentMovement, degrees);
}
void moveCircle(int direction, position anchor, position target, movement mov, double degrees){
  int Schritte = 360;
  Coordinates point = Coordinates();
  Coordinates point2 = Coordinates();
  point.fromCartesian(posLib[0].posX - anchor.posX, posLib[0].posY - anchor.posY);
  double radius = sqrt(pow(anchor.posX, 2) + pow(anchor.posY, 2));
  if(degrees != 0){
    point2.fromPolar(radius, point.getAngle() + degrees);
  }else{
    point2.fromCartesian(target.posX - anchor.posX, target.posY - anchor.posY);
  }
  anchor.posX = posLib[0].posX + anchor.posX;
  anchor.posY = posLib[0].posY + anchor.posY;
  ENABLEDISPLAY = 0;
  position allpositions[360];
  int i = 0;
  if(direction == 0){
    while(point.getAngle() > (point2.getAngle() + 2*((2.0*M_PI)/Schritte))){
      point.fromCartesian(posLib[0].posX - anchor.posX, posLib[0].posY - anchor.posY);
      double angle = point.getAngle();
      point.fromPolar(radius, angle + 2.0*M_PI/Schritte);
      allpositions[i].posX = point.getX() + anchor.posX;
      allpositions[i].posY = point.getY() + anchor.posY;
      i++;
    }
  }else{
    while(point.getAngle() > (point2.getAngle() + 2*((2.0*M_PI)/Schritte))){
      point.fromCartesian(posLib[0].posX - anchor.posX, posLib[0].posY - anchor.posY);
      double angle = point.getAngle();
      point.fromPolar(radius, angle - 2.0*M_PI/Schritte);
      allpositions[i].posX = point.getX() + anchor.posX;
      allpositions[i].posY = point.getY() + anchor.posY;
      i++;
    }
  }
  for(int i = 0; i < 360; i++){
    targetPosition.posX = allpositions[i].posX;
    targetPosition.posY = allpositions[i].posY;
    moveToTargetPosition();
  }
  ENABLEDISPLAY = 1;
  targetPosition.posX = point2.getX();
  targetPosition.posY = point2.getY();
  moveToTargetPosition();
}
void back(){
  delay(200);
}
void editEEPROM(){
  int currentItem = 0;
  lcd.setCursor(0, 0);
  lcd.print("  EEPROM Werte  ");
  lcd.setCursor(0, 1);
  lcd.print(eepromItems[currentItem]);
  while(digitalRead(MOVETO_BUTTON) == 1){
    delay(50);
  }
  while(digitalRead(MOVETO_BUTTON) != 1){
    if(digitalRead(ENCODER_X_BUTTON) == 1){
      if(currentItem == numberOfeepromItems - 1){
        currentItem = 0;
      }else{
        currentItem++;
      }
      lcd.setCursor(0, 1);
      lcd.print(eepromItems[currentItem]);
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delay(50);
       }
    }
  }
  while(digitalRead(MOVETO_BUTTON) == 1){
    delay(50);
  }
  //setting selected
  if(currentItem == 0){
    lcd.setCursor(0, 0);
    lcd.print(" X Invertieren  ");
    int value;
    if(INVERT_X == 1){
      value = 0;
    }else{
      value = 1;
    }
    lcd.setCursor(0, 1);
    lcd.print("        " + String(value) + "       ");
    while(digitalRead(MOVETO_BUTTON) != 1){
      if(digitalRead(ENCODER_X_BUTTON) == 1){
        if(value == 0){
          value = 1;
        }else{
          value = 0;
        }
        lcd.setCursor(0, 1);
        lcd.print("        " + String(value) + "       ");
        while(digitalRead(ENCODER_X_BUTTON) == 1){
          delay(50);
        }
      }
    }
    if(value == 0){
      INVERT_X = 1;
    }else{
      INVERT_X = -1;
    }
    config.Data.INVERT_X = INVERT_X;
    setConfig();
  }
  else if(currentItem == 1){
    lcd.setCursor(0, 0);
    lcd.print(" Y Invertieren  ");
    int value;
    if(INVERT_Y == 1){
      value = 0;
    }else{
      value = 1;
    }
    lcd.setCursor(0, 1);
    lcd.print("        " + String(value) + "       ");
    while(digitalRead(MOVETO_BUTTON) != 1){
      if(digitalRead(ENCODER_X_BUTTON) == 1){
        if(value == 0){
          value = 1;
        }else{
          value = 0;
        }
        lcd.setCursor(0, 1);
        lcd.print("        " + String(value) + "       ");
        while(digitalRead(ENCODER_X_BUTTON) == 1){
          delay(50);
        }
      }
    }
    if(value == 0){
      INVERT_Y = 1;
    }else{
      INVERT_Y = -1;
    }
    config.Data.INVERT_Y = INVERT_Y;
    setConfig();
  }
  else if(currentItem == 2){
    lcd.setCursor(0, 0);
    lcd.print(" Umkehrspiel X  ");
    double finalValue = BACKLASH_X;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    BACKLASH_X = finalValue;
    config.Data.BACKLASH_X = BACKLASH_X;
    setConfig();
  }
  else if(currentItem == 3){
    lcd.setCursor(0, 0);
    lcd.print(" Umkehrspiel Y  ");
    double finalValue = BACKLASH_Y;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    BACKLASH_Y = finalValue;
    config.Data.BACKLASH_X = BACKLASH_Y;
    setConfig();
  }
  else if(currentItem == 4){
    lcd.setCursor(0, 0);
    lcd.print(" Beschleunigung ");
    double finalValue = STANDARDACCELERATION;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    STANDARDACCELERATION = finalValue;
    config.Data.STANDARDACCELERATION = STANDARDACCELERATION;
    setConfig();
  }
  else if(currentItem == 5){
    lcd.setCursor(0, 0);
    lcd.print(" Geschwindigkeit");
    double finalValue = STANDARDSPEED;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    STANDARDSPEED = finalValue;
    config.Data.STANDARDSPEED = STANDARDSPEED;
    setConfig();
  }
  else if(currentItem == 6){
    lcd.setCursor(0, 0);
    lcd.print("    Haltezeit   ");
    double finalValue = HOLDTIME;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1000; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    HOLDTIME = finalValue;

    setConfig();
  }
  else if(currentItem == 7){
    lcd.setCursor(0, 0);
    lcd.print(" Microschritte  ");
    double finalValue = MICROSTEPPING;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1000; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    MICROSTEPPING = finalValue;
    config.Data.MICROSTEPPING = MICROSTEPPING;
    setConfig();
  }
  else if(currentItem == 8){
    lcd.setCursor(0, 0);
    lcd.print("SchritteProUmdre");
    double finalValue = STEPS_PER_ROTATION;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1000; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    STEPS_PER_ROTATION = finalValue;
    config.Data.STEPS_PER_ROTATION = STEPS_PER_ROTATION;
    setConfig();
  }
  else if(currentItem == 9){
    lcd.setCursor(0, 0);
    lcd.print("MM pro Umdrehung");
    double finalValue = MM_PER_ROTATION;
    lcd.setCursor(0,1);
    lcd.print(generatePositionDisplayString("Y", finalValue) + "  SET ");
    for(long value = 1000000; value > 1; value = value / 10){
      highlightChar("Y", value);
      int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
            xEncoderStartPos -= 1;
          }
          lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
    }
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("      DONE      ");
    delay(500);
    MM_PER_ROTATION = finalValue;
    config.Data.MM_PER_ROTATION = MM_PER_ROTATION;
    setConfig();
  }
}
void about(){
  lcd.setCursor(0, 0);
  lcd.print(" Version  " + VERSION + " ");
  lcd.setCursor(0, 1);
  lcd.print(" Aaron Beckmann ");
  while(digitalRead(MOVETO_BUTTON) == 1){
    delay(50);
  }
  while(digitalRead(MOVETO_BUTTON) != 1){
    delay(50);
  }
}

void processInputs(){
  //Speed Button
  if(digitalRead(SPEED_BUTTON) == 1 && speedFlag == 0){
    speedTimer.startTime = millis();
    speedFlag = 1;
  }else if(digitalRead(SPEED_BUTTON) == 0 && speedFlag == 1){
    speedTimer.stopTime = millis();
    if(speedTimer.stopTime - speedTimer.startTime > HOLDTIME){
      //HOLD
      startMode(openAdvancedMenu());
    }else{
      //PRESS
      setMoveSpeed();
    }
    speedFlag = 0;
  }
  //MoveTo Button
  if(digitalRead(MOVETO_BUTTON) == 1 && moveFlag == 0){
    moveTimer.startTime = millis();
    moveFlag = 1;
  }else if(digitalRead(MOVETO_BUTTON) == 0 && moveFlag == 1){
    moveTimer.stopTime = millis();
    if(moveTimer.stopTime - moveTimer.startTime > HOLDTIME){
      //HOLD
      moveToPreviousPosition();
    }else{
      //PRESS
      moveToTargetPosition();
    }
    moveFlag = 0;
  }
  //X Enc Button
  if(digitalRead(ENCODER_X_BUTTON) == 1 && xencFlag == 0){
    xencTimer.startTime = millis();
    xencFlag = 1;
  }else if(digitalRead(ENCODER_X_BUTTON) == 0 && xencFlag == 1){
    xencTimer.stopTime = millis();
    if(xencTimer.stopTime - xencTimer.startTime > HOLDTIME){
      //HOLD
      zeroXValues();
    }else{
      //PRESS
      setTargetPositionX();
    }
    xencFlag = 0;
  }
  //Y Enc Button
  if(digitalRead(ENCODER_Y_BUTTON) == 1 && yencFlag == 0){
    yencTimer.startTime = millis();
    yencFlag = 1;
  }else if(digitalRead(ENCODER_Y_BUTTON) == 0 && yencFlag == 1){
    yencTimer.stopTime = millis();
    if(yencTimer.stopTime - yencTimer.startTime > HOLDTIME){
      //HOLD
      zeroYValues();
    }else{
      //PRESS
      setTargetPositionY();
    }
    yencFlag = 0;
  }
}
void zeroXValues(){
  for(int i = 9; i >= 0; i--){
    posLib[i].posX = posLib[i].posX - posLib[0].posX;
  }
  targetPosition.posX = 0.0;
}
void zeroYValues(){
  for(int i = 9; i >= 0; i--){
    posLib[i].posY = posLib[i].posY - posLib[0].posY;
  }
  targetPosition.posY = 0.0;
}
void setTargetPositionX(){
  double finalValue = targetPosition.posX;
  lcd.setCursor(0,0);
  lcd.print(generatePositionDisplayString("X", targetPosition.posX) + "  SET ");
  for(long value = 1000000; value > 1; value = value / 10){
    highlightChar("X", value);
    int xEncoderStartPos = oldXEncoderPos;
    while(digitalRead(ENCODER_X_BUTTON) == 1){
      delayMicroseconds(50);
    }
    while(digitalRead(ENCODER_X_BUTTON) != 1){
      if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
        oldXEncoderPos = xEnc.read() / 4;
        lcd.setCursor(0, 0);
        if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 999.999){
          xEncoderStartPos += 1;
        }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < -999.999){
          xEncoderStartPos -= 1;
        }
        lcd.print(generatePositionDisplayString("X", finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + "  SET ");
      }
    }
    finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("      DONE      ");
  delay(500);
  targetPosition.posX = finalValue;
}
void setTargetPositionY(){
  double finalValue = targetPosition.posY;
  lcd.setCursor(0,1);
  lcd.print(generatePositionDisplayString("Y", targetPosition.posY) + "  SET ");
  for(long value = 1000000; value > 1; value = value / 10){
    highlightChar("Y", value);
    int yEncoderStartPos = oldYEncoderPos;
    while(digitalRead(ENCODER_Y_BUTTON) == 1){
      delayMicroseconds(50);
    }
    while(digitalRead(ENCODER_Y_BUTTON) != 1){
      if(oldYEncoderPos != (yEnc.read() / 4) && yEnc.read() % 4 == 0){
        oldYEncoderPos = yEnc.read() / 4;
        lcd.setCursor(0, 1);
        if(finalValue + value / 10000.0 * (oldYEncoderPos - yEncoderStartPos) > 999.999){
          yEncoderStartPos += 1;
        }else if(finalValue + value / 10000.0 * (oldYEncoderPos - yEncoderStartPos) < -999.999){
          yEncoderStartPos -= 1;
        }
        lcd.print(generatePositionDisplayString("Y", finalValue + value / 10000.0 * (oldYEncoderPos - yEncoderStartPos)) + "  SET ");
      }
    }
    finalValue += value / 10000.0 * (oldYEncoderPos - yEncoderStartPos);
  }
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("      DONE      ");
  delay(500);
  targetPosition.posY = finalValue;
}
void moveToTargetPosition(){
  moveToPosition(targetPosition, currentMovement);
}
void moveToPreviousPosition(){
  targetPosition.posX = posLib[1].posX;
  targetPosition.posY = posLib[1].posY;
  moveToTargetPosition();
}
void setMoveSpeed(){
  lcd.setCursor(0,0);
  lcd.print("SET MOTORSPEED: ");
  lcd.setCursor(0, 1);
  lcd.print("   " + generateSpeedDisplayString(currentMovement.speed) + " MM/S   ");
  delay(600);
  double finalValue = currentMovement.speed;
  for(long value = 10000; value > 1; value = value / 10){
    highlightSpeedChar(value);
    int xEncoderStartPos = oldXEncoderPos;
      while(digitalRead(ENCODER_X_BUTTON) == 1){
        delayMicroseconds(50);
      }
      while(digitalRead(ENCODER_X_BUTTON) != 1){
        if(oldXEncoderPos != (xEnc.read() / 4) && xEnc.read() % 4 == 0){
          oldXEncoderPos = xEnc.read() / 4;
          lcd.setCursor(0, 1);
          if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) > 9.999){
            xEncoderStartPos += 1;
          }else if(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos) < 0.000){
            xEncoderStartPos -= 1;
          }
          lcd.print("   " + generateSpeedDisplayString(finalValue + value / 10000.0 * (oldXEncoderPos - xEncoderStartPos)) + " MM/S   ");
        }
      }
      finalValue += value / 10000.0 * (oldXEncoderPos - xEncoderStartPos);
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("      DONE      ");
  delay(500);
  currentMovement.speed = finalValue;
}

void updateDisplay(){
  if(ENABLEDISPLAY == 1){
    lcd.setCursor(0, 0);
    lcd.print(generatePositionDisplayString("X", targetPosition.posX) + " " + generateSpeedDisplayString(currentMovement.speed));
    lcd.setCursor(0, 1);
    lcd.print(generatePositionDisplayString("Y", targetPosition.posY) + "  MM/S");
  }
}

void highlightSpeedChar(long value){
  lcd.setCursor(0, 0);
  if(value == 10000){
    lcd.print("   v              ");
  }
  if(value == 1000){
    lcd.print("     v            ");
  }
  if(value == 100){
    lcd.print("      v           ");
  }
  if(value == 10){
    lcd.print("       v          ");
  }
}
void highlightChar(String axis, long value){
  if(axis.equals("X")){
    lcd.setCursor(0,1);
    if(value == 1000000){
      lcd.print("   ^            ");
    }
    if(value == 100000){
      lcd.print("    ^           ");
    }
    if(value == 10000){
      lcd.print("     ^          ");
    }
    if(value == 1000){
      lcd.print("       ^        ");
    }
    if(value == 100){
      lcd.print("        ^       ");
    }
    if(value == 10){
      lcd.print("         ^      ");
    }
  }else if(axis.equals("Y")){
    lcd.setCursor(0,0);
    if(value == 1000000){
      lcd.print("   v            ");
    }
    if(value == 100000){
      lcd.print("    v           ");
    }
    if(value == 10000){
      lcd.print("     v          ");
    }
    if(value == 1000){
      lcd.print("       v        ");
    }
    if(value == 100){
      lcd.print("        v       ");
    }
    if(value == 10){
      lcd.print("         v      ");
    }
  }
}
String generatePositionDisplayString(String axis, double displayVar){
  if (displayVar >= 0){
    if(displayVar < 10){
      return axis + " +00" + String(displayVar, 3);
    }else if(displayVar < 100){
      return axis + " +0" + String(displayVar, 3);
    }else{
      return axis + " +" + String(displayVar, 3);
    }
  }else{
    if(displayVar > -10){
      return axis + " -00" + String(displayVar * -1, 3);
    }else if(displayVar > -100){
      return axis + " -0" + String(displayVar * -1, 3);
    }else{
      return axis + " -" + String(displayVar * -1, 3);
    }
  }
}
String generateSpeedDisplayString(double displayVar){
  return String(displayVar, 3);
}

void initialize(){
  initIO();
  initConfig();
  initSteppers();
  initVars();
  initDisplay();
}
void initSteppers(){
  stepperX.connectToPins(MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
  stepperY.connectToPins(MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
}
void initConfig(){
  config.Load();
  INVERT_X = config.Data.INVERT_X;
  INVERT_Y = config.Data.INVERT_Y;
  MICROSTEPPING = config.Data.MICROSTEPPING;
  STEPS_PER_ROTATION = config.Data.STEPS_PER_ROTATION;
  MM_PER_ROTATION = config.Data.MM_PER_ROTATION;
  STANDARDACCELERATION = config.Data.STANDARDACCELERATION;
  STANDARDSPEED = config.Data.STANDARDSPEED;
  BACKLASH_X = config.Data.BACKLASH_X;
  BACKLASH_Y = config.Data.BACKLASH_Y;

  posLib[10] = config.Data.posLib[10];
}
void initIO(){
  pinMode(ENCODER_X_BUTTON, INPUT);
  pinMode(ENCODER_Y_BUTTON, INPUT);
  pinMode(SPEED_BUTTON, INPUT);
  pinMode(MOVETO_BUTTON, INPUT);
}
void initVars(){
  currentMovement.acceleration = STANDARDACCELERATION;
  currentMovement.speed = STANDARDSPEED;
  targetPosition.posX = 0.0;
  targetPosition.posY = 0.0;
  speedFlag = 0;
  moveFlag = 0;
  xencFlag = 0;
  yencFlag = 0;
  oldXEncoderPos = 0;
  oldYEncoderPos = 0;
}
void initDisplay(){
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(generatePositionDisplayString("X", targetPosition.posX) + " " + generateSpeedDisplayString(currentMovement.speed));
  lcd.setCursor(0, 1);
  lcd.print(generatePositionDisplayString("Y", targetPosition.posY) + "  MM/S");
}

position addBacklash(){
  position backlash;
  backlash.posX = 0.0;
  backlash.posY = 0.0;
  double previousMovementX = posLib[2].posX - posLib[1].posX;
  double currentMovementX = posLib[1].posX - posLib[0].posX;
  double previousMovementY = posLib[2].posY - posLib[1].posY;
  double currentMovementY = posLib[1].posY - posLib[0].posY;
  if(previousMovementX < 0 && currentMovementX > 0){
    backlash.posX = -BACKLASH_X;
  }
  else if(previousMovementX > 0 && currentMovementX < 0){
    backlash.posX = BACKLASH_X;
  }
  if(previousMovementY < 0 && currentMovementY > 0){
    backlash.posY = -BACKLASH_Y;
  }
  else if(previousMovementY > 0 && currentMovementY < 0){
    backlash.posY = BACKLASH_Y;
  }
  return backlash;
}
void moveToPosition(position pos, movement mov){
  if(managePosLib(pos) == 0){
    position backlash = addBacklash();
    positionSteps steps = convertPositionToSteps(INVERT_X * (posLib[1].posX - posLib[0].posX + backlash.posX), INVERT_Y * (posLib[1].posY - posLib[0].posY + backlash.posY));
    movementSteps move = convertMovementToSteps(mov);
    if(ENABLEDISPLAY == 1){
      lcd.setCursor(0, 0);
      lcd.print("     MOVING     ");
      lcd.setCursor(0, 1);
      lcd.print("     ......     ");
    }
    moveXYWithCoordination(steps.stepsX, steps.stepsY, move.stepsSpeed, move.stepsAccerleration);
    updateDisplay();
  }
}

int managePosLib(position newPosition){
  if(newPosition.posX != posLib[0].posX || newPosition.posY != posLib[0].posY){
    for(int i = 8; i >= 0; i--){
      posLib[i + 1] = posLib[i];
    }
    posLib[0] = newPosition;
    return 0;
  }else{
    return 1;
  }
}
positionSteps convertPositionToSteps(double differenceX, double differenceY){
  positionSteps steps;
  steps.stepsX = MICROSTEPPING * STEPS_PER_ROTATION * (differenceX / MM_PER_ROTATION);
  steps.stepsY = MICROSTEPPING * STEPS_PER_ROTATION * (differenceY / MM_PER_ROTATION);
  return steps;
}
movementSteps convertMovementToSteps(movement mov){
  movementSteps move;
  move.stepsSpeed = (mov.speed / MM_PER_ROTATION) * MICROSTEPPING * STEPS_PER_ROTATION;
  move.stepsAccerleration = (mov.acceleration / MM_PER_ROTATION) * MICROSTEPPING * STEPS_PER_ROTATION;
  return move;
}

void setConfig(){
  config.Save();
}
void restoreDefaultValues(){
  config.Reset();
  config.Save();
}

void moveXYWithCoordination(long stepsX, long stepsY, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond){
  float speedInStepsPerSecond_X;
  float accelerationInStepsPerSecondPerSecond_X;
  float speedInStepsPerSecond_Y;
  float accelerationInStepsPerSecondPerSecond_Y;
  long absStepsX;
  long absStepsY;

  //
  // setup initial speed and acceleration values
  //
  speedInStepsPerSecond_X = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_X = accelerationInStepsPerSecondPerSecond;

  speedInStepsPerSecond_Y = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_Y = accelerationInStepsPerSecondPerSecond;


  //
  // determine how many steps each motor is moving
  //
  if (stepsX >= 0)
    absStepsX = stepsX;
  else
    absStepsX = -stepsX;

  if (stepsY >= 0)
    absStepsY = stepsY;
  else
    absStepsY = -stepsY;


  //
  // determine which motor is traveling the farthest, then slow down the
  // speed rates for the motor moving the shortest distance
  //
  if ((absStepsX > absStepsY) && (stepsX != 0))
  {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float) absStepsY / (float) absStepsX;
    speedInStepsPerSecond_Y = speedInStepsPerSecond_Y * scaler;
    accelerationInStepsPerSecondPerSecond_Y = accelerationInStepsPerSecondPerSecond_Y * scaler;
  }

  if ((absStepsY > absStepsX) && (stepsY != 0))
  {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float) absStepsX / (float) absStepsY;
    speedInStepsPerSecond_X = speedInStepsPerSecond_X * scaler;
    accelerationInStepsPerSecondPerSecond_X = accelerationInStepsPerSecondPerSecond_X * scaler;
  }


  //
  // setup the motion for the X motor
  //
  stepperX.setSpeedInStepsPerSecond(speedInStepsPerSecond_X);
  stepperX.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_X);
  stepperX.setTargetPositionRelativeInSteps(stepsX);


  //
  // setup the motion for the Y motor
  //
  stepperY.setSpeedInStepsPerSecond(speedInStepsPerSecond_Y);
  stepperY.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_Y);
  stepperY.setTargetPositionRelativeInSteps(stepsY);


  //
  // now execute the moves, looping until both motors have finished
  //
  while((!stepperX.motionComplete()) || (!stepperY.motionComplete()))
  {
    stepperX.processMovement();
    stepperY.processMovement();
  }
}
