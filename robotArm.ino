#include <Servo.h> 
#include <Wire.h>

#include <NintendoExtensionCtrl.h>
Nunchuk nunchuck;

//#define DEBUG

//Change this value to change the servo speed
//bigger value = slower speed
uint16_t servoSpeed = 30;

/**********************************
 * joystick initialisation
**********************************/
//joystick pin definitions
int jsClawY = A2;
int jsClawX = A3;
int jsArmY = A6;
int jsArmX = A7;
int jsClawSw = 3;
int jsArmSw = 2;

//joystick boolean values for the switch press
//boolean (bool) can only be 2 values: true or false.
bool jsClawPressed = false;
bool jsArmPressed = false;

//joystick reading minimum/maximum and threshold values
//min = joystick all the way to left or down
//max = joystick all the way irght or up
//max = 1024 because Analog to digital converter (ADC) - converts an analog voltage 
//into a digital reading has a 10 bit resolution so max is 2^10 = 1024
//this means a reading of 0 = 0 volts and a reading of 1024= 5 volts
//for more info... https://learn.sparkfun.com/tutorials/analog-to-digital-conversion/all
//https://www.arrow.com/en/research-and-events/articles/engineering-resource-basics-of-analog-to-digital-converters
//reading is less than negative threshold = turn servo anti-clockwise
//reading is more than positive threshold = turn servo clockwise
//uint16 is a 16 bit integer, for info on data types...
// https://www.geeksforgeeks.org/data-types-in-c/
uint16_t jsMax = 1024;
uint16_t jsMin = 0;
uint16_t jsNegThresh = 300;
uint16_t jsPosThresh = 700;
uint16_t jsVal = 0;

/********************************
 * Servo initialisation
*********************************/
//All servos
Servo sArmBase;
Servo sArmShoulder;
Servo sArmElbow;
Servo sClawSpin;
Servo sClawAngle;
Servo sClawGrip;

//servo positions, change these to start arm in different position
//uint8 is 8 bit unsigned integer, link describing more above on the uint16 bit
uint8_t sArmBasePos = 90;
uint8_t sArmShoulderPos = 90;
uint8_t sArmElbowPos = 90;
uint8_t sClawSpinPos = 90;
uint8_t sClawAnglePos = 90;
uint8_t sClawGripPos = 0;

//minimum degrees you can send servo is 0, maximum is 180
uint8_t servoMin = 0;
uint8_t servoMax = 180;

//variables that signal which servos the arm joystick operates
//these are changed when the arm joystick switch is pressed
uint8_t shoulderMode = 0;
uint8_t elbowMode = 1;
uint8_t ArmMode = shoulderMode;

//variables that signal which servos the claw joystick operates
//these are changed when the claw joystick switch is pressed
uint8_t angleMode = 0;
uint8_t spinMode = 1;
uint8_t clawMode = angleMode;

//speed resolution no longer used was used to change 
//servo speed depending on how far the joystick was moved
const uint8_t speedResolution = 3;

//pressTime is used to make sure the button is pressed
//and it isnt just voltage jitter, and for debounce...
//more info on debouce...https://www.allaboutcircuits.com/technical-articles/switch-bounce-how-to-deal-with-it/
unsigned long pressTime = 0;

uint8_t nc_js_neg_thresh = 50;
uint8_t nc_js_pos_thresh = 200;
uint16_t nc_acc_neg_thresh = 400;
uint16_t nc_acc_pos_thresh = 600;

uint8_t nc_jsx = 0;
uint8_t nc_jsy = 0;
uint16_t nc_accx = 0;
uint16_t nc_accy = 0;
uint16_t nc_accz = 0;
bool nc_butz = 0;
bool nc_butc = 0;

//This interrupt function is called when the claw joystick switch is pressed
//it just sets the jsCLawPressed boolean value to true
void jsClawSwitched() {
  jsClawPressed = true;
}

//This interrupt function is called when the arm joystick switch is pressed
//it just sets the jsArmPressed boolean value to true
void jsArmSwitched(){
  jsArmPressed = true;
}

//This function is called first to set up the arduino and is only run through once
void setup() {
  //setting the joystick switch pins up
  //pull up pulls the pins up to 5 volts, when the switch is pressed 
  //it drops the pin to 5 volts which causes the interrupts above to trigger
  pinMode(jsClawSw, INPUT_PULLUP);
  pinMode(jsArmSw, INPUT_PULLUP);

  //sets the LED on the arduino board to be an output
  pinMode(LED_BUILTIN, OUTPUT);

  //sets up serial communication to print debug info
  Serial.begin(250000);
  Serial.print("START UP");
  
  //These two lines link the joystick switch pins to the interrupt functions above
  //and causes the fucntions to trigger when the switch pin falls from 5v to 0v
  //thats why we have 'FALLING' 
  attachInterrupt(digitalPinToInterrupt(jsClawSw), jsClawSwitched, FALLING);
  attachInterrupt(digitalPinToInterrupt(jsArmSw), jsArmSwitched, FALLING);

  //This attaches the servos to these digital pins
  sArmBase.attach(13);
  sArmShoulder.attach(11);
  sArmElbow.attach(10);
  sClawSpin.attach(9);
  sClawAngle.attach(8);
  sClawGrip.attach(7);

  //This moves servos to start positions defined at the top
  servoStartPos(sArmBase, sArmBasePos);
  servoStartPos(sArmShoulder, sArmShoulderPos);
  servoStartPos(sArmElbow, sArmElbowPos);
  servoStartPos(sClawSpin, sClawSpinPos);
  servoStartPos(sClawAngle, sClawAnglePos);
  servoStartPos(sClawGrip, sClawGripPos);

  /*Wire.begin();
  Wire.setClock(400000);
  nunchuk_init();*/

  nunchuck.begin();

  uint8_t tries = 0;
  while (!nunchuck.connect() && tries < 3){
    Serial.print("nunchuck not detected\n\r");
    delay(500);
    tries++;
  }

}

//ServoStartPos function slowly moves servo to the start position
void servoStartPos(Servo s, int pos){
  //first reads the position of the servo
  int sVal = s.read();
  //if the servo position is more than the position to set
  //loop and subtract 1 degree every 50milliseconds
  while(sVal > pos){
    s.write(sVal);
    sVal--;
    Serial.print(sVal);
    delay(50);
  }
  //if the servo position is less than the position to set
  //loop and add 1 degree every 50milliseconds
  while(sVal < pos){
    s.write(sVal);
    sVal++;
    Serial.print(sVal);
    delay(50);
  }
}


//this loop function happens after the setup function above
//and it continously loops around forever until the arduino is turned off 
void loop() {
  
  if(nunchuck.update()){   
    nc_jsx = nunchuck.joyX();
    nc_jsy = nunchuck.joyY();
    nc_accx = nunchuck.accelX();
    nc_accy = nunchuck.accelY();
    nc_accz = nunchuck.accelZ();
    nc_butz = nunchuck.buttonZ();
    nc_butc = nunchuck.buttonC();

#ifdef DEBUG
    Serial.print("\n\rjs x: "); Serial.print(nc_jsx);
    Serial.print("\n\rjs y: "); Serial.print(nc_jsy);
    Serial.print("\n\racc x: "); Serial.print(nc_accx);
    Serial.print("\n\racc y: "); Serial.print(nc_accy);
    Serial.print("\n\racc z: "); Serial.print(nc_accz);
    Serial.print("\n\rbut z: "); Serial.print(nc_butz);
    Serial.print("\n\rbut c: "); Serial.print(nc_butc); 
#endif
    
    }
    else
    {
      nc_jsx = nc_jsy = nc_accx = nc_accy = nc_accz = nc_butz = nc_butc = 0;
      Serial.print("POOP\n\r");
    }
  //read the arm joystick signal and if it is outside of the thresholds
  //then start moving the arm, always move base
  
  jsVal = readAnSig(jsArmY);

  if(jsVal < jsNegThresh || nc_jsx < nc_js_neg_thresh){
    sArmBasePos = checkServo(sArmBase, sArmBasePos, false); 
  }
  else if (jsVal > jsPosThresh || nc_jsx > nc_js_pos_thresh){
    sArmBasePos = checkServo(sArmBase, sArmBasePos, true);
  }

  //move shoulder or elbow depending on mode, which is changed by the arm joystick switch
  jsVal = readAnSig(jsArmX);
  
  if((jsVal < jsNegThresh && ArmMode == shoulderMode) || nc_jsy < nc_js_neg_thresh){
    sArmShoulderPos = checkServo(sArmShoulder, sArmShoulderPos, false);    
  }  
  else if ((jsVal > jsPosThresh && ArmMode == shoulderMode) || nc_jsy > nc_js_pos_thresh){
    sArmShoulderPos = checkServo(sArmShoulder, sArmShoulderPos, true);
  }
  else if ((jsVal < jsNegThresh && ArmMode == elbowMode) || nc_accy < nc_acc_neg_thresh){
    sArmElbowPos = checkServo(sArmElbow, sArmElbowPos, false);
  }
  else if ((jsVal > jsPosThresh && ArmMode == elbowMode) || nc_accy > nc_acc_pos_thresh){
    sArmElbowPos = checkServo(sArmElbow, sArmElbowPos, true);
  }

  //read the claw joystick signal and if it is outside of the thresholds
  //then start moving the claw, always move the grip
  jsVal = readAnSig(jsClawY);
  if(jsVal < jsNegThresh || (!nc_butc && sClawGripPos != 0)){
    sClawGripPos  = checkServo(sClawGrip, sClawGripPos, false);
  }
  else if (jsVal > jsPosThresh || nc_butc){
    sClawGripPos  = checkServo(sClawGrip, sClawGripPos, true);
  }

  //move angle or spin depending on mode, which is changed by the claw joystick switch
  jsVal = readAnSig(jsClawX);
  if((jsVal < jsNegThresh && clawMode == angleMode) || (!nc_butz && sClawAnglePos != 0)){
     sClawAnglePos = checkServo(sClawAngle, sClawAnglePos, false);
  }
  else if ((jsVal > jsPosThresh && clawMode == angleMode) || nc_butz){
    sClawAnglePos = checkServo(sClawAngle, sClawAnglePos, true);
  }
  else if((jsVal < jsNegThresh && clawMode == spinMode) || nc_accx < nc_acc_neg_thresh){
      sClawSpinPos = checkServo(sClawSpin, sClawSpinPos, false);
  }
  else if((jsVal > jsPosThresh && clawMode == spinMode) || nc_accx > nc_acc_pos_thresh){
      sClawSpinPos = checkServo(sClawSpin, sClawSpinPos, true);
  }

  if(jsClawPressed){
    //if the claw joystick switch pressed value is set to true
    //and the joystick is still pressed 30milliseconds later (for debounce)
    //change the claw mode
    pressTime = millis();
    while(millis() - pressTime > 30){
    }
    if(!digitalRead(jsClawSw)){    
      if(clawMode == angleMode){
        clawMode = spinMode;
        Serial.print("Spin Mode\n\r\n\r");
      }
      else if(clawMode == spinMode){
        clawMode = angleMode;
        Serial.print("Angle Mode\n\r\n\r");
      }
      //set value 
      jsClawPressed = false;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  if(jsArmPressed){
    //if the arm joystick switch pressed value is set to true
    //and the joystick is still pressed 30milliseconds later (for debounce)
    //change the claw mode
    pressTime = millis();
    while(millis() - pressTime > 30){
    }    
    if(!digitalRead(jsArmSw)){
      if(ArmMode == shoulderMode){
        ArmMode = elbowMode;
        Serial.print("Elbow Mode\n\r\n\r");
      }
      else if(ArmMode == elbowMode){
        ArmMode = shoulderMode;
        Serial.print("Shoulder Mode\n\r\n\r");
      }
      jsArmPressed = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  
}

//readAnSig function reads an analog signal from the pin given
uint16_t readAnSig(int pin){
  //create large value anRead 
  uint16_t anRead = 0;
  //in this loop we take 8 analogue reading and then average them
  //by dividing by 8, this gets rid of any errors in the reading
  //so will stop the servos moving in strange ways if there is a 
  //reading error
  uint8_t loop = 0;
  for(loop=0; loop<8;loop++){
    anRead += analogRead(pin);
  }

  anRead /= 8;

  return anRead;
}

//checkServo function uses the joystick value to set the servo position
int checkServo(Servo s, int servoPos, bool move_pos){ 
  //if the joystick value is less than negative threshold we turn the 
  //servo anticlockwise (subtract from current position value) 
  Serial.print("move pos: "); Serial.print(move_pos); Serial.print("\n\r");
  if(!move_pos){
    //These print the joystick value for debug
    int armSpeed =  1; 
    //change the position value of the servo by subtracting armSpeed(1)
    if(servoPos-armSpeed > servoMin){
      servoPos -= armSpeed;         
    }
    //if the joystick is trying to make the servo go to less than the minimum 
    //position just set the servo to the minimum position
    else if (servoPos-armSpeed <= servoMin){
      servoPos = servoMin;
    }
    //write the value to the servo
    s.write(servoPos);
    //print servo position for debug
    Serial.print("servoPos: ");
    Serial.print(servoPos);
    Serial.print("\n\r");
    //delay for 'servoSpeed' milliseconds, this changes the speed of the servo
    delay(servoSpeed);  
  }
  //if the joystick value is greater than positive threshold we turn the 
  //servo clockwise (add from current position value) 
  else if(move_pos){
    //These print the joystick value for debug
    int armSpeed =  1; 
    //change the position value of the servo by adding armSpeed (1)
    if(servoPos+armSpeed < servoMax){
      servoPos += armSpeed;
    }
    //if the joystick is trying to make the servo go to further than the maximum 
    //position just set the servo to the maxmum position
    else if (servoPos+armSpeed >= servoMax){
      servoPos = servoMax;
    }
    //write the value to the servo
    s.write(servoPos);
    //print servo position for debug
    Serial.print("servoPos: ");
    Serial.print(servoPos);
    Serial.print("\n\r");
    //delay for 'servoSpeed' milliseconds, this changes the speed of the servo
    delay(servoSpeed);  
  }
    
  //returns the new servo position
  return servoPos;
}
