#include <Servo.h> 

//joystick init
int jsClawY = A3;
int jsClawX = A4;
int jsArmY = A6;
int jsArmX = A7;
int jsClawSw = 3;
int jsArmSw = 2;
bool jsClawPressed = false;
bool jsArmPressed = false;
int jsMax = 1024;
int jsMin = 0;
int jsNegThresh = 300;
int jsPosThresh = 700;
int jsVal = 0;

//Servo init
Servo sArmBase;
Servo sArmShoulder;
Servo sArmElbow;
Servo sClawSpin;
Servo sClawAngle;
Servo sClawGrip;

int sArmBasePos = 90;
int sArmShoulderPos = 90;
int sArmElbowPos = 90;
int sClawSpinPos = 90;
int sClawAnglePos = 90;
int sClawGripPos = 90;

int servoMin = 0;
int servoMax = 180;
int shoulderMode = 0;
int elbowMode = 1;
int ArmMode = shoulderMode;

int angleMode = 0;
int spinMode = 1;
int clawMode = angleMode;
 
const int speedResolution = 3;
unsigned long time_now = 0, pressTime = 0;

void jsClawSwitched() {
  jsClawPressed = true;
}

void jsArmSwitched(){
  jsArmPressed = true;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(jsClawSw, INPUT_PULLUP);
  pinMode(jsArmSw, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(250000);
  Serial.print("START UP");
  
  attachInterrupt(digitalPinToInterrupt(jsClawSw), jsClawSwitched, FALLING);
  attachInterrupt(digitalPinToInterrupt(jsArmSw), jsArmSwitched, FALLING);
  
  sArmBase.attach(13);
  sArmShoulder.attach(11);
  sArmElbow.attach(10);
  sClawSpin.attach(9);
  sClawAngle.attach(8);
  sClawGrip.attach(7);

  servoStartPos(sArmBase, sArmBasePos);
  servoStartPos(sArmShoulder, sArmShoulderPos);
  servoStartPos(sArmElbow, sArmElbowPos);
  servoStartPos(sClawSpin, sClawSpinPos);
  servoStartPos(sClawAngle, sClawAnglePos);
  servoStartPos(sClawGrip, sClawGripPos);

}

void servoStartPos(Servo s, int pos){
  int sVal = s.read();
  while(sVal > pos){
    s.write(sVal);
    sVal--;
    Serial.print(sVal);
    delay(50);
  }
  while(sVal < pos){
    s.write(sVal);
    sVal++;
    Serial.print(sVal);
    delay(50);
  }
}

void loop() {

  jsVal = readAnSig(jsArmY);
  if(jsVal < jsNegThresh || jsVal > jsPosThresh){
    sArmBasePos = checkServo(sArmBase, sArmBasePos, jsVal); 
  }

  jsVal = readAnSig(jsArmX);
  if(jsVal < jsNegThresh || jsVal > jsPosThresh){
    if(ArmMode == shoulderMode){
      sArmShoulderPos = checkServo(sArmShoulder, sArmShoulderPos, jsVal);
    }
    else if(ArmMode == elbowMode){
      sArmElbowPos = checkServo(sArmElbow, sArmElbowPos, jsVal);
    }
  }

  jsVal = readAnSig(jsClawY);
  if(jsVal < jsNegThresh || jsVal > jsPosThresh){
    sClawGripPos = checkServo(sClawGrip, sClawGripPos, jsVal);
  }

  jsVal = readAnSig(jsClawX);
  if(jsVal < jsNegThresh || jsVal > jsPosThresh){
    if(clawMode == angleMode){
       sClawAnglePos = checkServo(sClawAngle, sClawAnglePos, jsVal);
    }
    else if(clawMode == spinMode){
      sClawSpinPos = checkServo(sClawSpin, sClawSpinPos, jsVal);
    }
  } 
  
  if(jsClawPressed){
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
      jsClawPressed = false;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  if(jsArmPressed){
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

uint16_t readAnSig(int pin){
  uint16_t anRead = 0;
  uint8_t loop = 0;
  for(loop=0; loop<8;loop++){
    anRead += analogRead(pin);
  }

  anRead /= 8;

  return anRead;
}

int checkServo(Servo s, int servoPos, int jsVal){ 
  if(jsVal < jsNegThresh){
    Serial.print(jsVal);
    Serial.print("\n\r");
    int armSpeed =  1; //map(jsVal, jsNegThresh, jsMin, 0, speedResolution);
    if(servoPos-armSpeed > servoMin){
      servoPos -= armSpeed;         
    }
    else if (servoPos-armSpeed < servoMin){
      servoPos = servoMin;
    }
    s.write(servoPos);
    Serial.print("servoPos: ");
    Serial.print(servoPos);
    Serial.print("\n\r");
    //time_now = millis();
    //while(millis() < time_now + 5){
    //}
    //delay(5);  
  }
  else if(jsVal > jsPosThresh){
    Serial.print(jsVal);
    Serial.print("\n\r");
    int armSpeed =  1; //map(jsVal, jsPosThresh, jsMax, 0, speedResolution);
    if(servoPos+armSpeed < servoMax){
      servoPos += armSpeed;
    }
    else if (servoPos+armSpeed > servoMax){
      servoPos = servoMax;
    }
    s.write(servoPos);
    Serial.print("servoPos: ");
    Serial.print(servoPos);
    Serial.print("\n\r");
    //time_now = millis();
    //while(millis() < time_now + 5){
    //}
  }
    //delay(5);  

  return servoPos;
}
