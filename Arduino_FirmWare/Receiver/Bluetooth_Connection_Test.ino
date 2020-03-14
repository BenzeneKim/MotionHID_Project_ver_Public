#include <SoftwareSerial.h>
#include <Mouse.h>
#include <Keyboard.h>
SoftwareSerial mySerial(8, 9); //set Blutooth Tx, Rx pin to (8, 9)
                               //Arduino pro micro (8, 9), Arduino nano(2, 3)
//===============BT serial==============
String inputData;
char rawInputByte;
//======================================
int i;
//================Sensor value==========
String splVal[7];
char bridge_gyro[4];
char bridge_finger[3];
int sensorVal[7];
//======================================

//============act=======================
bool action();
bool grab();
bool snap();
bool arrow();
bool point();
bool spiderman();

//======================================
//===========actionNum==================
int actionNum;
int totalActionNumber;
//======================================

//===========Function===================
void BT();//input Data, split Data
void command(int actNum);
//======================================
void setup() {
  // set Serial baudrate to 57600
  Serial.begin(57600);
  Serial.println("Hello World!");

  pinMode(5,OUTPUT);
  // set Bluetooth serial baudrate to 9600(to change this, you have to change the setting of the HC-05(by using AT-Mode))
  mySerial.begin(9600);
  Mouse.begin();
  Keyboard.begin();
  digitalWrite(5,HIGH);
  delay(100);
  digitalWrite(5,LOW);
  delay(100);
}

void loop() { 
  digitalWrite(5,LOW);
  if (mySerial.available()){
    if(mySerial.read() == 's'){
      digitalWrite(5,HIGH);
      BTsignalChecker();
      SensorValueRead();
  
      for(i = 0; i < 5; i++){
        if(action(i)) {
          command(i);
        }
      }
  
      for(i = 0; i < 7; i++){
        Serial.print(sensorVal[i]);
        Serial.print(" : ");
      }
      Serial.println();
      Keyboard.releaseAll();     
    }
  }
}

int BTread(){ // read raw data from HC-05
  while (mySerial.available()<2);
  int v0 = (unsigned char)mySerial.read();
  int v1 = (unsigned char)mySerial.read();
  return((v0|v1 << 8) - 2000);  // integrate two int to one and minus 2000(added from Transmitter)
}

void SensorValueRead(){
   for(int i = 0; i < 7; i++){
     sensorVal[i] = BTread(); // put BT signal into the integer sensorvalue array 
   }
   sensorVal[1] = sensorVal[1]*-1;
}

void BTsignalChecker(){ // LED indicator for BT connection
  if (mySerial.available())
    digitalWrite(5, HIGH);
  else
    digitalWrite(5, LOW);
}
    

void BT()
{
  if (mySerial.available()){
    if(mySerial.read() == 's'){
      digitalWrite(5,HIGH);
      BTsignalChecker();
      SensorValueRead();   
    }
  }
}
bool action(int actionID){
  if (actionID == 0) return grab();
  if (actionID == 1) return snap();
  if (actionID == 2) return arrow();
  if (actionID == 3) return spiderman();
  if (actionID == 4) return point(); 
}

bool grab(){
  return sensorVal[3] > 600 && sensorVal[4] > 650 && sensorVal[5] > 650 && sensorVal[6] > 650;
}
bool arrow(){
  return sensorVal[3] > 600 && sensorVal[4]> 500 && sensorVal[5] < 400 && sensorVal[6] < 450;
}
bool snap(){
  return sensorVal[3] > 600 && sensorVal[4]> 500 && 600 > sensorVal[5]&& sensorVal[5] > 400 && 600 > sensorVal[6]&& sensorVal[6] > 450;
}
bool spiderman(){
  return sensorVal[3] < 550 && sensorVal[4] > 500 && sensorVal[5] > 500 && sensorVal[6] < 500 && sensorVal[1]< -15;
}
bool point(){
  return sensorVal[3] > 650 && sensorVal[4] > 650 && sensorVal[5] > 500 && sensorVal[6] < 450;
}


void LeftClick(){
  
  while(1){
    int deltatime = 0;
    int oldfingerValue = sensorVal[5];
    BT();
    Serial.println("snap");
    if (sensorVal[5] - oldfingerValue > 70){ 
      Mouse.click(MOUSE_LEFT);
      break;
    }
    if (deltatime>3) break;
    deltatime++;
  }
}

void RightClick(){
}

void Drag(){
  Mouse.press(MOUSE_LEFT);
  while(grab()){
    BT();
    Mouse.move(sensorVal[2]/2,-1 * sensorVal[1]/2,0);
  }
  Mouse.release(MOUSE_LEFT);
}

void Move(){
  while(point()){
    BT();
    Mouse.move(sensorVal[2]/2,-1 * sensorVal[1]/2,0);
  }
}

void Scroll(){
  Mouse.move(0, 0, sensorVal[2]);
}

void PressRightArrow(){
  Keyboard.press(KEY_RIGHT_ARROW);
  delay(100);
}

void PressLeftArrow(){
  Keyboard.press(KEY_LEFT_ARROW);
  delay(100);
}

void PressUpArrow(){
  Keyboard.press(KEY_UP_ARROW);
  delay(100);
}

void PressDownArrow(){
  Keyboard.press(KEY_DOWN_ARROW);
  delay(100);
}

void Arrow_press(){

 /*I     Serial.println("drag");

  if (sensorVal[1] > 30){
    return PressRightArrow();
  }
  if (sensorVal[1] < -30){
    return PressLeftArrow();
  }
  if (sensorVal[0] > 30){
    return PressUpArrow();
  }
  if (sensorVal[0] < 30){
    return PressLeftArrow();
  }
  while(arrow()){
    BT();
    Keyboard.releaseAll();  
    delay(100);
  }
  //Keyboard.releaseAll();
  delay(10);*/
}



void PressSpaceBar(){
  /*Keyboard.press(32);
  delay(100);
  //jinKeyboard.releaseAll();
  delay(100);*/
}

void command(int actNum){
  if(actNum == 0){
    return Drag();
  }
  if(actNum == 1){
    return LeftClick();
  }
  if (actNum == 2){
    return Arrow_press();
  }
  if (actNum == 3){
    return PressSpaceBar();
  }
  if (actNum == 4){
    return Move();
  }
}
