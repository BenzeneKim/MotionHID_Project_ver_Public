#include <SoftwareSerial.h>
#include <Mouse.h>
#include <Keyboard.h>
SoftwareSerial mySerial(8, 9); //블루투스의 Tx, Rx핀을 2번 3번핀으로 설정
                               //아두이노 프로 마이크로 (8, 9), 아두이노 나노(2, 3)
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
  // 시리얼 통신의 속도를 9600으로 설정
  Serial.begin(57600);
  Serial.println("Hello World!");

  pinMode(5,OUTPUT);
  //블루투스와 아두이노의 통신속도를 9600으로 설정
  mySerial.begin(9600);
  Mouse.begin();
  Keyboard.begin();
  digitalWrite(5,HIGH);
  delay(100);
  digitalWrite(5,LOW);
  delay(100);
}

void loop() { //코드를 무한반복합니다.
  BT();

  for(i = 0; i < 5; i++){
    if(action(i)) {
      command(i);
    }
  }
    Keyboard.releaseAll();                                                                                           

}

void BT()
{

  while(!mySerial);
  if (mySerial.available()) {
    digitalWrite(5,HIGH);
    rawInputByte = mySerial.read();
    inputData += String(rawInputByte);
  }
  else{
    digitalWrite(5,LOW);
  }
  //if (mySerial.available() == false) Serial.println("no");
  //Serial.println(inputData);
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
  if(inputData.endsWith("/") == true) 
  {  
     Serial.println("input Data is " + inputData);
     
    for(i = 0; i < 3; i++)
    {
      splVal[i] = inputData.substring(4 * i + 1, 4 * i + 4);
      splVal[i].toCharArray(bridge_gyro, 4);
      if(inputData.substring(4 * i, 4 * i + 1) == ">") 
      {
        sensorVal[i] = atoi(bridge_gyro);
      }
      if(inputData.substring(4 * i, 4 * i + 1) == "<")
      {
        sensorVal[i] = -1 * atoi(bridge_gyro); 
      }
    }
    for(i = 3; i<7; i++)
    {
      splVal[i] = inputData.substring(12 + 3 * (i-3), 12 + 3 * (i-3) + 3);
      splVal[i].toCharArray(bridge_finger, 4);
      sensorVal[i] = atoi(bridge_finger);
    }
   // Serial.println("");
    inputData = "";
    sensorVal[1] = sensorVal[1] * -1;
    for(i = 0; i < 7; i++){
      Serial.print(sensorVal[i]);
      Serial.print("  :  ");
    }
    Serial.println("");
  }
  
  delay(4);
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
