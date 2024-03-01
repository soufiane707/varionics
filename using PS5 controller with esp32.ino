//don't forget to change the Mac address to yours, you can find your controller's Mac address with the DS4 software sorry for the extra spacing lines

// tested with esp32 s2 on Arduino ide, make sure you download the library from here https://github.com/rodneybakiskan/ps5-esp32.git

#include <ps5Controller.h>
void setup() {

 Serial.begin(921600);

 pinMode(16,OUTPUT);

 pinMode(17,OUTPUT);

 pinMode(18,OUTPUT);

 pinMode(19,OUTPUT);

 pinMode(22,OUTPUT);

 pinMode(23,OUTPUT);

 ps5.begin("1a:2b:3c:01:01:01"); //replace with MAC address of your controller

 Serial.println("Ready.");

 digitalWrite(22,HIGH);

 digitalWrite(23,HIGH);

}



void loop() {

// while (ps5.isConnected() == false) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing

//  Serial.println("PS5 controller not found");

//  delay(300);

// }



 while (ps5.isConnected() == true) {

  if (ps5.Right()) Serial.println("Right Button");

  if (ps5.Down()) Serial.println("Down Button");

  if (ps5.Up()) Serial.println("Up Button");

  if (ps5.Left()) Serial.println("Left Button");



  if (ps5.Square()) Serial.println("Square Button");

  if (ps5.Cross()) Serial.println("Cross Button");

  if (ps5.Circle()) Serial.println("Circle Button");

  if (ps5.Triangle()) Serial.println("Triangle Button");



  if (ps5.UpRight()) Serial.println("Up Right");

  if (ps5.DownRight()) Serial.println("Down Right");

  if (ps5.UpLeft()) Serial.println("Up Left");

  if (ps5.DownLeft()) Serial.println("Down Left");



  if (ps5.L1()) Serial.println("L1 Button");

  if (ps5.R1()) Serial.println("R1 Button");



  if (ps5.Share()) Serial.println("Share Button");

  if (ps5.Options()) Serial.println("Options Button");

  if (ps5.L3()) Serial.println("L3 Button");

  if (ps5.R3()) Serial.println("R3 Button");



  if (ps5.PSButton()) Serial.println("PS Button");

  if (ps5.Touchpad()) Serial.println("Touch Pad Button");



  if (ps5.L2()) {

   Serial.printf("L2 button at %d\n", ps5.L2Value());

  }

  if (ps5.R2()) {

   Serial.printf("R2 button at %d\n", ps5.R2Value());

  }



  if (ps5.LStickX()) {

   Serial.printf("Left Stick x at %d\n", ps5.LStickX());


  }

  if (ps5.LStickY()) {

   Serial.printf("Left Stick y at %d\n", ps5.LStickY());

   if(ps5.LStickY()>100){

    digitalWrite(16,HIGH);

    digitalWrite(17,LOW);

   }

   else{

    digitalWrite(17,LOW);

    digitalWrite(16,LOW);

   }

   }

   if(ps5.LStickY()<20){

    digitalWrite(17,HIGH);

    digitalWrite(16,LOW);

   }

   else{

    digitalWrite(16,LOW);

    digitalWrite(17,LOW);

   }

  }

  if (ps5.RStickX()) {

   Serial.printf("Right Stick x at %d\n", ps5.RStickX());

  }

  if (ps5.RStickY()) {

   Serial.printf("Right Stick y at %d\n", ps5.RStickY());

    if(ps5.RStickY()>100){

    digitalWrite(18,HIGH);

    digitalWrite(19,LOW);

   }

   else{

    digitalWrite(19,LOW);

    digitalWrite(18,LOW);

   }

   if(ps5.RStickY()<20){

    digitalWrite(19,HIGH);

    digitalWrite(18,LOW);

   }

   else{

    digitalWrite(19,LOW);

    digitalWrite(18,LOW);

   }

  }



  Serial.println();

  // This delay is to make the output more human readable

  // Remove it when you're not trying to see the output

  //delay(300);

 }
