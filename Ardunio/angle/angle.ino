#include <Servo.h>
#include "Command.h"

Servo hServo[4];
int c=0;

void setup() {
  Serial.begin(115200);
  Serial.println("Robotic Arm Controller");
  //Serial.setTimeout(100); /* to avoid lags in communication */

  hServo[0].attach(2);
  hServo[1].attach(3);
  hServo[2].attach(5);
  hServo[3].attach(6);
  Serial.println("READY");
}

void loop() {
  if ( Serial.available() > 0 ) {
      String serialData = Serial.readStringUntil('\n');
      Command* cmd = new Command(serialData);
      Serial.println(cmd->getParam(1).toInt());
      Serial.println(cmd->getParam(2).toInt());
      Serial.println(cmd->getParam(3).toInt());
      Serial.println(cmd->getParam(4).toInt());
      hServo[0].write(cmd->getParam(1).toInt());
      hServo[1].write(cmd->getParam(2).toInt());
      hServo[2].write(cmd->getParam(3).toInt());
      hServo[3].write(cmd->getParam(4).toInt());
      delay(500);
      Serial.println("MOVE!");
 }
}
