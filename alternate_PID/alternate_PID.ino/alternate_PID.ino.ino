#include <Sabertooth.h>
#include <SoftwareSerial.h>

int none, ntwo, oone, otwo = 0;
float stone, sttwo, xone, xtwo = 0;
int done, dtwo, ione, itwo = 0;

SoftwareSerial SWSerial(NOT_A_PIN, 30);
Sabertooth ST(129, SWSerial);   //1st motor driver
Sabertooth ST1(128, SWSerial);  //2nd motor driver

void roverCallBack() {
  oone = xone;
  otwo = xtwo;
  none = -127;
  ntwo = 0;  //cmd_vel.linear.y;
  if (none < 0) {
    none = -none;
    done = -1;
  } else if (none == 0) {
    done = done;
  } else {
    done = 1;
  }
  if (ntwo < 0) {
    ntwo = -ntwo;
    dtwo = 0;
  } else if (ntwo == 0) {
    dtwo = dtwo;
  } else {
    dtwo = 1;
  }
  if (none > 127 || none < 0) {
    none = 0;
  }
  if (ntwo > 127 || ntwo < 0) {
    ntwo = 0;
  }
  stone = (none - oone) / 10000.;
  sttwo = (ntwo - otwo) / 5.;

  ione = 0;
  itwo = 0;
}

void setup() {
  SWSerial.begin(9600);
  ST.setBaudRate(115200);
  ST1.setBaudRate(115200);
  SWSerial.end();
  SWSerial.begin(115200);
  Serial.begin(115200);

}

void loop() {  
  if (ione == 0) {
    roverCallBack();
  }
  if (xone != none) {
    xone = stone * ione + oone;
    ione = ione + 1;
    if (xone > 127 || xone < 0) {
      xone = 0;
      oone = 0;
    }

    ST.motor(1, done*xone);
    // analogWrite(9, xone);
  }
  Serial.println(none);
  Serial.println(xone);

  // if (xtwo != ntwo) {
  //   xtwo = sttwo * itwo + otwo;
  //   itwo = itwo + 1;
  //   if (xtwo > 127 || xtwo < 0) {
  //     xtwo = 0;
  //     otwo = 0;
  //   }
  //   // msg.linear.y = xtwo;
  //   // analogWrite(6, xtwo);
  //   // analogWrite(5, xtwo);
  // }


  //  msg.linear.z = ione;
}
}