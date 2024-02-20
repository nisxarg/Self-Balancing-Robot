//LINE - FOLLOWING ROBOT

/*
    IR input --> s6 s5 s4 s3 s2 s1

    Error 0 : forward 
    1 1 1 1 1 1 0 : error 7
    1 1 1 1 1 0 0 : error 6
    1 1 1 1 0 0 0 : error 5
    1 1 1 1 1 0 1 : error 5
    1 1 1 1 0 0 1 : error 4
    1 1 1 0 0 0 1 : error 3
    1 1 1 1 0 1 1 : error 3
    1 1 1 0 0 1 1 : error 2
    1 1 0 0 0 1 1 : error 1
    1 1 1 0 1 1 1 : error 1
    1 1 0 0 1 1 1 : error 0
    1 0 0 0 1 1 1 : error -1
    1 1 0 1 1 1 1 : error -1
    1 0 0 1 1 1 1 : error -2
    0 0 0 1 1 1 1 : error -3
    1 0 1 1 1 1 1 : error -3
    0 0 1 1 1 1 1 : error -4
    0 1 1 1 1 1 1 : error -5

*/

  #define i0 29
  #define i1 31
  #define i2 33
  #define i3 35
  #define i4 37
  #define i5 39
  #define i6 41
  #define pl 6
  #define pr 8
  #define inl 7
  #define inr 9

  int s0,s1,s2,s3,s4,s5,s6;
  int e = 0;
  int prev_e = 0;
  long t = 0;
  long time = 0;
  long prev_time = 0;
  float ref_error = 0;
  float error = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float I = 0;
  float D = 0;
  float u = 0;
  bool stop = 0;

  void setup() {
    // put your setup code here, to run once:
    pinMode(i0, INPUT_PULLUP);
    pinMode(i1, INPUT_PULLUP);
    pinMode(i2, INPUT_PULLUP);
    pinMode(i3, INPUT_PULLUP);
    pinMode(i4, INPUT_PULLUP);
    pinMode(i5, INPUT_PULLUP);
    pinMode(i6, INPUT_PULLUP);
    
    pinMode(pl, OUTPUT);
    pinMode(pr, OUTPUT);
    pinMode(inl, OUTPUT);
    pinMode(inr, OUTPUT);

    Serial.begin(115200);
  }

  void loop() {
  

    cal_error();
    PID();

    while(stop) {
      motor_free();
      cal_error();
    }


  }

  void cal_error() {

    s0 = digitalRead(i0);
    s1 = digitalRead(i1);
    s2 = digitalRead(i2);
    s3 = digitalRead(i3);
    s4 = digitalRead(i4);
    s5 = digitalRead(i5);
    s6 = digitalRead(i6);
    // take input from port resister to get binary data
    // Ports vs Pins -  https://github.com/arduino/ArduinoCore-avr/blob/master/variants/mega/pins_arduino.h

    if(s6 & s5 & s4 & s3 & s2 & s1 & !s0) {      //1 1 1 1 1 1 0 : error 7
      error = 7;
      stop = 0;
    }

    else if(s6 & s5 & s4 & s3 & s2 & !s1 & !s0) {      //1 1 1 1 1 0 0 : error 6
      error = 6;
      stop = 0;
    }

    else if(s6 & s5 & s4 & s3 & !s2 & !s1 & !s0) {      //1 1 1 1 0 0 0 : error 5
      error = 5;
      stop = 0;
    }

    else if(s6 & s5 & s4 & s3 & s2 & !s1 & s0) {      //1 1 1 1 1 0 1 : error 5
      error = 5;
      stop = 0;
    }

    else if(s6 & s5 & s4 & s3 & !s2 & !s1 & s0) {      //1 1 1 1 0 0 1 : error 4
      error = 4;
      stop = 0;
    }

    else if(s6 & s5 & s4 & !s3 & !s2 & !s1 & s0) {      //1 1 1 0 0 0 1 : error 3
      error = 3;
      stop = 0;
    }

    else if(s6 & s5 & s4 & s3 & !s2 & s1 & s0) {     //1 1 1 1 0 1 1 : error 3
      error = 3;
      stop = 0;
    }

    else if(s6 & s5 & s4 & !s3 & !s2 & s1 & s0) {      //1 1 1 0 0 1 1 : error 2
      error = 2;
      stop = 0;
    }

    else if(s6 & s5 & !s4 & !s3 & !s2 & s1 & s0) {      //1 1 0 0 0 1 1 : error 1
      error = 1;
      stop = 0;
    }

    else if(s6 & s5 & s4 & !s3 & s2 & s1 & s0) {      //1 1 1 0 1 1 1 : error 1
      error = 1;
      stop = 0;
    }

    else if(s6 & s5 & !s4 & !s3 & s2 & s1 & s0) {      //1 1 0 0 1 1 1 : error 0
      error = 0;
      stop = 0;
    }

    else if(s6 & !s5 & !s4 & !s3 & s2 & s1 & s0) {      //1 0 0 0 1 1 1 : error -1
      error = -1;
      stop = 0;
    }

    else if(s6 & s5 & !s4 & s3 & s2 & s1 & s0) {      //1 1 0 1 1 1 1 : error -1
      error = -1;
      stop = 0;
    }

    else if(s6 & !s5 & !s4 & s3 & s2 & s1 & s0) {      //1 0 0 1 1 1 1 : error -2
      error = -2;
      stop = 0;
    }

    else if(!s6 & !s5 & !s4 & s3 & s2 & s1 & s0) {      //0 0 0 1 1 1 1 : error -3
      error = -3;
      stop = 0;
    }

    else if(s6 & !s5 & s4 & s3 & s2 & s1 & s0) {      //1 0 1 1 1 1 1 : error -3
      error = -3;
      stop = 0;
    }

    else if(!s6 & !s5 & s4 & s3 & s2 & s1 & s0) {      //0 0 1 1 1 1 1 : error -4
      error = -4;
      stop = 0;
    }

    else if(!s6 & s5 & s4 & s3 & s2 & s1 & s0) {      //0 1 1 1 1 1 1 : error -5
      error = -5;
      stop = 0;
    }

    else if(s6 & s5 & s4 & s3 & s2 & s1 & s0){      
      stop = 1;
      motor_free();
      // Serial.print("all off");
    }

    else if(!s6 & !s5 & !s4 & !s3 & !s2 & !s1 & !s0) {   
      stop = 1;       
      motor_free();
      // Serial.print("all on");
    }

  }

  void PID() {
    kp = 18;
    kd = 0.01;
    ki = 0.01;
    time = millis();
    if(time - prev_time > 50) {
      t = time - prev_time;
      e = error - ref_error;
      D = (e - prev_e)*20;
      I = I + e*0.05;
      u = kp*e + kd*D + ki*I;

      // if(u>0) {
      //   left_motor(30 + u);
      //   right_motor(30 - u);
      // }
      // else {
      //   left_motor(30 + u);
      //   right_motor(30 - u);
      // }

      if(u>0) { 
        set_motor(20 + u, 20 - u);
      }
      else {
        set_motor(20 + u, 20 - u);
      }

      // Serial.println(u);
      // Serial.print(" , ");
      // Serial.println(D);
      prev_e = e;
      prev_time = millis();
    }
  }

  void right_motor(float pwm) {
    if(pwm < 0) {
      pwm = 0;
    }
    analogWrite(pl, pwm);
    digitalWrite(inl, 1);     //inl = 1 : anticlockwise

  }

  void left_motor(float pwm) {
    if(pwm < 0) {
      pwm = 0;
    }
    analogWrite(pr, pwm);
    digitalWrite(inr, 1);     //inr = 1 : clockwise

  }

  void motor_free() {
    left_motor(0);
    right_motor(0);
    analogWrite(pl, 0);
    analogWrite(pr, 0);
  }

void set_motor(float l, float r) {
  if (l >= 0) {
    analogWrite(pl, l);
    digitalWrite(inl, LOW);  // inl = 0: clockwise
  } else {
    analogWrite(pl, -l);
    digitalWrite(inl, HIGH);  // inl = 1: anticlockwise
  }

  if (r >= 0) {
    analogWrite(pr, r);
    digitalWrite(inr, LOW);  // inr = 0: clockwise
  } else {
    analogWrite(pr, -r);
    digitalWrite(inr, HIGH);  // inr = 1: anticlockwise
  }
}

