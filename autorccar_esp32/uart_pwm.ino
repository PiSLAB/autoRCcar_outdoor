#include "esp32-hal-cpu.h"

#define Pin_GLED 25   // Green LED
#define Pin_RLED 26   // Red LED
#define Pin_YLED 32   // Yellow LED
#define Pin_steer 17  // PWM Port
#define Pin_accel 16  // PWM Port
#define Pin_empty 4   // PWM Port (empty)

int pwm_ch0 = 0;
int pwm_ch1 = 1;
int pwm_ch2 = 2;
int pwm_freq = 50;        // 50Hz
int pwm_resolution = 16;  // 16bit

int dat[3];

void servoWrite(int ch, int deg) {
  int duty = deg*18.2 + 3277;
  ledcWrite(ch, duty);
}

void error_loop(){
  while(1)
  {
    digitalWrite(Pin_RLED, !digitalRead(Pin_RLED));
    delay(100);
  }
}

void setup() {
  // initialize serial:
  Serial.begin(115200);

  setCpuFrequencyMhz(80); // Set CPU clock to 80, 160, 240MHz
  
  pinMode(Pin_GLED, OUTPUT);  
  pinMode(Pin_RLED, OUTPUT);
  pinMode(Pin_YLED, OUTPUT);
  
  digitalWrite(Pin_GLED, LOW);
  digitalWrite(Pin_RLED, LOW);
  digitalWrite(Pin_YLED, LOW);  

  ledcSetup(pwm_ch0, pwm_freq, pwm_resolution);
  ledcSetup(pwm_ch1, pwm_freq, pwm_resolution);
  ledcSetup(pwm_ch2, pwm_freq, pwm_resolution);
  ledcAttachPin(Pin_steer, pwm_ch0);
  ledcAttachPin(Pin_accel, pwm_ch1);
  ledcAttachPin(Pin_empty, pwm_ch2);

  servoWrite(pwm_ch0, 90);
  servoWrite(pwm_ch1, 90);
  delay(2000);

}

void loop() {
  int xa, xs, xm;

  if (Serial.available() >= 8)
  {
    if (Serial.read() == 0xFF)
    {
      if (Serial.read() == 0xFE)
      {
        for(int i=0; i<3; i++)
          dat[i]= (Serial.read() << 8) | Serial.read();

        xs = dat[0];
        xa = dat[1];
        xm = dat[2];

        if (xs >= 180)
          xs = 180;
        if (xs <= 0)
          xs = 0;
        if (xa >= 180)
          xa = 180;
        if (xa <= 0)
          xa = 0;
 
        servoWrite(pwm_ch0, xs);  // steer pwm 0-180
        servoWrite(pwm_ch1, xa);  // accel pwm 0(reverse) 90(static) 180(accel)
        //servoWrite(pwm_ch2, msg->z);
      
        if (xa < 90)  // backward
          digitalWrite(Pin_YLED, HIGH);
        else
          digitalWrite(Pin_YLED, LOW);
      
        if (xm > 0)  // operating 
          digitalWrite(Pin_GLED, HIGH);
        else
          digitalWrite(Pin_GLED, LOW);
      }
    }
  }
}
