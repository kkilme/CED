#include <Servo.h>
// CH0500-03 ASSY
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

//#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
//#define _INTERVAL_DIST  25 // USS interval (unit: ms)
//#define _INTERVAL_SERVO  20 // servo interval (unit: ms)
//#define _INTERVAL_SERIAL  100 // serial interval (unit: ms)
//#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
//#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.3

#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 60
#define INTERVAL 20
#define SENSOR_400 205
#define SENSOR_100 82

// global variables
float dist_min, dist_max, dist_center, dist_raw; // unit: mm
float scale; // used for pulse duration to distance conversion
float raw_dist = 0;
float dist_cali;
float dist_ema;
float alpha;
int duty_chg_per_interval, duty_target;
int duty_curr = 1350;
Servo myservo;
unsigned long last_sampling_time; // unit: ms

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1350);

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

// initialize USS related variables
  alpha = _DIST_ALPHA;
// initialize serial port
  Serial.begin(57600);  

// initialize event variables
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;

  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  raw_dist = ir_distance();
  dist_cali = 100 + 300.0 / (SENSOR_400 - SENSOR_100) *(raw_dist - SENSOR_100);
  dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;
  Serial.print("min:50,max:450,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);
//  myservo.writeMicroseconds(1350);
  if(dist_ema > 355){
    duty_target = 1050;
  } else {
    duty_target = 1750;
  }

  if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }
  myservo.writeMicroseconds(duty_curr);
  last_sampling_time += INTERVAL;
  
}
