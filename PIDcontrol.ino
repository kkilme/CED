#include <Servo.h>
/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9            // [20200000] LED핀 설정
#define PIN_SERVO 10         //[20191979] Servo 핀 설정
#define PIN_IR A0            // [20191979] 적외선 핀(아날로그핀)

// Framework setting
#define _DIST_TARGET 255   // [20213085] 정지하려는 위치 목표값
#define _DIST_MIN 100         //[20191979] 최소거리
#define _DIST_MAX 410        //[20191979] 최대거리

// Distance sensor
#define _DIST_ALPHA 0.25 //[20203118] DIST_ALPHA 값 설정

// Servo range
#define _DUTY_MIN 1050    // [20213083]서보 각도의 최솟값 설정
#define _DUTY_NEU 1350   // [20213090]서보 수평 각도 펄스 값
#define _DUTY_MAX 1750        // [20213081] 서보 각도의 최댓값

// Servo speed control
#define _SERVO_ANGLE 30 //[20203118] 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 50 //[20203118] 서보 속도 설정
#define _RAMPUP_TIME 60

#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100

// Event periods
#define _INTERVAL_DIST 10   // [20213099] 거리 센서 주기 설정
#define _INTERVAL_SERVO 20  // [20213099] 서보 주기 설정
#define _INTERVAL_SERIAL 100  // [20213099] 시리얼 표시 주기 설정

// PID parameters
#define _KP 1.5  //[20191979] 비례제어 값
#define _KD 50
#define _KI 0.01
//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  // [20213090] 서보 인스턴스 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, dist_cali, dist_average; //[20203118] 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수
float alpha;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; //[20203118] 마지막으로 측정한 거리, 마지막으로 측정한 서보 각도(각 이벤트별로 업데이트를 위해 측정하는 시간)
bool event_dist, event_servo, event_serial; //[20203118] 이벤트 별로 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; //[20213086]interval 당 최대 duty 변화량
int duty_target, duty_curr; //[20213086] 목표 duty와 현재 duty 값
int duty_chg_max;
int duty_chg_adjust;
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
int i_count = 0;
int i_abs = 0;
//[20213086] error_curr: 현재 주기 오차값 / error_prev : 이전 주기 오차 값 / control : PID 제어량 / pterm, dterm, iterm : 비례,적분,미분 이득값

int a = 90;
int b = 265;

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  
// move servo to neutral position
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = START;
  myservo.writeMicroseconds(duty_curr);

  // initialize global variables
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * (float)(_SERVO_SPEED / 180) * _INTERVAL_SERVO / 1000;
  duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.
  pause_time = 0.5;
  toggle_interval = ((float)(180.0 / _SERVO_SPEED) + pause_time) * 1000 / _INTERVAL_SERVO;
  toggle_interval_cnt = toggle_interval;
  
  dist_target=_DIST_TARGET; //[20203118] dist_target 값 초기화
  alpha = _DIST_ALPHA;
  dist_raw = dist_ema = dist_average = ir_distance_filtered();
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  error_curr = error_prev = dist_target - dist_ema;
  pterm = 0;
  dterm = 0;
  iterm = 0;

  // initialize serial port
  Serial.begin(115200);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = int((_DUTY_MAX-_DUTY_MIN))*((float)_SERVO_SPEED/_SERVO_ANGLE)*(float(_INTERVAL_SERVO)/1000.0); //duty_chg_per_interval 값 설정   
}
  


void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  // [20213090]
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
          last_sampling_time_dist += _INTERVAL_DIST;
          event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
          last_sampling_time_servo += _INTERVAL_SERVO;
          event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
          last_sampling_time_serial += _INTERVAL_SERIAL;
          event_serial = true;
  }
  
  
  ////////////////////
  // Event handlers //
  ////////////////////
  
  if(event_dist) {
    event_dist = false;
    // get a distance reading from the distance sensor
//    dist_raw = ir_distance_filtered();
//    dist_cali = 100 + 300.0 / (b - a) *(dist_raw - a);
//    dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;
    dist_average = 0;
    
    for(int i = 0; i < 100; ++i){
      dist_raw = ir_distance_filtered();
      dist_cali = 100 + 300.0 / (b - a) *(dist_raw - a);
      dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;
      dist_average += dist_ema;
      delay(0.1);
    }
    dist_average /= 100;

//    i_abs = abs((int)dist_average - _DIST_TARGET);
//    if(i_abs < 20 && i_count <= 99){
//      i_count += 1;
//    } else if (i_count>= 1) {
//      i_count -= 1;
//    }
    
  // PID control logic
    error_curr = dist_target - dist_average;
    pterm = _KP * error_curr; //[20213083]
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm; //[20203118]  
  
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; //[20213086]

//    if(i_count >= 100){
//      iterm = 0;
//    }

    if(abs(iterm)>20){iterm = 0;}
  
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}
    if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}

    error_prev = error_curr;
  }
    
  if(event_servo) {
    
    event_servo=false;
  
     // adjust duty_curr toward duty_target by duty_chg_per_interval
   if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
      }
    else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
      }
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  
  }
  if(event_serial) {
    event_serial = false; //[20203118]
    Serial.print("IR:");
    Serial.print(dist_average);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
//    Serial.print(",Ireal:");
//    Serial.print(iterm);
//    Serial.print(",i_count:");
//    Serial.print(i_count);
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
//    Serial.print(",Min:400,duty_target:");
//    Serial.print(duty_target);
//    Serial.print(",duty_curr:");
//    Serial.print(duty_curr);
//    Serial.print(",duty_chg_max:");
//    Serial.print(duty_chg_max);
//    Serial.print(",duty_chg_adjust:");
//    Serial.print(duty_chg_adjust);  
//    Serial.print(",duty_chg_per_interval:");
//    Serial.print(duty_chg_per_interval);  
//    Serial.println(",Max:2400");
  }
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}
