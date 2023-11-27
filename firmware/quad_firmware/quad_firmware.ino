#include <radio.h>
#include "quad_radio.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <QuadClass_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>

rfInfo_t info;
uint8_t armed = 0;
unsigned long prevTimeout;
unsigned long prevReceive;
unsigned long blink;
int prevLEDState;
unsigned long prevLoop;
quad_data_t orientation;
float cf_angle_pitch;
float cf_angle_roll;
float yaw_rate;

float gain;
float pitch_offset;
float roll_offset;
float pitch_rate_offset;
float roll_rate_offset;
float yaw_rate_offset;
float pwm_offset[4] = {0,0,0,0};
float rpy_cum[3] = {0,0,0};
float rpy_prev[3] = {0,0,0};

int fl_pin = 34;
int bl_pin = 35;
int br_pin = 9;
int fr_pin = 8;

uint8_t * info_buffer;

int fl_pin = 34;
int bl_pin = 35;
int br_pin = 9;
int fr_pin = 8;


QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL

void setupSensor()
{
 if (!lsm.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  lsm.setAccelDataRate(LSM6DS_RATE_208_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_208_HZ);
  // 208/4 = 26 hz
  lsm.setAccelCompositeFilter(LSM6DS_CompositeFilter_LPF2, LSM6DS_CompositeFilter_ODR_DIV_100);
  lsm.setGyroHPF(true, LSM6DS_gyroHPF_0);

  _accel = lsm.getAccelerometerSensor();
  _gyro = lsm.getGyroSensor();
  ahrs = new Adafruit_Simple_AHRS(_accel, _mag, _gyro);
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rfBegin(13);
  Serial.println("serial is working");
  pinMode(18, OUTPUT);
  info.armed = 0;
  info.yaw = 0;
  info.throttle = 0;
  info.roll = 0;
  info.pitch = 0;
  info_buffer = (uint8_t *)malloc(sizeof(rfInfo_t));
  uint8_t* msg = get_msg(info);
  rfWrite(msg, sizeof(rfInfo_t));
  free(msg);
  prevLEDState = 0;
  setupSensor();
  Serial.println("sensor setup");
  init_gyro();
  Serial.println("gyro setup");

  gain = 0.98;
}

#define RAD_TO_DEG 57.295779513082320876798154814105

void loop() {
 // digitalWrite(16, HIGH);
 // digitalWrite(17, HIGH);
  // analogWrite(fl_pin, 0); //white rotor on battery side, spinning is sus
  // analogWrite(bl_pin, 0); //blue rotor non battery side
  // analogWrite(br_pin, 0); //white rotor non battery side
  // analogWrite(fr_pin, 0); //blue rotor battery side
 
  float dt = (millis() - prevLoop)/1000.0;
  prevLoop = millis();
  // check timeout
  receive();
  
  analogWrite(fl_pin, 50); //white rotor on battery side, spinning is sus
  analogWrite(bl_pin, 50); //blue rotor non battery side
  analogWrite(br_pin, 50); //white rotor non battery side
  analogWrite(fr_pin, 50); //blue rotor battery side
  if (ahrs->getQuadOrientation(&orientation)){
      cf_angle_pitch = (gain)*(cf_angle_pitch + (orientation.pitch_rate - pitch_rate_offset) * RAD_TO_DEG * dt) + (1-gain)*(orientation.pitch - pitch_offset);
      cf_angle_roll = (gain)*(cf_angle_roll + (orientation.roll_rate - roll_rate_offset) * RAD_TO_DEG * dt) + (1-gain)*(orientation.roll - roll_offset);
      Serial.print(F("acc_angle "));
      Serial.print(orientation.pitch-pitch_offset);
      Serial.print(F(" gyro_raw "));
      Serial.print(orientation.pitch_rate-pitch_rate_offset);
      Serial.print(F(" cf_angle "));
      Serial.println(cf_angle_pitch);
  }
  if (millis()-prevTimeout >= 500) {
    // Serial.println("Timed out");
    digitalWrite(18, LOW);
    analogWrite(fl_pin, 0); //white rotor on battery side, spinning is sus
    analogWrite(bl_pin, 0); //blue rotor non battery side
    analogWrite(br_pin, 0); //white rotor non battery side
    analogWrite(fr_pin, 0); //blue rotor battery side
    armed = 0;
  } else {
    // if (!armed) {
    //   init_gyro();
    // }
    armed = info.armed;
  }
  if(armed){
    if(prevLEDState == 0){
      digitalWrite(18, HIGH);
    }
    else{
      digitalWrite(18, LOW);
    }
    prevLEDState = 1 - prevLEDState;
    // calc_pid(dt);
    // int drive8 = constrain((float)info.throttle/4+pwm_offset[0], 0, 255);
    // int drive3 = constrain((float)info.throttle/4+pwm_offset[1], 0, 255);
    // int drive4 = constrain((float)info.throttle/4+pwm_offset[2], 0, 255);
    // int drive5 = constrain((float)info.throttle/4+pwm_offset[3], 0, 255);

    // Serial.print("drive8: ");
    // Serial.print(drive8);
    // Serial.print(" drive3: ");
    // Serial.print(drive3);
    // Serial.print(" drive4: ");
    // Serial.print(drive4);
    // Serial.print(" drive5: ");
    // Serial.println(drive5);
    if(info.throttle > 10){
      analogWrite(fl_pin, info.throttle); //white rotor on battery side, spinning is sus
      analogWrite(bl_pin, info.throttle); //blue rotor non battery side
      analogWrite(br_pin, info.throttle); //white rotor non battery side
      analogWrite(fr_pin, info.throttle); //blue rotor battery side
    }else {
      rpy_cum[2] = 0; 
      rpy_cum[1] = 0;
      rpy_cum[0] = 0;     
      analogWrite(fl_pin, 0); //white rotor on battery side, spinning is sus
      analogWrite(bl_pin, 0); //blue rotor non battery side
      analogWrite(br_pin, 0); //white rotor non battery side
      analogWrite(fr_pin, 0); //blue rotor battery side 
    }  
    // analogWrite(fl_pin, (int)((info.throttle+pwm_offset[0])/6)); //white rotor on battery side, spinning is sus
    // analogWrite(bl_pin, (int)((info.throttle+pwm_offset[1])/6)); //blue rotor non battery side
    // analogWrite(br_pin, (int)((info.throttle+pwm_offset[2])/6)); //white rotor non battery side
    // analogWrite(fr_pin, (int)((info.throttle+pwm_offset[3])/6)); //blue rotor battery side
  }
  // delay(1);
}

void receive(){
  if (rfAvailable() >= sizeof(rfInfo_t))  // If serial comes in...
	{
    Serial.println("trying to read");
		int l = (int) rfRead(info_buffer, sizeof(rfInfo_t));
		if (l == sizeof(rfInfo_t)) {
      Serial.println("got msg");
      // Serial.print("raw magic: ");
      // Serial.println(info_buffer[0]);
      Serial.print("arm: ");
      Serial.println(info_buffer[1]);

			if (parse_msg(info_buffer, info)) {
        // reset timeout
        if (info.armed){
          prevTimeout = millis();
        }
				Serial.println("Good packet: ");
        
			} else {
				rfFlush();
				// Serial.println("Bad packet: ");
			}
		} 
	}
}

void init_gyro() {
  cf_angle_pitch = 0;
  cf_angle_roll = 0;
  if (ahrs->getQuadOrientation(&orientation)) {
    pitch_offset = orientation.pitch;
    roll_offset = orientation.roll;
    pitch_rate_offset = orientation.pitch_rate;
    roll_rate_offset = orientation.roll_rate;
    yaw_rate_offset = orientation.yaw_rate;
  } else {
    Serial.println("Quad init BROKE");
    pitch_offset = 0;
    roll_offset = 0;
    pitch_rate_offset = 0;
    roll_rate_offset = 0;
    yaw_rate_offset = 0;
  }
}

void calc_pid(float dt) {
  // yaw
  // pwm_offset[0] = ((float)info.yaw-512)/2;
  // pwm_offset[1] = -1 * ((float)info.yaw-512)/2;
  // pwm_offset[2] = ((float)info.yaw-512)/2;
  // pwm_offset[3] = -1 * ((float)info.yaw-512)/2;
  if (ahrs->getQuadOrientation(&orientation)){
      cf_angle_pitch = (gain)*(cf_angle_pitch + (orientation.pitch_rate - pitch_rate_offset) * RAD_TO_DEG * dt) + (1-gain)*(orientation.pitch - pitch_offset);
      cf_angle_roll = (gain)*(cf_angle_roll + (orientation.roll_rate - roll_rate_offset) * RAD_TO_DEG * dt) + (1-gain)*(orientation.roll - roll_offset);
      yaw_rate = (orientation.yaw_rate - yaw_rate_offset)*RAD_TO_DEG;
      // Serial.print(F("acc_angle "));
      // Serial.print(orientation.roll-roll_offset);
      // Serial.print(F(" gyro_raw "));
      // Serial.print(orientation.roll_rate-roll_rate_offset);
      // Serial.print(F(" cf_angle "));
      // Serial.println(cf_angle_pitch);
  }

  float target_yaw = -((float)info.yaw-512)/4;
  float p_error_yaw = target_yaw - yaw_rate;
  float d_error_yaw = (p_error_yaw - rpy_prev[2])/dt;
  if (info.pid_yaw[1] > 0) {
    rpy_cum[2] += p_error_yaw*dt;
  }
  float pid_yaw_offset = info.pid_yaw[0]*p_error_yaw + info.pid_yaw[1]*rpy_cum[2] + info.pid_yaw[2]*d_error_yaw;

  // pitch offset
  float target_pitch = -((float)info.pitch-512)/11;
  float p_error_pitch = target_pitch - cf_angle_pitch;
  float d_error_pitch = (p_error_pitch - rpy_prev[1])/dt;
  if (info.pid_pitch[1] > 0) {
    rpy_cum[1] += p_error_pitch*dt;
  }
  float pid_pitch_offset = info.pid_pitch[0]*p_error_pitch + info.pid_pitch[1]*rpy_cum[1] + info.pid_pitch[2]*d_error_pitch;

  // roll offset
  float target_roll = ((float)info.roll-512)/11;
  float p_error_roll = target_roll - cf_angle_roll;
  float d_error_roll = (p_error_roll - rpy_prev[0])/dt;
  if (info.pid_roll[1] > 0) {
    rpy_cum[0] += p_error_roll*dt;
  }
  float pid_roll_offset = info.pid_roll[0]*p_error_roll + info.pid_roll[1]*rpy_cum[0] + info.pid_roll[2]*d_error_roll;


  
  // print yaw offset
  // Serial.print("target yaw: ");
  // Serial.println(target_yaw);
  // Serial.print("yaw rate: ");
  // Serial.println(yaw_rate);


  pwm_offset[0] = (pid_yaw_offset + pid_pitch_offset- pid_roll_offset)/3;
  pwm_offset[1] = (-1 * (pid_yaw_offset) - pid_pitch_offset- pid_roll_offset)/3;
  pwm_offset[2] = pid_yaw_offset-pid_pitch_offset+pid_roll_offset;
  pwm_offset[3] = -1 * pid_yaw_offset+pid_pitch_offset+pid_roll_offset;

  rpy_prev[2] = p_error_yaw;
  rpy_prev[1] = p_error_pitch;
  rpy_prev[0] = p_error_roll;

  
  // Serial.println("PWM OFFSETS:");
  // Serial.println(pwm_offset[0]);
  // Serial.println(pwm_offset[1]);
  // Serial.println(pwm_offset[2]);
  // Serial.println(pwm_offset[3]);

}
