#include <radio.h>
#include <quad_remote.h>      // Header file with pin definitions and setup
#include <serLCD.h>
#include "quad_radio.h"



String menu[3] = {"CALIBRATION", "REQUEST ARM", "ADJUST PID"};
//String calMenu[8] = {"MIN YAW(LEFT)", "MAX YAW(RIGHT)", "MIN THROTTLE(DOWN)", "MAX THROTTLE(UP)", "MIN PITCH(LEFT)", "MAX PITCH(RIGHT)", "MIN ROLL(DOWN)", "MAX ROLL(UP)"};
String calMenu[4] = {"YAW", "THROTTLE", "ROLL", "PITCH"};
String pidMenu[3] = {"AXIS: YAW  ", "AXIS: ROLL ", "AXIS: PITCH"}; 
String armMSG = "ARMING";
//String pidMenu[size]....;
int intMenuSizes[3] = {4, 1, 3};
uint8_t * info_buffer = (uint8_t *) malloc(sizeof(rfInfo_t));
rfInfo_t rcv_info;


int menuIndex = 0; //external menu index
int intMenuIndex = 0; //internal menu index
bool withinMenu = false; //iterate over external or internal menu
boolean firstArm = false;

int pidSelection = 0;
int prevKnobPos;

rfInfo_t info;
unsigned long prev;
unsigned long timeArmed;
bool armed = false; //request arming state
bool btnPress = false;
int gimbalValues[8] = {1000, 0, 1000, 0, 1000, 0, 1000, 0}; //gimbal limits
float pidValues[9] = {0,0,0,0,0,0,0,0,0}; //pid coefficients in pattern of p, i, d, YAW, ROLL, PITCH
void knobs_update();
void knob_pressed(bool);
void btn1_pressed(bool); //define to be disarm quadcopter
void btn2_pressed(bool); //define to be exit mnue item

void btn_up_pressed(bool down);
void btn_down_pressed(bool down);
void btn_left_pressed(bool down);
void btn_right_pressed(bool down);
void btn_center_pressed(bool down);


void setup() {
  // put your setup code here, to run once:
  quad_remote_setup();
  Serial.begin(9600);
  rfBegin(13);
  info = rfInfo_t();
  prev = millis();
  analogReference(INTERNAL);
  pinMode(BATTERY_SENSE_PIN, INPUT);
  Serial.println("serial is working");
  lcd.setBacklight(0,128,0);
  update_display(); //print menu
  knob1.setCurrentPos(0);


  //read old gimbals values from EEPROM, if -1 mean has never written so don't override default values
   for(int i = 0; i < 8; i++){ //ee_promload or smth too?
    int temp;
     EEPROM.get(i * sizeof(int), temp);
     if(temp != -1){
       gimbalValues[i] = temp;
     }
   }
   for(int i = 0; i < 9; i++){ //ee_promload or smth too?
    float temp;
     EEPROM.get(8*sizeof(int) + i * sizeof(float), temp);
    //  Serial.println(temp);
     if(temp != -1){
       pidValues[i] = temp;
     }
   }


  // The buttons and the knob trigger these call backs.       
	knobs_update_cb = knobs_update; 
	knob1_btn_cb = knob_pressed;
	btn1_cb = btn1_pressed;
	btn2_cb = btn2_pressed;
	btn_up_cb =  btn_up_pressed;
	btn_down_cb = btn_down_pressed;
	btn_left_cb =  btn_left_pressed;
	btn_right_cb = btn_right_pressed;
	btn_center_cb =  btn_center_pressed;
     
	knobs_update(); // Initialize the knob
}

void read_gimbals(){
  int raw_yaw = analogRead(PIN_YAW);
  int raw_throttle = analogRead(PIN_THROTTLE);
  //yaw and pitch store min in right index, and max in left index
  int yaw = map(constrain(raw_yaw, gimbalValues[0], gimbalValues[1]), gimbalValues[0], gimbalValues[1], 1023, 0);
  int throttle = map(constrain(raw_throttle, gimbalValues[2], gimbalValues[3]), gimbalValues[2], gimbalValues[3], 0, 1023);

  int raw_roll = analogRead(PIN_ROLL);
  int raw_pitch = analogRead(PIN_PITCH);
  int roll = map(constrain(raw_roll, gimbalValues[4], gimbalValues[5]), gimbalValues[4], gimbalValues[5], 0, 1023);
  int pitch = map(constrain(raw_pitch, gimbalValues[6], gimbalValues[7]), gimbalValues[6], gimbalValues[7], 1023, 0);
 
  if(!info.armed && armed && yaw <= 10 && throttle <= 10 && roll >= 1013  && pitch <= 10){
    //transmit something
    info.armed = 1;
    lcd.setBacklight(255,0,0);
    timeArmed = millis();
    firstArm = true;
    update_display();
  }

  // deadband
  yaw = 492 < yaw && yaw < 532 ? 512 : yaw;
  throttle = throttle < 5 ? 0 : throttle;
  roll = 509 < roll && roll < 515 ? 512 : roll;
  pitch = 509 < pitch && pitch < 515 ? 512 : pitch;

  info.yaw = yaw;
  info.throttle = throttle;
  info.roll = roll;
  info.pitch = pitch;
  info.pid_yaw[0] = pidValues[0];
  info.pid_yaw[1] = pidValues[1];
  info.pid_yaw[2] = pidValues[2];
  info.pid_pitch[0] = pidValues[6];
  info.pid_pitch[1] = pidValues[7];
  info.pid_pitch[2] = pidValues[8];
  info.pid_roll[0] = pidValues[3];
  info.pid_roll[1] = pidValues[4];
  info.pid_roll[2] = pidValues[5];
}

void read_batteries(){
  int batteryVoltage = analogRead(BATTERY_SENSE_PIN);
  Serial.print("Battery V: ");
  Serial.println(batteryVoltage);

}

void loop() {
  // put your main code here, to run repeatedly:
  //read_gimbals();

//disable any menu interaction when armed
  if (is_pressed(BUTTON_UP_PIN)) {
		Serial.println("Up is pressed");
	}

  //is in calibration mode
   //don't send any information to when unarmed so can't motors can't run/prevent screen flickering
   if(millis() - prev > 100){
     if(withinMenu && menuIndex == 0){
      calibrateGimbals();
    }
    else if(withinMenu && menuIndex == 2){
      //calibratePid();
    }
     read_gimbals();
     prev = millis();
     //Serial.println(prev);
     send();
     receive();
   }
 // receive();
  delay(1);  // delay in between reads for stability
}

void send(){
  uint8_t* msg = get_msg(info);
  Serial.print("Magic: ");
  Serial.println(msg[0]);
  rfWrite(msg, sizeof(rfInfo_t));
  free(msg);
}
void receive() {
  if (rfAvailable() >= sizeof(rfInfo_t))  // If serial comes in...
	{
    Serial.println("Message received");
		int l = (int) rfRead(info_buffer, sizeof(rfInfo_t));
		if (l == sizeof(rfInfo_t)) {
      // Serial.println(rcv_info.magic);
			if (parse_msg(info_buffer, rcv_info)) {
        Serial.println("Validated Message received"); //why does it receive so often?
      //  Serial.println(rcv_info.armed);
        // reset timeout
        if (rcv_info.armed){
          //prev = millis();
        } else if(info.armed && firstArm){//if(info.armed && rcv_info.armed != info.armed && millis() - timeArmed > 5000){ //exit armed state if received state does not match current arm + enough time has passed
          firstArm = false; //think this should resolve multiple messages received
          // armed = false;
          // withinMenu = true;
          // menuIndex = 1;
          // intMenuIndex = 0;
          lcd.setBacklight(0, 128, 0);
          armed = false;
          withinMenu = false;
          info.armed = 0;
          update_display();
        }
        
			} else {
				rfFlush();
			}
		}
	}
  if(btnPress){
    if(menuIndex == 0){ //if in calibration mode, store values in EEPROM
        EEPROM.put((intMenuIndex*2)*sizeof(int), gimbalValues[intMenuIndex*2]);
        EEPROM.put((intMenuIndex*2+1)*sizeof(int), gimbalValues[intMenuIndex*2+1]); //store in EEPROM
      }
      else if(menuIndex == 2){
        pidSelection = 0;
        EEPROM.put(8*sizeof(int) + (intMenuIndex * 3) * sizeof(float), pidValues[intMenuIndex*3]);
        EEPROM.put(8*sizeof(int) + (intMenuIndex * 3 + 1) * sizeof(float), pidValues[intMenuIndex*3 + 1]);
        EEPROM.put(8*sizeof(int) + (intMenuIndex * 3 + 2) * sizeof(float), pidValues[intMenuIndex*3 + 2]);
    }
    btnPress = false;
  }
}


int row = 0;
int column = 0;

void update_display() {
	lcd.clear();
  //lcd.setBacklight(0, 128, 0);
	//lcd.setCursor(column, row);
  lcd.setCursor(0,0);
  String disp;
  if(!withinMenu){
    disp = menu[menuIndex];
  }else{
    if(menuIndex == 0){
      disp = calMenu[intMenuIndex];
    }
    else if(menuIndex == 1){
     if(info.armed){ //if quad is armed
        disp = "ARMED"; 
      } else{ //in request arming state
        disp = armMSG;
      }
    }
    else{
      calibratePid();
    }
  }
	lcd.print(disp);
}

float offset = 0.1;
void knobs_update() {
	Serial.print("Knob: ");
  if(!armed){
    if(withinMenu && menuIndex == 2){
      float off = offset;
      if(knob1.getCurrentPos() < 0)
        off = -1 * offset;
      if(knob1.getCurrentPos() != 0){
        pidValues[intMenuIndex * 3 + pidSelection] += off;
        calibratePid();
      }
    }
  }
  Serial.println(knob1.getCurrentPos());
  knob1.setCurrentPos(0);
}


void btn1_pressed(bool down) {
    if(down) {
      if (armed) { //exit armed mode
        armed = false;
        withinMenu = false;
        Serial.println("btn1 down");
        lcd.setBacklight(0, 128, 0);
        info.armed = 0;
        update_display();
      } else if (withinMenu && menuIndex == 0) { //reset values
        gimbalValues[intMenuIndex*2] = 1000;
        gimbalValues[intMenuIndex*2+1] = 0;
      }
    } else {
        Serial.println("btn1 up");
    }
}

void btn2_pressed(bool down) {
  if(!armed){
	if(down) {
		Serial.println("btn2 down");
    btnPress = true;
    withinMenu = false;
    update_display();
	}else {
		Serial.println("btn2 up");    
	}
  }

  
}

void knob_pressed(bool down) {
	if(down) {
		Serial.println("knob down");
		knob1.setCurrentPos(0);
		update_display();
	}else {
		Serial.println("knob up");    
	}
}


void btn_up_pressed(bool down) {
  if(!armed){
	if(down) {
		Serial.println("up down");	
	} else {
    row = (row - 1) %2;
        if(!withinMenu){
          menuIndex = (menuIndex - 1);
          if(menuIndex < 0){
            menuIndex = 2;
          }
        }
        else{
          btnPress = true;
          intMenuIndex = (intMenuIndex - 1) % intMenuSizes[menuIndex]; 
          if(intMenuIndex < 0){
            intMenuIndex = intMenuIndex + intMenuSizes[menuIndex];
          }
        }
		update_display();
		Serial.println("up up");    
	}
  }
}

void btn_down_pressed(bool down) {
  if(!armed){
	if(down) {
		Serial.println("down down");
	} else {
    row = (row + 1) %2;
    if(!withinMenu){
      menuIndex = (menuIndex + 1) % (sizeof(menu)/sizeof(menu[0]));
      Serial.println(menuIndex);
    }
    else{
      //calibrateGimbals(intMenuIndex); //hold gimbal at max limit + press down at same time
      btnPress = true;
      intMenuIndex = (intMenuIndex + 1) % intMenuSizes[menuIndex]; 
    }
    update_display();
		Serial.println("down up");    
	}
  }
}

void btn_left_pressed(bool down) {
  if(!armed){
	if(down) {
		Serial.println("left down");
		column = (column - 1) %16;
    if(withinMenu && menuIndex == 2){
      pidSelection = (pidSelection - 1) % 3;
    }
    update_display();
	} else {
		Serial.println("left up");
	}
  }
}

void btn_right_pressed(bool down) {
  if(!armed){
	if(down) {
		Serial.println("right down");
		column = (column + 1) %16;
    if(withinMenu && menuIndex == 2){
      pidSelection = (pidSelection + 1) % 3;
    }
    update_display();
	} else {
		Serial.println("right up");    
	}
  }
}

//enter menu
void btn_center_pressed(bool down) {
  if(!armed){
	if(down) {
		Serial.println("center down");
    if(!armed){
        withinMenu = true;
        if(menuIndex == 1){
          armed = true;
        }
        update_display();
    }
	} else {
		Serial.println("center up");    
	}
  }
}

void calibrateGimbals(){
   int pinNumber;
    if(intMenuIndex == 0){
      pinNumber = PIN_YAW;
    }
    else if(intMenuIndex == 1 ){
      pinNumber = PIN_THROTTLE;
    }
    else if(intMenuIndex == 2){
      pinNumber = PIN_ROLL;
    }
    else {
      pinNumber = PIN_PITCH;
    }
    int val = analogRead(pinNumber);
    if (withinMenu) {
      gimbalValues[intMenuIndex*2] = min(gimbalValues[intMenuIndex*2], val);
      gimbalValues[intMenuIndex*2 + 1] = max(gimbalValues[intMenuIndex*2+1], val);

      lcd.clear();

      lcd.print(calMenu[intMenuIndex] + " " + String(val));
      lcd.print(" min:" + String(gimbalValues[intMenuIndex*2]) + " max:" + String(gimbalValues[intMenuIndex*2+1]));
    }
}
void calibratePid(){
      String pCh = "p";
      String iCh = " i";
      String dCh = " d";       
      if(pidSelection == 0)
        pCh = "P";
      else if(pidSelection == 1)
        iCh = " I";
      else
        dCh = " D";
      lcd.clear();
      lcd.print(pidMenu[intMenuIndex] + "     ");
      lcd.print(pCh + String(pidValues[intMenuIndex*3 + 0], 1) + iCh + String(pidValues[intMenuIndex*3 + 1],1 ) + 
        dCh + String(pidValues[intMenuIndex*3 + 2], 1));
      
    }
