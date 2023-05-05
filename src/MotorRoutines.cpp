#define UP_LIMIT_DEG   60   //Elevation Up limit of tracker
#define DOWN_LIMIT_DEG 0    //Elevation Down limit of tracker
#define EAST_LIMIT_DEG 120  //Azimuth East limit of tracker
#define WEST_LIMIT_DEG 240  //Azimuth West limit of tracker
#define SOUTH_DEG      180  //Reference Degrees of South
#define elev_pot 35       // Analog input pin for elevation potentiometer
#define azim_pot 34       // Analog input pin for azimuth potentiometer
//Motor Driver 1 Pin Designation
const int M1_LPwm = 19;   // I/O channel setup ESP32 pin 
const int M1_RPwm = 18;   // I/O channel setup ESP32 pin
const int En_M1    = 5;   // I/O pin for BTS Driver Enable
//Motor Driver 2 Pin Designation
const int M2_LPwm = 2;    // I/O channel setup ESP32 pin 
const int M2_RPwm = 4;    // I/O chennel setup ESP32 pin
const int En_M2    =15;   // I/O pin for BTS Driver Enable
//Channel Variables
const int  freq = 1000;   // PWM Frequency
const int  res = 8;       // PWM Resolution
//- - - - - - - - - - VALUE ASSIGNMENTS  - - - - - - - - - -
//Potentiometer Value Assignment
int pot1value = 2047;     // Potentiometer Analog Raw Value Scale
int dirM1 = 2;            // Motor control: dirM1 {1=leftTurn 2=stop 3=rightTurn}
int Speed1 = 255 ;        // Speed Control (@maxSpeed)
int pot2value = 2047;     // Potentiometer Analog Raw Value Scale
int dirM2 = 2;            // Motor control: dirM1 {1=leftTurn 2=stop 3=rightTurn}
int Speed2 = 255 ;        // Speed Control (@maxSpeed)
const int  Channel_15 = 15;   // PWM channel 0, for BTS pin M1_LPwm 
const int  Channel_14 = 14;   // PWM channel 1, for BTS pin M1_RPwm
const int  Channel_13 = 13;   // PWM channel 0, for BTS pin M2_LPwm
const int  Channel_12 = 12;   // PWM channel 1, for BTS pin M2_RPwm
//Control Flow Flags' Assignment 
int elev_pot_up_limit=450;    // Potentiometer Reading at Elevation Up limit of tracker
int elev_pot_down_limit=0;    // Potentiometer Reading at Elevation Down limit of tracker
int azim_pot_east_limit=0;    // Potentiometer Reading at Azimuth East limit of tracker
int azim_pot_west_limit=600;  // Potentiometer Reading at Azimuth West limit of tracker
int num_pot_readings=10;
int motion_flag=1;        //General Motion Flag for movement 8=UP, 4=DOWN, 2=EAST, 1=WEST, 0=STOP
int autoMoveElevFlag;     //Motion Flag for Azimuth Movement
int autoMoveAzimFlag;     //Motion Flag for Elevation Movement
int autoMoveElevDeg;      //Motion Flag for Elevation Movement
int autoMoveAzimDeg;      //Motion Flag for Azimuth Movement

#define elev_pot 35       // Analog input pin for elevation potentiometer
#define azim_pot 34       // Analog input pin for azimuth potentiometer

const int Elev_Tolerance = 1;

void automove_elev(int deg, int &autoMoveElevFlag, const int  Channel_15, const int  Channel_14, int Speed1) {
  Serial.println("elev Move Auto");
  if (read_elev_pot()>=(deg-Elev_Tolerance) && read_elev_pot()<=(deg+Elev_Tolerance)) {
    autoMoveElevFlag=1; 
    elevation_stop(Channel_15, Channel_14); 
    Serial.println("Elev Case 1");
  } else if(read_elev_pot()<(deg-Elev_Tolerance)) {
    move_elev_DOWN(Channel_15, Channel_14, Speed1); 
    Serial.println("Elev Case 2");
  } else if(read_elev_pot()>(deg+Elev_Tolerance)) {
    move_elev_UP(Channel_15, Channel_14, Speed1); 
    Serial.println("Elev Case 3");
  } else  elevation_stop(Channel_15, Channel_14);
}

// Elevetion UP
void move_elev_UP(const int  Channel_15, const int  Channel_14, int Speed1){
ledcWrite(Channel_15, 0);       // stop channel 15 (L_PWM)
ledcWrite(Channel_14, Speed1);   // run channel 14 (R_PWM) at "speed"
Serial.println("Move elevation to UP");
}

//--------------------------------------------------------//
//Elevation DOWN
void move_elev_DOWN(const int  Channel_15, const int  Channel_14, int Speed1){
ledcWrite(Channel_14,0);       // stop channel 14  (R_PWM)     
ledcWrite(Channel_15,Speed1);   // run channel 15 (L_PWM) at "speed"
Serial.println("Move elevation to Down");
}

//--------------------------------------------------------//
//Azimuth EAST
const int Azim_Tolerance = 2;

void automove_azim(int deg, int &autoMoveAzimFlag, const int  Channel_12, const int  Channel_13, int Speed2){
  Serial.println("azim Move Auto");
  if (read_azim_pot()>=(deg-Azim_Tolerance) && read_azim_pot()<=(deg+Azim_Tolerance)) {
    autoMoveAzimFlag=1; 
    azimuth_stop(Channel_12, Channel_13); 
    Serial.println("Azim Case 1");
  } else if (read_azim_pot()<(deg-Azim_Tolerance)) {
    move_azim_EAST(Channel_12, Channel_13, Speed2);
    Serial.println("Azim Case 2");
  } else if (read_azim_pot()>(deg+Azim_Tolerance)) {
    move_azim_WEST(Channel_12, Channel_13, Speed2); 
    Serial.println("Azim Case 3");
  } else  azimuth_stop(Channel_12, Channel_13);
}

void move_azim_EAST(const int  Channel_12, const int  Channel_13, int Speed2){
ledcWrite(Channel_12,0);       // stop channel 14  (R_PWM)     
ledcWrite(Channel_13,Speed2);   // run channel 15 (L_PWM) at "speed"
Serial.println("Move azimuth to EAST");
}

//Azimuth WEST
void move_azim_WEST(const int  Channel_12, const int  Channel_13, int Speed2){
ledcWrite(Channel_13,0);       // stop channel 15 (L_PWM)
ledcWrite(Channel_12,Speed2);   // run channel 14 (R_PWM) at "speed"
Serial.println("Move azimuth to West");
}

void both_stop(const int  Channel_12, const int  Channel_13, const int  Channel_15, const int  Channel_14){
ledcWrite(Channel_14,0);       // stop channel 14  (R_PWM)     
ledcWrite(Channel_15,0);   // run channel 15 (L_PWM) at "speed"
ledcWrite(Channel_13,0);       // stop channel 15 (L_PWM)
ledcWrite(Channel_12,0);
Serial.println("Halt Movement");
}

void azimuth_stop(const int  Channel_12, const int  Channel_13){
  ledcWrite(Channel_13,0);       // stop channel 15 (L_PWM)
  ledcWrite(Channel_12,0);
  Serial.println("Halt Azimuth Movement");
}

void elevation_stop(const int  Channel_15, const int  Channel_14){
ledcWrite(Channel_14,0);       // stop channel 14  (R_PWM)     
ledcWrite(Channel_15,0);   // run channel 15 (L_PWM) at "speed"
//ledcWrite(Channel_13,0);       // stop channel 15 (L_PWM)
//ledcWrite(Channel_12,0);
Serial.println("Halt Elevation Movement");
}

int read_azim_pot(){
  int azim_pot_val;
  int azim_pot_reading;
  //azim_pot_east_limit=AZIM_EAST_POT_LIMIT;
  azim_pot_val=0;
  for(int i=0;i<num_pot_readings;i++){
    if (motion_flag!=0) azim_pot_reading=(analogRead(azim_pot)/2); else azim_pot_reading=analogRead(azim_pot)/2;
    azim_pot_val=azim_pot_val+azim_pot_reading;
  }
azim_pot_val=azim_pot_val/num_pot_readings;
Serial.print("azim RAW: ");Serial.println(azim_pot_val);
azim_pot_val=constrain(azim_pot_val,azim_pot_east_limit,azim_pot_west_limit);
azim_pot_val=map(azim_pot_val,azim_pot_east_limit,azim_pot_west_limit,EAST_LIMIT_DEG,WEST_LIMIT_DEG);
Serial.print("azim Deg: ");Serial.println(azim_pot_val);
return azim_pot_val;
}

int read_elev_pot(){
int elev_pot_val;
int elev_pot_reading;
elev_pot_val=0;
  for(int i=0;i<num_pot_readings;i++){if(motion_flag!=0) elev_pot_reading=(analogRead(elev_pot)/3); else elev_pot_reading=analogRead(elev_pot)/3;
      elev_pot_val=elev_pot_val+elev_pot_reading;
  }
  elev_pot_val=elev_pot_val/num_pot_readings;
  Serial.print("elev RAW: ");Serial.println(elev_pot_val);

  elev_pot_val = constrain(elev_pot_val,elev_pot_down_limit,elev_pot_up_limit);
  elev_pot_val = map(elev_pot_val,elev_pot_down_limit,elev_pot_up_limit,DOWN_LIMIT_DEG,UP_LIMIT_DEG);
  Serial.print("elev Deg: ");Serial.println(elev_pot_val);
  return elev_pot_val;
}