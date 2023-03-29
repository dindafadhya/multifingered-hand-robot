#include <Servo.h>

Servo jempol, act1, act2, act3, act4;

// pins initialization
const int FlexPin0 = A0, FlexPin1 = A1, FlexPin2 = A2, FlexPin3 = A3, FlexPin4 = A4;
const int FBPin1 = A10, FBPin2 = A11, FBPin3 = A12, FBPin4 = A13;

// flexes and filter complementaries
float x0, fil_x0, x0x = 0, fil_x0x = 0; // jempol
float x1, fil_x1, x1x = 0, fil_x1x = 0; // telunjuk
float x2, fil_x2, x2x = 0, fil_x2x = 0; // jari tengah
float x3, fil_x3, x3x = 0, fil_x3x = 0; // jari manis
float x4, fil_x4, x4x = 0, fil_x4x = 0; // kelingking

// set points
float sp0, sp1, sp2, sp3, sp4;
float sp1_prev = 0;
float sp2_prev = 0;
float sp3_prev = 0;
float sp4_prev = 0;

// 2nd filter complimentaries
float sp1x = 0, fil_sp1x = 0, fil_sp1;
float sp2x = 0, fil_sp2x = 0, fil_sp2;
float sp3x = 0, fil_sp3x = 0, fil_sp3;
float sp4x = 0, fil_sp4x = 0, fil_sp4;
  
// position sensors and filter complementaries
float pres1, pos_1, poscal_1, fpres_1, poscal_1x = 0, fpres_1x = 0; // telunjuk
float pres2, pos_2, poscal_2, fpres_2, poscal_2x = 0, fpres_2x = 0; // jari tengah
float pres3, pos_3, poscal_3, fpres_3, poscal_3x = 0, fpres_3x = 0; // jari manis
float pres4, pos_4, poscal_4, fpres_4, poscal_4x = 0, fpres_4x = 0; // kelingking

// errors
float e1, e1_prev = 0;
float e2, e2_prev = 0;
float e3, e3_prev = 0;
float e4, e4_prev = 0;

// control variables
float PID_pos1, PID_pos2, PID_pos3, PID_pos4;
float g1, g2, g3, g4;
int final0, final1, final2, final3, final4; // PWM outputs (microseconds)

// error integrals
float inte1, inte1_prev;
float inte2, inte2_prev;
float inte3, inte3_prev;
float inte4, inte4_prev;

// controlled variable limitations
// values from potentiometer feedback sensors of linear actuators, showed in 10-bit ADC values
float pos1_max = 140.0, pos1_min = 0.0; // telunjuk
float pos2_max = 190.0, pos2_min = 0.0; // jari tengah
float pos3_max = 100.0, pos3_min = 0.0; // jari manis
float pos4_max = 110.0, pos4_min = 0.0; // kelingking

int dt; unsigned long t; unsigned long t_prev = 0;
unsigned long timer = 0;

//PID gains
double kp = 5;
double ki_1 = 0.005;
double ki_2 = 0.004;
double kd_1 = 5;
double kd_2 = 4;

void setup(){
  Serial.begin(1000000);
  act1.attach(2); // linear actuators
  act2.attach(3); 
  act3.attach(44); 
  act4.attach(45); 
  jempol.attach(46);
  pinMode(FlexPin0, INPUT); // flex sensors
  pinMode(FlexPin1, INPUT);
  pinMode(FlexPin2, INPUT); 
  pinMode(FlexPin3, INPUT); 
  pinMode(FlexPin4, INPUT);
  pinMode(FBPin1, INPUT); // potentiometer feedback sensors (linear actuator)
  pinMode(FBPin2, INPUT); 
  pinMode(FBPin3, INPUT); 
  pinMode(FBPin4, INPUT);
}

void loop(){
  x0 = analogRead(FlexPin0); 
  x1 = analogRead(FlexPin1); 
  x2 = analogRead(FlexPin2); 
  x3 = analogRead(FlexPin3); 
  x4 = analogRead(FlexPin4);

  // flex sensor lowpass filter, 0.5 Hz for cut-off frequency
  fil_x0 = 0.9727*fil_x0x + 0.0364*x0 + 0.0364*x0x; 
  x0x = x0; fil_x0x = fil_x0;
  fil_x1 = 0.9727*fil_x1x + 0.0364*x1 + 0.0364*x1x; 
  x1x = x1; fil_x1x = fil_x1;
  fil_x2 = 0.9727*fil_x2x + 0.0364 *x2 + 0.0364*x2x; 
  x2x = x2; fil_x2x = fil_x2;
  fil_x3 = 0.9727*fil_x3x + 0.0364*x3 + 0.0364*x3x; 
  x3x = x3; fil_x3x = fil_x3;
  fil_x4 = 0.9727*fil_x4x + 0.0364*x4 + 0.0364*x4x; 
  x4x = x4; fil_x4x = fil_x4;

  // flex sensor value limitations
  if(fil_x0>=380) {fil_x0=380;} else if (fil_x0<=290) {fil_x0=290;}
  if(fil_x1>=420) {fil_x1=420;} else if (fil_x1<=280) {fil_x1=280;}
  if(fil_x2>=500) {fil_x2=500;} else if (fil_x2<=310) {fil_x2=310;}
  if(fil_x3>=460) {fil_x3=460;} else if (fil_x3<=360) {fil_x3=360;}
  if(fil_x4>=380) {fil_x4=380;} else if (fil_x4<=270) {fil_x4=270;}

  // setpoints in flex unit, initializing values to zero
  sp0 = fil_x0 - 290;
  sp1 = fil_x1 - 280;
  sp2 = fil_x2 - 310;
  sp3 = fil_x3 - 360;
  sp4 = fil_x4 - 270;
  
  // thumb motor servo calculations, 130 degree in range
  final0 = (sp0/90)*130;

  //2nd filter, 0.5 Hz for cut-off frequency
  fil_sp1 = 0.9727*fil_sp1x + 0.0364*sp1 + 0.0364*sp1x;
  sp1x = sp1; fil_sp1x = fil_sp1;
  fil_sp2 = 0.9727*fil_sp2x + 0.0364*sp2 + 0.0364*sp2x;
  sp2x = sp2; fil_sp2x = fil_sp2;
  fil_sp3 = 0.9727*fil_sp3x + 0.0364*sp3 + 0.0364*sp3x;
  sp3x = sp3; fil_sp3x = fil_sp3;
  fil_sp4 = 0.9727*fil_sp4x + 0.0364*sp4 + 0.0364*sp4x;
  sp4x = sp4; fil_sp4x = fil_sp4;

  if (fil_sp1>0){
    pres1 = analogRead(FBPin1);
    if (pres1<=8) {pres1 = 8;} else if (pres1>=648) {pres1 = 648;} 
    pos_1 = (648-pres1)/640;
    poscal_1 = pos_1*140; // normalization
    fpres_1 = 0.9727*fpres_1x + 0.0364*poscal_1 + 0.0364*poscal_1x; 
    poscal_1x = poscal_1; 
    fpres_1x = fpres_1;}

  if (fil_sp2>0){
    pres2 = analogRead(FBPin2); 
    if (pres2<=50) {pres2 = 50;} else if (pres2>=652) {pres2 = 652;} 
    pos_2 = (652-pres2)/602;
    poscal_2 = pos_2*190; // normalization
    fpres_2 = 0.9727*fpres_2x + 0.0364*poscal_2 + 0.0364*poscal_2x; 
    poscal_2x = poscal_2; 
    fpres_2x = fpres_2;}

  if (fil_sp3>0){
    pres3 = analogRead(FBPin3); 
    if (pres3<=33) {pres3 = 33;} else if (pres3>=655) {pres3 = 655;} 
    pos_3 = (655-pres3)/622;
    poscal_3 = pos_3*100; // normalization
    fpres_3 = 0.9727*fpres_3x + 0.0364*poscal_3 + 0.0364*poscal_3x; 
    poscal_3x = poscal_3; 
    fpres_3x = fpres_3;}

  if (fil_sp4>0){
    pres4 = analogRead(FBPin4); 
    if (pres4<=13) {pres4 = 13;} else if (pres4>=648) {pres4 = 648;} 
    pos_4 = (648-pres4)/635;
    poscal_4 = pos_4*110; // normalization
    fpres_4 = 0.9727*fpres_4x + 0.0364*poscal_4 + 0.0364*poscal_4x; 
    poscal_4x = poscal_4; 
    fpres_4x = fpres_4;}

  t = millis(); dt = (t-t_prev);

  // calculate error (SP-PV), direct
  e1 = fil_sp1 - fpres_1;
  e2 = fil_sp2 - fpres_2;
  e3 = fil_sp3 - fpres_3;
  e4 = fil_sp4 - fpres_4;
  
  inte1 = inte1 + (dt*(e1+e1_prev)/2);
  inte2 = inte2 + (dt*(e2+e2_prev)/2);
  inte3 = inte3 + (dt*(e3+e3_prev)/2); 
  inte4 = inte4 + (dt*(e4+e4_prev)/2);

  // PID control
  PID_pos1 = (kp*e1) + (ki_1*inte1) + (kd_1*(e1-e1_prev)/dt);
  if (PID_pos1 > pos1_max) {PID_pos1 = pos1_max; inte1 = inte1_prev;}  
  if (PID_pos1 < pos1_min) {PID_pos1 = pos1_min; inte1 = inte1_prev; sp1_prev = sp1;}
  PID_pos2 = (kp*e2) + (ki_1*inte2) + (kd_1*(e2-e2_prev)/dt); 
  if (PID_pos2 > pos2_max) {PID_pos2 = pos2_max; inte2 = inte2_prev;}  
  if (PID_pos2 < pos2_min) {PID_pos2 = pos2_min; inte2 = inte2_prev; sp2_prev = sp2;}
  PID_pos3 = (kp*e3) + (ki_2*inte3) + (kd_1*(e3-e3_prev)/dt); // algoritma PID
  if (PID_pos3 > pos3_max) {PID_pos3 = pos3_max; inte3 = inte3_prev;}  
  if (PID_pos3 < pos3_min) {PID_pos3 = pos3_min; inte3 = inte3_prev; sp3_prev = sp3;}
  PID_pos4 = (kp*e4) + (ki_1*inte4) + (kd_2*(e4-e4_prev)/dt); // algoritma PID
  if (PID_pos4 > pos4_max) {PID_pos4 = pos4_max; inte4 = inte4_prev;}  
  if (PID_pos4 < pos4_min) {PID_pos4 = pos4_min; inte4 = inte4_prev; sp4_prev = sp4;}

  t_prev = t;
  inte1_prev = inte1;
  inte2_prev = inte2;
  inte3_prev = inte3;
  inte4_prev = inte4;

  // normalization of position value
  g1 = abs(PID_pos1)/140.0; 
  g2 = abs(PID_pos2)/190.0; 
  g3 = abs(PID_pos3)/100.0; 
  g4 = abs(PID_pos4)/110.0; 

  // control signal (pulse in microseconds)
  final1 = (g1*969)+1028;
  final2 = (g2*969)+1028;
  final3 = (g3*969)+1028;
  final4 = (g4*969)+1028;
  
  jempol.write(final0);
  act1.writeMicroseconds(final1);
  act2.writeMicroseconds(final2);
  act3.writeMicroseconds(final3);
  act4.writeMicroseconds(final4);
  delay(25); // sampling time of 25 ms
}
