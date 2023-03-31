#include <Servo.h>
#include <math.h>

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

// experiments
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

// delta errors
float de1 = e1 - e1_prev;
float de2 = e2 - e2_prev;
float de3 = e3 - e3_prev;
float de4 = e4 - e4_prev;

// define fuzzy error input 
float Ne1[] = {-140,-140,0};
float Ze1[] = {-140,0,140};
float Pe1[] = {0,140,140};

float Ne2[] = {-190,-190,0};
float Ze2[] = {-190,0,190};
float Pe2[] = {0,190,190};

float Ne3[] = {-100,-100,0};
float Ze3[] = {-100,0,100};
float Pe3[] = {0,100,100};

float Ne4[] = {-110,110,0};
float Ze4[] = {-110,0,110};
float Pe4[] = {0,110,110};

float Nde[] = {-2,-2,0};
float Zde[] = {-2,0,2};
float Pde[] = {0,2,2};

float fNde, fZde, fPde;

float rule1_1,rule1_2,rule1_3,rule1_4,rule1_5,rule1_6,rule1_7,rule1_8,rule1_9;
float rule2_1,rule2_2,rule2_3,rule2_4,rule2_5,rule2_6,rule2_7,rule2_8,rule2_9;
float rule3_1,rule3_2,rule3_3,rule3_4,rule3_5,rule3_6,rule3_7,rule3_8,rule3_9;
float rule4_1,rule4_2,rule4_3,rule4_4,rule4_5,rule4_6,rule4_7,rule4_8,rule4_9;
int z1, z2, z3, z4;

// ------------------------------ fuzzification ------------------------------ //
// ------ 1. telunjuk
float fNe1(){
  if(e1<=Ne1[0]){return 1; }
  else if(e1>Ne1[0]&&e1<Ne1[1]){return (Ne1[1]-e1)/(Ne1[1]-Ne1[0]); }
  else if(e1>=Ne1[2]){return 0; }
}
float fZe1(){
  if(e1<=Ze1[0]){return 0; }
  else if(e1>Ze1[0]&&e1<Ze1[1]){return (e1-Ze1[0])/(Ze1[1]-Ze1[0]); }
  else if(e1>=Ze1[1]&&e1<Ze1[2]){return (Ze1[2]-e1)/(Ze1[2]-Ze1[1]); }
  else if(e1>=Ze1[2]){return 0; }
}
float fPe1(){
  if(e1<=Pe1[0]){return 0; }
  else if(e1>Pe1[0]&&e1<Pe1[1]){return (e1-Pe1[0])/(Pe1[1]-Pe1[0]); }
  else if(e1>=Pe1[2]){return 1; }
}

// ------ 2. jari tengah
float fNe2(){
  if(e2<=Ne2[0]){return 1; }
  else if(e2>Ne2[0]&&e2<Ne2[1]){return (Ne2[1]-e2)/(Ne2[1]-Ne2[0]); }
  else if(e2>=Ne2[2]){return 0;}
}
float fZe2(){
  if(e2<=Ze2[0]){return 0; }
  else if(e2>Ze2[0]&&e2<Ze2[1]){return (e2-Ze2[0])/(Ze2[1]-Ze2[0]); }
  else if(e2>=Ze2[1]&&e2<Ze2[2]){return (Ze2[2]-e2)/(Ze2[2]-Ze2[1]); }
  else if(e2>=Ze2[2]){return 0;}
}
float fPe2(){
  if(e2<=Pe2[0]){return 0;}
  else if(e2>Pe2[0]&&e2<Pe2[1]){return (e2-Pe2[0])/(Pe2[1]-Pe2[0]);}
  else if(e2>=Pe2[2]){return 1;}
}

// ------ 3. jari manis
float fNe3(){
  if(e3<=Ne3[0]){return 1; }
  else if(e3>Ne3[0]&&e3<Ne3[1]){return (Ne3[1]-e3)/(Ne3[1]-Ne3[0]); }
  else if(e3>=Ne3[2]){return 0; }
}
float fZe3(){
  if(e3<=Ze3[0]){return 0;}
  else if(e3>Ze3[0]&&e3<Ze3[1]){return (e3-Ze3[0])/(Ze3[1]-Ze3[0]);}
  else if(e3>=Ze3[1]&&e3<Ze3[2]){return (Ze3[2]-e3)/(Ze3[2]-Ze3[1]);}
  else if(e3>=Ze3[2]){return 0;}
}
float fPe3(){
  if(e3<=Pe3[0]){return 0;}
  else if(e3>Pe3[0]&&e3<Pe3[1]){return (e3-Pe3[0])/(Pe3[1]-Pe3[0]);}
  else if(e3>=Pe3[2]){return 1;}
}

// ------ 4. kelingking
float fNe4(){
  if(e4<=Ne4[0]){return 1;}
  else if(e4>Ne4[0]&&e4<Ne4[1]){return (Ne4[1]-e4)/(Ne4[1]-Ne4[0]);}
  else if(e4>=Ne4[2]){return 0;}
}
float fZe4(){
  if(e4<=Ze4[0]){return 0;}
  else if(e4>Ze4[0]&&e4<Ze4[1]){return (e4-Ze4[0])/(Ze4[1]-Ze4[0]);}
  else if(e4>=Ze4[1]&&e4<Ze4[2]){return (Ze4[2]-e4)/(Ze4[2]-Ze4[1]);}
  else if(e4>=Ze4[2]){return 0;}
}
float fPe4(){
  if(e4<=Pe4[0]){return 0;}
  else if(e4>Pe4[0]&&e4<Pe4[1]){return (e4-Pe4[0])/(Pe4[1]-Pe4[0]);}
  else if(e4>=Pe4[2]){return 1;}
}

// ------------------------------ DELTA ERROR ------------------------------ //
float N_derror(int de){
  if(de<=Nde[0]){fNde = 1;}
  else if(de>Nde[0]&&de<Nde[1]){fNde = (Nde[1]-de)/(Nde[1]-Nde[0]);}
  else if(de>=Nde[2]){fNde = 0;}
  return fNde;
}
float Z_derror(int de){
  if(de<=Zde[0]){fZde = 0;}
  else if(de>Zde[0]&&de<Zde[1]){fZde = (de-Zde[0])/(Zde[1]-Zde[0]);}
  else if(de>=Zde[1]&&de<Zde[2]){fZde = (Zde[2]-de)/(Zde[2]-Zde[1]);}
  else if(de>=Zde[2]){fZde = 0;}
  return fZde;
}

float P_derror(int de){
  if(de<=Pde[0]){
    fPde = 0;}
  else if(de>Pde[0]&&de<Pde[1]){
    fPde = (de-Pde[0])/(Pde[1]-Pde[0]);}
  else if(de>=Pde[2]){
    fPde = 1;}
  return fPde;
}

// ------------------------------ FUZZY RULE ------------------------------ // 
void rule1(){
  int N_de1 = N_derror(de1);
  int Z_de1 = Z_derror(de1);
  int P_de1 = P_derror(de1);
  rule1_1 = min(fNe1(),N_de1);
  rule1_2 = min(fZe1(),N_de1);
  rule1_3 = min(fPe1(),N_de1);
  rule1_4 = min(fNe1(),Z_de1);
  rule1_5 = min(fZe1(),Z_de1);
  rule1_6 = min(fPe1(),Z_de1);
  rule1_7 = min(fNe1(),P_de1);
  rule1_8 = min(fZe1(),P_de1);
  rule1_9 = min(fPe1(),P_de1); 
}

void rule2(){
  int N_de2 = N_derror(de2);
  int Z_de2 = Z_derror(de2);
  int P_de2 = P_derror(de2);
  rule2_1 = min(fNe2(),N_de2);
  rule2_2 = min(fZe2(),N_de2);
  rule2_3 = min(fPe2(),N_de2);
  rule2_4 = min(fNe2(),Z_de2);
  rule2_5 = min(fZe2(),Z_de2);
  rule2_6 = min(fPe2(),Z_de2);
  rule2_7 = min(fNe2(),P_de2);
  rule2_8 = min(fZe2(),P_de2);
  rule2_9 = min(fPe2(),P_de2); 
}

void rule3(){
  int N_de3 = N_derror(de3);
  int Z_de3 = Z_derror(de3);
  int P_de3 = P_derror(de3);
  rule3_1 = min(fNe3(),N_de3);
  rule3_2 = min(fZe3(),N_de3);
  rule3_3 = min(fPe3(),N_de3);
  rule3_4 = min(fNe3(),Z_de3);
  rule3_5 = min(fZe3(),Z_de3);
  rule3_6 = min(fPe3(),Z_de3);
  rule3_7 = min(fNe3(),P_de3);
  rule3_8 = min(fZe3(),P_de3);
  rule3_9 = min(fPe3(),P_de3); 
}

void rule4(){
  int N_de4 = N_derror(de4);
  int Z_de4 = Z_derror(de4);
  int P_de4 = P_derror(de4);
  rule4_1 = min(fNe4(),N_de4);
  rule4_2 = min(fZe4(),N_de4);
  rule4_3 = min(fPe4(),N_de4);
  rule4_4 = min(fNe4(),Z_de4);
  rule4_5 = min(fZe4(),Z_de4);
  rule4_6 = min(fPe4(),Z_de4);
  rule4_7 = min(fNe4(),P_de4);
  rule4_8 = min(fZe4(),P_de4);
  rule4_9 = min(fPe4(),P_de4); 
}

//float defuzzyfikasi(){
//  
//}

// control variables
int final0, final1, final2, final3, final4;

// controlled variable limitations
float pos1_max = 140.0, pos1_min = 0.0; // telunjuk
float pos2_max = 190.0, pos2_min = 0.0; // jari tengah
float pos3_max = 100.0, pos3_min = 0.0; // jari manis
float pos4_max = 110.0, pos4_min = 0.0; // kelingking

void setup() {
  Serial.begin(1000000);
  pinMode(FlexPin0, INPUT);
  pinMode(FlexPin1, INPUT);
  pinMode(FlexPin2, INPUT);
  pinMode(FlexPin3, INPUT);
  pinMode(FlexPin4, INPUT);
  pinMode(FBPin1, INPUT);
  pinMode(FBPin2, INPUT);
  pinMode(FBPin3, INPUT);
  pinMode(FBPin4, INPUT);
  jempol.attach(46);
  act1.attach(2);
  act2.attach(3);
  act3.attach(44);
  act4.attach(45);
  jempol.write(0);
  act1.writeMicroseconds(1028);
  act2.writeMicroseconds(1028);
  act3.writeMicroseconds(1028);
  act4.writeMicroseconds(1028);
}

void loop() {
  // read flex sensor (ADC)
  x0 = analogRead(FlexPin0); 
  x1 = analogRead(FlexPin1); 
  x2 = analogRead(FlexPin2); 
  x3 = analogRead(FlexPin3); 
  x4 = analogRead(FlexPin4);

  // flex sensor lowpass filter
  fil_x0 = 0.9272*fil_x0x + 0.0364*x0 + 0.0364*x0x; 
  x0x = x0; 
  fil_x0x = fil_x0;
  fil_x1 = 0.9272*fil_x1x + 0.0364*x1 + 0.0364*x1x; 
  x1x = x1; 
  fil_x1x = fil_x1;
  fil_x2 = 0.9272*fil_x2x + 0.0364*x2 + 0.0364*x2x; 
  x2x = x2; 
  fil_x2x = fil_x2;
  fil_x3 = 0.9272*fil_x3x + 0.0364*x3 + 0.0364*x3x; 
  x3x = x3; 
  fil_x3x = fil_x3;
  fil_x4 = 0.9272*fil_x4x + 0.0364*x4 + 0.0364*x4x; 
  x4x = x4; 
  fil_x4x = fil_x4;

  // flex sensor limitations
  if (fil_x0>=380) {fil_x0=380;} else if (fil_x0<=290) {fil_x0=290;}
  if (fil_x1>=420) {fil_x1=420;} else if (fil_x1<=280) {fil_x1=280;}
  if (fil_x2>=500) {fil_x2=500;} else if (fil_x2<=310) {fil_x2=310;}
  if (fil_x3>=460) {fil_x3=460;} else if (fil_x3<=360) {fil_x3=360;}
  if (fil_x4>=380) {fil_x4=380;} else if (fil_x4<=270) {fil_x4=270;}

  // setpoints in flex unit
  sp0 = fil_x0 - 290;
  sp1 = fil_x1 - 280;
  sp2 = fil_x2 - 310;
  sp3 = fil_x3 - 360;
  sp4 = fil_x4 - 270;
  final0 = ((sp0/90)*110)+20;

  // 2nd flex sensor lowpass filter
  fil_sp1 = 0.9272*fil_sp1x + 0.0364*sp1 + 0.0364*sp1x;
  sp1x = sp1; fil_sp1x = fil_sp1;
  fil_sp2 = 0.9272*fil_sp2x + 0.0364*sp2 + 0.0364*sp2x;
  sp2x = sp2; fil_sp2x = fil_sp2;
  fil_sp3 = 0.9272*fil_sp3x + 0.0364*sp3 + 0.0364*sp3x;
  sp3x = sp3; fil_sp3x = fil_sp3;
  fil_sp4 = 0.9272*fil_sp4x + 0.0364*sp4 + 0.0364*sp4x;
  sp4x = sp4; fil_sp4x = fil_sp4;
  
  if (fil_sp1>0){
    pres1 = analogRead(FBPin1); // baca sensor posisi (ADC)
    if (pres1<=8) {pres1 = 8;} else if (pres1>=648) {pres1 = 648;} // limitasi range nilai ADC sensor posisi
    pos_1 = (648-pres1)/640;
    poscal_1 = pos_1*140; // normalisasi nilai ADC sensor posisi
    fpres_1 = 0.9272*fpres_1x + 0.0364*poscal_1 + 0.0364*poscal_1x; //filter nilai sensor flex
    poscal_1x = poscal_1; 
    fpres_1x = fpres_1;}

  if (fil_sp2>0){
    pres2 = analogRead(FBPin2); // baca sensor posisi (ADC)
    if (pres2<=50) {pres2 = 50;} else if (pres2>=652) {pres2 = 652;} // limitasi range nilai ADC sensor posisi
    pos_2 = (652-pres2)/602;
    poscal_2 = pos_2*190; // normalisasi nilai ADC sensor posisi
    fpres_2 = 0.9272*fpres_2x + 0.0364*poscal_2 + 0.0364*poscal_2x; //filter nilai sensor flex
    poscal_2x = poscal_2; 
    fpres_2x = fpres_2;}

  if (fil_sp3>0){
    pres3 = analogRead(FBPin3); // baca sensor posisi (ADC)
    if (pres3<=33) {pres3 = 33;} else if (pres3>=655) {pres3 = 655;} // limitasi range nilai ADC sensor posisi
    pos_3 = (655-pres3)/622;
    poscal_3 = pos_3*100; // normalisasi nilai ADC sensor posisi
    fpres_3 = 0.9272*fpres_3x + 0.0364*poscal_3 + 0.0364*poscal_3x; //filter nilai sensor flex
    poscal_3x = poscal_3; 
    fpres_3x = fpres_3;}

  if (fil_sp4>0){
    pres4 = analogRead(FBPin4); // baca sensor posisi (ADC)
    if (pres4<=13) {pres4 = 13;} else if (pres4>=648) {pres4 = 648;} // limitasi range nilai ADC sensor posisi
    pos_4 = (648-pres4)/635;
    poscal_4 = pos_4*110; // normalisasi nilai ADC sensor posisi
    fpres_4 = 0.9272*fpres_4x + 0.0364*poscal_4 + 0.0364*poscal_4x; //filter nilai sensor flex
    poscal_4x = poscal_4; 
    fpres_4x = fpres_4;}

  // calculate error (SP-PV), direct
  e1 = fil_sp1 - fpres_1;
  e2 = fil_sp2 - fpres_2;
  e3 = fil_sp3 - fpres_3;
  e4 = fil_sp4 - fpres_4;

  // FUZZY LOGIC control

  //
  
  jempol.write(final0);
  act1.writeMicroseconds(final1);
  act2.writeMicroseconds(final2);
  act3.writeMicroseconds(final3);
  act4.writeMicroseconds(final4);
  delay(25);
}
