//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}

//************************************************ *************************
//                                                      
// KNCT-MMEdu for Arduino Kumamoto National College of Technology Seiki Hayama
//                                                                              
// Menu selection with set switch. Displayed on LED from       
// 00: Sensor check, 01: Reserve, 10: Extended left method, 11: Shortest run       
// 2010.11.28 Ver 1.0
//*********** **************************************************** *************
#include <MsTimer2.h>

#define PT_FR 0 // Front right sensor, analog input
#define PT_FL 1 // Front left sensor, analog input
#define PT_R 2 // Right sensor , analog input
#define PT_L 3 // left sensor, analog input
#define GYRO 4 // reserved for gyro, analog input
#define LEDOUT 19 // LED output for wall sensor (use AN5 as digital 19)
#define SET 2 // Set switch, digital input
#define START 3 // Start switch, digital input 

// Various reference values
​​#define LSPD 3 // Low speed timer count value
#define HSPD 2 // High speed timer count value
#define STEP1 665 // Steps per step 
#define R90 265 // Steps rotated 90 degrees right 
#define L90 265 // Steps rotated left 90 degrees 
#define R180 530 // Steps rotated right 180 degrees 
#define DISFR 599 // Front Right wall standard distance sensor value 
#define DISFL 625 // Front left wall standard distance sensor value 
#define DISR 200 // Right wall standard distance sensor value 
#define DISL 190 // Left wall standard distance sensor Value    
#define DISFMAX 60 // Sensor value for front wall maximum distance (sensor value is minimum) 
#define DISRMAX 80 // Right wall maximum distance sensor value (sensor value is minimum) 
#define DISLMAX 80 // Left wall Maximum distance sensor value (minimum sensor value) 

// Motor rotation pattern table, right and left are reversed, so reverse rotation
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00}; // right motor is output to PORTB
const by te LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00}; // Left motor output to PORTD

const byte DtoR[]={0,2,4,0,8,0,0,0,1} ; // a table giving the right direction relative to the current direction 
const byte DtoL[]={0,8,1,0,2,0,0,0,4}; // left relative to the current directionmodeR       

byte pmode=0; // Program mode

// Variables whose values ​​change in interrupts must be declared as Volatile
=0, modeL=0; Mode  
volatile int stepR, stepL; // Motor rotation speed indication variable  
volatile byte patR=0, patL=0; // Motor excitation pattern phase 
volatile int cntR, cntL; // Motor rotation speed count 
volatile int sensFR, sensFL, sensR, sensL; // sensor values 
​​volatile int sensFRB, sensFLB, sensRB, sensLB; // sensor values ​​when LED is off 
volatile byte timR=0, timL=0; L motor timer 
volatile byte timS; // Light sensor timer used in interrupts 
volatile byte fS=0  
; / R, L motor low/high speed switching flag 0: low speed, 1: high speed  

union { // Array structure definition for storing map  
       byte all; // Access mmap by byte  
       struct { byte n:1;absence of wall in north direction (0: no, 1: yes)  
                byte e:1; // Presence/absence of wall in east direction (0: no, 1: yes)  
                byte s:1; // Presence/absence of wall in south direction (0: No, 1: Yes)  
                byte w: 1; // Presence or absence of wall in west direction (0: No, 1: Yes)  
                byte d: 4;  
              };
        [16][16]; 

mmap------------------------------------------------- 
// LED display                                                   
//------------------------------------------------ -------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111; // turn off the LED once
 PORTB |= (n< <4); // LED display
}

//--------------------------------------- ----------------------------------- 
// timer interrupt handling                                                  
//------------------------------------------------ -------------------------- 
void SensAndMotor(void) {    

// Motor rotation mode = 0: free, 1: forward, 2: backward, 3: brake
// right motor processing 
  if (timR>0) timR--; // count down timR, processing when timR=0
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3;     
    cntR++; // right motor step count 
  }
// left motor processing
  if (timL>0) timL--; // count down timL, process when timL=0
  if (timL==0) {
    if (fL== else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL-- ; else patL = 3; }          
    cntL++; // left motor step count 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; } // free    
  PORTB in mode0 &=0xf0; PORTB |= RMOTOR[patR]; Output
  PORTD &=0x0f; PORTD |= LMOTOR[patL]; // Pattern output to the left motor

// Sensor processing
// Get the reference value when the LED is turned off, and read the sensor value with the difference after the LED is turned on.
  if (timS<20) timS++; else timS=0; // sensor reading cycle counter  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR); // input initial value when LED is off
    sensFLB=analogRead(PT_FL)sensLB
    sensRB=analogRead(PT_R);
    (PT_L); 
    digitalWrite(LEDOUT, HIGH); // LED-ON  
    delayMicroseconds(50);
    analogRead-sensFRB; // Input initial value when LED is off 
    sensFL=analogRead(PT_FL)-sensFLB;
    sensR =analogRead(PT_R) -sensRB;
    sensL =analogRead(PT_L) -sensLB;  
    digitalWrite(LEDOUT, LOW); // LED -OFF
  }  

// Distance correction by left and right sensors //
If there is a right wall, use only the right wall and ignore the left wall. If there is only a left wall, use only the left wall.
// Standard distance (sensor value) and currentsensR
  If (fS==1){ // When correcting the distance to the wall, perform the following processing
    fR=fL=1; // Set the left and right first
    >DISRMAX ){ // If the right wall exists, adjust only the right wall  
      if ((sensR-DISR)>20) fL=0; // When the right wall is too close, slow down the left motor
      if ((sensR- DISR)<-20) fR=0; // Decelerate right motor when too far from right wall      
    } else if(sensL>DISLMAX){ // Adjustment  
      if only left wall exists if ((sensL-DISL )>20) fR=0; // The idea is the same as the right wall  
      if ((sensL-DISL)<-20) fL=0;
    }
  } else { fR=fL=0; } // When fS=0, set to slow

}

//-------------------------------------------- ------------------------------ 
// sensor adjustment, display sensor value on LED                                                                                                
//------------------------------------------------ -------------------------- 
void check_sens() {  
  while (1){   
    Serial.print(0x0c,BYTE); // page break
    Serial .print("Sensor FR:"); Serial.println(sensFR); // Output right front sensor value  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // Output left front sensor value  
    Serial .print("Sensor R:"); Serial.println(sensR); // Output right sensor value  
    Serial.print("Sensor L:"); Serial.println(sensL); // Output left sensor value 
    delay (500);
  }
}

//------------------------------------------ -------------------------------- 
//  Brake                                                                
//------------------------------------------------ -------------------------- 
void run_break(){
  modeR=0; modeL=0; // stop motor  
}

//---- -------------------------------------------------- -------------------- 
//  Angle adjustment by front wall                                                       
//------------------------------------------------ -------------------------- 
void adjust(){
  fS=0; // slow   
  while(abs((sensFR-DISFR)-(sensFL -DISFL))>20){ // Correct when the difference between the previous sensor values ​​is large
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1; // turn right
    } else { 
      modeR=1; modeL=2; // left turn
    } 
  } 
  run_break();
}

//--------------------------------------- ---------------------------------------------- 
//  When moving forward start slow                
//--------------------------------------------- ----------------------------- 
void slow_start(){
  fS=0; // slow setting
  modeR=modeL=1; // Mode setting Right: Forward, Left: Forward  
  cntR=0; stepR=20; // Advance 50 steps at low speed
  while (cntR<stepR);
                                        // If nothing is done, it will be ignored by optimization, so delay is included.  
}

//----------------------------------------------- --------------------------- 
//  step forward                
//----------------- -------------------------------------------------- ------- 
void run_step(){
  slow_start();
  fS=1; // fast the remaining distance
  cntR=0; stepR=STEP1-20;             
  while (cntR<stepR) delay(1);
  run_break();
}  

//------------------------------------------ ------------------------------- 
//  Rotate right 90 degrees                                                  
//------------------------------------------------ -------------------------- 
void run_R90(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R90; // right 90 degree rotation step count  
  modeR=2; modeL=1; // Right: backward, left: forward   
  while (cntR<stepR) delay(1);
  run_break();
}  

//--------- -------------------------------------------------- --------------- 
//  Rotate left 90 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_L90(){
  fS=0; // low speed setting  
  cntL=0; stepL=L90; // left modeR  
  =1; modeL=2; // right: forward, left: backward  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//----- -------------------------------------------------- ------------------- 
//  Rotate right 180 degrees                                                   
//------------------------------------------------ -------------------------- 
void run_R180(){ 
  fS=0; // low speed setting 
  cntR=0; stepR=R180; // right 90-degree rotation step count  
  modeR=2; modeL=1; // Right: Backward, Left: Forward   
  while (cntR<stepR);
  run_break();
}  

//------------- -------------------------------------------------- -----------
//  U-turn or right-turn travel, accelerate forward, decelerate if there is a wall, turn after distance correction
//--------------- -------------------------------------------------- ---------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1; // fast the remaining distance
    while (sensFR<DISFR); ) run to standard distance with sensor   
    adjust(); // angle adjustment by front wall   
    if (n==0) run_R180(); else run_R90(); // right 180 degree turn or right 90 degree turn
  }
} 

// -------------------------------------------------- ------------------------ 
//  Extended left method (record history)                                                 
//Determine direction with priority of left, front, right, if dead end U-turn                    
//---------------------------------------------- ---------------------------- 
void run_Hidarite(){
  byte wF, wR, wL; flag byte wS; // flag whether or not  
  wall sense has been performed  
  byte mapF, mapR, mapL; // variables used when reading the history on the map 
  byte mx,my; Initial is 0,0, y increment north, x increment east  
  byte md; // mouse direction, north: 1 , East: 2, South: 4, West: 8 
  
  =0; // Initialize map read variables 
  mx=0; my=0; md=1;  
  mapL=1; wL=1; // Start point is from forward  
  mmap[0][0].d=1; // Set initial position history to north direction 

  while (digitalRead(START)==HIGH){ / / Repeat until the blue button is pressed  

  // Read history (previous, right, left partitions in mapF, mapR, mapL)
  // However, when the current location is in the outer circumference, do not access the partitions that protrude from the mmap array .
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // if current direction is North
              if (mx<15) mapR=mmap[my][ mx+1].d; 
              if (mx>0) mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1]. d; // current direction is east
              if (my>0) mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].dcase 
              break;
      4: if (my>0) mapF=mmap[my-1][mx].d; // if current direction is south
              if (mx>0) mapR=mmap[my][mx- 1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0) mapF=mmap[my][mx-1].d; // if current direction is west
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0) mapL=mmap[my-1][mx].d; 
              break;
    }
  
// Left-hand method conditional judgment from here
    if (wL ==0 && (mapL==0 || mapL==DtoL[md])) // Turn left when there is no left wall, no entry or left turn is allowed in history  
      { run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){} // If there is no front wall and entry is possible, move forward (below elseif pass) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; } // When there is no right wall and entry is allowed turn right  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];} // U-turn  

// From here, move forward while looking at the front, rear, left, and right walls with sensors.
    wS=0; wF=0; wR=0; wL=0; // reset wall judgment flag 
    (); // slow start
    stepR=STEP1;
    fS=1;
    slow_start{ // Rotate one step.   
      if (cntR > (STEP1*2/3) && wS==0){ // Wall detection wS=1 at 2/3 steps to prevent malfunction due to step count error    
        ; // Wall detection only once set flag (wS)                               
        if (sensR > DISRMAX) wR=1; else wR=0; // detect right wall   
        if (sensL > DISLMAX) wL=1; else wL=0; // detect left wall
        if ((sensFR >DISFMAX || sensFL>DISFMAX)){ wF=1; break; } // front wall detection, break the loop if there is a front wall  
      }
    } 
    
    // When the front wall is detected and exits the loop, proceed to the front wall
    if (wF==1){                                     
      while (sensFR<DISFR); // Distance adjustment with wall 
      adjust(); // Angle adjustment with front wall
    } 

// Recorded in the coordinates before proceeding to the history (in the opposite direction of the leaving direction).
// Wall information is recorded after updating the coordinates according to the direction of the mouse.                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my] break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my] [mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w =wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); // The search ends when you return to the starting point  
  }
}

//------------------------------------- ------------------------------------- 
//  the shortest distance from the map and run the shortest                                       
// -------------------------------------------------- ------------------------ 
void run_saitan(){
  byte i,j,k,m; // General purpose variable 
  byte smap[16][16] ; // Map for finding the shortest distance  
  byte run[256]; // Array byte md to put 
  the shortest running pattern; // Mouse pointing direction, north: 1, east: 2, south: 4, west: 8 
    
// Clear the shortest distance map, assume that all unexplored areas have walls.
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0 ){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j< 15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0) mmap[i-1][j].n=1
        ; i][j].w=1; if (j>0) mmap[i][j-1].e=1;
      }
    }
  }

// Create step count map
// Set 1 to goal position, initial m The value is 1. Scan all sections and set the next number of steps (m+1) in the section where it is possible to move to the position of the number
of steps m. Repeat this and exit the loop when the start point is reached.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // set goal to 1
  m=1 ; // Initial value set for m  
  for(k=0;k<255;k++){ // Repeat up to 255 times  
    for(i=0;i<16;i++){
      for(j=0;j<16; j++){ // full parcel scan  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j] ==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0 && smap[i-1][j]==0) smap[i- 1][j]=m+1;
          if (mmap[i][j].w==0 && j>0 && smap[i][j-1]==0) smap[i][j-1 ]=m+1;
        }
      }
    }
    m++; // advance the number of steps 
    if (smap[0][0]!=0) break  
  ;
        
// Trace the step count map backwards to create a running pattern for the shortest route in run[k]
// k is the number of patterns. 1: Go straight, 2: Turn right, 3: Turn left
  m=smap[0][0]  
  ;
  -1=1;
  while (m>0){ // after reaching the goal  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap [i+1][j]==m && i<15) {run[k]=1; i++; m--; break;}
              if (mmap[i][j].e==0 && smap[ i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[ i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s= =0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--; break;}
              if (mmap[i][j].w ==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e ==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j ].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--; break;}
              if (mmap[i][ j].n==0 && smap[i+1][j]==m && i<15) {run[k ]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k ]=3; md=DtoL[md]; break;}
    k
    ++;
  }
    
// run shortest path
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90(); i++; }
    if (run[i]==3) { run_L90(); i++; }
  }
}

//------------------------- -------------------------------------------------
/ /   initialization                                                      
//------------------------------------------------ --------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111; // PB0-4 right motor, PB5 ,PB6 is LED L0,L1
  DDRD |=B11110000; // PD4-7 left motor, PD1 is TX, PD2 is SET, PD3 is START
  pinMode(LEDOUT, OUTPUT); // LED output for wall sensor
  digitalWrite(LEDOUT, LOW); // LED off initially

// Map initialization
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0 ; // first clear
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1; set 
     mmap[0][i].s=1; mmap[15][i].n=1; } // set north and south perimeter wall 
  mmap[0][0].e=1;    
 
  MsTimer2::set(1, SensAndMotor); // Period setting, 1ms 
  MsTimer2::start(); // Timer start 
  Serial.begin(9600); // Open serial port, for wall sensor check
}

//-- -------------------------------------------------- ----------------------
//    main loop                                                       
//------------------------------------------------ --------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)== LOW) { // advance program mode when set switch is pressed
      delay(10); // prevent chattering
      while(digitalRead(SET)==LOW);
      delay(10); // prevent chattering
      pmode++; if (pmode> 3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500); // wait 0.5 seconds

  // program execution
  switch(pmode){
/* 
    case 0: run_step(); 1 step run
    case 1: run_R90(); break; // turn right 90 degrees 
    case 2: run_L90(); break; // turn left 90 degrees    
    case 3: run_R180(); break; // turn 180 degrees       
*/  
    case 0: check_sens(); break; // Sensor check   
    case 1: run_Turn(0); break; // U-turn run
    case 2: run_Hidarite()   
    ; // drive the shortest path   
  }
}
