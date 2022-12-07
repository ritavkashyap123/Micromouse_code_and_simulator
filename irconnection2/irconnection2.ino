//----------------------------Libaries--------------------------
#include <ArduinoQueue.h>
// All variables being used here
//---------------------------- Sensors -------------------------
int centerIR;
int nearLeftIR;
int farLeftIR;
int farRightIR;
int nearRightIR;

//---------------------------- Counter Flags --------------------
int leftFlag;
int rightFlag;
int centerFlag;

//---------------------------- Threshold Values -----------------
int sideFarThreshold = 200;
int centerThreshold = 200;

//---------------------------- Map Matrix --------------------

int horizontalWalls[16][16] ={{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},};  

int verticalWalls[16][16] ={{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},}; 
                            
int manhattenDistance[16][16] ={{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},}; 

ArduinoQueue<int> ffQueue(512);
// intQueue.enqueue(1);    // Adds number 1 to the queue
// intQueue.enqueue(123);    // Adds number 123 to the queue
// int number = intQueue.dequeue();    // Will return number 1 and remove it from the queue
// int number2 = intQueue.getHead();    // Will return number 123 but leave it still in the queue
// int number3 = intQueue.dequeue();    // Will return number 123 and remove it from the queue



//---------------------------- Function -------------------------
void counterAssign();
void selDeviation();

//---------------------------- Motion Functions -----------------
void deviateRight();
void deviateLeft();
void froward();
void reverse();
 
 //---------------------------- Setup ---------------------------

void setup(){
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);

    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);

    Serial.begin(115200);
 }

//---------------------------- Execution -------------------------
void loop(){
    // Senosor pin assignment
    centerIR = analogRead(A0);
    nearLeftIR = analogRead(A1);
    farLeftIR = analogRead(A2);
    farRightIR = analogRead(A3);
    nearRightIR = analogRead(A4);

    // Motion Operation
    counterAssign();
    selDeviation(); // Navigation directed motion
     
}

//---------------------------- Function ---------------------------
void counterAssign(){
    if (farLeftIR < sideFarThreshold){
        leftFlag = 1;
    }
    else {
        leftFlag = 0;
    }
    if (farRightIR < sideFarThreshold){
        rightFlag = 1;
    }
    else {
        rightFlag = 0;
    }
    if (centerIR < centerThreshold){
        centerFlag = 0;
    }
}

void selDeviation(){
    if ((leftFlag ==1) || (rightFlag ==1) || (centerFlag == 1)){
        if (centerFlag == 1){
            if (leftFlag ==1){
                deviateRight();
            }
            else if (rightFlag ==1)
                deviateLeft();
            else
                deviateLeft();
        }
        else if (leftFlag == 1){
            deviateRight();
        }
        else if (rightFlag == 1){
            deviateLeft();
        }
        else{
            deviateLeft();
        }
    }
    else{
        forward();
    }
}

void reverse(){
    Serial.println("Backing up");
    digitalWrite(9, LOW);
    analogWrite(6, 75);
    analogWrite(11, 75);
    digitalWrite(10, LOW);
    delay(1000);
}

void deviateRight(){
    Serial.println("Right");
    reverse();
    digitalWrite(6, LOW);
    digitalWrite(9, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, 350);
    delay(300);
}

void deviateLeft(){
    Serial.println("Left");
    reverse();
    analogWrite(9, 350);
    digitalWrite(6, LOW);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    delay(300);
}

void forward(){
    Serial.println("Forward");
    analogWrite(9, 75);
    digitalWrite(6, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, 75);
}
