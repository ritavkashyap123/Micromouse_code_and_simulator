//----------------------------Libaries--------------------------
#include <ArduinoQueue.h>


//---------------------------- Map Matrix --------------------

int horizontalWalls[16][16] ={{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
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
                              {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2},};  // 1 for top, 2 for bottom, 3 for both    
                                                               

int verticalWalls[16][16] =  {{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
                              {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},}; // 4 for left, 5 for right, 6 for both
                            
int manhattenDistance[16][16] ={{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
                                {13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
                                {12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
                                {11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
                                {10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
                                {9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
                                {8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
                                {7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
                                {7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
                                {8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
                                {9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
                                {10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
                                {11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
                                {12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
                                {13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
                                {14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},}; 

//---------------------------- Coordinate Sys --------------------

struct pos
{
   int row, col;
   char dir; //n,s,e,w
};
//---------------------------- FF Matrix --------------------
ArduinoQueue<pos> ffQueue(512);
struct pos currentPos = {0, 0, 'e'};
// intQueue.enqueue(1);    // Adds number 1 to the queue
// intQueue.enqueue(123);    // Adds number 123 to the queue
// int number = intQueue.dequeue();    // Will return number 1 and remove it from the queue
// int number2 = intQueue.getHead();    // Will return number 123 but leave it still in the queue
// int number3 = intQueue.dequeue();    // Will return number 123 and remove it from the queue
//---------------------------- FF Matrix --------------------




void setup() {
    Serial.begin(19200);
}

void loop() {
    
    log("Running...");
    setColor(0, 0, 'G');
    setText(0, 0, "abc");
    ffQueue.enqueue(currentPos);
    
    while (true) {
        
        if (!wallLeft()) {
            turnLeft();
            log("Left");
            switch (currentPos.dir) {
            case 'n' : currentPos.dir ='w';
                       break;
            case 's' : currentPos.dir= 'e';
                       break;
            case 'e' : currentPos.dir= 'n';
                       break;
            case 'w' : currentPos.col= 's';
                       break;
           default  : log("Issue in left-turn at pos \t");
                       log((String)currentPos.row);
                       log(" ");
                       log((String)currentPos.col);
                       log("\n");
                       break;
         }
        }
        while (wallFront()) {
            turnRight();
            log("Right");
            switch (currentPos.dir) {
            case 'n' : currentPos.dir ='e';
                       break;
            case 's' : currentPos.dir= 'w';
                       break;
            case 'e' : currentPos.dir= 's';
                       break;
            case 'w' : currentPos.col= 'n';
                       break;
           default  : log("Issue in right-turn at pos \t");
                       log((String)currentPos.row);
                       log(" ");
                       log((String)currentPos.col);
                       log("\n");
                       break;
         }
        }
        moveForward();
        log("fuck");
        switch (currentPos.dir) {
            case 'n' : currentPos.row -=1;
                       break;
            case 's' : currentPos.row +=1;
                       break;
            case 'e' : currentPos.col +=1;
                       break;
            case 'w' : currentPos.col -=1;
                       break;
           default  : log("Issue in forward at pos \t");
                       log((String)currentPos.row);
                       log(" ");
                       log((String)currentPos.col);
                       log("\n");
                       break;
         }

    }
    log((String)currentPos.dir);
    log(" ");
    log((String)currentPos.row);
    log(" ");
    log((String)currentPos.col);
    log(" ");
    checkWalls();
}
