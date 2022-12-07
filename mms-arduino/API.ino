// ----- API -----

void log(String message) {
  Serial.print("log " + message + "\n");
}

int mazeWidth() {
  return getInteger("mazeWidth");
}

int mazeHeight() {
  return getInteger("mazeHeight");
}

bool wallFront() {
  return getBoolean("wallFront");
}

bool wallRight() {
  return getBoolean("wallRight");
}

bool wallLeft() {
  return getBoolean("wallLeft");
}

bool moveForward() {
  return getAck("moveForward");
}

void turnRight() {
  getAck("turnRight");
}

void turnLeft() {
  getAck("turnLeft");
}

void setWall(int x, int y, char direction) {
  Serial.print(
    "setWall " + String(x) + " " + String(y) + " " + String(direction) + "\n");
}

void clearWall(int x, int y, char direction) {
  Serial.print(
    "clearWall " + String(x) + " " + String(y) + " " + String(direction) + "\n");
}

void setColor(int x, int y, char color) {
  Serial.print(
    "setColor " + String(x) + " " + String(y) + " " + String(color) + "\n");
}

void clearColor(int x, int y) {
  Serial.print(
    "clearColor " + String(x) + " " + String(y) + "\n");
}

void clearAllColor() {
  Serial.print("clearAllColor\n");
}

void setText(int x, int y, String text) {
  Serial.print(
    "setText " + String(x) + " " + String(y) + " " + text + "\n");
}

void clearText(int x, int y) {
  Serial.print(
    "clearText " + String(x) + " " + String(y) + "\n");
}

void clearAllText() {
  Serial.print("clearAllText\n");
}

bool wasReset() {
  return getBoolean("wasReset");
}

void ackReset() {
  getAck("ackReset");
}

// ----- Helpers -----

String readline() {
  String response = "";
  while (response == "") {
    response = Serial.readStringUntil('\n');
  }
  return response;
}

String communicate(String command) {
  Serial.print(command + "\n");
  return readline();
}

bool getAck(String command) {
  String response = communicate(command);
  return response == "ack";
}

bool getBoolean(String command) {
  String response = communicate(command);
  return response == "true";
}

int getInteger(String command) {
  String response = communicate(command);
  return response.toInt();
}
void checkWalls() {
  if (wallFront()) {
    switch (currentPos.dir) {
      case 'n':
        if (horizontalWalls[currentPos.row][currentPos.col] == 2) {
          horizontalWalls[currentPos.row][currentPos.col] = 3;
        } else {
          horizontalWalls[currentPos.row][currentPos.col] = 1;
        }
        break;
      case 's':
        if (horizontalWalls[currentPos.row][currentPos.col] == 1) {
          horizontalWalls[currentPos.row][currentPos.col] = 3;
        } else {
          horizontalWalls[currentPos.row][currentPos.col] = 2;
        }
        break;
      case 'e':
        if (verticalWalls[currentPos.row][currentPos.col] == 4) {
          verticalWalls[currentPos.row][currentPos.col] = 6;
        } else {
          verticalWalls[currentPos.row][currentPos.col] = 5;
        }
        break;
      case 'w':
        if (verticalWalls[currentPos.row][currentPos.col] == 5) {
          verticalWalls[currentPos.row][currentPos.col] = 6;
        } else {
          verticalWalls[currentPos.row][currentPos.col] = 6;
        }
        break;
      default:
        log("Issue in forward at pos \t");
        log(currentPos.row);
        log(" ");
        log(currentPos.col);
        log("\n");
        break;
    }
  } else if (wallRight()) {
    // if (wallLeft()) {
    //   verticalWalls[currentPos.row][currentPos.col] = 6;
    // } else {
    //   verticalWalls[currentPos.row][currentPos.col] = 5;
    // }
    switch (currentPos.dir) {
      case 'n':
        if (horizontalWalls[currentPos.row][currentPos.col] == 2) {
          horizontalWalls[currentPos.row][currentPos.col] = 3;
        } else {
          horizontalWalls[currentPos.row][currentPos.col] = 1;
        }
        break;
      case 's':
        if (horizontalWalls[currentPos.row][currentPos.col] == 1) {
          horizontalWalls[currentPos.row][currentPos.col] = 3;
        } else {
          horizontalWalls[currentPos.row][currentPos.col] = 2;
        }
        break;
      case 'e':
        if (verticalWalls[currentPos.row][currentPos.col] == 4) {
          verticalWalls[currentPos.row][currentPos.col] = 6;
        } else {
          verticalWalls[currentPos.row][currentPos.col] = 5;
        }
        break;
      case 'w':
        if (verticalWalls[currentPos.row][currentPos.col] == 5) {
          verticalWalls[currentPos.row][currentPos.col] = 6;
        } else {
          verticalWalls[currentPos.row][currentPos.col] = 6;
        }
        break;
      default:
        log("Issue in forward at pos \t");
        log(currentPos.row);
        log(" ");
        log(currentPos.col);
        log("\n");
        break;
    }
  } else if (wallLeft()) {
    verticalWalls[currentPos.row][currentPos.col] = 4;
  }
}
