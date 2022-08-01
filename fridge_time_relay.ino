#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define Relay 3

// Enable init screen
bool initScreen = true;

int timerState[2] = { 0, 0 };

int currentTick = 0;

const int connectedTime[2] = { 30, 00};

const int disconnectedTime[2] = { 45, 00};

int calculateSeconds(int *timeArray){
  return timeArray[0] * 60 + timeArray[1];
}

const int connectedTicks = calculateSeconds(connectedTime);

const int disconnectedTicks = calculateSeconds(disconnectedTime);

bool relayState = true;

void setup() {

  lcd.begin(16, 2);
  
  if (initScreen) printInitScreen();
  
  initPins();
  relayConnect();
  delay(1000);
}

void loop() {
  lcd.clear();

  connectedTimer();
  disconnectedTimer();

  delay(1000);
}

void connectedTimer(){
  if (relayState) {
    if(checkTimer(connectedTicks)) relayDisconnect();
    printConnectedTime();
  }
}

void disconnectedTimer(){
  if (!relayState) {
    if(checkTimer(disconnectedTicks)) relayConnect();
    printDisconnectedTime();
  }
}

bool checkTimer(int *ticks){
  increaseTimerState();
  if (currentTick >= ticks) {
    resetTimerState();
    return true;
  }
  return false;
}

void increaseTimerState(){
  ++currentTick;
  if(timerState[1] < 59) ++timerState[1];
  else if (timerState[1] >= 59) {
    ++timerState[0];
    timerState[1] = 0;
  }
}

void resetTimerState(){
  timerState[0] = 0;
  timerState[1] = 0;
  currentTick = 0;
}

void initPins() {
  pinMode(Relay, OUTPUT);
}

void relayConnect(){
  relayState = true;
  digitalWrite(Relay, HIGH);
};

void relayDisconnect(){
  relayState = false;
  digitalWrite(Relay, LOW);
};

void printConnectedTime(){
  lcd.setCursor(0, 0);
  lcd.print("Freeze");
  lcd.setCursor(0, 1);
  printTimer(connectedTime, connectedTicks);
}

void printDisconnectedTime(){
  lcd.setCursor(0, 0);
  lcd.print("Rest");
  lcd.setCursor(0, 1);
  printTimer(disconnectedTime, disconnectedTicks);
}

void printTimer(int *timeArray, int ticks) {
  lcd.print(timerState[0]);
  lcd.print(":");
  lcd.print(timerState[1]);
  lcd.print("/");
  lcd.print(timeArray[0]);
  lcd.print(":");
  lcd.print(timeArray[1]);
}

void printInitScreen() {
  lcd.setCursor(0, 0);
  lcd.print("     Fridge     ");
  lcd.setCursor(0, 1);
  lcd.print("   Time Relay   ");
  delay(2000);
  lcd.clear();  // Clears the display

  lcd.setCursor(0, 0);
  lcd.print("Sergey Bezugliy");
  lcd.setCursor(3, 1);
  lcd.print("codenv.top");
  delay(3000);
  lcd.clear();  // Clears the display
};

void printParameter(char *parameterAbbreviation, int startCol, int Row, int *valueRef) {
  lcd.setCursor(startCol, Row);
  lcd.print(parameterAbbreviation);
  lcd.setCursor(strlen(parameterAbbreviation) + startCol, Row);
  lcd.print(*valueRef);
}
