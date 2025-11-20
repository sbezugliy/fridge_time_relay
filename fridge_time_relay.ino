
#include <LiquidCrystal_I2C.h>
#include <stdio.h>

const uint8_t LCD_COLUMNS = 16;
const uint8_t LCD_ROWS = 2;
const uint8_t RELAY_PIN = 3;

LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);  // set the LCD address to 0x27 for a 16 chars and 2 line display

struct PhaseConfig {
  const char *label;
  int durationSeconds;
  bool relayConnected;
};

int minutesToSeconds(int minutes, int seconds) {
  return minutes * 60 + seconds;
}

const PhaseConfig PHASES[] = {
  {"Freeze", minutesToSeconds(30, 0), true},
  {"Rest", minutesToSeconds(45, 0), false}
};

const size_t PHASE_COUNT = sizeof(PHASES) / sizeof(PHASES[0]);

const bool SHOW_INIT_SCREEN = true;

bool relayState = false;
size_t currentPhaseIndex = 0;
int elapsedSeconds = 0;

void setup() {
  lcd.init();
  lcd.backlight();

  if (SHOW_INIT_SCREEN) {
    printInitScreen();
  }

  initPins();
  activatePhase(currentPhaseIndex);
}

void loop() {
  const PhaseConfig &phase = PHASES[currentPhaseIndex];
  displayPhase(phase);
  delay(1000);
  advanceTimer();

  if (elapsedSeconds >= phase.durationSeconds) {
    switchToNextPhase();
  }
}

void initPins() {
  pinMode(RELAY_PIN, OUTPUT);
}

void switchToNextPhase() {
  size_t nextIndex = (currentPhaseIndex + 1) % PHASE_COUNT;
  activatePhase(nextIndex);
}

void activatePhase(size_t phaseIndex) {
  currentPhaseIndex = phaseIndex;
  resetTimerState();
  applyRelayState(PHASES[currentPhaseIndex].relayConnected);
  lcd.clear();
}

void applyRelayState(bool shouldConnect) {
  if (shouldConnect) {
    relayConnect();
  } else {
    relayDisconnect();
  }
}

void advanceTimer() {
  ++elapsedSeconds;
}

void resetTimerState() {
  elapsedSeconds = 0;
}

void relayConnect() {
  relayState = true;
  digitalWrite(RELAY_PIN, HIGH);
}

void relayDisconnect() {
  relayState = false;
  digitalWrite(RELAY_PIN, LOW);
}

void displayPhase(const PhaseConfig &phase) {
  printLine(0, phase.label);

  char timerLine[17];
  buildTimerLine(timerLine, sizeof(timerLine), elapsedSeconds, phase.durationSeconds);
  printLine(1, timerLine);
}

void buildTimerLine(char *buffer, size_t length, int elapsed, int target) {
  char elapsedBuffer[8];
  char targetBuffer[8];
  formatTime(elapsedBuffer, sizeof(elapsedBuffer), elapsed);
  formatTime(targetBuffer, sizeof(targetBuffer), target);
  snprintf(buffer, length, "%s/%s", elapsedBuffer, targetBuffer);
}

void formatTime(char *buffer, size_t length, int totalSeconds) {
  int minutes = totalSeconds / 60;
  int seconds = totalSeconds % 60;
  snprintf(buffer, length, "%02d:%02d", minutes, seconds);
}

void printLine(uint8_t row, const char *text) {
  lcd.setCursor(0, row);
  uint8_t col = 0;
  while (col < LCD_COLUMNS && text[col] != '\0') {
    lcd.print(text[col]);
    ++col;
  }
  while (col < LCD_COLUMNS) {
    lcd.print(' ');
    ++col;
  }
}

void printInitScreen() {
  printLine(0, "     Fridge     ");
  printLine(1, "   Time Relay   ");
  delay(1000);
  lcd.clear();

  printLine(0, "Sergey Bezugliy");
  printLine(1, "   codenv.top   ");
  delay(1000);
  lcd.clear();
}
