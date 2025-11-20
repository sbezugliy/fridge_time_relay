
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum OperationMode {
  OPERATION_MODE_TIMER = 0,
  OPERATION_MODE_THERMOSTAT = 1
};

const uint8_t LCD_COLUMNS = 16;
const uint8_t LCD_ROWS = 2;
const uint8_t RELAY_PIN = 3;
const uint8_t TEMPERATURE_SENSOR_PIN = 2;
const uint8_t DOOR_SENSOR_PIN = 4;

const bool ENABLE_TEMPERATURE_SENSOR = true;
const bool ENABLE_TEMPERATURE_REGULATION = true;
const bool ENABLE_DOOR_SENSOR = true;
const unsigned long TEMPERATURE_CONVERSION_INTERVAL_MS = 1000UL;
const float THERMOSTAT_HYSTERESIS_C = 1.0f;
const float DEFAULT_THERMOSTAT_TARGET_C = -12.0f;
const float THERMOSTAT_MIN_TARGET_C = -30.0f;
const float THERMOSTAT_MAX_TARGET_C = 5.0f;
const float DEFAULT_TIMER_STOP_TEMP_C = -18.0f;
const float DEFAULT_TIMER_RESUME_TEMP_C = -15.0f;
const float TIMER_TEMP_MIN_C = -40.0f;
const float TIMER_TEMP_MAX_C = 15.0f;
const unsigned long DEFAULT_FREEZE_DURATION_SECONDS = 30UL * 60UL;
const unsigned long DEFAULT_WAIT_DURATION_SECONDS = 45UL * 60UL;
const unsigned long MIN_PHASE_DURATION_SECONDS = 30UL;
const unsigned long MAX_PHASE_DURATION_SECONDS = 6UL * 60UL * 60UL;
const char DOOR_INDICATOR_OPEN[] = "<>";
const char DOOR_INDICATOR_CLOSED[] = "[]";
const char DOOR_INDICATOR_UNKNOWN[] = "??";
const uint8_t CONFIG_VERSION = 2;
const int CONFIG_EEPROM_ADDRESS = 0;
const uint8_t ENCODER_PIN_A = 5;
const uint8_t ENCODER_PIN_B = 6;
const uint8_t ENCODER_BUTTON_PIN = 7;
const uint16_t DISPLAY_REFRESH_INTERVAL_MS = 250;
const uint16_t DOOR_SAMPLE_INTERVAL_MS = 50;
const uint16_t TEMPERATURE_POLL_INTERVAL_MS = 250;
const uint16_t TIMER_SECOND_INTERVAL_MS = 1000;
const uint16_t THERMOSTAT_CONTROL_INTERVAL_MS = 500;
const uint16_t ENCODER_SAMPLE_INTERVAL_MS = 5;

LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);  // set the LCD address to 0x27 for a 16 chars and 2 line display
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature temperatureSensor(&oneWire);

struct DeviceConfig {
  uint8_t version;
  uint8_t mode;
  float thermostatTargetC;
  uint32_t freezeDurationSeconds;
  uint32_t waitDurationSeconds;
  float timerStopTempC;
  float timerResumeTempC;
};

struct PhaseConfig {
  const char *label;
  int durationSeconds;
  bool relayConnected;
};

int minutesToSeconds(int minutes, int seconds) {
  return minutes * 60 + seconds;
}

PhaseConfig PHASES[] = {
  {"Freeze", static_cast<int>(DEFAULT_FREEZE_DURATION_SECONDS), true},
  {"Rest", static_cast<int>(DEFAULT_WAIT_DURATION_SECONDS), false}
};

const size_t PHASE_COUNT = sizeof(PHASES) / sizeof(PHASES[0]);

const bool SHOW_INIT_SCREEN = true;

enum UiState {
  UI_STATE_IDLE,
  UI_STATE_MENU_SELECT,
  UI_STATE_EDIT_VALUE
};

enum MenuItemType {
  MENU_ITEM_MODE,
  MENU_ITEM_FREEZE_DURATION,
  MENU_ITEM_WAIT_DURATION,
  MENU_ITEM_STOP_TEMP,
  MENU_ITEM_RESUME_TEMP,
  MENU_ITEM_THERMOSTAT_TARGET,
  MENU_ITEM_EXIT
};

struct MenuItem {
  const char *label;
  MenuItemType type;
};

const MenuItem MENU_ITEMS[] = {
  {"Mode", MENU_ITEM_MODE},
  {"Freeze", MENU_ITEM_FREEZE_DURATION},
  {"Rest", MENU_ITEM_WAIT_DURATION},
  {"Stop", MENU_ITEM_STOP_TEMP},
  {"Resume", MENU_ITEM_RESUME_TEMP},
  {"Thermo", MENU_ITEM_THERMOSTAT_TARGET},
  {"Exit", MENU_ITEM_EXIT}
};

const size_t MENU_ITEM_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);

struct EditState {
  MenuItemType itemType;
  OperationMode modeValue;
  unsigned long secondsValue;
  float floatValue;
};

struct EncoderRuntimeState {
  bool pinALast;
  bool pinBLast;
  bool buttonLastSample;
  bool buttonStableState;
  uint8_t buttonStableCounter;
};

bool relayState = false;
bool desiredRelayState = false;
size_t currentPhaseIndex = 0;
int elapsedSeconds = 0;
bool temperatureSensorAvailable = false;
bool temperatureConversionPending = false;
bool timerCoolingPauseActive = false;
float currentTemperatureC = 0.0f;
unsigned long lastTemperatureRequestMillis = 0;
DeviceConfig deviceConfig;
bool thermostatCoolingDemand = false;
bool doorSensorAvailable = false;
bool doorOpen = false;
UiState uiState = UI_STATE_IDLE;
size_t menuSelectionIndex = 0;
EditState editState = {MENU_ITEM_MODE, OPERATION_MODE_TIMER, 0UL, 0.0f};
EncoderRuntimeState encoderState = {true, true, true, true, 0};
int16_t encoderPendingDelta = 0;
bool encoderButtonClicked = false;
volatile bool displayUpdateRequested = false;
volatile bool temperatureUpdateRequested = false;
volatile bool doorSampleRequested = false;
volatile bool timerSecondTickRequested = false;
volatile bool thermostatControlRequested = false;
volatile bool encoderSampleRequested = false;

void handleTimerSecondTick();
void initEncoderInput();
void updateEncoderInput();
int16_t consumeEncoderDelta();
bool consumeEncoderButtonClick();
void processEncoderInterface();
void enterMenuSelection();
void exitMenu();
void startEditingCurrentItem();
void adjustEditValue(int16_t delta);
void commitEditValue();
void displayMenuScreen();
void buildMenuValueLine(char *buffer, size_t length, const MenuItem &item, bool editing);
void formatDuration(char *buffer, size_t length, unsigned long totalSeconds);
const char *doorIndicatorSymbol();
void appendDoorIndicator(char *line, size_t length);
void refreshDisplay();

void requestDisplayRefresh() {
  noInterrupts();
  displayUpdateRequested = true;
  interrupts();
}

void requestSchedulerKickstart() {
  noInterrupts();
  displayUpdateRequested = true;
  temperatureUpdateRequested = true;
  doorSampleRequested = true;
  thermostatControlRequested = true;
  encoderSampleRequested = true;
  interrupts();
}

bool consumeSchedulerFlag(volatile bool &flag) {
  bool pending = false;
  noInterrupts();
  if (flag) {
    pending = true;
    flag = false;
  }
  interrupts();
  return pending;
}

void initTaskScheduler() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 249;  // 1 ms tick with 64 prescaler (16 MHz / 64 / 1000)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void initEncoderInput() {
  encoderState.pinALast = digitalRead(ENCODER_PIN_A);
  encoderState.pinBLast = digitalRead(ENCODER_PIN_B);
  bool buttonPressed = digitalRead(ENCODER_BUTTON_PIN) == LOW;
  encoderState.buttonLastSample = buttonPressed;
  encoderState.buttonStableState = buttonPressed;
  encoderState.buttonStableCounter = 0;
  encoderPendingDelta = 0;
  encoderButtonClicked = false;
}

void updateEncoderInput() {
  bool pinA = digitalRead(ENCODER_PIN_A);
  bool pinB = digitalRead(ENCODER_PIN_B);

  if (pinA != encoderState.pinALast) {
    if (pinA == HIGH) {
      if (pinB == HIGH) {
        ++encoderPendingDelta;
      } else {
        --encoderPendingDelta;
      }
    }
    encoderState.pinALast = pinA;
    encoderState.pinBLast = pinB;
  }

  bool buttonPressed = digitalRead(ENCODER_BUTTON_PIN) == LOW;
  if (buttonPressed == encoderState.buttonLastSample) {
    if (encoderState.buttonStableCounter < 5) {
      ++encoderState.buttonStableCounter;
    }
  } else {
    encoderState.buttonStableCounter = 0;
    encoderState.buttonLastSample = buttonPressed;
  }

  if (encoderState.buttonStableCounter >= 3 && buttonPressed != encoderState.buttonStableState) {
    encoderState.buttonStableState = buttonPressed;
    if (!buttonPressed) {
      encoderButtonClicked = true;
    }
  }
}

int16_t consumeEncoderDelta() {
  int16_t delta = encoderPendingDelta;
  encoderPendingDelta = 0;
  return delta;
}

bool consumeEncoderButtonClick() {
  bool clicked = encoderButtonClicked;
  encoderButtonClicked = false;
  return clicked;
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  lcd.init();
  lcd.backlight();

  if (SHOW_INIT_SCREEN) {
    printInitScreen();
  }

  loadConfig();
  initPins();
  initEncoderInput();
  initTemperatureSensor();
  initTaskScheduler();
  requestSchedulerKickstart();
  applyOperationMode(deviceConfig.mode);
  printConfigToSerial();
}

void loop() {
  handleSerialCommands();

  if (consumeSchedulerFlag(doorSampleRequested)) {
    updateDoorSensor();
  }

  if (consumeSchedulerFlag(temperatureUpdateRequested)) {
    updateTemperature();
  }

  if (consumeSchedulerFlag(timerSecondTickRequested) && deviceConfig.mode == OPERATION_MODE_TIMER) {
    handleTimerSecondTick();
  }

  if (consumeSchedulerFlag(thermostatControlRequested) && deviceConfig.mode == OPERATION_MODE_THERMOSTAT) {
    updateThermostatControl();
  }

  if (consumeSchedulerFlag(encoderSampleRequested)) {
    updateEncoderInput();
  }

  if (consumeSchedulerFlag(displayUpdateRequested)) {
    refreshDisplay();
  }

  processEncoderInterface();
}

void enterMenuSelection() {
  if (uiState != UI_STATE_IDLE) {
    return;
  }
  uiState = UI_STATE_MENU_SELECT;
  menuSelectionIndex = 0;
  requestDisplayRefresh();
}

void exitMenu() {
  uiState = UI_STATE_IDLE;
  requestDisplayRefresh();
}

void startEditingCurrentItem() {
  const MenuItem &item = MENU_ITEMS[menuSelectionIndex];
  editState.itemType = item.type;
  switch (item.type) {
    case MENU_ITEM_MODE:
      editState.modeValue = static_cast<OperationMode>(deviceConfig.mode);
      break;
    case MENU_ITEM_FREEZE_DURATION:
      editState.secondsValue = deviceConfig.freezeDurationSeconds;
      break;
    case MENU_ITEM_WAIT_DURATION:
      editState.secondsValue = deviceConfig.waitDurationSeconds;
      break;
    case MENU_ITEM_STOP_TEMP:
      editState.floatValue = deviceConfig.timerStopTempC;
      break;
    case MENU_ITEM_RESUME_TEMP:
      editState.floatValue = deviceConfig.timerResumeTempC;
      break;
    case MENU_ITEM_THERMOSTAT_TARGET:
      editState.floatValue = deviceConfig.thermostatTargetC;
      break;
    case MENU_ITEM_EXIT:
      break;
  }
  if (item.type != MENU_ITEM_EXIT) {
    uiState = UI_STATE_EDIT_VALUE;
    requestDisplayRefresh();
  }
}

void adjustEditValue(int16_t delta) {
  if (delta == 0) {
    return;
  }

  bool changed = false;
  switch (editState.itemType) {
    case MENU_ITEM_MODE: {
      int value = static_cast<int>(editState.modeValue);
      value += delta;
      if (value < 0) {
        value = 0;
      }
      if (value > 1) {
        value = 1;
      }
      OperationMode nextMode = value == 0 ? OPERATION_MODE_TIMER : OPERATION_MODE_THERMOSTAT;
      if (nextMode != editState.modeValue) {
        editState.modeValue = nextMode;
        changed = true;
      }
      break;
    }
    case MENU_ITEM_FREEZE_DURATION:
    case MENU_ITEM_WAIT_DURATION: {
      long updated = static_cast<long>(editState.secondsValue) + static_cast<long>(delta) * 60L;
      if (updated < static_cast<long>(MIN_PHASE_DURATION_SECONDS)) {
        updated = static_cast<long>(MIN_PHASE_DURATION_SECONDS);
      }
      if (updated > static_cast<long>(MAX_PHASE_DURATION_SECONDS)) {
        updated = static_cast<long>(MAX_PHASE_DURATION_SECONDS);
      }
      if (static_cast<unsigned long>(updated) != editState.secondsValue) {
        editState.secondsValue = static_cast<unsigned long>(updated);
        changed = true;
      }
      break;
    }
    case MENU_ITEM_STOP_TEMP: {
      float updated = editState.floatValue + static_cast<float>(delta) * 0.5f;
      float maxAllowed = deviceConfig.timerResumeTempC - 0.5f;
      if (maxAllowed < TIMER_TEMP_MIN_C) {
        maxAllowed = TIMER_TEMP_MIN_C;
      }
      if (updated > maxAllowed) {
        updated = maxAllowed;
      }
      if (updated < TIMER_TEMP_MIN_C) {
        updated = TIMER_TEMP_MIN_C;
      }
      if (updated > TIMER_TEMP_MAX_C) {
        updated = TIMER_TEMP_MAX_C;
      }
      if (updated != editState.floatValue) {
        editState.floatValue = updated;
        changed = true;
      }
      break;
    }
    case MENU_ITEM_RESUME_TEMP: {
      float updated = editState.floatValue + static_cast<float>(delta) * 0.5f;
      float minAllowed = deviceConfig.timerStopTempC + 0.5f;
      if (updated < minAllowed) {
        updated = minAllowed;
      }
      if (updated < TIMER_TEMP_MIN_C) {
        updated = TIMER_TEMP_MIN_C;
      }
      if (updated > TIMER_TEMP_MAX_C) {
        updated = TIMER_TEMP_MAX_C;
      }
      if (updated != editState.floatValue) {
        editState.floatValue = updated;
        changed = true;
      }
      break;
    }
    case MENU_ITEM_THERMOSTAT_TARGET: {
      float updated = editState.floatValue + static_cast<float>(delta) * 0.5f;
      if (updated < THERMOSTAT_MIN_TARGET_C) {
        updated = THERMOSTAT_MIN_TARGET_C;
      }
      if (updated > THERMOSTAT_MAX_TARGET_C) {
        updated = THERMOSTAT_MAX_TARGET_C;
      }
      if (updated != editState.floatValue) {
        editState.floatValue = updated;
        changed = true;
      }
      break;
    }
    case MENU_ITEM_EXIT:
      break;
  }

  if (changed) {
    requestDisplayRefresh();
  }
}

void commitEditValue() {
  switch (editState.itemType) {
    case MENU_ITEM_MODE:
      setOperationMode(editState.modeValue);
      break;
    case MENU_ITEM_FREEZE_DURATION:
      setFreezeDurationSeconds(editState.secondsValue);
      break;
    case MENU_ITEM_WAIT_DURATION:
      setWaitDurationSeconds(editState.secondsValue);
      break;
    case MENU_ITEM_STOP_TEMP:
      setTimerTempRange(editState.floatValue, deviceConfig.timerResumeTempC);
      break;
    case MENU_ITEM_RESUME_TEMP:
      setTimerTempRange(deviceConfig.timerStopTempC, editState.floatValue);
      break;
    case MENU_ITEM_THERMOSTAT_TARGET:
      setThermostatTarget(editState.floatValue);
      break;
    case MENU_ITEM_EXIT:
      break;
  }
  uiState = UI_STATE_MENU_SELECT;
  requestDisplayRefresh();
}

void processEncoderInterface() {
  int16_t delta = consumeEncoderDelta();
  bool click = consumeEncoderButtonClick();

  if (delta == 0 && !click) {
    return;
  }

  if (uiState == UI_STATE_IDLE && (delta != 0 || click)) {
    enterMenuSelection();
    if (click) {
      click = false;
    }
  }

  if (uiState == UI_STATE_MENU_SELECT) {
    if (delta != 0) {
      int index = static_cast<int>(menuSelectionIndex);
      index += delta;
      while (index < 0) {
        index += static_cast<int>(MENU_ITEM_COUNT);
      }
      while (index >= static_cast<int>(MENU_ITEM_COUNT)) {
        index -= static_cast<int>(MENU_ITEM_COUNT);
      }
      if (menuSelectionIndex != static_cast<size_t>(index)) {
        menuSelectionIndex = static_cast<size_t>(index);
        requestDisplayRefresh();
      }
    }
    if (click) {
      const MenuItem &item = MENU_ITEMS[menuSelectionIndex];
      if (item.type == MENU_ITEM_EXIT) {
        exitMenu();
      } else {
        startEditingCurrentItem();
      }
    }
  } else if (uiState == UI_STATE_EDIT_VALUE) {
    if (delta != 0) {
      adjustEditValue(delta);
    }
    if (click) {
      commitEditValue();
    }
  }
}

void initPins() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  if (ENABLE_DOOR_SENSOR) {
    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  }
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
}

void switchToNextPhase() {
  size_t nextIndex = (currentPhaseIndex + 1) % PHASE_COUNT;
  activatePhase(nextIndex);
}

void handleTimerSecondTick() {
  const PhaseConfig &phase = PHASES[currentPhaseIndex];
  advanceTimer();
  if (elapsedSeconds >= phase.durationSeconds) {
    switchToNextPhase();
  }
}

void activatePhase(size_t phaseIndex) {
  currentPhaseIndex = phaseIndex;
  resetTimerState();
  applyRelayState(PHASES[currentPhaseIndex].relayConnected);
  if (uiState == UI_STATE_IDLE) {
    lcd.clear();
  }
  requestDisplayRefresh();
}

void applyRelayState(bool shouldConnect) {
  desiredRelayState = shouldConnect;
  updateRelayOutput();
}

void refreshDisplay() {
  if (uiState == UI_STATE_IDLE) {
    if (deviceConfig.mode == OPERATION_MODE_TIMER) {
      displayPhase(PHASES[currentPhaseIndex]);
    } else {
      displayThermostatScreen();
    }
  } else {
    displayMenuScreen();
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
  char phaseLine[17];
  buildStatusLine(phaseLine, sizeof(phaseLine), phase.label);
  printLine(0, phaseLine);

  char timerLine[17];
  buildTimerLine(timerLine, sizeof(timerLine), elapsedSeconds, phase.durationSeconds);
  printLine(1, timerLine);
}

void displayThermostatScreen() {
  char statusLine[17];
  buildStatusLine(statusLine, sizeof(statusLine), "Thermo");
  printLine(0, statusLine);

  char statsLine[17];
  buildThermostatLine(statsLine, sizeof(statsLine));
  printLine(1, statsLine);
}

void displayMenuScreen() {
  const MenuItem &item = MENU_ITEMS[menuSelectionIndex];
  const char *prefix = uiState == UI_STATE_MENU_SELECT ? "Menu" : "Edit";
  char headerText[17];
  snprintf(headerText, sizeof(headerText), "%-4s %-11s", prefix, item.label);
  char headerLine[17];
  snprintf(headerLine, sizeof(headerLine), "%-16s", headerText);
  appendDoorIndicator(headerLine, sizeof(headerLine));
  printLine(0, headerLine);

  char valueLine[17];
  buildMenuValueLine(valueLine, sizeof(valueLine), item, uiState == UI_STATE_EDIT_VALUE);
  printLine(1, valueLine);
}

void buildMenuValueLine(char *buffer, size_t length, const MenuItem &item, bool editing) {
  char value[13];
  value[0] = '\0';
  switch (item.type) {
    case MENU_ITEM_MODE:
      strncpy(value, deviceConfig.mode == OPERATION_MODE_TIMER ? "Timer" : "Thermo", sizeof(value) - 1);
      value[sizeof(value) - 1] = '\0';
      if (editing) {
        strncpy(value, editState.modeValue == OPERATION_MODE_TIMER ? "Timer" : "Thermo", sizeof(value) - 1);
        value[sizeof(value) - 1] = '\0';
      }
      break;
    case MENU_ITEM_FREEZE_DURATION:
    case MENU_ITEM_WAIT_DURATION: {
      unsigned long seconds = (item.type == MENU_ITEM_FREEZE_DURATION) ? deviceConfig.freezeDurationSeconds : deviceConfig.waitDurationSeconds;
      if (editing) {
        seconds = editState.secondsValue;
      }
      formatDuration(value, sizeof(value), seconds);
      break;
    }
    case MENU_ITEM_STOP_TEMP: {
      float temp = deviceConfig.timerStopTempC;
      if (editing) {
        temp = editState.floatValue;
      }
      snprintf(value, sizeof(value), "%4.1fC", temp);
      break;
    }
    case MENU_ITEM_RESUME_TEMP: {
      float temp = deviceConfig.timerResumeTempC;
      if (editing) {
        temp = editState.floatValue;
      }
      snprintf(value, sizeof(value), "%4.1fC", temp);
      break;
    }
    case MENU_ITEM_THERMOSTAT_TARGET: {
      float temp = deviceConfig.thermostatTargetC;
      if (editing) {
        temp = editState.floatValue;
      }
      snprintf(value, sizeof(value), "%4.1fC", temp);
      break;
    }
    case MENU_ITEM_EXIT:
      strncpy(value, "Press", sizeof(value) - 1);
      value[sizeof(value) - 1] = '\0';
      break;
  }

  const char *prefix = editing ? "Set:" : "Cur:";
  if (item.type == MENU_ITEM_EXIT) {
    snprintf(buffer, length, "%-16s", "Press to exit");
  } else if (editing) {
    snprintf(buffer, length, "%s%-10sOK", prefix, value);
  } else {
    snprintf(buffer, length, "%s%-12s", prefix, value);
  }
}

void formatDuration(char *buffer, size_t length, unsigned long totalSeconds) {
  unsigned long minutes = totalSeconds / 60UL;
  unsigned long seconds = totalSeconds % 60UL;
  snprintf(buffer, length, "%3lu:%02lu", minutes, seconds);
}

const char *doorIndicatorSymbol() {
  if (!ENABLE_DOOR_SENSOR) {
    return "  ";
  }
  if (!doorSensorAvailable) {
    return DOOR_INDICATOR_UNKNOWN;
  }
  return doorOpen ? DOOR_INDICATOR_OPEN : DOOR_INDICATOR_CLOSED;
}

void appendDoorIndicator(char *line, size_t length) {
  if (length < LCD_COLUMNS + 1) {
    return;
  }
  const char *indicator = doorIndicatorSymbol();
  for (size_t i = 0; i < LCD_COLUMNS; ++i) {
    if (line[i] == '\0') {
      line[i] = ' ';
    }
  }
  line[LCD_COLUMNS - 2] = indicator[0];
  line[LCD_COLUMNS - 1] = indicator[1];
  line[LCD_COLUMNS] = '\0';
}

void buildStatusLine(char *buffer, size_t length, const char *label) {
  if (length < LCD_COLUMNS + 1) {
    return;
  }

  char content[LCD_COLUMNS - 1];  // leave room for indicator and null
  if (ENABLE_TEMPERATURE_SENSOR && temperatureSensorAvailable) {
    snprintf(content, sizeof(content), "%-8.8s%5.1fC", label, currentTemperatureC);
  } else {
    snprintf(content, sizeof(content), "%-14.14s", label);
  }

  const char *indicator = doorIndicatorSymbol();

  size_t mainLength = LCD_COLUMNS - 2;
  for (size_t i = 0; i < mainLength; ++i) {
    char c = content[i];
    if (c == '\0') {
      buffer[i] = ' ';
    } else {
      buffer[i] = c;
    }
  }

  buffer[mainLength] = indicator[0];
  buffer[mainLength + 1] = indicator[1];
  buffer[LCD_COLUMNS] = '\0';
}

void buildThermostatLine(char *buffer, size_t length) {
  const char *relayLabel = relayState ? "R:ON" : "R:OFF";
  if (ENABLE_TEMPERATURE_SENSOR && temperatureSensorAvailable) {
    snprintf(buffer, length, "%-5s%5.1f/%5.1f", relayLabel, currentTemperatureC, deviceConfig.thermostatTargetC);
  } else {
    snprintf(buffer, length, "%-5sTarget%5.1f", relayLabel, deviceConfig.thermostatTargetC);
  }
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

DeviceConfig makeDefaultConfig() {
  DeviceConfig config;
  config.version = CONFIG_VERSION;
  config.mode = OPERATION_MODE_TIMER;
  config.thermostatTargetC = DEFAULT_THERMOSTAT_TARGET_C;
  config.freezeDurationSeconds = DEFAULT_FREEZE_DURATION_SECONDS;
  config.waitDurationSeconds = DEFAULT_WAIT_DURATION_SECONDS;
  config.timerStopTempC = DEFAULT_TIMER_STOP_TEMP_C;
  config.timerResumeTempC = DEFAULT_TIMER_RESUME_TEMP_C;
  return config;
}

void applyConfigBounds(DeviceConfig &config) {
  if (config.mode != OPERATION_MODE_TIMER && config.mode != OPERATION_MODE_THERMOSTAT) {
    config.mode = OPERATION_MODE_TIMER;
  }

  if (config.thermostatTargetC < THERMOSTAT_MIN_TARGET_C || config.thermostatTargetC > THERMOSTAT_MAX_TARGET_C) {
    config.thermostatTargetC = DEFAULT_THERMOSTAT_TARGET_C;
  }

  if (config.freezeDurationSeconds < MIN_PHASE_DURATION_SECONDS || config.freezeDurationSeconds > MAX_PHASE_DURATION_SECONDS) {
    config.freezeDurationSeconds = DEFAULT_FREEZE_DURATION_SECONDS;
  }

  if (config.waitDurationSeconds < MIN_PHASE_DURATION_SECONDS || config.waitDurationSeconds > MAX_PHASE_DURATION_SECONDS) {
    config.waitDurationSeconds = DEFAULT_WAIT_DURATION_SECONDS;
  }

  if (config.timerStopTempC < TIMER_TEMP_MIN_C || config.timerStopTempC > TIMER_TEMP_MAX_C) {
    config.timerStopTempC = DEFAULT_TIMER_STOP_TEMP_C;
  }

  if (config.timerResumeTempC < TIMER_TEMP_MIN_C || config.timerResumeTempC > TIMER_TEMP_MAX_C) {
    config.timerResumeTempC = DEFAULT_TIMER_RESUME_TEMP_C;
  }

  if (config.timerStopTempC >= config.timerResumeTempC) {
    config.timerStopTempC = DEFAULT_TIMER_STOP_TEMP_C;
    config.timerResumeTempC = DEFAULT_TIMER_RESUME_TEMP_C;
  }
}

void saveConfig() {
  deviceConfig.version = CONFIG_VERSION;
  EEPROM.put(CONFIG_EEPROM_ADDRESS, deviceConfig);
}

void loadConfig() {
  EEPROM.get(CONFIG_EEPROM_ADDRESS, deviceConfig);
  if (deviceConfig.version != CONFIG_VERSION) {
    deviceConfig = makeDefaultConfig();
    saveConfig();
  } else {
    applyConfigBounds(deviceConfig);
  }
  syncPhaseDurationsFromConfig();
}

void syncPhaseDurationsFromConfig() {
  PHASES[0].durationSeconds = static_cast<int>(deviceConfig.freezeDurationSeconds);
  PHASES[1].durationSeconds = static_cast<int>(deviceConfig.waitDurationSeconds);
}

void setOperationMode(OperationMode mode) {
  if (deviceConfig.mode == mode) {
    applyOperationMode(mode);
    return;
  }

  deviceConfig.mode = mode;
  saveConfig();
  applyOperationMode(mode);
  Serial.print(F("Mode updated: "));
  Serial.println(mode == OPERATION_MODE_TIMER ? F("timer") : F("thermostat"));
}

void applyOperationMode(uint8_t modeValue) {
  OperationMode mode = modeValue == OPERATION_MODE_THERMOSTAT ? OPERATION_MODE_THERMOSTAT : OPERATION_MODE_TIMER;
  deviceConfig.mode = mode;

  if (mode == OPERATION_MODE_TIMER) {
    syncPhaseDurationsFromConfig();
    timerCoolingPauseActive = false;
    currentPhaseIndex = 0;
    activatePhase(currentPhaseIndex);
  } else {
    elapsedSeconds = 0;
    thermostatCoolingDemand = false;
    desiredRelayState = false;
    timerCoolingPauseActive = false;
    relayDisconnect();
    lcd.clear();
    requestDisplayRefresh();
    noInterrupts();
    thermostatControlRequested = true;
    interrupts();
  }
}

void setThermostatTarget(float targetC) {
  if (targetC < THERMOSTAT_MIN_TARGET_C) {
    targetC = THERMOSTAT_MIN_TARGET_C;
  } else if (targetC > THERMOSTAT_MAX_TARGET_C) {
    targetC = THERMOSTAT_MAX_TARGET_C;
  }

  deviceConfig.thermostatTargetC = targetC;
  saveConfig();
  Serial.print(F("Thermostat target updated: "));
  Serial.print(targetC, 1);
  Serial.println(F(" C"));

  if (deviceConfig.mode == OPERATION_MODE_THERMOSTAT) {
    updateThermostatControl();
  }
  requestDisplayRefresh();
}

void setFreezeDurationSeconds(unsigned long seconds) {
  if (seconds < MIN_PHASE_DURATION_SECONDS) {
    seconds = MIN_PHASE_DURATION_SECONDS;
  } else if (seconds > MAX_PHASE_DURATION_SECONDS) {
    seconds = MAX_PHASE_DURATION_SECONDS;
  }

  deviceConfig.freezeDurationSeconds = seconds;
  saveConfig();
  syncPhaseDurationsFromConfig();
  Serial.print(F("Freeze time updated: "));
  Serial.print(seconds / 60);
  Serial.print(F("m "));
  Serial.print(seconds % 60);
  Serial.println(F("s"));

  if (deviceConfig.mode == OPERATION_MODE_TIMER && currentPhaseIndex == 0 && elapsedSeconds > PHASES[0].durationSeconds) {
    elapsedSeconds = PHASES[0].durationSeconds;
  }
  requestDisplayRefresh();
}

void setWaitDurationSeconds(unsigned long seconds) {
  if (seconds < MIN_PHASE_DURATION_SECONDS) {
    seconds = MIN_PHASE_DURATION_SECONDS;
  } else if (seconds > MAX_PHASE_DURATION_SECONDS) {
    seconds = MAX_PHASE_DURATION_SECONDS;
  }

  deviceConfig.waitDurationSeconds = seconds;
  saveConfig();
  syncPhaseDurationsFromConfig();
  Serial.print(F("Wait time updated: "));
  Serial.print(seconds / 60);
  Serial.print(F("m "));
  Serial.print(seconds % 60);
  Serial.println(F("s"));

  if (deviceConfig.mode == OPERATION_MODE_TIMER && currentPhaseIndex == 1 && elapsedSeconds > PHASES[1].durationSeconds) {
    elapsedSeconds = PHASES[1].durationSeconds;
  }
  requestDisplayRefresh();
}

void setTimerTempRange(float stopTempC, float resumeTempC) {
  if (stopTempC < TIMER_TEMP_MIN_C) {
    stopTempC = TIMER_TEMP_MIN_C;
  }
  if (stopTempC > TIMER_TEMP_MAX_C) {
    stopTempC = TIMER_TEMP_MAX_C;
  }
  if (resumeTempC < TIMER_TEMP_MIN_C) {
    resumeTempC = TIMER_TEMP_MIN_C;
  }
  if (resumeTempC > TIMER_TEMP_MAX_C) {
    resumeTempC = TIMER_TEMP_MAX_C;
  }

  if (stopTempC >= resumeTempC) {
    Serial.println(F("Stop temp must be lower than resume temp."));
    return;
  }

  deviceConfig.timerStopTempC = stopTempC;
  deviceConfig.timerResumeTempC = resumeTempC;
  saveConfig();
  Serial.print(F("Timer temp range updated: stop "));
  Serial.print(stopTempC, 1);
  Serial.print(F(" C, resume "));
  Serial.print(resumeTempC, 1);
  Serial.println(F(" C"));
  updateRelayOutput();
  requestDisplayRefresh();
}

void updateDoorSensor() {
  if (!ENABLE_DOOR_SENSOR) {
    return;
  }

  bool previousAvailable = doorSensorAvailable;
  bool previousState = doorOpen;
  doorSensorAvailable = true;
  doorOpen = digitalRead(DOOR_SENSOR_PIN) == HIGH;

  if (!previousAvailable || previousState != doorOpen) {
    requestDisplayRefresh();
  }
}

void printConfigToSerial() {
  Serial.println(F("--- Device Configuration ---"));
  Serial.print(F("Mode: "));
  Serial.println(deviceConfig.mode == OPERATION_MODE_TIMER ? F("timer") : F("thermostat"));
  Serial.print(F("Thermostat target: "));
  Serial.print(deviceConfig.thermostatTargetC, 1);
  Serial.println(F(" C"));
  Serial.print(F("Freeze time: "));
  Serial.print(deviceConfig.freezeDurationSeconds / 60);
  Serial.print(F("m "));
  Serial.print(deviceConfig.freezeDurationSeconds % 60);
  Serial.println(F("s"));
  Serial.print(F("Wait time: "));
  Serial.print(deviceConfig.waitDurationSeconds / 60);
  Serial.print(F("m "));
  Serial.print(deviceConfig.waitDurationSeconds % 60);
  Serial.println(F("s"));
  Serial.print(F("Timer stop/resume: "));
  Serial.print(deviceConfig.timerStopTempC, 1);
  Serial.print(F(" / "));
  Serial.print(deviceConfig.timerResumeTempC, 1);
  Serial.println(F(" C"));
  Serial.println(F("Commands: mode, temp, frost time/temp, wait time, status, help"));
}

void printHelpToSerial() {
  Serial.println(F("Available commands:"));
  Serial.println(F("  mode timer   - run by timers"));
  Serial.println(F("  mode thermo  - run by temperature"));
  Serial.println(F("  temp <value> - set thermostat target in C"));
  Serial.println(F("  frost time <mm:ss> - set freeze duration"));
  Serial.println(F("  wait time <mm:ss>  - set rest duration"));
  Serial.println(F("  frost temp <stop> <resume> - set timer temp limits"));
  Serial.println(F("  status       - print current config"));
  Serial.println(F("  help         - show this help"));
}

const char *skipSpaces(const char *text) {
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }
  return text;
}

void trimInPlace(char *text) {
  if (text == nullptr) {
    return;
  }

  size_t length = strlen(text);
  size_t start = 0;
  while (start < length && isspace(static_cast<unsigned char>(text[start]))) {
    ++start;
  }

  size_t end = length;
  while (end > start && isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }

  if (start > 0) {
    memmove(text, text + start, end - start);
  }
  text[end - start] = '\0';
}

bool equalsIgnoreCase(const char *lhs, const char *rhs) {
  while (*lhs != '\0' && *rhs != '\0') {
    if (tolower(static_cast<unsigned char>(*lhs)) != tolower(static_cast<unsigned char>(*rhs))) {
      return false;
    }
    ++lhs;
    ++rhs;
  }
  return *lhs == '\0' && *rhs == '\0';
}

bool startsWithIgnoreCase(const char *text, const char *prefix) {
  while (*prefix != '\0') {
    if (*text == '\0') {
      return false;
    }
    if (tolower(static_cast<unsigned char>(*text)) != tolower(static_cast<unsigned char>(*prefix))) {
      return false;
    }
    ++text;
    ++prefix;
  }
  return true;
}

bool parseFloatArgument(const char *text, float &value) {
  const char *start = skipSpaces(text);
  if (*start == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  value = static_cast<float>(strtod(start, &endPtr));
  if (endPtr == start) {
    return false;
  }

  const char *remainder = skipSpaces(endPtr);
  return *remainder == '\0';
}

bool parseDurationArgument(const char *text, unsigned long &seconds) {
  const char *start = skipSpaces(text);
  if (*start == '\0') {
    return false;
  }

  const char *colon = strchr(start, ':');
  if (colon != nullptr) {
    char *endPtr = nullptr;
    long minutes = strtol(start, &endPtr, 10);
    if (endPtr != colon) {
      return false;
    }
    char *endSeconds = nullptr;
    long secs = strtol(colon + 1, &endSeconds, 10);
    const char *remainder = skipSpaces(endSeconds);
    if (*remainder != '\0') {
      return false;
    }
    if (minutes < 0 || secs < 0) {
      return false;
    }
    seconds = static_cast<unsigned long>(minutes * 60L + secs);
    return true;
  }

  char *endPtr = nullptr;
  double minutes = strtod(start, &endPtr);
  if (endPtr == start) {
    return false;
  }
  const char *remainder = skipSpaces(endPtr);
  if (*remainder != '\0') {
    return false;
  }
  if (minutes < 0.0) {
    return false;
  }
  seconds = static_cast<unsigned long>(minutes * 60.0 + 0.5);
  return true;
}

bool parseFloatPair(const char *text, float &first, float &second) {
  const char *start = skipSpaces(text);
  if (*start == '\0') {
    return false;
  }

  char *endFirst = nullptr;
  first = static_cast<float>(strtod(start, &endFirst));
  if (endFirst == start) {
    return false;
  }

  second = 0.0f;
  const char *secondStart = skipSpaces(endFirst);
  if (*secondStart == '\0') {
    return false;
  }

  char *endSecond = nullptr;
  second = static_cast<float>(strtod(secondStart, &endSecond));
  if (endSecond == secondStart) {
    return false;
  }

  const char *remainder = skipSpaces(endSecond);
  return *remainder == '\0';
}

void handleFrostCommand(const char *args) {
  const char *keyword = skipSpaces(args);
  if (*keyword == '\0') {
    Serial.println(F("Usage: frost time <mm:ss> OR frost temp <stop> <resume>."));
    return;
  }

  if (startsWithIgnoreCase(keyword, "time")) {
    unsigned long seconds = 0;
    if (!parseDurationArgument(keyword + 4, seconds)) {
      Serial.println(F("Provide frost time as mm:ss or minutes."));
      return;
    }
    setFreezeDurationSeconds(seconds);
  } else if (startsWithIgnoreCase(keyword, "temp")) {
    float stopTemp = 0.0f;
    float resumeTemp = 0.0f;
    if (!parseFloatPair(keyword + 4, stopTemp, resumeTemp)) {
      Serial.println(F("Provide stop and resume temps, e.g. frost temp -18 -15"));
      return;
    }
    setTimerTempRange(stopTemp, resumeTemp);
  } else {
    Serial.println(F("Unknown frost command. Use 'frost time' or 'frost temp'."));
  }
}

void handleWaitCommand(const char *args) {
  const char *keyword = skipSpaces(args);
  if (*keyword == '\0') {
    Serial.println(F("Usage: wait time <mm:ss>"));
    return;
  }

  if (startsWithIgnoreCase(keyword, "time")) {
    unsigned long seconds = 0;
    if (!parseDurationArgument(keyword + 4, seconds)) {
      Serial.println(F("Provide wait time as mm:ss or minutes."));
      return;
    }
    setWaitDurationSeconds(seconds);
  } else if (startsWithIgnoreCase(keyword, "temp")) {
    Serial.println(F("Wait stage uses resume temp from 'frost temp'."));
  } else {
    Serial.println(F("Unknown wait command. Use 'wait time'."));
  }
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char buffer[40];
  size_t length = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
  buffer[length] = '\0';
  trimInPlace(buffer);

  if (buffer[0] == '\0') {
    return;
  }

  if (startsWithIgnoreCase(buffer, "mode")) {
    const char *argument = skipSpaces(buffer + 4);
    if (*argument == '\0') {
      Serial.println(F("Mode command requires 'timer' or 'thermo'."));
      return;
    }

    if (equalsIgnoreCase(argument, "timer")) {
      setOperationMode(OPERATION_MODE_TIMER);
    } else if (equalsIgnoreCase(argument, "thermo") || equalsIgnoreCase(argument, "sensor")) {
      setOperationMode(OPERATION_MODE_THERMOSTAT);
    } else {
      Serial.println(F("Unknown mode. Use 'timer' or 'thermo'."));
    }
  } else if (startsWithIgnoreCase(buffer, "temp")) {
    const char *argument = skipSpaces(buffer + 4);
    float value = 0.0f;
    if (!parseFloatArgument(argument, value)) {
      Serial.println(F("Provide numeric temperature, e.g. 'temp -12'."));
      return;
    }

    setThermostatTarget(value);
  } else if (startsWithIgnoreCase(buffer, "frost")) {
    handleFrostCommand(buffer + 5);
  } else if (startsWithIgnoreCase(buffer, "freeze")) {
    handleFrostCommand(buffer + 6);
  } else if (startsWithIgnoreCase(buffer, "wait")) {
    handleWaitCommand(buffer + 4);
  } else if (startsWithIgnoreCase(buffer, "rest")) {
    handleWaitCommand(buffer + 4);
  } else if (equalsIgnoreCase(buffer, "status")) {
    printConfigToSerial();
  } else if (equalsIgnoreCase(buffer, "help")) {
    printHelpToSerial();
  } else {
    Serial.println(F("Unknown command. Type 'help' for a list."));
  }
}

void initTemperatureSensor() {
  if (!ENABLE_TEMPERATURE_SENSOR) {
    return;
  }

  temperatureSensor.begin();
  temperatureSensor.setWaitForConversion(false);
  scheduleTemperatureMeasurement();
}

void scheduleTemperatureMeasurement() {
  if (!ENABLE_TEMPERATURE_SENSOR) {
    return;
  }

  temperatureSensor.requestTemperatures();
  temperatureConversionPending = true;
  lastTemperatureRequestMillis = millis();
}

void updateTemperature() {
  if (!ENABLE_TEMPERATURE_SENSOR) {
    return;
  }

  if (!temperatureConversionPending) {
    scheduleTemperatureMeasurement();
    return;
  }

  unsigned long now = millis();
  if (now - lastTemperatureRequestMillis < TEMPERATURE_CONVERSION_INTERVAL_MS) {
    return;
  }

  float temperature = temperatureSensor.getTempCByIndex(0);
  bool previousAvailability = temperatureSensorAvailable;
  float previousTemperature = currentTemperatureC;
  if (temperature > DEVICE_DISCONNECTED_C) {
    currentTemperatureC = temperature;
    temperatureSensorAvailable = true;
  } else {
    temperatureSensorAvailable = false;
  }

  temperatureConversionPending = false;
  scheduleTemperatureMeasurement();
  updateRelayOutput();
  if (previousAvailability != temperatureSensorAvailable ||
      (temperatureSensorAvailable && currentTemperatureC != previousTemperature)) {
    requestDisplayRefresh();
  }
}

void updateThermostatControl() {
  if (deviceConfig.mode != OPERATION_MODE_THERMOSTAT) {
    return;
  }

  if (!ENABLE_TEMPERATURE_SENSOR || !temperatureSensorAvailable) {
    thermostatCoolingDemand = false;
    desiredRelayState = false;
    updateRelayOutput();
    return;
  }

  float upperBound = deviceConfig.thermostatTargetC + THERMOSTAT_HYSTERESIS_C;
  float lowerBound = deviceConfig.thermostatTargetC - THERMOSTAT_HYSTERESIS_C;

  if (!thermostatCoolingDemand && currentTemperatureC >= upperBound) {
    thermostatCoolingDemand = true;
  } else if (thermostatCoolingDemand && currentTemperatureC <= lowerBound) {
    thermostatCoolingDemand = false;
  }

  desiredRelayState = thermostatCoolingDemand;
  updateRelayOutput();
}

void updateRelayOutput() {
  bool shouldConnect = desiredRelayState;

  if (deviceConfig.mode == OPERATION_MODE_TIMER && ENABLE_TEMPERATURE_SENSOR && ENABLE_TEMPERATURE_REGULATION) {
    if (temperatureSensorAvailable) {
      if (!timerCoolingPauseActive && currentTemperatureC <= deviceConfig.timerStopTempC) {
        timerCoolingPauseActive = true;
      } else if (timerCoolingPauseActive && currentTemperatureC >= deviceConfig.timerResumeTempC) {
        timerCoolingPauseActive = false;
      }
    } else {
      timerCoolingPauseActive = false;
    }

    if (timerCoolingPauseActive) {
      shouldConnect = false;
    }
  }

  if (shouldConnect && !relayState) {
    relayConnect();
  } else if (!shouldConnect && relayState) {
    relayDisconnect();
  }
}

ISR(TIMER1_COMPA_vect) {
  static uint16_t displayCounter = 0;
  static uint16_t doorCounter = 0;
  static uint16_t temperatureCounter = 0;
  static uint16_t timerCounter = 0;
  static uint16_t thermostatCounter = 0;
  static uint16_t encoderCounter = 0;

  ++displayCounter;
  ++doorCounter;
  ++temperatureCounter;
  ++timerCounter;
  ++thermostatCounter;
  ++encoderCounter;

  if (displayCounter >= DISPLAY_REFRESH_INTERVAL_MS) {
    displayCounter = 0;
    displayUpdateRequested = true;
  }

  if (doorCounter >= DOOR_SAMPLE_INTERVAL_MS) {
    doorCounter = 0;
    doorSampleRequested = true;
  }

  if (temperatureCounter >= TEMPERATURE_POLL_INTERVAL_MS) {
    temperatureCounter = 0;
    temperatureUpdateRequested = true;
  }

  if (timerCounter >= TIMER_SECOND_INTERVAL_MS) {
    timerCounter = 0;
    timerSecondTickRequested = true;
  }

  if (thermostatCounter >= THERMOSTAT_CONTROL_INTERVAL_MS) {
    thermostatCounter = 0;
    thermostatControlRequested = true;
  }

  if (encoderCounter >= ENCODER_SAMPLE_INTERVAL_MS) {
    encoderCounter = 0;
    encoderSampleRequested = true;
  }
}
