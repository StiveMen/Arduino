/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║                    SIM_RUL V1.0                                  ║
 * ║             Контроллер руля, педалей и КПП                       ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * Плата: Arduino Pro Micro, 5 В / 16 МГц.
 *
 * КАК РАБОТАЕТ ПРОШИВКА
 * • Руль и три педали передаются в игру как аналоговые HID-оси.
 * • КПП распознаётся по двум аналоговым координатам X/Y.
 *   Координаты КПП не передаются в игру как оси — вместо них
 *   отправляются отдельные кнопки передач 1–6.
 * • Кнопки HID: передачи 1–6, повышенная передача и зажигание. Сцепление — аналоговая ось.
 * • Контроллер запускается сразу с настройками по умолчанию, без обязательной калибровки.
 * • Если при включении удерживать «Зажигание», запускается калибровка.
 * • При обычном запуске текущее положение энкодера принимается за ноль.
 * • Подробные значения датчиков доступны в диагностике Serial Monitor.
 *
 * ВАЖНО: распиновка и назначение выводов сохранены без изменений.
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <Joystick.h>

// ════════════════════════════════════════════════════════════════════
// 1. НАСТРОЙКИ И КАЛИБРОВКА
// ════════════════════════════════════════════════════════════════════
// Рабочие пределы АЦП педалей и датчиков КПП.
constexpr int GAS_MIN = 0, GAS_MAX = 1023;
constexpr int BRAKE_MIN = 0, BRAKE_MAX = 1023;
constexpr int CLUTCH_MIN = 0, CLUTCH_MAX = 1023;
constexpr int GEAR_X_MIN = 0, GEAR_X_MAX = 1023;
constexpr int GEAR_Y_MIN = 0, GEAR_Y_MAX = 1023;
// ДИАПАЗОНЫ КПП
// Передача определяется, только если X И Y одновременно попадают в её зону.
// Значения датчиков сначала переводятся в диапазон 0–1023.

// Передача 1.
constexpr int GEAR_1_X_MIN =   0, GEAR_1_X_MAX =  100;
constexpr int GEAR_1_Y_MIN =   0, GEAR_1_Y_MAX =  100;

// Передача 2.
constexpr int GEAR_2_X_MIN = 700, GEAR_2_X_MAX = 1023;
constexpr int GEAR_2_Y_MIN =   0, GEAR_2_Y_MAX =  100;

// Передача 3.
constexpr int GEAR_3_X_MIN =   0, GEAR_3_X_MAX =  100;
constexpr int GEAR_3_Y_MIN = 400, GEAR_3_Y_MAX =  600;

// Передача 4.
constexpr int GEAR_4_X_MIN = 700, GEAR_4_X_MAX = 1023;
constexpr int GEAR_4_Y_MIN = 400, GEAR_4_Y_MAX =  600;

// Передача 5.
constexpr int GEAR_5_X_MIN = 300, GEAR_5_X_MAX =  400;
constexpr int GEAR_5_Y_MIN = 700, GEAR_5_Y_MAX = 1023;

// Передача 6.
constexpr int GEAR_6_X_MIN = 700, GEAR_6_X_MAX = 1023;
constexpr int GEAR_6_Y_MIN = 700, GEAR_6_Y_MAX = 1023;
// true — использовать сохранённую калибровку педалей, если она действительна.
constexpr bool USE_SAVED_PEDAL_CALIBRATION = true;

// ════════════════════════════════════════════════════════════════════
// 2. РАСПИНОВКА — НЕ МЕНЯТЬ БЕЗ ПЕРЕСМОТРА СХЕМЫ
// ════════════════════════════════════════════════════════════════════
// Педали.
constexpr uint8_t PIN_GAS = A1;
constexpr uint8_t PIN_BRAKE = A2;
constexpr uint8_t PIN_CLUTCH = A3;

// Два аналоговых датчика положения КПП.
constexpr uint8_t PIN_GEAR_X = A8;
constexpr uint8_t PIN_GEAR_Y = A9;

// Энкодер руля.
constexpr uint8_t PIN_ENC_A = 0;
constexpr uint8_t PIN_ENC_B = 1;

// Кнопки: зажигание, концевик нуля руля, повышенная передача.
constexpr uint8_t PIN_IGNITION = 5;
constexpr uint8_t PIN_CENTER_SW = 6;
constexpr uint8_t PIN_HIGH_GEAR = 7;
// ════════════════════════════════════════════════════════════════════
// 3. СЛУЖЕБНЫЕ ПАРАМЕТРЫ
// ════════════════════════════════════════════════════════════════════
// Подавление дребезга кнопок и интервалы ожидания.
constexpr uint16_t BUTTON_DEBOUNCE_MS = 25;
constexpr uint32_t ZERO_CAL_TIMEOUT_MS = 15000UL;
constexpr uint32_t FIRST_CAL_TIMEOUT_MS = 30000UL;
constexpr uint32_t MONITOR_INTERVAL_MS = 250;

// Диапазон оси руля, который поддерживает USB HID.
constexpr int32_t STEERING_HID_MIN = -32767L;
constexpr int32_t STEERING_HID_MAX = 32767L;

// Маркер и версия структуры, хранящейся в EEPROM.
constexpr uint16_t EEPROM_MAGIC = 0x5347;
constexpr uint8_t EEPROM_VERSION = 5;  // Версия формата данных в EEPROM.

// Структура сохранённых данных калибровки.
// Поля gear оставлены в структуре для совместимости формата;
// зоны передач задаются константами в начале этого файла.
struct GearCal {
  int16_t x, y;
};

struct CalData {
  uint16_t magic;
  uint8_t version;
  uint8_t reserved;
  int32_t steeringExtreme;

  uint16_t gasMin, gasMax;
  uint16_t brakeMin, brakeMax;
  uint16_t clutchMin, clutchMax;

  GearCal gear[6];
  uint16_t checksum;
};

CalData cal{};

// Рассчитывает контрольную сумму сохранённых данных.
uint16_t checksum(const CalData &data) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&data);
  uint32_t sum = 0;

  for (size_t i = 0; i < sizeof(CalData) - sizeof(data.checksum); ++i) {
    sum = (sum + bytes[i]) & 0xFFFF;
  }

  return static_cast<uint16_t>(sum);
}

// Проверяет, что калибровка сохранена и не повреждена.
bool calValid() {
  return cal.magic == EEPROM_MAGIC &&
         cal.version == EEPROM_VERSION &&
         cal.steeringExtreme > 10 &&
         cal.checksum == checksum(cal);
}

// Записывает текущую калибровку в энергонезависимую память.
void saveCal() {
  cal.magic = EEPROM_MAGIC;
  cal.version = EEPROM_VERSION;
  cal.reserved = 0;
  cal.checksum = checksum(cal);
  EEPROM.put(0, cal);
}

// USB HID-устройство игрового типа: 8 кнопок и четыре оси.
// X — руль; Rx — газ; Ry — тормоз; Rz — сцепление.
// Оси X/Y самой КПП в HID-устройство не включены.
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  8, 0,
  true, false, false,  // X, Y, Z
  true, true, true,    // Rx, Ry, Rz
  false, false, false, false, false
);

// Номера кнопок в игровом контроллере.
// Нумерация внутри прошивки с нуля: кнопки 1–6 — передачи,
// кнопка 7 — повышенная передача, кнопка 8 — зажигание.
// Сцепление передаётся только аналоговой осью Rz.
constexpr uint8_t HID_GEAR_BASE = 0;
constexpr uint8_t HID_HIGH_GEAR = 6;
constexpr uint8_t HID_IGNITION = 7;

// Текущее значение энкодера. Переменные меняются также из прерывания.
volatile long encoderRaw = 0;
volatile uint8_t encoderLastAB = 0;

// Смещение, которое соответствует найденному нулю руля.
long centerOffset = 0;

// Обработчик прерывания энкодера: определяет направление вращения.
void encoderISR() {
  uint8_t ab = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  uint8_t transition = (encoderLastAB << 2) | ab;

  switch (transition) {
    case 1:
    case 7:
    case 14:
    case 8:
      ++encoderRaw;
      break;

    case 2:
    case 11:
    case 13:
    case 4:
      --encoderRaw;
      break;
  }

  encoderLastAB = ab;
}

long readEncoder() {
  noInterrupts();
  long value = encoderRaw;
  interrupts();
  return value;
}

void resetEncoder() {
  noInterrupts();
  encoderRaw = 0;
  interrupts();
}

// Кнопка с программным подавлением дребезга контактов.
struct DebouncedInput {
  uint8_t pin;
  bool stable = false;
  bool lastRaw = false;
  uint32_t changed = 0;

  void begin(uint8_t inputPin) {
    pin = inputPin;
    pinMode(pin, INPUT_PULLUP);
    stable = lastRaw = (digitalRead(pin) == LOW);
    changed = millis();
  }

  void update() {
    bool rawPressed = (digitalRead(pin) == LOW);

    if (rawPressed != lastRaw) {
      lastRaw = rawPressed;
      changed = millis();
    }

    if (millis() - changed >= BUTTON_DEBOUNCE_MS) {
      stable = rawPressed;
    }
  }

  bool pressed() const {
    return stable;
  }
};

DebouncedInput ignition;
DebouncedInput centerSwitch;
DebouncedInput highGear;

// Читает аналоговый вход несколько раз и усредняет показания.
uint16_t analogStable(uint8_t pin) {
  uint32_t sum = 0;

  for (uint8_t i = 0; i < 4; ++i) {
    sum += analogRead(pin);
  }

  return sum / 4;
}

// Переводит значение датчика в диапазон 0–1023.
int mapAxis(uint16_t value, int minimum, int maximum) {
  if (maximum <= minimum) {
    return 0;
  }

  value = constrain((int)value, minimum, maximum);
  return map(value, minimum, maximum, 0, 1023);
}

// Педаль калибруется по двум физическим крайним положениям; поддерживается
// и обратное направление потенциометра (значение АЦП может уменьшаться при нажатии).
// Переводит показание педали в диапазон 0–1023.
// Поддерживает потенциометр, у которого значение АЦП уменьшается при нажатии.
int mapPedal(uint16_t raw, uint16_t rest, uint16_t full) {
  if (rest == full) {
    return 0;
  }

  int low = min((int)rest, (int)full);
  int high = max((int)rest, (int)full);
  int value = constrain((int)raw, low, high);
  int normalized = map(value, (int)rest, (int)full, 0, 1023);

  return constrain(normalized, 0, 1023);
}

// Возвращает сохранённые пределы педалей, если калибровка действительна.
// Если калибровки нет, используются значения по умолчанию из настроек.
void pedalRanges(
  int &gasMin, int &gasMax,
  int &brakeMin, int &brakeMax,
  int &clutchMin, int &clutchMax
) {
  if (USE_SAVED_PEDAL_CALIBRATION && calValid()) {
    gasMin = cal.gasMin;
    gasMax = cal.gasMax;
    brakeMin = cal.brakeMin;
    brakeMax = cal.brakeMax;
    clutchMin = cal.clutchMin;
    clutchMax = cal.clutchMax;
  } else {
    gasMin = GAS_MIN;
    gasMax = GAS_MAX;
    brakeMin = BRAKE_MIN;
    brakeMax = BRAKE_MAX;
    clutchMin = CLUTCH_MIN;
    clutchMax = CLUTCH_MAX;
  }
}

// ════════════════════════════════════════════════════════════════════
// 4. ОБРАБОТКА НАЖАТИЯ КНОПКИ «ЗАЖИГАНИЕ»
// ════════════════════════════════════════════════════════════════════
bool ignPrev = false;
bool ignArmed = false;

// Готовит кнопку к регистрации следующего нажатия.
void armIgn() {
  ignPrev = ignition.pressed();
  ignArmed = !ignPrev;
}

// Возвращает true только один раз в момент нового нажатия.
bool ignEdge() {
  bool nowPressed = ignition.pressed();

  if (!nowPressed) {
    ignArmed = true;
  }

  bool newPress = nowPressed && !ignPrev && ignArmed;

  if (newPress) {
    ignArmed = false;
  }

  ignPrev = nowPressed;
  return newPress;
}

// ════════════════════════════════════════════════════════════════════
// 5. ПОИСК НУЛЕВОГО ПОЛОЖЕНИЯ РУЛЯ
// ════════════════════════════════════════════════════════════════════
// Алгоритм поиска нуля:
// 1. Повернуть руль вправо.
// 2. Медленно повернуть влево до срабатывания концевика.
// 3. Продолжить движение влево до отпускания концевика.
// 4. Сохранить количество импульсов до отпускания как нулевое смещение.
// 5. Немного повернуть вправо для проверки.
// После калибровки концевик больше не участвует в управлении рулём.
bool zeroCalibrate(){
  uint32_t start=millis();
  long prev=readEncoder();
  bool sawRight=false, sawLeftAfterReset=false;
  bool pressed=centerSwitch.pressed(), releasedAfterReset=false;

  uint32_t lastCalDebug=0;
  while(millis()-start<ZERO_CAL_TIMEOUT_MS){
    centerSwitch.update();
    if(millis()-lastCalDebug>=500){lastCalDebug=millis();Serial.print(F("[DBG] ZERO_CAL elapsed_ms="));Serial.print(lastCalDebug-start);Serial.print(F(" ENC="));Serial.print(readEncoder());Serial.print(F(" CENTER_SW="));Serial.println(centerSwitch.pressed()?F("PRESSED"):F("OPEN"));}
    long now=readEncoder();
    long d=now-prev;
    prev=now;

    if(d>0)sawRight=true;

    // Концевик должен сработать после движения руля вправо и при возврате влево.
    // Из-за подавления дребезга изменение его состояния может не совпасть
    // по времени с импульсом энкодера.
    if(!pressed && centerSwitch.pressed()){
      if(!sawRight)return false;
      resetEncoder();
      pressed=true;
      releasedAfterReset=true;
      sawLeftAfterReset=false;
    }

    if(releasedAfterReset){
      if(d<0)sawLeftAfterReset=true;
      if(pressed && !centerSwitch.pressed()){
        long releaseCount=readEncoder();
        if(!sawLeftAfterReset || releaseCount==0)return false;
        centerOffset=releaseCount;
        break;
      }
    }

    pressed=centerSwitch.pressed();
    delay(1);
  }

  if(centerOffset==0)return false;

  // Проверка: после нахождения нуля поверни руль вправо.
  uint32_t vstart=millis();
  long verify=readEncoder();
  while(millis()-vstart<ZERO_CAL_TIMEOUT_MS){
    if(readEncoder()>verify+20)return true;
    delay(1);
  }
  return false;
}

// ════════════════════════════════════════════════════════════════════
// 6. ПЕРВИЧНАЯ КАЛИБРОВКА
// ════════════════════════════════════════════════════════════════════
enum FStage:uint8_t{ZERO,STEERING_EXTREME,PEDAL_MIN,PEDAL_MAX,SAVE,ERROR,DONE};
FStage fstage=FStage::ZERO;
uint32_t fstarted=0;
long steeringPeak=0;
uint16_t pedalPeakG=0,pedalPeakB=0,pedalPeakC=0;
uint32_t saveHoldStarted=0;
bool justSavedCalibration=false;
bool calibrationMode=false;

void printStageInstructions(){
  Serial.println();
  switch(fstage){
    case FStage::ZERO:
      Serial.println(F("[CAL 1/4] НОЛЬ РУЛЯ: поверни руль ВПРАВО, затем медленно ВЛЕВО до срабатывания концевика; продолжай влево до отпускания концевика, затем поверни вправо для проверки."));
      break;
    case FStage::STEERING_EXTREME:
      Serial.println(F("[CAL 2/4] РУЛЬ: верни руль к найденному нулю, затем поверни до крайнего положения (полный ход) и нажми ЗАЖИГАНИЕ для сохранения диапазона."));
      break;
    case FStage::PEDAL_MIN:
      Serial.println(F("[CAL 3/4] ПЕДАЛИ: отпусти газ, тормоз и сцепление; нажми ЗАЖИГАНИЕ, чтобы записать их исходные положения."));
      break;
    case FStage::PEDAL_MAX:
      Serial.println(F("[CAL 3/4] ПЕДАЛИ: нажми каждую педаль до упора, затем отпусти. После этого нажми ЗАЖИГАНИЕ для сохранения полного диапазона."));
      break;
    case FStage::SAVE:
      Serial.println(F("[SAVE] Переведи КПП в НЕЙТРАЛЬ и удерживай ЗАЖИГАНИЕ 1,2 секунды. Калибровка сохранится в EEPROM."));
      break;
    case FStage::ERROR:
      Serial.println(F("[ERROR] Калибровка прервана/не пройдена. Перезапусти контроллер с зажатым ЗАЖИГАНИЕМ и повтори процедуру."));
      break;
    case FStage::DONE:
      Serial.println(F("[OK] Калибровка завершена."));
      break;
  }
}
void setStage(uint8_t s){
  fstage=(FStage)s; fstarted=millis(); armIgn(); steeringPeak=0;
  if(fstage==FStage::PEDAL_MAX){
    pedalPeakG=cal.gasMin; pedalPeakB=cal.brakeMin; pedalPeakC=cal.clutchMin;
  }
  printStageInstructions();
}
void blinkSaved(){
  // Отдельный светодиод для индикации калибровки не используется.
  // Подтверждение выводится в окне последовательного монитора.
  Serial.println(F("CALIBRATION_SAVED"));
}
void firstCalibration() {
  ignition.update();
  centerSwitch.update();
  highGear.update();

  // Если этап затянулся, просим пользователя начать калибровку заново.
  if (fstage != FStage::SAVE &&
      millis() - fstarted > FIRST_CAL_TIMEOUT_MS) {
    setStage(FStage::ERROR);
    return;
  }

  // Этап 1: поиск нулевого положения руля.
  if (fstage == FStage::ZERO) {
    if (zeroCalibrate()) {
      setStage(FStage::STEERING_EXTREME);
    } else {
      setStage(FStage::ERROR);
    }
    return;
  }

  // Этап 2: пользователь поворачивает руль до крайнего положения.
  if (fstage == FStage::STEERING_EXTREME) {
    long position = labs(readEncoder() - centerOffset);

    if (position > steeringPeak) {
      steeringPeak = position;
    }

    if (ignEdge()) {
      if (steeringPeak < 10) {
        setStage(FStage::ERROR);
        return;
      }

      cal.steeringExtreme = steeringPeak;
      setStage(FStage::PEDAL_MIN);
    }
    return;
  }

  // Этап 3а: записываем исходное положение всех педалей.
  if (fstage == FStage::PEDAL_MIN) {
    if (ignEdge()) {
      cal.gasMin = analogStable(PIN_GAS);
      cal.brakeMin = analogStable(PIN_BRAKE);
      cal.clutchMin = analogStable(PIN_CLUTCH);
      setStage(FStage::PEDAL_MAX);
    }
    return;
  }

  // Этап 3б: отслеживаем полный ход каждой педали.
  if (fstage == FStage::PEDAL_MAX) {
    uint16_t gas = analogStable(PIN_GAS);
    uint16_t brake = analogStable(PIN_BRAKE);
    uint16_t clutch = analogStable(PIN_CLUTCH);

    if (abs((int)gas - (int)cal.gasMin) >
        abs((int)pedalPeakG - (int)cal.gasMin)) {
      pedalPeakG = gas;
    }

    if (abs((int)brake - (int)cal.brakeMin) >
        abs((int)pedalPeakB - (int)cal.brakeMin)) {
      pedalPeakB = brake;
    }

    if (abs((int)clutch - (int)cal.clutchMin) >
        abs((int)pedalPeakC - (int)cal.clutchMin)) {
      pedalPeakC = clutch;
    }

    if (ignEdge()) {
      cal.gasMax = pedalPeakG;
      cal.brakeMax = pedalPeakB;
      cal.clutchMax = pedalPeakC;

      // Если педаль не изменила показание, калибровка считается ошибочной.
      if (cal.gasMax == cal.gasMin ||
          cal.brakeMax == cal.brakeMin ||
          cal.clutchMax == cal.clutchMin) {
        setStage(FStage::ERROR);
        return;
      }

      setStage(FStage::SAVE);
    }
    return;
  }

  // Этап 4: сохраняем результат. Перед этим КПП должна быть в нейтрали.
  if (fstage == FStage::SAVE) {
    if (ignition.pressed()) {
      if (saveHoldStarted == 0) {
        saveHoldStarted = millis();
      }

      if (millis() - saveHoldStarted >= 1200) {
        saveCal();
        blinkSaved();

        // После сохранения необходим перезапуск контроллера.
        fstage = FStage::DONE;
        justSavedCalibration = true;
      }
    } else {
      saveHoldStarted = 0;
    }
  }
}

// ════════════════════════════════════════════════════════════════════
// 7. РАСПОЗНАВАНИЕ ПЕРЕДАЧ И ПОВЫШЕННАЯ ПЕРЕДАЧА
// ════════════════════════════════════════════════════════════════════
// 7. РАСПОЗНАВАНИЕ ПЕРЕДАЧ И ПОВЫШЕННАЯ ПЕРЕДАЧА
// ════════════════════════════════════════════════════════════════════
int8_t activeGear=0; bool highGearLatched=false;

// ════════════════════════════════════════════════════════════════════
// 8. ДИАГНОСТИКА В ПОСЛЕДОВАТЕЛЬНОМ МОНИТОРЕ
// ════════════════════════════════════════════════════════════════════
const __FlashStringHelper* stageName(){
  switch(fstage){
    case FStage::ZERO: return F("ZERO");
    case FStage::STEERING_EXTREME: return F("STEERING_EXTREME");
    case FStage::PEDAL_MIN: return F("PEDAL_MIN");
    case FStage::PEDAL_MAX: return F("PEDAL_MAX");
    case FStage::SAVE: return F("SAVE");
    case FStage::ERROR: return F("ERROR");
    case FStage::DONE: return F("DONE");
  }
  return F("UNKNOWN");
}
void printDiagnostics(bool force=false){
  static uint32_t last=0;
  if(!force && millis()-last<MONITOR_INTERVAL_MS)return;
  last=millis();
  long raw=readEncoder();
  Serial.print(F("[DBG] ms=")); Serial.print(millis());
  Serial.print(F(" CAL=")); Serial.print(calValid()?F("OK"):F("INVALID"));
  Serial.print(F(" STAGE=")); Serial.print(stageName());
  Serial.print(F(" ENC_RAW=")); Serial.print(raw);
  Serial.print(F(" ENC_ZEROED=")); Serial.print(raw-centerOffset);
  Serial.print(F(" CENTER_SW=")); Serial.print(centerSwitch.pressed()?F("PRESSED"):F("OPEN"));
  Serial.print(F(" IGN=")); Serial.print(ignition.pressed()?1:0);
  Serial.print(F(" HIGH_BTN=")); Serial.print(highGear.pressed()?F("ON"):F("OFF"));
  Serial.print(F(" ADC_GAS=")); Serial.print(analogStable(PIN_GAS));
  Serial.print(F(" ADC_BRAKE=")); Serial.print(analogStable(PIN_BRAKE));
  Serial.print(F(" ADC_CLUTCH=")); Serial.print(analogStable(PIN_CLUTCH));
  Serial.print(F(" ADC_GEAR_X=")); Serial.print(analogStable(PIN_GEAR_X));
  Serial.print(F(" ADC_GEAR_Y=")); Serial.print(analogStable(PIN_GEAR_Y));
  Serial.print(F(" GEAR=")); Serial.print(activeGear);
  Serial.print(F(" HIGH_LATCH=")); Serial.print(highGearLatched?1:0);
  Serial.print(F(" STEER_LIMIT=")); Serial.print(cal.steeringExtreme);
  Serial.print(F(" HID=")); Serial.println(calValid()?F("READY"):F("CALIBRATION"));
}
// Проверяет, попали ли координаты X/Y внутрь прямоугольника передачи.
bool gearZone(uint8_t gearIndex, int x, int y) {
  switch (gearIndex) {
    case 0:
      return x >= GEAR_1_X_MIN && x <= GEAR_1_X_MAX &&
             y >= GEAR_1_Y_MIN && y <= GEAR_1_Y_MAX;

    case 1:
      return x >= GEAR_2_X_MIN && x <= GEAR_2_X_MAX &&
             y >= GEAR_2_Y_MIN && y <= GEAR_2_Y_MAX;

    case 2:
      return x >= GEAR_3_X_MIN && x <= GEAR_3_X_MAX &&
             y >= GEAR_3_Y_MIN && y <= GEAR_3_Y_MAX;

    case 3:
      return x >= GEAR_4_X_MIN && x <= GEAR_4_X_MAX &&
             y >= GEAR_4_Y_MIN && y <= GEAR_4_Y_MAX;

    case 4:
      return x >= GEAR_5_X_MIN && x <= GEAR_5_X_MAX &&
             y >= GEAR_5_Y_MIN && y <= GEAR_5_Y_MAX;

    case 5:
      return x >= GEAR_6_X_MIN && x <= GEAR_6_X_MAX &&
             y >= GEAR_6_Y_MIN && y <= GEAR_6_Y_MAX;

    default:
      return false;
  }
}

// Определяет номер передачи по координатам КПП.
// 0 означает нейтраль. Если зоны передач пересекаются, также возвращается 0.
int8_t findGear(int x, int y) {
  int8_t foundGear = 0;

  for (uint8_t i = 0; i < 6; ++i) {
    if (gearZone(i, x, y)) {
      if (foundGear != 0) {
        return 0;
      }

      foundGear = i + 1;
    }
  }

  return foundGear;
}

// Обновляет выбранную передачу и состояние повышенной передачи.
void updateGear(int x, int y) {
  const bool highButtonPressed = highGear.pressed();

  activeGear = findGear(x, y);

  // Нажатие фиксируется, если рычаг находится в одной из передач.
  if (highButtonPressed && activeGear != 0) {
    highGearLatched = true;
  }

  // В нейтрали повышенная передача выключается после отпускания кнопки.
  if (activeGear == 0 && !highButtonPressed) {
    highGearLatched = false;
  }
}

// ════════════════════════════════════════════════════════════════════
// 9. ОТПРАВКА ДАННЫХ В ИГРУ И РЕЖИМ МОНИТОРА
// ════════════════════════════════════════════════════════════════════
void updateController(bool sendHid) {
  ignition.update();
  highGear.update();

  // Считываем датчики педалей и координаты рычага КПП.
  const uint16_t gasRaw = analogStable(PIN_GAS);
  const uint16_t brakeRaw = analogStable(PIN_BRAKE);
  const uint16_t clutchRaw = analogStable(PIN_CLUTCH);
  const uint16_t gearXRaw = analogStable(PIN_GEAR_X);
  const uint16_t gearYRaw = analogStable(PIN_GEAR_Y);

  // Переводим педали в диапазон 0–1023.
  int gasMin, gasMax;
  int brakeMin, brakeMax;
  int clutchMin, clutchMax;
  pedalRanges(
    gasMin, gasMax,
    brakeMin, brakeMax,
    clutchMin, clutchMax
  );

  const int gas = mapPedal(gasRaw, gasMin, gasMax);
  const int brake = mapPedal(brakeRaw, brakeMin, brakeMax);
  const int clutch = mapPedal(clutchRaw, clutchMin, clutchMax);

  // Координаты КПП нужны только для распознавания передачи.
  const int gearX = mapAxis(gearXRaw, GEAR_X_MIN, GEAR_X_MAX);
  const int gearY = mapAxis(gearYRaw, GEAR_Y_MIN, GEAR_Y_MAX);
  updateGear(gearX, gearY);

  // Нормализуем угол руля по сохранённому пределу калибровки.
  long steering = readEncoder() - centerOffset;
  const long steeringLimit = cal.steeringExtreme;

  if (steeringLimit > 0) {
    steering = constrain(steering, -steeringLimit, steeringLimit);
    steering = map(
      steering,
      -steeringLimit, steeringLimit,
      STEERING_HID_MIN, STEERING_HID_MAX
    );
  } else {
    steering = 0;
  }

  // Кнопки HID: передачи 1–6, повышенная передача и зажигание.
  // Сцепление остаётся аналоговой осью Rz, концевик не является HID-кнопкой.
  for (uint8_t i = 0; i < 6; ++i) {
    Joystick.setButton(
      HID_GEAR_BASE + i,
      activeGear == (int8_t)(i + 1)
    );
  }
  Joystick.setButton(HID_HIGH_GEAR, highGearLatched);
  Joystick.setButton(HID_IGNITION, ignition.pressed());

  // Важно: диапазоны этих осей заданы в setup().
  Joystick.setXAxis(steering);
  Joystick.setRxAxis(gas);
  Joystick.setRyAxis(brake);
  Joystick.setRzAxis(clutch);

  // Режим монитора читает датчики, но не посылает игровые события.
  if (sendHid) {
    Joystick.sendState();
  }
}
void printMonitor() {
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint < MONITOR_INTERVAL_MS) {
    return;
  }
  lastPrint = millis();

  Serial.println(F("\n=== SIM_RUL: МОНИТОР ==="));
  Serial.print(F("РУЛЬ, импульсы от нуля: "));
  Serial.println(readEncoder() - centerOffset);

  int gasMin, gasMax;
  int brakeMin, brakeMax;
  int clutchMin, clutchMax;

  pedalRanges(
    gasMin, gasMax,
    brakeMin, brakeMax,
    clutchMin, clutchMax
  );

  Serial.print(F("ГАЗ, АЦП / норма: "));
  Serial.print(analogStable(PIN_GAS));
  Serial.print('/');
  Serial.println(mapPedal(analogStable(PIN_GAS), gasMin, gasMax));

  Serial.print(F("ТОРМОЗ, АЦП / норма: "));
  Serial.print(analogStable(PIN_BRAKE));
  Serial.print('/');
  Serial.println(mapPedal(analogStable(PIN_BRAKE), brakeMin, brakeMax));

  Serial.print(F("СЦЕПЛЕНИЕ, АЦП / норма: "));
  Serial.print(analogStable(PIN_CLUTCH));
  Serial.print('/');
  Serial.println(mapPedal(analogStable(PIN_CLUTCH), clutchMin, clutchMax));

  Serial.print(F("КПП, X: "));
  Serial.println(analogStable(PIN_GEAR_X));
  Serial.print(F("КПП, Y: "));
  Serial.println(analogStable(PIN_GEAR_Y));

  Serial.print(F("ЗАЖИГАНИЕ: "));
  Serial.println(ignition.pressed() ? F("НАЖАТО") : F("ОТПУЩЕНО"));

  Serial.print(F("КНОПКА ПОВЫШЕННОЙ ПЕРЕДАЧИ: "));
  Serial.println(highGear.pressed() ? F("НАЖАТА") : F("ОТПУЩЕНА"));

  Serial.print(F("ПОВЫШЕННАЯ ПЕРЕДАЧА (HID): "));
  Serial.println(highGearLatched ? F("ВКЛ") : F("ВЫКЛ"));

  Serial.print(F("ЗАЖИГАНИЕ (HID): "));
  Serial.println(ignition.pressed() ? F("ВКЛ") : F("ВЫКЛ"));

  for (uint8_t i = 0; i < 6; ++i) {
    Serial.print(F("ПЕРЕДАЧА "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(activeGear == (int8_t)(i + 1) ? F("ВКЛ") : F("ВЫКЛ"));
  }

  Serial.println(F("ВЫВОД В ИГРУ: ОТКЛЮЧЁН (РЕЖИМ МОНИТОРА)"));
}
void monitor(){ printDiagnostics(); }

// ════════════════════════════════════════════════════════════════════
// 10. ЗАПУСК И ОСНОВНОЙ ЦИКЛ ПРОГРАММЫ
// ════════════════════════════════════════════════════════════════════
void setup(){
  Serial.begin(115200);
  pinMode(PIN_ENC_A,INPUT_PULLUP);pinMode(PIN_ENC_B,INPUT_PULLUP);
  encoderLastAB=(digitalRead(PIN_ENC_A)<<1)|digitalRead(PIN_ENC_B);
  ignition.begin(PIN_IGNITION);centerSwitch.begin(PIN_CENTER_SW);highGear.begin(PIN_HIGH_GEAR);
  analogReference(DEFAULT);
  // Настраиваем диапазоны HID-осей, чтобы игра получала полный ход педалей.
  Joystick.setXAxisRange(STEERING_HID_MIN, STEERING_HID_MAX);
  Joystick.setRxAxisRange(0, 1023);  // Газ: отпущен → нажат.
  Joystick.setRyAxisRange(0, 1023);  // Тормоз: отпущен → нажат.
  Joystick.setRzAxisRange(0, 1023);  // Сцепление: отпущено → нажато.

  // Регистрируем USB-устройство как игровой контроллер.
  // Состояние отправляется вручную из updateController().
  Joystick.begin(false);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encoderISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),encoderISR,CHANGE);
  delay(100);
  ignition.update();
  EEPROM.get(0, cal);

  // Если калибровки ещё нет, сразу задаём безопасные значения по умолчанию.
  // Это позволяет пользоваться контроллером без прохождения калибровки.
  if (!calValid()) {
    cal.steeringExtreme = 1200;
    cal.gasMin = GAS_MIN;
    cal.gasMax = GAS_MAX;
    cal.brakeMin = BRAKE_MIN;
    cal.brakeMax = BRAKE_MAX;
    cal.clutchMin = CLUTCH_MIN;
    cal.clutchMax = CLUTCH_MAX;
    saveCal();
    Serial.println(F("[CAL] Сохранены настройки по умолчанию. Калибровка не обязательна."));
  }

  // Удержание «Зажигания» при включении — добровольный запуск калибровки.
  if (ignition.pressed()) {
    calibrationMode = true;
    setStage(FStage::ZERO);
    Serial.println(F("[CAL] Запущена ручная калибровка по запросу пользователя."));
  } else {
    calibrationMode = false;
    fstage = FStage::DONE;
    Serial.println(F("[USB HID] Готов: 1–6 передачи, 7 повышенная, 8 зажигание. Сцепление — аналоговая ось; концевик без HID-кнопки."));
  }
}
void loop() {
  ignition.update();
  centerSwitch.update();

  // После сохранения калибровки требуется перезапуск контроллера.
  if (justSavedCalibration) {
    Serial.println(F("[RESTART REQUIRED] Калибровка сохранена. Перезапусти контроллер; до перезапуска HID не активируется."));
    delay(1000);
    return;
  }

  // Калибровка запускается только по желанию: удерживай «Зажигание» при включении.
  // Если её не запускали, контроллер сразу работает с настройками по умолчанию
  // или с ранее сохранёнными значениями.
  if (calibrationMode) {
    if (fstage != FStage::ERROR && fstage != FStage::DONE) {
      firstCalibration();
    } else if (fstage == FStage::ERROR) {
      updateController(false);
      delay(50);
    }

    printDiagnostics();
    return;
  }

  // Обычный запуск не выполняет поиск нуля и не ждёт действий пользователя.
  // Пока калибровка не запрошена, текущее положение энкодера считается нулём.
  // Если руль нужно откалибровать, перезапусти контроллер с зажатым «Зажиганием».
  updateController(true);
  printDiagnostics();
}
