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
 * • Кнопка «Зажигание» и кнопка повышенной передачи тоже являются HID-кнопками.
 * • При первом запуске выполняется калибровка руля и педалей.
 * • При обычном запуске определяется нулевое положение руля.
 * • Если при включении удерживать «Зажигание», запускается режим монитора:
 *   данные выводятся в Serial Monitor, но HID-состояние в игру не отправляется.
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
// ДИАПАЗОНЫ ПЕРЕДАЧ
// Значения X/Y нормализуются в диапазон 0–1023.
// Передача включается, только если ОБЕ координаты попали в её диапазон.
// Настрой диапазоны по показаниям своей КПП в режиме монитора.
// Значения передач 2–6 ниже — примеры, а не готовая калибровка.
// Передача 1 — пример заданного диапазона.
constexpr int GEAR_1_X_MIN = 100, GEAR_1_X_MAX = 200;
constexpr int GEAR_1_Y_MIN = 100, GEAR_1_Y_MAX = 200;

// Передача 2.
constexpr int GEAR_2_X_MIN = 100, GEAR_2_X_MAX = 200;
constexpr int GEAR_2_Y_MIN = 300, GEAR_2_Y_MAX = 400;

// Передача 3.
constexpr int GEAR_3_X_MIN = 100, GEAR_3_X_MAX = 200;
constexpr int GEAR_3_Y_MIN = 500, GEAR_3_Y_MAX = 600;

// Передача 4.
constexpr int GEAR_4_X_MIN = 300, GEAR_4_X_MAX = 400;
constexpr int GEAR_4_Y_MIN = 100, GEAR_4_Y_MAX = 200;

// Передача 5.
constexpr int GEAR_5_X_MIN = 300, GEAR_5_X_MAX = 400;
constexpr int GEAR_5_Y_MIN = 300, GEAR_5_Y_MAX = 400;

// Передача 6.
constexpr int GEAR_6_X_MIN = 300, GEAR_6_X_MAX = 400;
constexpr int GEAR_6_Y_MIN = 500, GEAR_6_Y_MAX = 600;
constexpr bool USE_SAVED_PEDAL_CALIBRATION=true;

// ════════════════════════════════════════════════════════════════════
// 2. РАСПИНОВКА — НЕ МЕНЯТЬ БЕЗ ПЕРЕСМОТРА СХЕМЫ
// ════════════════════════════════════════════════════════════════════
constexpr uint8_t PIN_GAS=A1, PIN_BRAKE=A2, PIN_CLUTCH=A3;
constexpr uint8_t PIN_GEAR_X=A8, PIN_GEAR_Y=A9;
constexpr uint8_t PIN_ENC_A=0, PIN_ENC_B=1;
constexpr uint8_t PIN_IGNITION=5, PIN_CENTER_SW=6, PIN_HIGH_GEAR=7;
// ════════════════════════════════════════════════════════════════════
// 3. СЛУЖЕБНЫЕ ПАРАМЕТРЫ
// ════════════════════════════════════════════════════════════════════
constexpr uint16_t BUTTON_DEBOUNCE_MS=25;
constexpr uint32_t ZERO_CAL_TIMEOUT_MS=15000UL;
constexpr uint32_t FIRST_CAL_TIMEOUT_MS=30000UL;
constexpr uint32_t MONITOR_INTERVAL_MS=250;
constexpr long MAX_ENCODER_TRAVEL=200000L;
constexpr int32_t STEERING_HID_MIN=-32767L, STEERING_HID_MAX=32767L;
constexpr uint16_t EEPROM_MAGIC=0x5347;
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

// Expose only steering and pedal axes. Gearbox X/Y are internal sensors.
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,8,0,
  true,false,false,true,true,true,false,false,false,false,false);
constexpr uint8_t HID_IGNITION=0, HID_HIGH_GEAR=1, HID_GEAR_BASE=2;

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
// и обратное направление потенциометра (ADC может уменьшаться при нажатии).
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
  // Подтверждение выводится в Serial Monitor.
  Serial.println(F("CALIBRATION_SAVED"));
}
void firstCalibration(){
  ignition.update();centerSwitch.update();highGear.update();
  if(fstage!=FStage::SAVE && millis()-fstarted>FIRST_CAL_TIMEOUT_MS){setStage(FStage::ERROR);return;}

  if(fstage==FStage::ZERO){
    if(zeroCalibrate()){setStage(FStage::STEERING_EXTREME);}
    else setStage(FStage::ERROR);
    return;
  }
  if(fstage==FStage::STEERING_EXTREME){
    long p=labs(readEncoder()-centerOffset);if(p>steeringPeak)steeringPeak=p;
    if(ignEdge()){if(steeringPeak<10){setStage(FStage::ERROR);return;}cal.steeringExtreme=steeringPeak;setStage(FStage::PEDAL_MIN);}
    return;
  }
  if(fstage==FStage::PEDAL_MIN){
    if(ignEdge()){cal.gasMin=analogStable(PIN_GAS);cal.brakeMin=analogStable(PIN_BRAKE);cal.clutchMin=analogStable(PIN_CLUTCH);setStage(FStage::PEDAL_MAX);}
    return;
  }
  if(fstage==FStage::PEDAL_MAX){
    // Запоминаем крайнее значение каждой педали относительно положения покоя.
    // Это работает и при обратном направлении потенциометра.
    uint16_t g=analogStable(PIN_GAS),b=analogStable(PIN_BRAKE),c=analogStable(PIN_CLUTCH);
    if(abs((int)g-(int)cal.gasMin)>abs((int)pedalPeakG-(int)cal.gasMin))pedalPeakG=g;
    if(abs((int)b-(int)cal.brakeMin)>abs((int)pedalPeakB-(int)cal.brakeMin))pedalPeakB=b;
    if(abs((int)c-(int)cal.clutchMin)>abs((int)pedalPeakC-(int)cal.clutchMin))pedalPeakC=c;
    if(ignEdge()){
      cal.gasMax=pedalPeakG; cal.brakeMax=pedalPeakB; cal.clutchMax=pedalPeakC;
      if(cal.gasMax==cal.gasMin||cal.brakeMax==cal.brakeMin||cal.clutchMax==cal.clutchMin){setStage(FStage::ERROR);return;}
      setStage(FStage::SAVE);
    }
    return;
  }
  if(fstage==FStage::SAVE){
    // Для сохранения переведи КПП в нейтраль и удерживай «Зажигание».
    if(ignition.pressed()){
      if(saveHoldStarted==0)saveHoldStarted=millis();
      if(millis()-saveHoldStarted>=1200){
      saveCal();
      blinkSaved();
      // Результат уже сохранён в EEPROM.
      // Перезапусти контроллер, чтобы начать работу с новой калибровкой.
      fstage=FStage::DONE;
      justSavedCalibration=true;
    }
    }else saveHoldStarted=0;
  }
}

// ════════════════════════════════════════════════════════════════════
// 7. РАСПОЗНАВАНИЕ ПЕРЕДАЧ И ПОВЫШЕННАЯ ПЕРЕДАЧА
// ════════════════════════════════════════════════════════════════════
int8_t activeGear=0; bool highGearLatched=false;

// ════════════════════════════════════════════════════════════════════
// 8. ДИАГНОСТИКА В SERIAL MONITOR
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
// 9. ОТПРАВКА ДАННЫХ В ИГРУ И РЕЖИМ МОНИТОРА
// ════════════════════════════════════════════════════════════════════
void updateController(bool sendHid){
  ignition.update();highGear.update();
  uint16_t gr=analogStable(PIN_GAS),br=analogStable(PIN_BRAKE),cl=analogStable(PIN_CLUTCH);
  uint16_t xr=analogStable(PIN_GEAR_X),yr=analogStable(PIN_GEAR_Y);
  int gmn,gmx,bmn,bmx,cmn,cmx; pedalRanges(gmn,gmx,bmn,bmx,cmn,cmx);
  int gas=mapPedal(gr,gmn,gmx),brake=mapPedal(br,bmn,bmx),clutch=mapPedal(cl,cmn,cmx);
  int gx=mapAxis(xr,GEAR_X_MIN,GEAR_X_MAX),gy=mapAxis(yr,GEAR_Y_MIN,GEAR_Y_MAX);
  updateGear(gx,gy);

  long steer=readEncoder()-centerOffset,e=cal.steeringExtreme;
  steer=constrain(steer,-e,e);steer=map(steer,-e,e,STEERING_HID_MIN,STEERING_HID_MAX);
  Joystick.setButton(HID_IGNITION,ignition.pressed());
  Joystick.setButton(HID_HIGH_GEAR,highGearLatched);
  for(uint8_t i=0;i<6;++i)Joystick.setButton(HID_GEAR_BASE+i,activeGear==(int8_t)(i+1));
  // HID-оси: X — руль, Rx — газ, Ry — тормоз, Rz — сцепление.
  Joystick.setXAxis(steer);Joystick.setRxAxis(gas);Joystick.setRyAxis(brake);Joystick.setRzAxis(clutch);
  // gx и gy нужны только для распознавания передачи и не отправляются в USB HID.
  if(sendHid)Joystick.sendState();
}
void printMonitor(){
  static uint32_t last=0;
  if(millis()-last<MONITOR_INTERVAL_MS)return;
  last=millis();
  Serial.println(F("\n=== SIM_RUL MONITOR ==="));
  Serial.print(F("STEERING: "));Serial.println(readEncoder()-centerOffset);
  int gmn,gmx,bmn,bmx,cmn,cmx; pedalRanges(gmn,gmx,bmn,bmx,cmn,cmx);
  Serial.print(F("GAS ADC/NORM: "));Serial.print(analogStable(PIN_GAS));Serial.print('/');
  Serial.println(mapPedal(analogStable(PIN_GAS),gmn,gmx));
  Serial.print(F("BRAKE ADC/NORM: "));Serial.print(analogStable(PIN_BRAKE));Serial.print('/');
  Serial.println(mapPedal(analogStable(PIN_BRAKE),bmn,bmx));
  Serial.print(F("CLUTCH ADC/NORM: "));Serial.print(analogStable(PIN_CLUTCH));Serial.print('/');
  Serial.println(mapPedal(analogStable(PIN_CLUTCH),cmn,cmx));
  Serial.print(F("GEAR X: "));Serial.println(analogStable(PIN_GEAR_X));
  Serial.print(F("GEAR Y: "));Serial.println(analogStable(PIN_GEAR_Y));
  Serial.print(F("IGNITION: "));Serial.println(ignition.pressed()?F("ON"):F("OFF"));
  Serial.print(F("HIGH GEAR BUTTON: "));Serial.println(highGear.pressed()?F("PRESSED"):F("RELEASED"));
  Serial.print(F("HIGH GEAR HID LATCH: "));Serial.println(highGearLatched?F("ON"):F("OFF"));
  Serial.print(F("IGNITION HID BUTTON: "));Serial.println(ignition.pressed()?F("ON"):F("OFF"));
  for(uint8_t i=0;i<6;++i){
    Serial.print(F("GEAR "));Serial.print(i+1);Serial.print(F(": "));
    Serial.println(activeGear==(int8_t)(i+1)?F("ON"):F("OFF"));
  }
  Serial.println(F("HID OUTPUT: DISABLED (MONITOR MODE)"));
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
  Joystick.setXAxisRange(STEERING_HID_MIN,STEERING_HID_MAX);Joystick.begin(false);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encoderISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),encoderISR,CHANGE);
  delay(100);ignition.update();EEPROM.get(0,cal);
  if(!calValid()){
    cal.steeringExtreme=1200; // Временный диапазон руля, пока калибровка ещё не выполнена.
    if(ignition.pressed())setStage(FStage::ZERO);
    else { fstage=FStage::ERROR; printStageInstructions(); }
  }else{
    // Если «Зажигание» удерживается при запуске, включается режим монитора.
    // Поиск нулевого положения руля всё равно выполняется первым.
    fstage=FStage::DONE;
  }
}
void loop(){
  ignition.update();  centerSwitch.update();
  if(justSavedCalibration){
    Serial.println(F("[RESTART REQUIRED] Калибровка сохранена. Перезапусти контроллер; до перезапуска HID не активируется."));
    delay(1000);
    return;
  }
  if(!calValid()){
    if(fstage!=FStage::ERROR&&fstage!=FStage::DONE) firstCalibration();
    else if(fstage==FStage::ERROR) { cal.steeringExtreme=1200; updateController(false); delay(50); }
    printDiagnostics();
    return;
  }
  static bool initialized=false,monitorMode=false;
  if(!initialized){
    initialized=true;
    monitorMode=ignition.pressed();
    Serial.println(monitorMode?F("[MODE] Serial monitor; HID output disabled."):F("[MODE] Normal HID mode."));
    if(!zeroCalibrate()){
      Serial.println(F("[WARN] Steering zero calibration failed. HID remains active; centerOffset=0."));
      centerOffset=0;
    } else {
      Serial.println(F("[OK] Steering zero calibration complete."));
    }
  }
  updateController(!monitorMode);
  if(monitorMode) printMonitor();
  printDiagnostics();
}
