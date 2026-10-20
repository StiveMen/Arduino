/* Sim_Rul V1.0 — Arduino Pro Micro 5V/16MHz
 * Gearbox X/Y are processed internally and are NOT exposed as HID axes.
 * No button matrix. Ignition + High Gear + 6 virtual gears = 8 HID buttons.
 * First start: full calibration with Ignition held at power-up.
 * Later starts: steering zero calibration only; center switch is then ignored.
 * Hold Ignition at boot to select monitor mode; monitor mode never sends HID state.
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <Joystick.h>

// ===== EASY MANUAL CALIBRATION =====
constexpr int GAS_MIN=0, GAS_MAX=1023;
constexpr int BRAKE_MIN=0, BRAKE_MAX=1023;
constexpr int CLUTCH_MIN=0, CLUTCH_MAX=1023;
constexpr int GEAR_X_MIN=0, GEAR_X_MAX=1023;
constexpr int GEAR_Y_MIN=0, GEAR_Y_MAX=1023;
constexpr int GEAR_CALIBRATION_MARGIN=15;
constexpr bool USE_SAVED_PEDAL_CALIBRATION=true;

// ===== PINS =====
constexpr uint8_t PIN_GAS=A1, PIN_BRAKE=A2, PIN_CLUTCH=A3;
constexpr uint8_t PIN_GEAR_X=A8, PIN_GEAR_Y=A9;
constexpr uint8_t PIN_ENC_A=0, PIN_ENC_B=1;
constexpr uint8_t PIN_IGNITION=5, PIN_CENTER_SW=6, PIN_HIGH_GEAR=7;
// ===== PARAMETERS =====
constexpr uint16_t BUTTON_DEBOUNCE_MS=25;
constexpr uint32_t ZERO_CAL_TIMEOUT_MS=15000UL;
constexpr uint32_t FIRST_CAL_TIMEOUT_MS=30000UL;
constexpr uint32_t MONITOR_INTERVAL_MS=250;
constexpr long MAX_ENCODER_TRAVEL=200000L;
constexpr int32_t STEERING_HID_MIN=-32767L, STEERING_HID_MAX=32767L;
constexpr uint16_t EEPROM_MAGIC=0x5347;
constexpr uint8_t EEPROM_VERSION=2;

struct GearCal { int16_t x,y; };
struct CalData {
  uint16_t magic; uint8_t version; uint8_t reserved;
  int32_t steeringExtreme;
  uint16_t gasMin,gasMax,brakeMin,brakeMax,clutchMin,clutchMax;
  GearCal gear[6];
  uint16_t checksum;
};
CalData cal{};

uint16_t checksum(const CalData &d){
  const uint8_t *p=reinterpret_cast<const uint8_t*>(&d); uint32_t s=0;
  for(size_t i=0;i<sizeof(CalData)-sizeof(d.checksum);++i) s=(s+p[i])&0xFFFF;
  return (uint16_t)s;
}
bool calValid(){
  return cal.magic==EEPROM_MAGIC && cal.version==EEPROM_VERSION &&
         cal.steeringExtreme>10 && cal.checksum==checksum(cal);
}
void saveCal(){ cal.magic=EEPROM_MAGIC; cal.version=EEPROM_VERSION; cal.reserved=0;
  cal.checksum=checksum(cal); EEPROM.put(0,cal); }

// Expose only steering and pedal axes. Gearbox X/Y are internal sensors.
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,8,0,
  true,true,true,true,false,false,false,false,false,false,false);
constexpr uint8_t HID_IGNITION=0, HID_HIGH_GEAR=1, HID_GEAR_BASE=2;

volatile long encoderRaw=0;
volatile uint8_t encoderLastAB=0;
long centerOffset=0;

void encoderISR(){
  uint8_t ab=(digitalRead(PIN_ENC_A)<<1)|digitalRead(PIN_ENC_B);
  uint8_t t=(encoderLastAB<<2)|ab;
  switch(t){
    case 1: case 7: case 14: case 8: ++encoderRaw; break;
    case 2: case 11: case 13: case 4: --encoderRaw; break;
  }
  encoderLastAB=ab;
}
long readEncoder(){ noInterrupts(); long v=encoderRaw; interrupts(); return v; }
void resetEncoder(){ noInterrupts(); encoderRaw=0; interrupts(); }

struct DebouncedInput{
  uint8_t pin; bool stable=false,lastRaw=false; uint32_t changed=0;
  void begin(uint8_t p){pin=p;pinMode(pin,INPUT_PULLUP);stable=lastRaw=(digitalRead(pin)==LOW);changed=millis();}
  void update(){bool r=digitalRead(pin)==LOW;if(r!=lastRaw){lastRaw=r;changed=millis();}
    if(millis()-changed>=BUTTON_DEBOUNCE_MS)stable=r;}
  bool pressed()const{return stable;}
};
DebouncedInput ignition,centerSwitch,highGear;


uint16_t analogStable(uint8_t p){uint32_t s=0;for(uint8_t i=0;i<4;++i)s+=analogRead(p);return s/4;}
int mapAxis(uint16_t v,int mn,int mx){if(mx<=mn)return 0;v=constrain((int)v,mn,mx);return map(v,mn,mx,0,1023);}
void pedalRanges(int &gmn,int &gmx,int &bmn,int &bmx,int &cmn,int &cmx){
  if(USE_SAVED_PEDAL_CALIBRATION && calValid()){gmn=cal.gasMin;gmx=cal.gasMax;bmn=cal.brakeMin;bmx=cal.brakeMax;cmn=cal.clutchMin;cmx=cal.clutchMax;}
  else {gmn=GAS_MIN;gmx=GAS_MAX;bmn=BRAKE_MIN;bmx=BRAKE_MAX;cmn=CLUTCH_MIN;cmx=CLUTCH_MAX;}
}

// ===== IGNITION EDGE =====
bool ignPrev=false,ignArmed=false;
void armIgn(){ignPrev=ignition.pressed();ignArmed=!ignPrev;}
bool ignEdge(){
  bool now=ignition.pressed(); if(!now)ignArmed=true;
  bool e=now&&!ignPrev&&ignArmed; if(e)ignArmed=false; ignPrev=now; return e;
}

// ===== STEERING ZERO =====
// Right -> left. When center switch is hit, encoder is reset to 0.
// Continue left until the switch releases; that release count is the zero.
// Then move right for verification. After this function the switch is ignored.
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

    // The switch must be reached while the wheel is moving left.
    // Do not require a non-zero encoder delta in the exact millisecond
    // when the debounced switch changes state.
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

  // Reliability check: after zero is found, turn right again.
  uint32_t vstart=millis();
  long verify=readEncoder();
  while(millis()-vstart<ZERO_CAL_TIMEOUT_MS){
    if(readEncoder()>verify+20)return true;
    delay(1);
  }
  return false;
}

// ===== FIRST CALIBRATION =====
enum FStage:uint8_t{ZERO,STEERING_EXTREME,PEDAL_MIN,PEDAL_MAX,G1,G2,G3,G4,G5,G6,SAVE,ERROR,DONE};
FStage fstage=FStage::ZERO;
uint32_t fstarted=0;
long steeringPeak=0;
uint16_t pedalPeakG=0,pedalPeakB=0,pedalPeakC=0;
uint32_t saveHoldStarted=0;

void setStage(uint8_t s){fstage=(FStage)s;fstarted=millis();armIgn();steeringPeak=0;
  if(fstage==FStage::PEDAL_MAX){pedalPeakG=cal.gasMin;pedalPeakB=cal.brakeMin;pedalPeakC=cal.clutchMin;}
}
uint8_t gearStage(){
  switch(fstage){case FStage::G1:return 1;case FStage::G2:return 2;case FStage::G3:return 3;
    case FStage::G4:return 4;case FStage::G5:return 5;case FStage::G6:return 6;default:return 0;}
}
void blinkSaved(){
  // D17 is occupied by the center switch, so no onboard LED is used as a
  // calibration indicator. USB Serial is the confirmation channel.
  Serial.println(F("CALIBRATION_SAVED"));
}
void firstCalibration(){
  ignition.update();centerSwitch.update();highGear.update();
  if(fstage!=FStage::SAVE && millis()-fstarted>FIRST_CAL_TIMEOUT_MS){fstage=FStage::ERROR;return;}

  if(fstage==FStage::ZERO){
    if(zeroCalibrate()){setStage(FStage::STEERING_EXTREME);}
    else fstage=FStage::ERROR;
    return;
  }
  if(fstage==FStage::STEERING_EXTREME){
    long p=labs(readEncoder()-centerOffset);if(p>steeringPeak)steeringPeak=p;
    if(ignEdge()){if(steeringPeak<10){fstage=FStage::ERROR;return;}cal.steeringExtreme=steeringPeak;setStage(FStage::PEDAL_MIN);}
    return;
  }
  if(fstage==FStage::PEDAL_MIN){
    if(ignEdge()){cal.gasMin=analogStable(PIN_GAS);cal.brakeMin=analogStable(PIN_BRAKE);cal.clutchMin=analogStable(PIN_CLUTCH);setStage(FStage::PEDAL_MAX);}
    return;
  }
  if(fstage==FStage::PEDAL_MAX){
    uint16_t g=analogStable(PIN_GAS),b=analogStable(PIN_BRAKE),c=analogStable(PIN_CLUTCH);
    if(g>pedalPeakG)pedalPeakG=g;if(b>pedalPeakB)pedalPeakB=b;if(c>pedalPeakC)pedalPeakC=c;
    if(ignEdge()){cal.gasMax=pedalPeakG;cal.brakeMax=pedalPeakB;cal.clutchMax=pedalPeakC;setStage(FStage::G1);}
    return;
  }
  uint8_t gn=gearStage();
  if(gn){
    if(ignEdge()){
      cal.gear[gn-1].x=mapAxis(analogStable(PIN_GEAR_X),GEAR_X_MIN,GEAR_X_MAX);
      cal.gear[gn-1].y=mapAxis(analogStable(PIN_GEAR_Y),GEAR_Y_MIN,GEAR_Y_MAX);
      setStage(gn<6?(FStage)((uint8_t)FStage::G1+gn):FStage::SAVE);
    }
    return;
  }
  if(fstage==FStage::SAVE){
    // User places gearbox in neutral and holds Ignition to save.
    if(ignition.pressed()){
      if(saveHoldStarted==0)saveHoldStarted=millis();
      if(millis()-saveHoldStarted>=1200){
      saveCal();
      blinkSaved();
      // Keep the calibration result in EEPROM and require a clean reboot.
      // The user can release Ignition and power-cycle/reset the controller.
      fstage=FStage::DONE;
    }
    }else saveHoldStarted=0;
  }
}

// ===== GEAR + HIGH GEAR =====
int8_t activeGear=0; bool highGearLatched=false;

// ===== PERMANENT SERIAL DIAGNOSTICS =====
const __FlashStringHelper* stageName(){
  switch(fstage){
    case FStage::ZERO: return F("ZERO");
    case FStage::STEERING_EXTREME: return F("STEERING_EXTREME");
    case FStage::PEDAL_MIN: return F("PEDAL_MIN");
    case FStage::PEDAL_MAX: return F("PEDAL_MAX");
    case FStage::G1: return F("GEAR_1");
    case FStage::G2: return F("GEAR_2");
    case FStage::G3: return F("GEAR_3");
    case FStage::G4: return F("GEAR_4");
    case FStage::G5: return F("GEAR_5");
    case FStage::G6: return F("GEAR_6");
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
bool gearZone(uint8_t i,int x,int y){
  int gx=cal.gear[i].x,gy=cal.gear[i].y;
  return x>=gx-GEAR_CALIBRATION_MARGIN&&x<=gx+GEAR_CALIBRATION_MARGIN&&
         y>=gy-GEAR_CALIBRATION_MARGIN&&y<=gy+GEAR_CALIBRATION_MARGIN;
}
int8_t findGear(int x,int y){
  int8_t found=0;for(uint8_t i=0;i<6;++i)if(gearZone(i,x,y)){if(found)return 0;found=i+1;}return found;
}
void updateGear(int x,int y){
  int8_t previousGear=activeGear;
  bool highPressed=highGear.pressed();
  activeGear=findGear(x,y);

  // High gear can latch only when its physical button is held while a gear is selected.
  if(highPressed && activeGear) highGearLatched=true;

  // On leaving a gear, keep high gear only while the physical button is still held.
  if(previousGear!=0 && activeGear==0 && !highPressed) highGearLatched=false;

  // If the user exits while holding the button, release it in neutral to clear the latch.
  if(activeGear==0 && !highPressed) highGearLatched=false;
}

// ===== HID / MONITOR =====
void updateController(bool sendHid){
  ignition.update();highGear.update();
  uint16_t gr=analogStable(PIN_GAS),br=analogStable(PIN_BRAKE),cl=analogStable(PIN_CLUTCH);
  uint16_t xr=analogStable(PIN_GEAR_X),yr=analogStable(PIN_GEAR_Y);
  int gmn,gmx,bmn,bmx,cmn,cmx; pedalRanges(gmn,gmx,bmn,bmx,cmn,cmx);
  int gas=mapAxis(gr,gmn,gmx),brake=mapAxis(br,bmn,bmx),clutch=mapAxis(cl,cmn,cmx);
  int gx=mapAxis(xr,GEAR_X_MIN,GEAR_X_MAX),gy=mapAxis(yr,GEAR_Y_MIN,GEAR_Y_MAX);
  updateGear(gx,gy);

  long steer=readEncoder()-centerOffset,e=cal.steeringExtreme;
  steer=constrain(steer,-e,e);steer=map(steer,-e,e,STEERING_HID_MIN,STEERING_HID_MAX);
  Joystick.setButton(HID_IGNITION,ignition.pressed());
  Joystick.setButton(HID_HIGH_GEAR,highGearLatched);
  for(uint8_t i=0;i<6;++i)Joystick.setButton(HID_GEAR_BASE+i,activeGear==(int8_t)(i+1));
  // HID: X=руль, Y=газ, Rx=тормоз, Z=сцепление.
  // Z назначена сцеплению; тормоз перенесён на Rx, чтобы оси не конфликтовали.
  Joystick.setXAxis(steer);Joystick.setYAxis(gas);Joystick.setRxAxis(brake);Joystick.setZAxis(clutch);
  // gx/gy are used only by updateGear(); never publish gearbox axes to USB HID.
  if(sendHid)Joystick.sendState();
}
void printMonitor(){
  static uint32_t last=0;
  if(millis()-last<MONITOR_INTERVAL_MS)return;
  last=millis();
  int gx=analogStable(PIN_GEAR_X);
  int gy=analogStable(PIN_GEAR_Y);
  int gmn,gmx,bmn,bmx,cmn,cmx;pedalRanges(gmn,gmx,bmn,bmx,cmn,cmx);
  Serial.println(F("\\n=== SIM_RUL MONITOR ==="));
  Serial.print(F("STEERING: "));Serial.println(readEncoder()-centerOffset);
  Serial.print(F("GAS: "));Serial.print(analogStable(PIN_GAS));Serial.print(F(" [MIN: "));Serial.print(gmn);Serial.print(F(" MAX: "));Serial.print(gmx);Serial.println(F("]"));
  Serial.print(F("BRAKE: "));Serial.print(analogStable(PIN_BRAKE));Serial.print(F(" [MIN: "));Serial.print(bmn);Serial.print(F(" MAX: "));Serial.print(bmx);Serial.println(F("]"));
  Serial.print(F("CLUTCH: "));Serial.print(analogStable(PIN_CLUTCH));Serial.print(F(" [MIN: "));Serial.print(cmn);Serial.print(F(" MAX: "));Serial.print(cmx);Serial.println(F("]"));
  Serial.print(F("GEAR X: "));Serial.print(gx);Serial.print(F(" [MIN: "));Serial.print(GEAR_X_MIN);Serial.print(F(" MAX: "));Serial.print(GEAR_X_MAX);Serial.println(F("]"));
  Serial.print(F("GEAR Y: "));Serial.print(gy);Serial.print(F(" [MIN: "));Serial.print(GEAR_Y_MIN);Serial.print(F(" MAX: "));Serial.print(GEAR_Y_MAX);Serial.println(F("]"));
  Serial.print(F("IGNITION: "));Serial.println(ignition.pressed()?F("ON"):F("OFF"));
  Serial.print(F("HIGH GEAR: "));Serial.println(highGear.pressed()?F("ON"):F("OFF"));
  for(uint8_t i=0;i<6;++i){Serial.print(F("GEAR "));Serial.print(i+1);Serial.print(F(": "));Serial.println(activeGear==(int8_t)(i+1)?F("ON"):F("OFF"));}
  Serial.println(F("HID OUTPUT: DISABLED (MONITOR MODE)"));
}
void monitor(){ printDiagnostics(); }

// ===== SETUP / LOOP =====
bool monitorModeAtBoot=false;
void setup(){
  Serial.begin(115200);
  pinMode(PIN_ENC_A,INPUT_PULLUP);pinMode(PIN_ENC_B,INPUT_PULLUP);
  encoderLastAB=(digitalRead(PIN_ENC_A)<<1)|digitalRead(PIN_ENC_B);
  ignition.begin(PIN_IGNITION);centerSwitch.begin(PIN_CENTER_SW);highGear.begin(PIN_HIGH_GEAR);
  analogReference(DEFAULT);
  Joystick.setXAxisRange(STEERING_HID_MIN,STEERING_HID_MAX);Joystick.begin(false);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encoderISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),encoderISR,CHANGE);
  delay(100);ignition.update();monitorModeAtBoot=ignition.pressed();EEPROM.get(0,cal);
  if(!calValid()){
    cal.steeringExtreme=1200; // Резервный диапазон оси руля до калибровки.
    if(ignition.pressed())setStage(FStage::ZERO);else fstage=FStage::ERROR;
  }else{
    // Holding Ignition at boot selects monitor mode after zero calibration.
    // The zero calibration itself is still performed first.
    fstage=FStage::DONE;
  }
}
void loop(){
  ignition.update();  centerSwitch.update();
  if(!calValid()){
    if(fstage!=FStage::ERROR&&fstage!=FStage::DONE) firstCalibration();
    else if(fstage==FStage::ERROR) { cal.steeringExtreme=1200; updateController(false); delay(50); }
    printDiagnostics();
    return;
  }
  static bool initialized=false,monitorMode=false;
  if(!initialized){
    initialized=true;
    monitorMode=monitorModeAtBoot;
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
