#include <GyverOLED.h>
#include <GyverButton.h>
#include <EEPROM.h>
#include <GyverPower.h>
#include <GyverBME280.h>

#define HALL_SENSOR_PIN 2
#define VREF_INTERNAL 1085

GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;
GButton button1(A3);
GButton button2(A2);
GyverBME280 bme;

unsigned long previousMillis = 0;
int seconds = 0;
int minutes = 0;
int hours = 0;
float temp;
int humid;
float WHEEL_DIAMETER_INCHES = 27.5;
float SaveRideDist[7];
int SaveRideMaxSPD[7];
int SaveRideTime[7];
volatile unsigned long lastTime;
volatile unsigned long currentTime;
volatile unsigned long curwrtim;
volatile unsigned long lastcrwrtim;
volatile unsigned long lastOfftim;
unsigned long curofftim;
volatile unsigned long deltaTime; 
volatile float wheelCircumference;
unsigned long lastWriteTime;
volatile int speedKPH;
int LastSpeed;
int LastMaxSpeed;
int MaxSpeed;
float distance = 0.00;
float Dist = 0.00;
const int centerX = 107;
const int centerY = 25;
const int radius = 19;
const int maxAngle = 180;
int currentAngle = 180;
bool isMoving = false;
bool isReturning = false;
int lastmVol;
int light = 127;
bool updatevol = true;
bool i = 1;
int Cursor = 0;
int BatteryPercent;
int MinBatV = 3300;
int MaxBatV = 4140;
bool HomeMenu = true;
bool bike = false;
bool modes = false;
bool battery = false;
bool settings = false;
bool ONlight = 0;
bool tick2 = 1;
bool tick1 = 1;
bool VeloAct = 0;
bool yesnomenu = 0;
bool SleepMode = 0;
bool WheathWiew = 0;
float pressure;
int remotelight = 3;
bool LightChangeON = 0;
int LightStop = 255;
int ProgBar = 0;
int addFunc = 3;
unsigned long secave = 0;
bool FSet = 0;
int tripchoice = 1;
bool TripVybor = 0;
bool SaveTripIneepr = 0;
bool BatInf = 0;
int mVol = 3100;
int page = 0;
bool diamwh = 0;
bool tech = 0;
int TechMaindist = 10;
int reminderTechMaindist = 10;
int Techcount = 10;
bool EcoOn = 0;
bool SitDriveOn = 0;
bool updatedisp = 1;
bool factRes = 0;
bool TechGO = 0;
bool AnalogSPDON = 1;
bool Toff = 1;
bool smoothlyON = 1;

const uint8_t TripsBit_21x21[] PROGMEM = {
	0x00, 0x00, 0x80, 0x80, 0x00, 0x88, 0xC8, 0x78, 0x28, 0x20, 0xA0, 0xE0, 0x60, 0x60, 0xFC, 0x84, 0x04, 0x14, 0x08, 0x00, 0x00, 
	0x06, 0x89, 0x90, 0x92, 0x0B, 0x07, 0x82, 0x82, 0x02, 0x03, 0x81, 0x00, 0x86, 0x89, 0x93, 0x10, 0x89, 0x86, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x0F, 0x00, 0x00, 0x0F, 0x00, 0x04, 0x0B, 0x00, 0x0E, 0x00, 0x0F, 0x02, 0x03, 0x00, 0x0B, 0x0A, 0x0E, 0x00, 0x00, 
};

const uint8_t OnOff_21x21[] PROGMEM = {
	0x00, 0x00, 0xC0, 0xE0, 0x70, 0x30, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x30, 0x70, 0xE0, 0xC0, 0x00, 0x00, 
	0x00, 0x7F, 0xFF, 0xE1, 0x80, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE1, 0xFF, 0x7F, 0x00, 
	0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x0E, 0x0C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x0C, 0x0E, 0x07, 0x07, 0x03, 0x01, 0x00, 0x00, 
};

const uint8_t Charging_10x15[] PROGMEM = {
	0x80, 0xC0, 0x20, 0x10, 0x08, 0x7C, 0x4E, 0x43, 0xC0, 0xC0, 
	0x01, 0x01, 0x61, 0x39, 0x1F, 0x08, 0x04, 0x02, 0x01, 0x00, 
};

const uint8_t batt_16x8[] PROGMEM = {
	0xC3, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t mode_21x21[] PROGMEM = {
	0x00, 0xC0, 0x30, 0x18, 0x0C, 0x04, 0x02, 0x82, 0x81, 0x81, 0x81, 0x81, 0x81, 0x82, 0x02, 0x04, 0x0C, 0x18, 0x30, 0xC0, 0x00, 
	0x1F, 0x60, 0x80, 0x00, 0x00, 0x00, 0x00, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x1F, 
	0x00, 0x00, 0x01, 0x03, 0x06, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x08, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00, 
};

const uint8_t EcoMode_16x9[] PROGMEM = {
	0xFC, 0x02, 0x7D, 0x55, 0x55, 0x01, 0x39, 0x45, 0x45, 0x01, 0x39, 0x45, 0x45, 0x39, 0x81, 0x7F, 
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 
};

const uint8_t Settings_27x25[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xB8, 0x18, 0x30, 0x60, 0x60, 0x38, 0x2E, 0x06, 0x0E, 0x38, 0x60, 0x60, 0x30, 0x18, 0xD8, 0xF0, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x38, 0x38, 0x6C, 0x44, 0xE7, 0x83, 0x00, 0x38, 0x7C, 0xEE, 0xC7, 0x83, 0xC7, 0xEE, 0x7C, 0x38, 0x00, 0x83, 0xCF, 0x44, 0x6C, 0x38, 0x38, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0C, 0x1E, 0x37, 0x33, 0x18, 0x0C, 0x0C, 0x38, 0xE1, 0xC1, 0xE9, 0x38, 0x0C, 0x0C, 0x18, 0x31, 0x3B, 0x1E, 0x0C, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t BikeMini_20x13[] PROGMEM = {
	0x00, 0x80, 0x40, 0x20, 0xA0, 0x44, 0xA4, 0x3C, 0x14, 0x90, 0xD0, 0x70, 0x30, 0x90, 0x50, 0x30, 0xFC, 0x42, 0x8A, 0x04, 
	0x03, 0x04, 0x08, 0x10, 0x11, 0x09, 0x05, 0x03, 0x01, 0x01, 0x01, 0x00, 0x03, 0x04, 0x08, 0x10, 0x11, 0x08, 0x04, 0x03, 
};

const uint8_t Back_21x21[] PROGMEM = {
	0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xFC, 0xFC, 0xFC, 0xE3, 0xE3, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x1C, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t BattInfo_21x21[] PROGMEM = {
	0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x18, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t Light_21x21[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xEE, 0xE0, 0xEE, 0xE0, 0xEE, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t WhBit_27x22[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0x40, 0x00, 0x80, 0xC4, 0xCC, 0xC8, 0xC0, 0xC0, 0xC0, 0xC0, 0xC8, 0x8C, 0x86, 0x00, 0x20, 0x30, 0x10, 0x18, 0x00, 0x00, 
	0xC6, 0x22, 0xB0, 0x10, 0x18, 0x0C, 0x04, 0x06, 0x02, 0x02, 0x06, 0x0C, 0x09, 0x09, 0x0B, 0x0B, 0x1B, 0x33, 0x77, 0x47, 0xCE, 0x9C, 0x3C, 0x70, 0x02, 0x42, 0x41, 
	0x07, 0x08, 0x13, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x10, 0x10, 0x10, 0x18, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t LightSet_27x25[] PROGMEM = {
	0x00, 0x00, 0x60, 0xC0, 0x80, 0x00, 0x08, 0x98, 0x90, 0xC0, 0x60, 0x60, 0x27, 0x27, 0x60, 0x60, 0x40, 0xD0, 0x98, 0x04, 0x00, 0x80, 0x80, 0xC0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x30, 0x30, 0x00, 0x78, 0xFE, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x38, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0x06, 0x06, 0x00, 0x41, 0x23, 0x26, 0x04, 0x0C, 0x08, 0xC8, 0xC8, 0x08, 0x0C, 0x04, 0x26, 0x63, 0x41, 0x00, 0x02, 0x06, 0x0C, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const uint8_t LowBatt_32x21[] PROGMEM = {
	0xFC, 0xFC, 0xFC, 0x1C, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1C, 0xFC, 0xFC, 0xFC, 
	0xFF, 0xFF, 0xFF, 0x00, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x3F, 0x3F, 0x0C, 0x0C, 0x00, 0xFF, 0xFF, 0xFF, 
	0x1F, 0x1F, 0x1F, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1F, 0x1F, 0x1F, 
};

void setup() {
  power.autoCalibrate();
  power.setSleepMode(POWERDOWN_SLEEP);
  power.hardwareEnable(PWR_ADC | PWR_TIMER1 | PWR_TIMER0);
  interrupts();
  Wire.begin();
  oled.init();
  oled.setContrast(0);
  oled.update();
  bme.setStandbyTime(STANDBY_1000MS);
  bme.begin();
  oled.setPower(1);
  oled.clear();
  oled.setScale(2);
  oled.setCursorXY(20, 13);
  
  if (!bme.begin()) {
  oled.home();
  oled.setScale(2);
  oled.print("ERR_BME");
  oled.update();
  delay(1000);
  }

  if(!factRes) {
    EprGet();
  }

  button1.setDebounce(20);
  button2.setDebounce(20);
  button2.setTimeout(400);
  button1.setTimeout(400);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);   // подтяжка пина датчика холла( Без подтяжки у вас будет скорость 3489567345678378678 км/ч )
  wheelCircumference = WHEEL_DIAMETER_INCHES * 0.0254 * PI;    // рассчет длины окружности колеса в метрах
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, RISING);   // прерывание для счета скорости
  oled.home();
  oled.clear();
  oled.update();
}

void loop() {
  pressure = bme.readPressure() / 133.322; // перевод давления из паскалей в мм рт.ст.
  temp = bme.readTemperature();
  humid = bme.readHumidity();
  oled.clear();
  oled.invertText(false);
  if(mVol < 3000) {
    oled.clear();
    oled.drawBitmap(47, 5, LowBatt_32x21, 32, 21);
    oled.update();
    delay(3000);
    Sleep();
  }
  if(SleepMode) {
    Sleep();
  }
  if(tick1) {
    button1.tick();
  }
  if(tick2) {
    button2.tick();
  }
  EcoMode();
if(VeloAct == 0){
if(HomeMenu == 0) {
  if(bike) {
    BikeMenu();
  }

  if(modes) {
    ModesMenu();
  }

  if(battery) {
    BatteryMenu();
  }

  if(settings) {
    Settings();
  }
}
}

  if(Cursor < 0) {
    Cursor = 3;
  }
  if(Cursor > 3) {
    Cursor = 0;
    page++;
  }
  if(Dist > 999.99) {
    Dist = 0.00;
  }
  mVol = getVcc();
  BatteryPercent = map(mVol, MinBatV, MaxBatV, 0, 100);
  BatteryPercent = constrain(BatteryPercent, 0, 100);

if(HomeMenu) {
  if(button2.isPress()) {
    Cursor++;
  }
  if(Cursor == 0) {
  oled.roundRect(5, 5, 31, 29, OLED_FILL);
  oled.drawBitmap(8, 9, BikeMini_20x13, 20, 13, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    bike = true;
    HomeMenu = 0;
    Cursor = 0;
  }
  }
  else {
  oled.drawBitmap(8, 9, BikeMini_20x13, 20, 13, BITMAP_NORMAL, BUF_ADD);
  oled.roundRect(5, 5, 31, 29, OLED_STROKE);
  }
  
  if(Cursor == 1){
  oled.roundRect(35, 5, 61, 29, OLED_FILL);
  oled.drawBitmap(38, 7, mode_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    modes = true;
    HomeMenu = 0;
    Cursor = 0;
  }
  }
  else {
  oled.roundRect(35, 5, 61, 29, OLED_STROKE);
  oled.drawBitmap(38, 7, mode_21x21, 21, 21, BITMAP_NORMAL, BUF_ADD);
  }

  if(Cursor == 2) {
  oled.roundRect(65, 5, 91, 29, OLED_FILL);
  oled.drawBitmap(70, 13, batt_16x8, 16, 8, BITMAP_INVERT, BUF_SUBTRACT);
  oled.setCursorXY(69, 13);
  drawBatteryinvert(BatteryPercent);
  oled.textMode(BUF_ADD);
  if(button1.isPress()) {
    battery = true;
    HomeMenu = 0;
    Cursor = 0;
  }
  }
  else {
  oled.setCursorXY(69, 13);
  drawBattery(BatteryPercent);
  if(updatevol == true) {
    oled.drawBitmap(73, 10, Charging_10x15, 10, 15, BITMAP_NORMAL, BUF_SUBTRACT);
  }
  oled.roundRect(65, 5, 91, 29, OLED_STROKE);
  }

  if(Cursor == 3) {
  oled.roundRect(95, 5, 121, 29, OLED_FILL);
  oled.drawBitmap(95, 5, Settings_27x25, 27, 25, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    FSet = 1;
    settings = 0;
    HomeMenu = 0;
    Cursor = 0;
    page = 0;
  }
  }
  else {
  oled.drawBitmap(95, 5, Settings_27x25, 27, 25);
  oled.roundRect(95, 5, 121, 29, OLED_STROKE);
  }

}

  if(mVol > lastmVol + 35) {
      updatevol = true;
    }
    if(mVol < lastmVol - 15) {
      updatevol = false;
    }

   if(ONlight) {
    tick2 = 0;
    oled.clear();
    oled.invertDisplay(ONlight);
    oled.setContrast(255);
   }
   
   if(ONlight != 1) {
    oled.invertDisplay(ONlight);
    tick2 = 1;
   }

   if(VeloAct) {
     VeloPCactive();
   }

   if(WheathWiew) {
    Wheather();
   }

  if(LightChangeON) {
    LightChange();
  }

  if(FSet) {
    FastSet();
  }

  if(SaveTripIneepr) {
    SaveTrips();
  }

  if(BatInf) {
    BatteryInfo();
  }

  if(diamwh) {
    SetDiameter();
  }

  if(tech) {
    SetTechMaindist();
  }

  if(updatedisp) {
    oled.update();
  }

  if(smoothlyON && updatedisp) {
    for(int zov = 0; zov < light; zov++) {
        oled.setContrast(zov);
        delay(8);
      }
      smoothlyON = 0;
  }

  if(!ONlight) {
    oled.setContrast(light);
  }

    curwrtim = millis();
    if(curwrtim - lastcrwrtim >= 20000){
    EEPROM.put(10, light);
    EEPROM.put(165, remotelight);
    lastcrwrtim = curwrtim;
    }
    lastmVol = mVol;
}

uint16_t getVcc(void) {
  ADMUX = (1 << REFS0) | 0b1110;
  ADCSRA = (1 << ADEN) | 0b111;
  for (uint8_t i = 0; i < 3; i++) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    volatile uint16_t a = ADC;
  }
  uint32_t result = 0;
  for (uint8_t i = 0; i < 4; i++) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    result += ADC;
  } result /= 4;
  return ((float)(VREF_INTERNAL * 1024UL) / result);
}

void drawBattery(byte percent) {
  oled.drawByte(0b00111100);
  oled.drawByte(0b00111100);
  oled.drawByte(0b11111111);
  for (byte i = 0; i < 100 / 8; i++) {
    if (i < (100 - percent) / 8) oled.drawByte(0b10000001);
    else oled.drawByte(0b11111111);
  }
  oled.drawByte(0b11111111);
}

void drawBatteryinvert(byte percent) {
  oled.textMode(BUF_SUBTRACT);
  oled.drawByte(0b11000011);
  oled.drawByte(0b11000011);
  oled.drawByte(0b00000000);
  for (byte i = 0; i < 100 / 8; i++) {
    if (i < (100 - percent) / 8) oled.drawByte(0b01111110);
    else oled.drawByte(0b00000000);
  }
  oled.drawByte(0b00000000);
}

void hallSensorISR() {

  if(SitDriveOn) {
    VeloAct = 1;
    addFunc = 3;
  }

  if(SleepMode) {
  power.wakeUp();
  SleepMode = 0;
  }

  lastTime = currentTime;
  currentTime = millis();
  deltaTime = currentTime - lastTime;
  speedKPH = (((wheelCircumference / deltaTime) * 3600) - 1);

  if(speedKPH < 0 ) {
    speedKPH = 0;
  }

  if(speedKPH > LastSpeed + 20) {
    speedKPH = LastSpeed;
  }
  
  if(speedKPH > MaxSpeed){
    MaxSpeed = speedKPH;
  }
  
  if(MaxSpeed > LastMaxSpeed + 20) {
    MaxSpeed = LastMaxSpeed;
  }

  currentTime = millis();
  Dist += wheelCircumference / 1051.993355;
  lastTime = currentTime;
  distance += wheelCircumference / 1051.993355;
  LastSpeed = speedKPH;
  LastMaxSpeed = MaxSpeed;
}

void VeloPCactive() {

  if(speedKPH > 15) {
    tick1 = 0;
    tick2 = 0;
  }
  else {
    tick1 = 1;
    tick2 = 1;
  }

  oled.clear();
  oled.setScale(3);
  oled.setCursorXY(0, 0);
  oled.print(speedKPH);
  oled.setScale(1);
  oled.setCursorXY(0, 24);
  oled.print(Dist);
   oled.setCursorXY(38, 24);
  oled.setScale(1);
  switch(addFunc) {
    case 0:
     oled.print(String(temp, 1) + "C");
    break;

    case 1:
     oled.print(String(humid) + "%");
    break;

    case 2:
     oled.print(String(pressure) + "mm");
    break;

    case 3:
     oled.print(hours);
     oled.print(":");
     oled.print(minutes);
     oled.print(":");
     oled.print(seconds);
    break;
  }
  oled.setScale(2);
  oled.setCursorXY(43, 4);
  oled.print(MaxSpeed);
  oled.line(36, 0, 36, 32);
  oled.line(0, 22, 80, 22);

  if(AnalogSPDON) {
   currentAngle = map(speedKPH, 0, 70, 180, 0);
  if(currentAngle > 180) {
    currentAngle = 180;
  }

  if(currentAngle < 0) {
    currentAngle = 0;
  }

    if (isMoving && currentAngle < maxAngle) {
        currentAngle++;
    } else if (!isMoving && currentAngle > 0) {
        isReturning = true;
    }

    if (isReturning && currentAngle > 0) {
        currentAngle--;
    }

    oled.circle(centerX, centerY, radius + 1, OLED_STROKE);
    oled.circle(centerX, centerY, radius, OLED_STROKE);

    oled.circle(centerX, centerY, 3);

    for (int i = 0; i <= 9; i++) {
        int angle = map(i, 0, 9, 0, maxAngle);
        int x1 = centerX + radius * cos(radians(angle));
        int y1 = centerY - radius * sin(radians(angle));
        int x2 = centerX + (radius - 5) * cos(radians(angle));
        int y2 = centerY - (radius - 5) * sin(radians(angle));
        oled.line(x1, y1, x2, y2);
    }

    int arrowX = centerX + radius * cos(radians(currentAngle));
    int arrowY = centerY - radius * sin(radians(currentAngle));
    oled.line(centerX, centerY, arrowX, arrowY);
    
  }
  else {
    oled.setCursorXY(90, 12);
    oled.print(String(BatteryPercent) + "%");
  }
  
    unsigned long currentMillisfortime = millis();
  if (currentMillisfortime - previousMillis >= 984) {
    previousMillis = currentMillisfortime;
    secave++;
    seconds++;
    if (seconds >= 60) {
      seconds = 0;
      minutes++;
      if (minutes >= 60) {
        minutes = 0;
        hours++;
        if (hours >= 99) {
          hours = 0;
        }
      }
    }

  }
    
    if (distance >= Techcount) {
        TechGO = true;
    } else {
        TechGO = false;
    }

    if(TechGO) {
    oled.clear();
    oled.setCursorXY(42, 13);
    oled.setScale(1);
    oled.print("TechMain");
    oled.update();
    Techcount = distance + TechMaindist;
    TechGO = 0;
    delay(5000);
    }


    if(button2.isHold()) {
      yesnomenu = 1;
      Cursor = 0;
    }

    if(button2.isDouble()) {}

    if(yesnomenu == 1) {
    if(button2.isClick()) {
      Cursor++;
    }
    oled.roundRect(30, 1, 100, 31, OLED_CLEAR);
    oled.roundRect(30, 1, 100, 31, OLED_STROKE);
    oled.setCursorXY(52, 23);
    oled.setScale(1);
    oled.textMode(BUF_ADD);
    oled.print("Exit?");

   if(Cursor == 1) {
  oled.roundRect(33, 4, 63 , 21, OLED_FILL);
  oled.setScale(1);
  oled.setCursorXY(43, 9);
  oled.textMode(BUF_SUBTRACT);
  oled.print("No");
  if(button1.isClick()) {
    VeloAct = 1;
    yesnomenu = 0;
  }
  }
  else {
  oled.textMode(BUF_ADD);
  oled.roundRect(33, 4, 63 , 21, OLED_STROKE);
  oled.setScale(1);
  oled.setCursorXY(43, 9);
  oled.print("No");
  }

  if(Cursor == 0) {
  oled.roundRect(66, 4, 97, 21, OLED_FILL);
  oled.setCursorXY(73, 9);
  oled.textMode(BUF_SUBTRACT);
  oled.setScale(1);
  oled.print("Yes");
  if(button1.isPress()) {
    EEPROM.put(1, distance);
    yesnomenu = 0;
    VeloAct = 0;
    Cursor = 0;
  }
  if(button2.isDouble()) {
    EEPROM.put(1, distance);
    yesnomenu = 0;
    VeloAct = 0;
    Cursor = 0;
    secave = 0;
    MaxSpeed = 0;
    Dist = 0;
    seconds = 0;
    minutes = 0;
    hours = 0;
  }
  }
  else {
  oled.textMode(BUF_ADD);
  oled.roundRect(66, 4, 97, 21, OLED_STROKE);
  oled.setCursorXY(73, 9);
  oled.print("Yes");
  }

  if(Cursor > 1) {
    Cursor = 0;
  }
  }
  
  // Костыли для исправления бага с кнопками (зато работает)
  if(button1.isPress()) {}
  if(button1.isClick()) {}

  if(button1.isHolded()) {
      addFunc++;
      if(addFunc > 3) {
      addFunc = 0;
      }
  }

  unsigned long WriteTime = millis();

  if(WriteTime - lastWriteTime >= 300000) {
    lastWriteTime = WriteTime;
    EEPROM.put(1, distance);
  }

}

void BikeMenu() {
  oled.clear();
  if(button2.isPress()) {
    Cursor++;
  }
  if(Cursor == 0) {
  oled.roundRect(5, 5, 31, 29, OLED_FILL);
  oled.setCursorXY(7, 10);
  oled.setScale(2);
  oled.textMode(BUF_SUBTRACT);
  oled.print("GO");
  if(button1.isPress()) {
    Cursor = 0;
    VeloAct = 1;
    yesnomenu = 0;
  }
  }
  else {
  oled.setCursorXY(7, 10);
  oled.setScale(2);
  oled.textMode(BUF_ADD);
  oled.print("GO");
  oled.roundRect(5, 5, 31, 29, OLED_STROKE);
  }
  
  if(Cursor == 1){
  oled.roundRect(35, 5, 61, 29, OLED_FILL);
  oled.drawBitmap(38, 6, TripsBit_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    bike = 0;
    Cursor = 0;
    SaveTripIneepr = 1;
  }
  }
  else {
  oled.roundRect(35, 5, 61, 29, OLED_STROKE);
  oled.drawBitmap(38, 6, TripsBit_21x21, 21, 21, BITMAP_NORMAL, BUF_ADD);
  }

  if(Cursor == 2) {
  oled.roundRect(65, 5, 91, 29, OLED_FILL);
  oled.setCursorXY(70, 13);
  oled.setScale(1);
  oled.textMode(BUF_SUBTRACT);
  oled.print("ODO");
  if(button1.isPress()) {
    EEPROM.get(1, distance);
    oled.clear();
    oled.setScale(2);
    oled.textMode(BUF_ADD);
    oled.home();
    oled.print(String(distance) + "Км");
    oled.update();
    delay(1000);
  }
  }
  else {
  oled.setCursorXY(70, 13);
  oled.setScale(1);
  oled.textMode(BUF_ADD);
  oled.print("ODO");
  oled.roundRect(65, 5, 91, 29, OLED_STROKE);
  }

  if(Cursor == 3) {
  oled.roundRect(95, 5, 121, 29, OLED_FILL);
  oled.drawBitmap(97, 8, Back_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    Cursor = 0;
    bike = 0;
    HomeMenu = 1;
  }
  }
  else {
  oled.drawBitmap(97, 8, Back_21x21, 21, 21);
  oled.roundRect(95, 5, 121, 29, OLED_STROKE);
  }
  tick1 = 1;
  tick2 = 1;
}

void ModesMenu() {
  oled.clear();
    if(button2.isPress()) {
    Cursor++;
  }

  if(Cursor > 2) {
    Cursor = 0;
  }
  
  if(Cursor == 0){
  oled.roundRect(35, 5, 61, 29, OLED_FILL);
  oled.drawBitmap(35, 6, WhBit_27x22, 27, 22, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
     WheathWiew = 1;
  }
  }
  else {
  oled.roundRect(35, 5, 61, 29, OLED_STROKE);
  oled.drawBitmap(35, 6, WhBit_27x22, 27, 22, BITMAP_NORMAL, BUF_ADD);
  }

  if(Cursor == 1) {
  oled.roundRect(65, 5, 91, 29, OLED_FILL);
  oled.drawBitmap(67, 8, Light_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    ONlight = !ONlight;
  }
  }
  else {
  oled.drawBitmap(67, 8, Light_21x21, 21, 21, BITMAP_NORMAL, BUF_ADD);
  oled.roundRect(65, 5, 91, 29, OLED_STROKE);
  }

  if(Cursor == 2) {
  oled.roundRect(95, 5, 121, 29, OLED_FILL);
  oled.drawBitmap(97, 8, Back_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    Cursor = 0;
    modes = 0;
    HomeMenu = 1;
  }
  }
  else {
  oled.drawBitmap(97, 8, Back_21x21, 21, 21);
  oled.roundRect(95, 5, 121, 29, OLED_STROKE);
  }
}

void BatteryMenu() {
  oled.clear();
    if(button2.isPress()) {
    Cursor++;
  }
  if(Cursor == 0) {
  oled.roundRect(5, 5, 31, 29, OLED_FILL);
  oled.drawBitmap(8, 6, BattInfo_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    battery = false;
    Cursor = 0;
    BatInf = 1;
  }
  }
  else {
  oled.drawBitmap(8, 6, BattInfo_21x21, 21, 21, BITMAP_NORMAL, BUF_ADD);
  oled.roundRect(5, 5, 31, 29, OLED_STROKE);
  }
  
  if(Cursor == 1){
  oled.roundRect(35, 5, 61, 29, OLED_FILL);
  oled.drawBitmap(41, 13, EcoMode_16x9, 16, 9, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    EcoOn = !EcoOn;
    if(EcoOn) {
      curofftim = millis();
      lastOfftim = curofftim;
      oled.clear();
      oled.setScale(2);
      oled.print("Eco ON");
      oled.update();
      delay(250);
    }
    if(!EcoOn) {
      EcoMode();
      oled.clear();
      oled.setScale(2);
      oled.print("Eco OFF");
      oled.update();
      delay(250);
    }
  }
  }
  else {
  oled.roundRect(35, 5, 61, 29, OLED_STROKE);
  oled.drawBitmap(41, 13, EcoMode_16x9, 16, 9, BITMAP_NORMAL, BUF_ADD);
  }

  if(Cursor == 2) {
  oled.roundRect(65, 5, 91, 29, OLED_FILL);
  oled.drawBitmap(68, 7, OnOff_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  oled.textMode(BUF_ADD);
  if(button1.isHold()) {
    SleepMode = 1;
  }
  }
  else {
  oled.drawBitmap(68, 7, OnOff_21x21, 21, 21, BITMAP_NORMAL, BUF_ADD);
  oled.roundRect(65, 5, 91, 29, OLED_STROKE);
  }

  if(Cursor == 3) {
  oled.roundRect(95, 5, 121, 29, OLED_FILL);
  oled.drawBitmap(97, 8, Back_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    Cursor = 0;
    battery = false;
    HomeMenu = 1;
  }
  }
  else {
  oled.drawBitmap(97, 8, Back_21x21, 21, 21);
  oled.roundRect(95, 5, 121, 29, OLED_STROKE);
  }
}

void EcoMode() {
  if(EcoOn) {
    curofftim = millis();
    if(!digitalRead(A2) || !digitalRead(A3)) {
      tick1 = 1;
      tick2 = 1;
      oled.setPower(1);
      updatedisp = 1;
      Toff = 1;
      lastOfftim = curofftim;
    }
    light = 80;
    if(curofftim - lastOfftim >= 30000 && Toff) {
      tick1 = 0;
      tick2 = 0;
      oled.clear();
      for(int zov = 255; zov > 0; zov--) {
        oled.setContrast(zov);
        delay(8);
      }
      oled.update();
      smoothlyON = 1;
      oled.setPower(0);
      updatedisp = 0;
      lastOfftim = curofftim;
      Toff = 0;
      tick2 = 1;
    }
  }
  if(!EcoOn) {
    light = map(remotelight, 0, 9, 0, 255);
  }
}

void SetDiameter() {
    oled.clear();

    // Изменение диаметра в зависимости от нажатия кнопок
    if (button2.isClick()) {
        WHEEL_DIAMETER_INCHES -= 0.5;
    } else if (button1.isClick()) {
        WHEEL_DIAMETER_INCHES += 0.5;
    }

    // Ограничение значения диаметра
    WHEEL_DIAMETER_INCHES = constrain(WHEEL_DIAMETER_INCHES, 0, 50);

    // Отображение обновленного диаметра
    oled.home();
    oled.setScale(1);
    oled.textMode(BUF_ADD);
    oled.print("Diameter: " + String(WHEEL_DIAMETER_INCHES));

    // Проверка удержания кнопки для сохранения настроек
    if (button2.isHold()) {
        wheelCircumference = WHEEL_DIAMETER_INCHES * 0.0254 * PI;    // рассчет длины окружности колеса в метрах
        settings = 1;
        Cursor = 0;
        page = 0;
        diamwh = 0;
        EEPROM.put(175, WHEEL_DIAMETER_INCHES);
    }
}

void SetTechMaindist() {
    oled.clear();

    if (button2.isClick()) {
        TechMaindist = max(10, TechMaindist - 10);
    }
    if (button1.isClick()) {
        TechMaindist = min(1500, TechMaindist + 10);
    }

    oled.home();
    oled.setScale(1);
    oled.textMode(BUF_ADD);
    oled.println("TechDist: " + String(TechMaindist) + "Km");

    reminderTechMaindist = Techcount - distance;
    oled.println("Dist to repair: " + String(reminderTechMaindist) + "Km");

    if (button2.isHold()) {
        Techcount = distance + TechMaindist;
        settings = 1;
        Cursor = 0;
        tech = 0;
    }
}

void displayMenu(const String options[], int numOptions, int cursor) {
    for (int i = 0; i < numOptions; i++) {
        if (i == cursor) {
            oled.invertText(true);
        } else {
            oled.invertText(false);
        }
        oled.println(options[i]);
    }
}

void handleButtonPress(int cursor, int page) {
    if (button1.isPress()) {
        if (page == 0) {
            switch (cursor) {
                case 0:
                    settings = 0;
                    Cursor = 0;
                    page = 0;
                    diamwh = 1;
                    EprGet();
                    break;
                case 1:
                    settings = 0;
                    Cursor = 0;
                    page = 0;
                    tech = 1;
                    break;
                case 2:
                    AnalogSPDON = !AnalogSPDON;
                    break;
                case 3:
                    noInterrupts();
                    asm volatile ("JMP 0x00");
                    break;
            }
        } else if (page == 1) {
            switch (cursor) {
                case 0:
                    SitDriveOn = !SitDriveOn;
                    break;
                case 1:
                    factRes = 1;
                    ClearEEPROM();
                    break;
                case 2:
                    HomeMenu = 1;
                    settings = 0;
                    Cursor = 0;
                    page = 0;
                    break;
            }
        }
    }
}

void Settings() {
    oled.clear();
    if (button2.isPress()) {
        Cursor++;
    }

    if (Cursor > 2 && page == 1) {
      Cursor = 0;
      page++;
    }

    if (Cursor > 3) {
        Cursor = 0;
        page++;
    }

    if(page > 1) {
      page = 0;
    }
    
    oled.home();
    oled.setScale(1);
    oled.textMode(BUF_ADD);
    
    const String menuPage0[] = {"Wheel Diameter", "TechMain", "AnalogSPD:" + String(AnalogSPDON), "Reboot"};
    const String menuPage1[] = {"SitDrive:" + String(SitDriveOn), "Factory Reset", "Back"};
    
    if (page == 0) {
        displayMenu(menuPage0, sizeof(menuPage0) / sizeof(menuPage0[0]), Cursor);
    } else if (page == 1) {
        displayMenu(menuPage1, sizeof(menuPage1) / sizeof(menuPage1[0]), Cursor);
    }

    handleButtonPress(Cursor, page);
}

void LightChange() {
    tick1 = 1;
    tick2 = 1;

    if (button1.isRelease()) {
        remotelight++;
    }
    if (button2.isRelease()) {
        remotelight--;
    }

    remotelight = constrain(remotelight, 0, 9);

    oled.clear();
    oled.setCursorXY(40, 0);
    oled.setScale(1);
    oled.textMode(BUF_ADD);
    oled.print("Light");
    oled.roundRect(5, 20, 122, 27, OLED_STROKE);

    light = map(remotelight, 0, 9, 0, LightStop);
    int progBarWidth = (remotelight > 0) ? map(remotelight, 1, 9, 17, 122) : 0;

    if (progBarWidth > 0) {
        oled.roundRect(5, 20, progBarWidth, 27, OLED_FILL);
    }

    if (button2.isHold()) {
        Cursor = 0;
        FSet = 1;
        LightChangeON = 0;
    }
    Cursor = 0;
}

void FastSet() {
  oled.clear();
  if(button2.isPress()) {
    Cursor++;
  }
  if(Cursor == 0) {
  oled.roundRect(5, 5, 31, 29, OLED_FILL);
  oled.drawBitmap(5, 5, LightSet_27x25, 27, 25, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    settings = false;
    Cursor = 0;
    FSet = 0;
    LightChangeON = 1;
  }
  }
  else {
  oled.drawBitmap(5, 5, LightSet_27x25, 27, 25, BITMAP_NORMAL, BUF_ADD);
  oled.roundRect(5, 5, 31, 29, OLED_STROKE);
  }
  
  if(Cursor == 1){
  oled.roundRect(35, 5, 61, 29, OLED_FILL);
  oled.setCursorXY(40, 13);
  oled.setScale(1);
  oled.textMode(BUF_SUBTRACT);
  oled.print("SET");
  if(button1.isPress()) {
    settings = 1;
    Cursor = 0;
    FSet = 0;
  }
  }
  else {
  oled.roundRect(35, 5, 61, 29, OLED_STROKE);
  oled.setCursorXY(40, 13);
  oled.setScale(1);
  oled.textMode(BUF_ADD);
  oled.print("SET");
  }

  if(Cursor == 2) {
  oled.roundRect(65, 5, 91, 29, OLED_FILL);
  oled.setCursorXY(70, 9);
  oled.setScale(1);
  oled.textMode(BUF_SUBTRACT);
  oled.print("EPR");
  oled.setCursorXY(70, 18);
  oled.print("RES");
  if(button1.isPress()) {
    ClearEEPROM();
  }
  }
  else {
  oled.setCursorXY(70, 9);
  oled.setScale(1);
  oled.textMode(BUF_ADD);
  oled.print("EPR");
  oled.setCursorXY(70, 18);
  oled.print("RES");
  oled.roundRect(65, 5, 91, 29, OLED_STROKE);
  }

  if(Cursor == 3) {
  oled.roundRect(95, 5, 121, 29, OLED_FILL);
  oled.drawBitmap(97, 8, Back_21x21, 21, 21, BITMAP_NORMAL, BUF_SUBTRACT);
  if(button1.isPress()) {
    settings = 0;
    bike = 0;
    modes = 0;
    battery = 0;
    HomeMenu = 1;
    Cursor = 0;
    FSet = 0;
  }
  }
  else {
  oled.drawBitmap(97, 8, Back_21x21, 21, 21);
  oled.roundRect(95, 5, 121, 29, OLED_STROKE);
  }
}

void ClearEEPROM() {
  oled.clear();
  oled.home();
  oled.textMode(BUF_ADD);
  if(!factRes) {
    oled.print("Clear EEPROM");
  }
  if(factRes) {
    oled.print("res to factory set");
  }
  oled.roundRect(5, 20, 122, 27, OLED_STROKE);
  oled.update();
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
    ProgBar = map(i, 6, 1023, 6, 122);
    oled.roundRect(6, 20, ProgBar, 27, OLED_FILL);
    oled.update();
  }
  oled.clear();
  oled.home();
  if(!factRes) {
    oled.print("Clear EEPROM done");
  }
  if(factRes) {
    oled.print("FactRes done");
    factRes = 0;
  }
  oled.update();
  delay(700);
  oled.clear();
  oled.home();
  oled.print("reboot");
  oled.update();
  delay(1000);
  noInterrupts();
  asm volatile ("JMP 0x00");
}

void Wheather() {
  tick2 = 0;
  Cursor = 1;
  oled.clear();
  oled.home();
  oled.textMode(BUF_ADD);
  oled.setScale(1);
  oled.print(temp, 1);
  oled.println("C");
  oled.setCursorXY(0, 10);
  oled.print(humid, 1);
  oled.println("%");
  oled.setCursorXY(0, 20);
  oled.print(pressure);
  oled.println("mm");
  oled.update();

  if(!digitalRead(A2)) {
    Cursor = 1;
    tick2 = 1;
    modes = 1;
    WheathWiew = 0;
  }
}

void BatteryInfo() {
  oled.clear();
  oled.home();
  oled.textMode(BUF_ADD);
  oled.setScale(1);
  oled.println(String(mVol) + "mV");
  oled.println(String(BatteryPercent) + "%");
  oled.println("ECO Mode:" + String(EcoOn));
  if(button2.isHold()) {
    battery = true;
    Cursor = 0;
    BatInf = 0;
  }
}

void SaveTrips() {
    oled.clear();
    oled.home();
    oled.textMode(BUF_ADD);
    oled.setScale(1);

    switch (TripVybor) {
        case 0:
            if (button1.isClick()) {
                tripchoice = constrain(tripchoice + 1, 1, 6);
            }
            if (button2.isClick()) {
                tripchoice = constrain(tripchoice - 1, 1, 6);
            }

            oled.print("SaveTrip:" + String(tripchoice));

            if (button1.isHold()) {
                bike = true;
                Cursor = 0;
                SaveTripIneepr = 0;
            }
            if (button2.isHold()) {
                TripVybor = 1;
            }
            break;

        case 1:
            EprGet();
            oled.println("      Savetrip " + String(tripchoice));
            oled.println(String(SaveRideDist[tripchoice]) + " Km");
            oled.println(String(SaveRideMaxSPD[tripchoice]) + " Km/h (Max)");
            oled.print("Time:" + String(SaveRideTime[tripchoice]) + "s");

            if (button2.isDouble()) {
                TripVybor = 0;
            }
            if (button1.isHold()) {
                SaveRideTime[tripchoice] = secave;
                SaveRideMaxSPD[tripchoice] = MaxSpeed;
                SaveRideDist[tripchoice] = Dist;
                EprPut();
                oled.clear();
                oled.home();
                oled.textMode(BUF_ADD);
                oled.print("Curr trip save");
                oled.update();
                delay(800);
            }
            break;
    }
}

void EprGet() {
  EEPROM.get(1, distance);
  EEPROM.get(10, light);
  for (int i = 0; i < 6; i++) {
    int currentAddress = 15 + i * 15;
    EEPROM.get(currentAddress, SaveRideDist[i]);
    EEPROM.get(currentAddress + 5, SaveRideMaxSPD[i]);
    EEPROM.get(currentAddress + 10, SaveRideTime[i]);
    if (currentAddress > 160) {
      break;
    }
  }
  EEPROM.get(165, remotelight);
  EEPROM.get(175, WHEEL_DIAMETER_INCHES);
}

void EprPut() {
  EEPROM.put(1, distance);
  EEPROM.put(10, light);
  for (int i = 0; i < 6; i++) {
    int currentAddress = 15 + i * 15;
    EEPROM.put(currentAddress, SaveRideDist[i]);
    EEPROM.put(currentAddress + 5, SaveRideMaxSPD[i]);
    EEPROM.put(currentAddress + 10, SaveRideTime[i]);
    if (currentAddress > 160) {
      break;
    }
  }
  EEPROM.put(165, remotelight);
  EEPROM.put(175, WHEEL_DIAMETER_INCHES);
}
void Sleep() {
  oled.clear();
  oled.update();
  SleepMode = 1;
  power.sleep(SLEEP_FOREVER);
}