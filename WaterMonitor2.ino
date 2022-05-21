#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFunLSM6DS3.h>
#include <ArduinoBLE.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define DISPLAY
#define USEIMU

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#ifdef DISPLAY
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#ifdef USEIMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);
#endif

#define ACCEL_Z_THRESH      0.02 
#define FLOW_MAX_LITRE_MIN  24.0

#define ORIENT_MIN          270.0  
#define ORIENT_TRESH        5.0  
#define ORIENT_MAX          300.0

#define FLOW_MAX_MILLILITRE_SEC  ((FLOW_MAX_LITRE_MIN * 1000.0) / 60.0)
#define CO2_GRAMS_PER_LITER   60.0

bool tapOpen = false;
int threshCntr = 0;
double totalMilliLitres = 0;
double partialMilliLitres = 0;
double totalCO2 = 0;
double partialCO2 = 0;
double lastAccelZ = 0;
double lastHeading = 0;
float accelZClose = 0.94;
float accelZFull = 0.78;

#define APP_NORMAL            0
#define APP_CALIBRATION1      1
#define APP_CALIBRATION2      2
int appStatus = APP_NORMAL;
int debounceCntr = 0;

#ifdef USEBLE
///////////////////////////////////////////////////////////////////////////////////
// BLE
// Define custom BLE service for position (read-only)
BLEService posService("95ff7bf8-aa6f-4671-82d9-22a8931c5387");
BLEStringCharacteristic pos("8f0e70fc-2a4b-4fd3-b487-6babf2e220e1", BLERead | BLENotify, 30);
#endif

void setupBLE()
{
#ifdef USEBLE
  Serial.print("Setting up BLE...");
  
  // Initialize BLE
  if(!BLE.begin()) {
    // Failed to initialize BLE, blink the internal LED
    Serial.println("AceBle: Failed initializing BLE");
    return;
  }

  // Set advertised local name and services UUID
  BLE.setDeviceName("Arduino Nano 33 IoT");
  BLE.setLocalName("ACE");

  posService.addCharacteristic(pos);
  BLE.addService(posService);

  // Start advertising
  BLE.advertise();
  Serial.println("DONE");
#endif
}

#define LITRES_X        0
#define LITRES_Y        0
#define LITRES_W        80
#define LITRES_H        16
#define LITRES_X_TOT    80
#define LITRES_Y_TOT    0
#define ACCELZ_Y        8
#define LITRES_UDM_X    73
#define LITRES_UDM_Y    12

#define CO2_X           0
#define CO2_Y           16
#define CO2_W           80
#define CO2_H           16
#define CO2_X_TOT       80
#define CO2_Y_TOT       16
#define HEADING_Y       24
#define CO2_UDM_X       73
#define CO2_UDM_Y       20

int x = 0;
bool lr = true;
void updateDisplay()
{
  char text[20];

  display.clearDisplay();

  // Litres
  display.fillRect(LITRES_X, LITRES_Y, LITRES_W, LITRES_H, SSD1306_BLACK);

  display.setCursor(LITRES_X, LITRES_Y);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  sprintf(text, "%6.1f", partialMilliLitres/1000.0);
  display.print(text);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(LITRES_UDM_X, LITRES_UDM_Y);
  display.print("l");

  display.setCursor(LITRES_X_TOT, LITRES_Y_TOT);
  sprintf(text, "%6.1f l", totalMilliLitres/1000.0);
  display.print(text);

  display.setCursor(LITRES_X_TOT, ACCELZ_Y);
  sprintf(text, "%6.0f", lastAccelZ);
  display.print(text);

  // CO2
  display.fillRect(CO2_X, CO2_Y, CO2_W, CO2_H, SSD1306_BLACK);

  display.setCursor(CO2_X, CO2_Y);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  sprintf(text, "%6.1f", partialCO2/1000.0);
  display.print(text);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(CO2_UDM_X, CO2_UDM_Y);
  display.print("kg");

  display.setCursor(CO2_X_TOT, CO2_Y_TOT);
  sprintf(text, "%6.1f g", totalCO2/1000.0);
  display.print(text);

  display.setCursor(CO2_X_TOT, HEADING_Y);
  sprintf(text, "%6.0f", lastHeading);
  display.print(text);

  display.display();

  /*
  if (lr)
  {
    x ++;
    if (x >= 128-30) lr = false;
  }
  else
  {
    x--;
    if (x == 0) lr = true;
  }

  display.clearDisplay();
  display.fillRect(x, 0, 30, 30, SSD1306_WHITE);
  display.display();
  */
}

void showCalibrationStatus1()
{
  display.clearDisplay();

  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.print("Turn\nfully on");

  display.display();
}

void showCalibrationStatus2()
{
  display.clearDisplay();

  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.print("Close\nthe tap");

  display.display();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Processor came out of reset.\n");

  pinMode(B10, INPUT_PULLUP);
  
#ifdef USEIMU
  //Call .begin() to configure the IMU
  myIMU.begin();
#endif

#ifdef DISPLAY
  // Clear the buffer
  display.begin();
  display.clearDisplay();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
#endif

  setupBLE();
  updateDisplay();
}

void loop()
{
  static auto prevMs = millis();
  static auto prevFastMs = millis();
  static auto printTime = millis();
  static float avgGX = 0;
  static float heading = 0; //orientation.heading(); 

  float aX, aY, aZ;
  float gX, gY, gZ;

  aX = myIMU.readFloatAccelX();
  aY = myIMU.readFloatAccelY();
  aZ = myIMU.readFloatAccelZ();
  gX = myIMU.readFloatGyroX();
  gY = myIMU.readFloatGyroY();
  gZ = myIMU.readFloatGyroZ();

  //Serial.println(String("rotation: ") + String(gX)+String(";")+String(gY)+String(";")+String(gZ)+String(";"));

  if (millis() - printTime >= 1000) {
    printTime = millis();

    Serial.println(String("acceleration: ") + String(aX)+String(";")+String(aY)+String(";")+String(aZ)+String(";"));
    Serial.println(String("rotation: ") + String(gX)+String(";")+String(gY)+String(";")+String(gZ)+String(";"));

    Serial.println(String("input: ") + String(digitalRead(B10)));
    Serial.println(String("status: ") + String(appStatus)+";"+String(debounceCntr));
    Serial.println(String("thresholds: ") + String(accelZClose)+";"+String(accelZFull));
    Serial.println(String("heading: ") + String(heading));
  }

  avgGX = (avgGX * 0.9) + (gX * 0.1);
  
  float delta = millis() - prevFastMs;
  if (delta > 0)
  {
    if ((appStatus == APP_NORMAL) && tapOpen && (abs(avgGX) > 5.0))
    {
      heading = heading + (avgGX * delta / 1000.0);
    }

    prevFastMs = millis();
  }
  
  if (millis() - prevMs >= 100) 
  {
    if (appStatus == APP_NORMAL)
    {
      float accelZ = aZ;
  
      if (accelZ < accelZClose-ACCEL_Z_THRESH)
      {
        if (!tapOpen)
        {
          threshCntr ++;
          if (threshCntr > 3)
          {
            partialMilliLitres = 0;
            partialCO2 = 0;
            tapOpen = true;
          }
        }
        else
        {
          threshCntr = 0;
        }
      }
      else if (accelZ > accelZClose-ACCEL_Z_THRESH)
      {
        if (tapOpen)
        {
          threshCntr ++;
          if (threshCntr > 3)
          {
            tapOpen = false;
          }
        }
        else
        {
          threshCntr = 0;
        }
      }
  
      if (tapOpen)
      {
        // compute current flow
        if (accelZ < accelZFull) accelZ = accelZFull;
        if (accelZ > accelZClose) accelZ = accelZClose;
  
        double perc = (accelZFull + (accelZ - accelZClose)) / (accelZClose - accelZFull);
        double flow = FLOW_MAX_MILLILITRE_SEC * perc;
        double deltaMs = millis() - prevMs;
        double milliLitres = flow * deltaMs / 1000.0; 
  
        partialMilliLitres += milliLitres;
        totalMilliLitres += milliLitres;
  
        if (heading < ORIENT_MIN) heading = ORIENT_MIN;
        if (heading > ORIENT_MAX) heading = ORIENT_MAX;
  
        double percHot = (heading - ORIENT_MIN) / (ORIENT_MAX - ORIENT_MIN);
        double co2 = CO2_GRAMS_PER_LITER * percHot * milliLitres / 1000.0;
  
        partialCO2 += co2;
        totalCO2 += co2;
  
        lastAccelZ = aZ;
        //lastHeading = orientation.heading();
  
        updateDisplay();
      }

      if (digitalRead(B10) == 0)
      {
        debounceCntr ++;
      }
      else 
      {
        if (debounceCntr >= 30)
        {
          Serial.println(String("Switching to APP_CALIBRATION1"));
          appStatus = APP_CALIBRATION1;
          debounceCntr = 0;

          showCalibrationStatus1();
        }
        else
        {
          debounceCntr = 0;
        }
      }
    }
  
    if (appStatus == APP_CALIBRATION1)
    {
      accelZFull = aZ;
      
      if (digitalRead(B10) == 0)
      {
        debounceCntr ++;
      }
      else
      {
        if (debounceCntr >= 10)
        {
          appStatus = APP_CALIBRATION2;
          debounceCntr = 0;

          showCalibrationStatus2();
        }
        else
        {
          debounceCntr = 0;
        }
      }
    }
  
    if (appStatus == APP_CALIBRATION2)
    {
      accelZClose = aZ;

      if (digitalRead(B10) == 0)
      {
        debounceCntr ++;
      }
      else
      {
        if (debounceCntr >= 10)
        {
          appStatus = APP_NORMAL;
          debounceCntr = 0;
  
          updateDisplay();
        }
        else
        {
          debounceCntr = 0;
        }
      }
    }

    prevMs = millis();
  }  
}
