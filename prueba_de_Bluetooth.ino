#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
BLEService batteryService("19B10012-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic batteryLevelChar("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 100);
//BLEStringCharacteristic batteryLevelChar("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 400);
boolean viewInSerialPlotter = true;    //true optimises for serial plotter, false for serial monitor
boolean global1 = true;
void filtropasaB(float*, float*, float*);
void eliminarG(float*, float*, float*);
void rectificarSen(float*, float*, float*);

float pasaB[] = {0.0, 0.0, 0.0};
float elimGrab[] = {0.0, 0.0, 0.0};
float alpha[] = {0.2, 0.9};  // factor entre 0 y 1, Para suavizar la señal resultante
float SB[] = {pasaB[0], pasaB[1], pasaB[2]};
float GraB[] = {elimGrab[0], elimGrab[1], elimGrab[2]};

void setup() {
  Serial.begin(9600);
  initBLE();
  aceleroInit();
}

void initBLE()
{
  while (Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin() && !IMU.begin())
  {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Ejemplo1");
  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop()
{
  BLEDevice central = BLE.central();
  float x, y, z;
  String cadena = "";
  if (central)
  {
    if (IMU.accelAvailable())
    {
      //Serial.print("Connected to central: ");
      //Serial.println(central.address());
      digitalWrite(LED_BUILTIN, HIGH);
      while (central.connected()) {
        IMU.readAccel(x, y, z);
        filtropasaB(&x, &y, &z);
        if (global1) {
          //eliminarG(&x, &y, &z);
        }
        if (global1) {
          rectificarSen(&x, &y, &z);
        }
        cadena = String(x, 3) + ',' + String(y, 3) + ',' + String(z, 3);
        Serial.print("x:");
        Serial.print(x);
        Serial.print('\t');
        Serial.print("y:");
        Serial.print(y);
        Serial.print('\t');
        Serial.print("z:");
        Serial.println(z);
        batteryLevelChar.writeValue(cadena);
        cadena = "";
        delay(50);
      }
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  //Serial.print("Disconnected from central: ");
  //Serial.println(central.address());
}

void filtropasaB(float* x, float* y, float* z) {
  pasaB[0] = *x;
  pasaB[1]  = *y;
  pasaB[2]  = *z;
  SB[0] = (alpha[0] * pasaB[0]) + ((1 - alpha[0]) *  SB[0]);
  SB[1] = (alpha[0] * pasaB[1]) + ((1 - alpha[0]) *  SB[1]);
  SB[2] = (alpha[0] * pasaB[2]) + ((1 - alpha[0]) *  SB[2]);

  *x = SB[0];
  *y = SB[1];
  *z = SB[2];
}

void eliminarG(float* x, float* y, float* z) {
  elimGrab[0] = *x;
  elimGrab[1]  = *y;
  elimGrab[2]  = *z;
  GraB[0] = (alpha[1] * elimGrab[0]) + ((1 - alpha[1]) *  GraB[0]);
  GraB[1] = (alpha[1] * elimGrab[1]) + ((1 - alpha[1]) *  GraB[1]);
  GraB[2] = (alpha[1] * elimGrab[2]) + ((1 - alpha[1]) *  GraB[2]);
  *x = elimGrab[0] - GraB[0];
  *y = elimGrab[1] - GraB[1];
  *z = elimGrab[2] - GraB[2];
}

void rectificarSen(float* x, float* y, float* z) {
  (*x >= 0 ) ? (*x = *x) : (*x = *x * -1);
  (*y >= 0 ) ? (*y = *y) : (*y = *y * -1);
  (*z >= 0 ) ? (*z = *z) : (*z = *z * -1);
}
void aceleroInit()
{
  if (!IMU.begin())
  {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  /*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
  ********************         Copy/Replace the lines below by the code output of the program              ****************/
  IMU.setAccelFS(3);
  IMU.setAccelODR(2);
  IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
  IMU.setAccelSlope (1, 1, 1);  //   uncalibrated
  /***********************************************************************************************************************************
  *******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
  ************************************************************************************************************************************/
  IMU.accelUnit =  METERPERSECOND2;   // or  METERPERSECOND2

  if (viewInSerialPlotter)
  { Serial.println("Gyroscope in degrees/second \n");
    Serial.print("Accelerometer Full Scale = ±");
    Serial.print(IMU.getAccelFS());
    Serial.println ("g");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.getAccelODR());        // alias  AccelerationSampleRate());
    Serial.println(" Hz \n");
    delay(4500);
  }
  Serial.println(" X \t Y \t Z ");
}
