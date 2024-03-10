/****************************************************************
 * Current Care for 2024 Health Hackathon
 * Original Creation Date: March 9, 2024
 * Written by Johnson Liu and Leah Lackey with some assistance
 * from programming guides from Sparkfun and Adafruit 
 ***************************************************************/
#include "ICM_20948.h" 
#include <LiquidCrystal.h>

#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.      
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// For force sensor
#define FORCE_SENSOR_PIN A0

// For ultrasonic sensor
int trigPin = 5;
int echoPin = 6;

// Initialize the LiquidCrystal library 
const int rs = 12, en = 11, d4 = 9, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void setup()
{
  lcd.begin(16, 2);
  SERIAL_PORT.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {
    //myICM.enableDebugging(Serial);
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

void loop()
{
  bool alert_triggered = false;

  // Force sensor reading
  int forceReading = analogRead(FORCE_SENSOR_PIN);
  Serial.print("Force sensor reading = ");
  Serial.print(forceReading);
  
  // Pressure length check
  if(forceReading >= 800) { // from 800 to 1023 is considered a big squeeze, which could be "long pressure"
    Serial.println(" -> big squeeze, ALERT");
    Serial.println("EMS treatment starting now");
    alert_triggered = true;
  } else {
    if (forceReading < 10)       // from 0 to 9
      Serial.println(" -> no pressure");
    else if (forceReading < 200) // from 10 to 199
      Serial.println(" -> light touch");
    else if (forceReading < 500) // from 200 to 499
      Serial.println(" -> light squeeze");
    else if (forceReading < 800) // from 500 to 799
      Serial.println(" -> medium squeeze");
  }
  
  // Ultrasonic Sensor Reading for Blood Flow Speed
  float distance1 = ultrasonicRead();
  delay(1000); // 1 second delay between measurements
  float distance2 = ultrasonicRead();
  float measured_speed = 10.0f*abs(distance2 - distance1); // Since time interval is 1 second, speed is the absolute distance change

  //Serial.print("Blood flow Speed in cm/s: ");
  // Added this statement as an alternative due to some error
  Serial.print("Blood flow value detected: ");
  Serial.println(measured_speed);

  // Blood flow speed check
  if(measured_speed < 50) {
    alert_triggered = true;
    Serial.println("Blood flow speed is less than 50 cm/s, ALERT");
    Serial.println("EMS treatment starting now");
  }

  // Here, implement data saving logic
  // For demonstration, we'll just print a placeholder message
  Serial.println("Data saved.");

  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(1000);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

  // Update LCD
  lcd.clear();
  if (alert_triggered) {
    lcd.write("ALERT! EMS Start");
  }
}

// Below here are some helper functions to print the data nicely!
float ultrasonicRead() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  const long duration = pulseIn(echoPin, HIGH);
  const float distance = duration * 0.034 / 2;
  return distance;
}

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}