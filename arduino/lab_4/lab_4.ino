/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X


#define SERIAL_PORT Serial

typedef struct {
  float x;
  float y;
  float z;
} THREE_AXIS;

class CAR{
  private:
    // SENSORS
    ICM_20948_I2C myICM;

    SFEVL53L1X distanceSensorA;
    SFEVL53L1X distanceSensorB;

    // DATA STRUCTURES
    struct {
      THREE_AXIS accel;
      THREE_AXIS gyro_delta;
      THREE_AXIS gyro;
      THREE_AXIS mag;
      int tof_a;
      int tof_b;
      long int imu_stamp;
      long int tof_a_stamp;
      long int tof_b_stamp;
    } sensor_readings;

    struct {
      THREE_AXIS rot;
      THREE_AXIS pos;
    } pose;

  public:
    void setup(){
      Serial.begin(115200);
      while (!SERIAL_PORT)
      {
      };
      Serial.println("Robot booting...");

      Wire.begin();
      Wire.setClock(400000);
      
      Serial.println("Initializing IMU...");
      bool initialized = false;
      while (!initialized)
      {
        myICM.begin(Wire, 1);

        Serial.print(F("Initialization of IMU returned: "));
        Serial.println(myICM.statusString());
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

      sensor_readings.gyro.x = 0;
      sensor_readings.gyro.y = 0;
      sensor_readings.gyro.z = 0;

      pose.rot.x = 0;
      pose.rot.y = 0;

      Serial.println("Initallizing ToF Sensors...");
      distanceSensorA = SFEVL53L1X(Wire, 7, -1);
      distanceSensorB = SFEVL53L1X(Wire, 8, -1);
      distanceSensorA.sensorOff();
      delay(500);
      distanceSensorA.sensorOn();
      distanceSensorB.sensorOff();
      initialized = false;
      while (!initialized){
        if(distanceSensorA.begin() != 0) //Begin returns 0 on a good init
        {
          Serial.println("Sensor A failed to begin. Trying again...");
          delay(500);
        }else{
          initialized = true;          
        }
      }
      Serial.println("Sensor A Online! Changing I2C address.");
      distanceSensorA.setI2CAddress(0x38);
      Serial.println("Enabling sensor B...");
      distanceSensorB.sensorOn(); // Enable B
      initialized = false;
      while (!initialized){
        if(distanceSensorA.begin() != 0) //Begin returns 0 on a good init
        {
          Serial.println("Sensor A failed to begin. Trying again...");
          delay(500);
        }else{
          initialized = true;          
        }
      }
      Serial.println("Sensor A and B Online!");
      distanceSensorA.setDistanceModeLong();
      distanceSensorB.setDistanceModeLong();

      Serial.println("Robot successfully booted!");      
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(100);                 
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(100);                      
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

    }
    void update(){
      update_sensor_readings();
      update_pose();
      Serial.print(sensor_readings.gyro.x);
      Serial.print(" ");
      Serial.print(sensor_readings.gyro.y);
      Serial.print(" ");
      Serial.print(sensor_readings.gyro.z);
      Serial.print(" ");
      Serial.print(pose.rot.x);
      Serial.print(" ");
      Serial.print(pose.rot.y);
      Serial.println();
    }  
    void update_sensor_readings(){
      // Update IMU
      if (myICM.dataReady())
      {
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        sensor_readings.accel.x = myICM.accX();
        sensor_readings.accel.y = myICM.accY();
        sensor_readings.accel.z = myICM.accZ();

        sensor_readings.mag.x = myICM.magX();
        sensor_readings.mag.y = myICM.magY();
        sensor_readings.mag.z = myICM.magZ();

        sensor_readings.gyro_delta.x = myICM.gyrX();
        sensor_readings.gyro_delta.y = myICM.gyrY();
        sensor_readings.gyro_delta.z = myICM.gyrZ();

        float dt = (float)(millis() - sensor_readings.imu_stamp) / 1000.0;
        //Serial.println(dt);
        sensor_readings.gyro.x += myICM.gyrX() * dt;
        sensor_readings.gyro.y -= myICM.gyrY() * dt;
        sensor_readings.gyro.z += myICM.gyrZ() * dt;

        //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
        //plotGyro(&myICM);
        //plotPitchRollYaw(&myICM);
        //plotAccel(&myICM);

        sensor_readings.imu_stamp = millis();
      }

      distanceSensorA.startRanging(); 
      if(distanceSensorA.checkForDataReady()){
        sensor_readings.tof_a = distanceSensorA.getDistance();
        distanceSensorA.clearInterrupt();

        sensor_readings.tof_a_stamp = millis();
      }

      distanceSensorB.startRanging();
      if(distanceSensorB.checkForDataReady()){
        sensor_readings.tof_b = distanceSensorB.getDistance();
        distanceSensorB.clearInterrupt();

        sensor_readings.tof_b_stamp = millis();
      }      
    }
    void update_pose(){
      // https://www.nxp.com/docs/en/application-note/AN3461.pdf
      float roll= atan2(sensor_readings.accel.y, sensor_readings.accel.z) * 180 / 3.14;
      float pitch = atan2(sensor_readings.accel.x, sensor_readings.accel.z) * 180 / 3.14;

      // https://seanboe.me/blog/complementary-filters
      float gyro_favor = 0.96;
      float dt = millis() - sensor_readings.imu_stamp;
      pose.rot.x = (gyro_favor) * (pose.rot.x + (sensor_readings.gyro.x * (1.00 / dt))) + (1.00 - gyro_favor) * (roll);
      pose.rot.y = (gyro_favor) * (pose.rot.y + (sensor_readings.gyro.y * (1.00 / dt))) + (1.00 - gyro_favor) * (pitch);
    }
};

CAR my_car;

void setup()
{
  my_car.setup();
}

void loop()
{
  my_car.update();
}

// Below here are some helper functions to print the data nicely!

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

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
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

void plotAccel(ICM_20948_I2C *sensor)
{
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(" ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(" ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.println();
}

void plotGyro(ICM_20948_I2C *sensor)
{
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(" ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(" ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.println();
}

void plotPitchRollYaw(ICM_20948_I2C *sensor)
{
  float pitch = atan2(sensor->accY(), sensor->accX()) * 180 / 3.14;
  float roll = atan2(sensor->accX(), sensor->accZ()) * 180 / 3.14;
  printFormattedFloat(pitch, 5, 2);
  SERIAL_PORT.print(" ");
  printFormattedFloat(roll, 5, 2);
  SERIAL_PORT.println();
}

void integrate_gyro(ICM_20948_I2C *sensor)
{
  
}
