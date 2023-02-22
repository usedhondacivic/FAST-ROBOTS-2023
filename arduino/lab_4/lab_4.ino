#include "BLECStringCharacteristic.h" // BLE Headers
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#include <Wire.h>
#include "SparkFun_VL53L1X.h" // Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include "LinkedList.h" // Linked List implementation: https://stackoverflow.com/questions/9986591/vectors-in-arduino


#define BLE_UUID_TEST_SERVICE "f74736e0-f5ac-4541-959d-e6c1f1b3f55c"
#define BLE_UUID_RX_STRING "58482b00-4146-4122-be67-2d89016731a8"

#define BLE_UUID_TX_FLOAT "51eed2ce-3329-4232-b8d5-8f022aaa2d1a"
#define BLE_UUID_TX_STRING "aa71399e-0f1d-411d-ac23-7ace2936fd5e"

#define SERIAL_PORT Serial

typedef struct {
  float x;
  float y;
  float z;
  long int stamp;
} THREE_AXIS;

typedef struct {
  int dist;
  long int stamp;  
} TOF_DATA;

class CAR{
  private:
    // SENSORS
    ICM_20948_I2C myICM;

    SFEVL53L1X distanceSensorA{Wire, 7, -1};
    SFEVL53L1X distanceSensorB{Wire, 8, -1};

    // DATA STRUCTURES
    struct {
      THREE_AXIS accel;
      THREE_AXIS gyro_delta;
      THREE_AXIS gyro;
      THREE_AXIS mag;
      TOF_DATA tof_a;
      TOF_DATA tof_b;
    } sensor_readings;

    struct {
      THREE_AXIS rot;
    } pose;

    long int start_time;

  public:
    enum  BUFFER_TYPE {ACCEL, GYRO, MAG, TOF, POSE, NA};
    struct {
      LinkedList<THREE_AXIS> accel;
      LinkedList<THREE_AXIS> gyro;
      LinkedList<THREE_AXIS> mag;
      LinkedList<TOF_DATA> tofA;
      LinkedList<TOF_DATA> tofB;
      LinkedList<THREE_AXIS> pose_rot;


      bool enabled[5];
    } data_buffers;

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
        if(distanceSensorB.begin() != 0) //Begin returns 0 on a good init
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
      distanceSensorA.startRanging();
      distanceSensorB.startRanging();
   
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(100);                 
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(100);                      
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

      data_buffers.enabled[ACCEL] = false;
      data_buffers.enabled[GYRO] = false;
      data_buffers.enabled[MAG] = false;
      data_buffers.enabled[TOF] = false;
      data_buffers.enabled[POSE] = false;

      start_time = millis();

      Serial.println("Robot successfully booted!");   
    }

    void update(){
      update_sensor_readings();
      // if(millis() - start_time > 1000 && millis() - start_time < 2000){
      //   data_buffers.enabled[data_buffers.ACCEL] = true;
      // }else{
      //   if(data_buffers.enabled[data_buffers.ACCEL]){
      //     Serial.print("Data points collected in one second: ");
      //     Serial.println(data_buffers.accel.getLength());          
      //   }
      //   data_buffers.enabled[data_buffers.ACCEL] = false;
      // }
    }  
    
    void update_sensor_readings(){
      //Update TOF
      if(distanceSensorA.checkForDataReady()){
        sensor_readings.tof_a.dist = distanceSensorA.getDistance();
        distanceSensorA.clearInterrupt();

        sensor_readings.tof_a.stamp = millis();

        if(data_buffers.enabled[TOF]){
          data_buffers.tofA.Append(sensor_readings.tof_a);
        }
      }

      if(distanceSensorB.checkForDataReady()){
        sensor_readings.tof_b.dist = distanceSensorB.getDistance();
        distanceSensorB.clearInterrupt();

        sensor_readings.tof_b.stamp = millis();

        if(data_buffers.enabled[TOF]){
          data_buffers.tofB.Append(sensor_readings.tof_b);
        }
      }    

      // Update IMU
      if (myICM.dataReady())
      {
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        sensor_readings.accel.x = myICM.accX();
        sensor_readings.accel.y = myICM.accY();
        sensor_readings.accel.z = myICM.accZ();

        sensor_readings.accel.stamp = millis();

        if(data_buffers.enabled[ACCEL]){
          data_buffers.accel.Append(sensor_readings.accel);
        }

        sensor_readings.mag.x = myICM.magX();
        sensor_readings.mag.y = myICM.magY();
        sensor_readings.mag.z = myICM.magZ();

        sensor_readings.mag.stamp = millis();

        if(data_buffers.enabled[MAG]){
          data_buffers.mag.Append(sensor_readings.mag);
        }

        sensor_readings.gyro_delta.x = myICM.gyrX();
        sensor_readings.gyro_delta.y = myICM.gyrY();
        sensor_readings.gyro_delta.z = myICM.gyrZ();

        sensor_readings.gyro_delta.stamp = millis();

        float dt = (float)(millis() - sensor_readings.gyro.stamp) / 1000.0;
        sensor_readings.gyro.x += myICM.gyrX() * dt;
        sensor_readings.gyro.y -= myICM.gyrY() * dt;
        sensor_readings.gyro.z += myICM.gyrZ() * dt;

        sensor_readings.gyro.stamp = millis();

        if(data_buffers.enabled[GYRO]){
          data_buffers.gyro.Append(sensor_readings.gyro);
        }

        float roll= atan2(sensor_readings.accel.y, sensor_readings.accel.z) * (180.0 / 3.14);
        float pitch = atan2(sensor_readings.accel.x, sensor_readings.accel.z) * (180.0 / 3.14);

        // https://seanboe.me/blog/complementary-filters
        float gyro_favor = 0.98;
        pose.rot.x = (gyro_favor) * (pose.rot.x + myICM.gyrX() * dt) + (1.00 - gyro_favor) * (roll);
        pose.rot.y = (gyro_favor) * (pose.rot.y - myICM.gyrY() * dt) + (1.00 - gyro_favor) * (pitch);

        pose.rot.stamp = millis();

        if(data_buffers.enabled[POSE]){
          data_buffers.pose_rot.Append(pose.rot);
        }
      }  
    }

    BUFFER_TYPE string_to_buf_type(char *str){
      if(strcmp(str, "GYRO") == 0){
        return GYRO;
      }
      if(strcmp(str, "ACCEL") == 0){
        return ACCEL;
      }
      if(strcmp(str, "MAG") == 0){
        return MAG;
      }
      if(strcmp(str, "TOF") == 0){
        return TOF;
      }
      if(strcmp(str, "POSE") == 0){
        return POSE;
      }
      return NA;
    }
};

class BLE_HANDLER{
  private:
    BLEService testService{BLE_UUID_TEST_SERVICE};

    BLECStringCharacteristic rx_characteristic_string{BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE};

    BLEFloatCharacteristic tx_characteristic_float{BLE_UUID_TX_FLOAT, BLERead | BLENotify};
    BLECStringCharacteristic tx_characteristic_string{BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE};

    // RX
    RobotCommand robot_cmd{":|"};

    // TX
    EString tx_estring_value;
    float tx_float_value{0.0};

    enum CommandTypes
    {
        PING,
        ECHO,
        GET_TIME_MILLIS,
        ENABLE_BUFFER,
        RETRIEVE_BUFFER,
        DISABLE_BUFFER
    };

    int interval = 500;
    long int last_write;

    CAR  *the_car;

    bool connected = false; 
    
  public:  
    void setup(CAR *car){
      the_car = car;

      Wire.begin();

      Serial.begin(115200);

      Serial.println("BLE Booting...");

      BLE.begin();
      
      // Set advertised local name and service
      BLE.setDeviceName("Artemis BLE");
      BLE.setLocalName("Artemis BLE");
      BLE.setAdvertisedService(testService);

      // Add BLE characteristics
      testService.addCharacteristic(tx_characteristic_float);
      testService.addCharacteristic(tx_characteristic_string);
      testService.addCharacteristic(rx_characteristic_string);

      // Add BLE service
      BLE.addService(testService);

      // Initial values for characteristics
      // Set initial values to prevent errors when reading for the first time on central devices
      tx_characteristic_float.writeValue(0.0);
      tx_estring_value.clear();
      tx_estring_value.append("init");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      // Output MAC Address
      Serial.print("Advertising BLE with MAC: ");
      Serial.println(BLE.address());

      BLE.advertise();

      Serial.println("BLE successfully booted!");
    }

    void update(){
      BLEDevice central = BLE.central();

      if (central) {
        if(!connected){
          Serial.print("Connected to: ");
          Serial.println(central.address());
        }

        // While central is connected
        if (central.connected()) {
          connected = true;
          read_data();
        }else if(connected){
          connected = false;
          Serial.println("Disconnected");          
        }
      }
    }

    void handle_command(){
      robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                              rx_characteristic_string.valueLength());

      bool success;
      int cmd_type = -1;

      success = robot_cmd.get_command_type(cmd_type);
      
      if (!success) {
          return;
      }


      char char_arr[MAX_MSG_SIZE];

      switch (cmd_type) {
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            break;

        case ECHO:
          // Extract the next value from the command string as a character array
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
              return;

          tx_estring_value.clear();
          tx_estring_value.append("Robot says -> ");
          tx_estring_value.append(char_arr);
          tx_estring_value.append(" :)");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());             
          
          break;
        
        case GET_TIME_MILLIS:
          tx_estring_value.clear();
          tx_estring_value.append("Time: ");
          tx_estring_value.append((float)millis());
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          break;

        case ENABLE_BUFFER:{
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
              return;
          CAR::BUFFER_TYPE buf = the_car->string_to_buf_type(char_arr);
          if(buf != CAR::NA){
            Serial.print("Enabling buffer: ");
            Serial.println(char_arr);
          }else{
            Serial.print("BLE attempted to enable unknown buffer: ");
            Serial.println(char_arr);
          }
          the_car->data_buffers.enabled[buf] = true;

        } break;

        case DISABLE_BUFFER:{
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
              return;
          CAR::BUFFER_TYPE buf = the_car->string_to_buf_type(char_arr);
          if(buf != CAR::NA){
            Serial.print("Disabling buffer: ");
            Serial.println(char_arr);
          }else{
            Serial.print("BLE attempted to disable unknown buffer: ");
            Serial.println(char_arr);
          }
          the_car->data_buffers.enabled[buf] = false;

        } break;

        case RETRIEVE_BUFFER:{
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
              return;
          CAR::BUFFER_TYPE buf = the_car->string_to_buf_type(char_arr);
          if(buf != CAR::NA){
            the_car->data_buffers.enabled[buf] = false;

            Serial.print("Sending buffer: ");
            Serial.println(char_arr);
            tx_estring_value.clear();
            tx_estring_value.append("<START BUFFER: ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(">");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());   

            if(buf == CAR::ACCEL){
              send_data_buffer(&(the_car->data_buffers.accel));
              the_car->data_buffers.accel.Clear();
            }
            if(buf == CAR::GYRO){
              send_data_buffer(&(the_car->data_buffers.gyro));
              the_car->data_buffers.gyro.Clear();
            }
            if(buf == CAR::MAG){
              send_data_buffer(&(the_car->data_buffers.mag));
              the_car->data_buffers.gyro.Clear();
            }
            if(buf == CAR::TOF){
              tx_estring_value.clear();
              tx_estring_value.append("[START TOF_A]");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());   
              send_data_buffer(&(the_car->data_buffers.tofA));
              the_car->data_buffers.tofA.Clear();
              tx_estring_value.clear();
              tx_estring_value.append("[END TOF_A]");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());

              tx_estring_value.clear();
              tx_estring_value.append("[START TOF_B]");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());   
              send_data_buffer(&(the_car->data_buffers.tofB));
              the_car->data_buffers.tofB.Clear();
              tx_estring_value.clear();
              tx_estring_value.append("[END TOF_B]");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());   
            }
            if(buf == CAR::POSE){
              send_data_buffer(&(the_car->data_buffers.pose_rot));
              the_car->data_buffers.pose_rot.Clear();
            }
            
            tx_estring_value.clear();
            tx_estring_value.append("<END BUFFER: ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(">");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());              
          }else{
            Serial.print("BLE requested unknown buffer: ");
            Serial.println(char_arr);
            tx_estring_value.clear();
            tx_estring_value.append("Unknown buffer: ");
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());  
            break;
          } 
        } break;

        default:
          Serial.print("Invalid Command Type: ");
          Serial.println(cmd_type);
          break;
      }
    }

    void send_data_buffer(LinkedList<THREE_AXIS> *buf){
      if(!buf->moveToStart()){
        Serial.println("No data to report. Make sure to enable buffer before requesting data.");
        return;
      }
      do{
        send_data_point(&(buf->getCurrent()));
      }while(buf->next());
    }

    void send_data_buffer(LinkedList<TOF_DATA> *buf){
      if(!buf->moveToStart()){
        Serial.println("No data to report. Make sure to enable buffer before requesting data.");
        return;
      }
      do{
        send_data_point(&(buf->getCurrent()));
      }while(buf->next());
    }

    void send_data_point(THREE_AXIS *send){
      tx_estring_value.clear();

      tx_estring_value.append("Time: ");
      tx_estring_value.append((int)send->stamp);
      tx_estring_value.append(" | ");

      tx_estring_value.append("X: ");
      tx_estring_value.append(send->x);
      tx_estring_value.append(" | ");

      tx_estring_value.append("Y: ");
      tx_estring_value.append(send->y);
      tx_estring_value.append(" | ");


      tx_estring_value.append("Z: ");
      tx_estring_value.append(send->z);
      
      tx_characteristic_string.writeValue(tx_estring_value.c_str());   
    }

    void send_data_point(TOF_DATA *send){
      tx_estring_value.clear();

      tx_estring_value.append("Time: ");
      tx_estring_value.append((int)send->stamp);
      tx_estring_value.append(" | ");

      tx_estring_value.append("Dist: ");
      tx_estring_value.append(send->dist);
      
      tx_characteristic_string.writeValue(tx_estring_value.c_str());  
    }

    void read_data()
    {
        // Query if the characteristic value has been written by another BLE device
        if (rx_characteristic_string.written()) {
            handle_command();
        }
    }
};

CAR my_car;
BLE_HANDLER my_ble_handler;

void setup()
{
  my_ble_handler.setup(&my_car);
  my_car.setup();
}

void loop()
{
 my_ble_handler.update();
 my_car.update();
}