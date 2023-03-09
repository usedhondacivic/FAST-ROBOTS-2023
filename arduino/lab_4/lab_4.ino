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


class PID_CONTROLLER{
  private:
    double p;
    double i;
    double d;

    double integrator = 0;
    double z_prev = NULL;
    unsigned long last_time = NULL;

  public:
    double output = 0;
    
    PID_CONTROLLER(){
      p = 0;
      i = 0;
      d = 0;
    }

    PID_CONTROLLER(double _p, double _i, double _d){
      p = _p;
      i = _i;
      d = _d;
    }

    void step(double u, double z){
      unsigned long now = millis();
      if(z_prev == NULL){
        z_prev = z;
      }
      if(last_time == NULL){
        last_time = millis();
      }
      int dt = now - last_time;
      double err = u - z;

      integrator += err;      

      double dz = z - z_prev;
      double der = -dz/dt;

      last_time = millis();
      z_prev = z;

      output = p * err + i * integrator + d * der;
    }

    void set_gains(double _p, double _i, double _d){
      p = _p;
      i = _i;
      d = _d;
    }
};

typedef struct {
  float x;
  float y;
  float z;
  long int stamp;
} THREE_AXIS;

typedef struct {
  int distA;
  int distB;
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
      TOF_DATA tof;
    } sensor_readings;

    struct {
      THREE_AXIS rot;
    } pose;

    long int start_time;

    bool robot_enabled;

    PID_CONTROLLER angle_controller;

  public:
    enum BUFFER_TYPE {ACCEL, GYRO, MAG, TOF, POSE, NA};
    static const int num_buffers = 5;
    struct {
      LinkedList<THREE_AXIS> accel;
      LinkedList<THREE_AXIS> gyro;
      LinkedList<THREE_AXIS> mag;
      LinkedList<TOF_DATA> tof;
      LinkedList<THREE_AXIS> pose_rot;

      bool enabled[num_buffers];
    } data_buffers;

    enum PID_CONTROLLER_TYPE{
      ROTATION
    };
    
    static const int num_pid_controllers = 1;

    struct {
      PID_CONTROLLER pid[num_pid_controllers];

      bool enabled[num_pid_controllers];
    } pid_controllers;

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

      robot_enabled = false;

      Serial.println("Robot successfully booted!");
    }

    void update(){
      if(!robot_enabled){
        return;
      }
      update_sensor_readings();
      update_pid_controllers();
      
      set_wheel_output(1.0, 1.0);
    }

    // 1.0 = full forward, -1.0 = full backwards
    void set_wheel_output(double left, double right){
      int deadband = 40;
      int remaining_band = 255-40;
      int left_sign = left / abs(left);
      int right_sign = right / abs(right);

      int output_left = left_sign * 40 + left * remaining_band;
      int output_right = right_sign * 40 + right * remaining_band;

      analogWrite(15, min(output_left, 0));
      analogWrite(12, max(output_left, 0));

      analogWrite(13, min(output_right, 0));
      analogWrite(14, max(output_right, 0));
    }

    void update_pid_controllers(){
      pid_controllers.pid[ROTATION].step(0, sensor_readings.gyro.z);
    }
    
    void update_sensor_readings(){
      //Update TOF
      if(distanceSensorA.checkForDataReady() && distanceSensorB.checkForDataReady()){
        sensor_readings.tof.distA = distanceSensorA.getDistance();
        distanceSensorA.clearInterrupt();

        sensor_readings.tof.distB = distanceSensorB.getDistance();
        distanceSensorB.clearInterrupt();

        sensor_readings.tof.stamp = millis();

        if(data_buffers.enabled[TOF]){
          data_buffers.tof.Append(sensor_readings.tof);
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
        pose.rot.z = sensor_readings.gyro.z;

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

    void set_enabled(bool enable){
      if(enable && !robot_enabled){
        start_time = millis();
      }
      robot_enabled = enable;
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
        ENABLE_ROBOT,
        DISABLE_ROBOT,
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
          the_car->set_enabled(false);
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

        case ENABLE_ROBOT:
          the_car->set_enabled(true);        
          break;

        case DISABLE_ROBOT:
          the_car->set_enabled(false);
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
            tx_estring_value.append("<START BUFFER ");
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
              send_data_buffer(&(the_car->data_buffers.tof));
              the_car->data_buffers.tof.Clear();
            }
            if(buf == CAR::POSE){
              send_data_buffer(&(the_car->data_buffers.pose_rot));
              the_car->data_buffers.pose_rot.Clear();
            }
            
            tx_estring_value.clear();
            tx_estring_value.append("<END BUFFER ");
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
      tx_estring_value.append("DistA: ");
      tx_estring_value.append(send->distA);


      tx_estring_value.append(" | ");
      tx_estring_value.append("DistB: ");
      tx_estring_value.append(send->distB);
      
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