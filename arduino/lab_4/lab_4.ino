#include "BLECStringCharacteristic.h"  // BLE Headers
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#include <Wire.h>
#include "SparkFun_VL53L1X.h"  // Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include "LinkedList.h"  // Linked List implementation: https://stackoverflow.com/questions/9986591/vectors-in-arduino

#include "BasicLinearAlgebra.h"   //Use this library to work with matrices:
using namespace BLA;               //This allows you to declare a matrix


#define BLE_UUID_TEST_SERVICE "f74736e0-f5ac-4541-959d-e6c1f1b3f55c"
#define BLE_UUID_RX_STRING "58482b00-4146-4122-be67-2d89016731a8"

#define BLE_UUID_TX_FLOAT "51eed2ce-3329-4232-b8d5-8f022aaa2d1a"
#define BLE_UUID_TX_STRING "aa71399e-0f1d-411d-ac23-7ace2936fd5e"

#define SERIAL_PORT Serial

//http://blog.lexique-du-net.com/index.php?post/Calculate-the-real-difference-between-two-angles-keeping-the-sign

double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
{
  double difference = secondAngle - firstAngle;
  while (difference < -180) difference += 360;
  while (difference > 180) difference -= 360;
  return difference;
 }

class KALMAN_FILTER{
private:
  Matrix<2,2> A;
  Matrix<2,1> B;
  Matrix<1,2> C;

  Matrix<2,2> sig_u;
  Matrix<1, 1> sig_z;

  unsigned long last_time = NULL;

public:
  Matrix<2,1> mu;
  Matrix<2,2> sig;

  KALMAN_FILTER(){}

  KALMAN_FILTER(Matrix<2,2> A_, Matrix<2,1> B_, Matrix<1,2> C_, Matrix<2,2> _sig_u, Matrix<1,1> _sig_z){
    A = A_;
    B = B_;
    C = C_;

    sig_u = _sig_u;
    sig_z = _sig_z;
  }

  void update(Matrix<1,1> u, Matrix<1,1> y, bool update) {
    if(last_time == NULL){
      last_time = micros();
    }
    float dt = (micros() - last_time) / 1E6;

    Matrix<2,2> eye = {1, 0, 0, 1};
    Matrix<2,2> A_d = eye + A * dt;
    Matrix<2,1> B_d = B * dt;
    
    // Thanks: https://anyafp.github.io/ece4960/labs/lab7/
    Matrix<2,1> x_p = A_d*mu + B_d*u;
    Matrix<2,2> sig_p = A_d*sig*(~A_d) + sig_u;

    if(!update){
      mu = x_p;
      sig = sig_p;
      last_time = micros();
      return;
    }

    Matrix<1,1> y_curr = y;
    Matrix<1,1> y_m = y_curr - C*x_p;
    Matrix<1,1> sig_m = C*sig_p*(~C) + sig_z;

    Matrix<1,1> sig_m_inv = sig_m;
    Invert(sig_m_inv);

    Matrix<2,1> kf_gain = sig_p*(~C)*(sig_m_inv);

    // Update
    mu = x_p + kf_gain*y_m;
    sig = (eye - kf_gain*C)*sig_p;

    last_time = micros();
  }

  void set_init(Matrix<2,1> mu_init, Matrix<2,2> sig_init){
    mu = mu_init;
    sig = sig_init;   
    last_time = NULL; 
  }
};

class PID_CONTROLLER {
private:
  double p;
  double i;
  double d;

  double integrator = 0;
  double integrator_cap = 1000;
  double z_prev = NULL;
  unsigned long last_time = NULL;

  int sample_rate_ms = 1;  //1khz

public:
  double output = 0;

  PID_CONTROLLER() {
    p = 0;
    i = 0;
    d = 0;
  }

  PID_CONTROLLER(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;

    if (_i != 0) {
      integrator_cap = 1 / i;
    }
  }

  void step(double u, double z) {
    unsigned long now = millis();
    if (z_prev == NULL) {
      z_prev = z;
    }
    if (last_time == NULL) {
      last_time = millis();
    }
    int dt = now - last_time;
    if (dt > sample_rate_ms) {
      double err = u - z;

      integrator += err;

      integrator = max(-integrator_cap, min(integrator_cap, integrator));

      double dz = z - z_prev;
      double der = -dz / dt;

      last_time = millis();
      z_prev = z;

      output = p * err + i * integrator + d * der;
    }
  }

  void set_gains(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;
  }

  void set_p_gain(double _p) {
    p = _p;
  }

  void set_i_gain(double _i) {
    i = _i;
  }

  void set_d_gain(double _d) {
    d = _d;
  }

  void reset(){
    z_prev = NULL;
    last_time = NULL;
    integrator = 0;
    output = 0;
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
  float kalmanEst;
  long int stamp;
} TOF_DATA;

class CAR {
private:
  // SENSORS
  ICM_20948_I2C myICM;

  SFEVL53L1X distanceSensorA{ Wire, 7, -1 };
  SFEVL53L1X distanceSensorB{ Wire, 8, -1 };

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

  KALMAN_FILTER distance_filter;

public:
  enum BUFFER_TYPE { ACCEL,
                     GYRO,
                     MAG,
                     TOF,
                     POSE,
                     MOTOR,
                     BUF_NA };
  static const int num_buffers = 6;
  struct {
    LinkedList<THREE_AXIS> accel;
    LinkedList<THREE_AXIS> gyro;
    LinkedList<THREE_AXIS> mag;
    LinkedList<TOF_DATA> tof;
    LinkedList<THREE_AXIS> pose_rot;
    LinkedList<THREE_AXIS> motor_input;

    bool enabled[num_buffers];
  } data_buffers;

  enum PID_CONTROLLER_TYPE { ROTATION,
                             ROTATION_RATE,                              
                             PID_NA };

  static const int num_pid_controllers = 2;

  struct {
    PID_CONTROLLER pid[num_pid_controllers];

    double setpoints[num_pid_controllers];
    bool enabled[num_pid_controllers];
  } pid_controllers;

  enum MODE_TYPE {SEEK_ANGLE, TAKE_READINGS, MOVE_FORWARD, BRAKE, BUG};

  MODE_TYPE current_mode = SEEK_ANGLE;
  bool mode_changed = true;
  double reading_start_rot = 0.0;
  double current_target = 0.0;
  long int mode_start = -1.0;

  void setup() {
    Serial.begin(115200);
    while (!SERIAL_PORT) {
    };
    Serial.println("Robot booting...");

    Wire.begin();
    Wire.setClock(400000);

    Serial.println("Initializing IMU...");
    bool initialized = false;
    while (!initialized) {
      myICM.begin(Wire, 1);

      ICM_20948_fss_t myFSS;
      myFSS.g = dps1000;  // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000

      myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

      Serial.print(F("Initialization of IMU returned: "));
      Serial.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok) {
        SERIAL_PORT.println("Trying again...");
        delay(500);
      } else {
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
    while (!initialized) {
      if (distanceSensorA.begin() != 0)  //Begin returns 0 on a good init
      {
        Serial.println("Sensor A failed to begin. Trying again...");
        delay(500);
      } else {
        initialized = true;
      }
    }
    Serial.println("Sensor A Online! Changing I2C address.");
    distanceSensorA.setI2CAddress(0x38);
    Serial.println("Enabling sensor B...");
    distanceSensorB.sensorOn();  // Enable B
    initialized = false;
    while (!initialized) {
      if (distanceSensorB.begin() != 0)  //Begin returns 0 on a good init
      {
        Serial.println("Sensor A failed to begin. Trying again...");
        delay(500);
      } else {
        initialized = true;
      }
    }
    Serial.println("Sensor A and B Online!");
    distanceSensorA.setDistanceModeLong();
    distanceSensorB.setDistanceModeLong();
    distanceSensorA.startRanging();
    distanceSensorB.startRanging();

    data_buffers.enabled[ACCEL] = false;
    data_buffers.enabled[GYRO] = false;
    data_buffers.enabled[MAG] = false;
    data_buffers.enabled[TOF] = false;
    data_buffers.enabled[POSE] = false;
    data_buffers.enabled[MOTOR] = false;

    pid_controllers.setpoints[ROTATION] = 0;
    pid_controllers.setpoints[ROTATION_RATE] = 20;

    Serial.println("Initalizing Kalman Filter...");

    int steady_state = 175000;
    int t = 225-25;

    float d = 1/(float)steady_state;
    float m = (-d*0.9*(float)t)/log(0.1);

    Matrix<2,2> A = {0, 1, 0, -d/m};
    Matrix<2,1> B = {0, 1/m};
    Matrix<1,2> C = {-1, 0};

    Matrix<2,2> sig_u = {pow(10, 2), 0, 0, pow(10, 2)};
    Matrix<1,1> sig_z = {pow(20, 2)};

    distance_filter = KALMAN_FILTER(A, B, C, sig_u, sig_z);

    Serial.println("Kalman Filter Online!");

    robot_enabled = false;

    Serial.println("Robot successfully booted!");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  }

  void update() {
    if (!robot_enabled) {
      set_wheel_output(0.0, 0.0);
      pid_controllers.setpoints[ROTATION] = 0;
      pid_controllers.pid[ROTATION].reset();
      pid_controllers.setpoints[ROTATION_RATE] = 20;
      pid_controllers.pid[ROTATION].reset();
      update_sensor_readings();
      sensor_readings.gyro.z = 0;
      // # inital state / uncertainty
      // x = np.array([[-tof_output[0, 1]],[0]])
      // sig = np.array([[5**2,0],[0,5**2]])

      int tof_avg = (sensor_readings.tof.distA + sensor_readings.tof.distB) / 2;
      Matrix<2,1> mu_init = {-(float)tof_avg, 0};
      Matrix<2,2> sig_init = {pow(5, 2), 0, 0, pow(5, 2)};
      distance_filter.set_init(mu_init, sig_init);

      mode_start = millis();
      return;
    }

    update_sensor_readings();
    update_pid_controllers();

    if(current_mode == TAKE_READINGS){
      if(mode_changed){
        reading_start_rot = sensor_readings.gyro.z;
        pid_controllers.pid[ROTATION].reset();
        pid_controllers.pid[ROTATION_RATE].reset();
        pid_controllers.setpoints[ROTATION] = reading_start_rot;
        pid_controllers.setpoints[ROTATION_RATE] = current_target;
        data_buffers.enabled[POSE] = true;
        data_buffers.enabled[TOF] = true;
        Serial.println("Start: TAKE_READINGS");
      }

      if(sensor_readings.gyro.z - reading_start_rot < 360.0){ // Do a 360 to take readings
        float out = pid_controllers.pid[ROTATION_RATE].output;
        if(out >= -0.1){
          out = -0.1;          
        }
        set_wheel_output(out, -out);
      }else{
        data_buffers.enabled[POSE] = false;
        data_buffers.enabled[TOF] = false;
        current_mode =  SEEK_ANGLE;
        current_target = reading_start_rot;
      }
    }else if(current_mode == SEEK_ANGLE){
      pid_controllers.setpoints[ROTATION] = current_target;
      float out = pid_controllers.pid[ROTATION].output;
      set_wheel_output(out, -out);
    }else if(current_mode == MOVE_FORWARD){
      if(mode_changed){
        pid_controllers.setpoints[ROTATION] = sensor_readings.gyro.z;
        mode_start = millis();
      }
      float out = pid_controllers.pid[ROTATION].output;
      if(millis() - mode_start < 100){
        set_wheel_output(0.5 + out, 0.5 - out);
      }
      else if(millis() - mode_start < current_target){
        set_wheel_output(0.2 + out, 0.2 - out);
      }else{
        set_wheel_output(0, 0, true);
      }
    }else if(current_mode == BRAKE){
      set_wheel_output(0, 0, true);
    }else if(current_mode == BUG){
      double left = ((float)sensor_readings.tof.distA) / 1000.0;
      double right = ((float)sensor_readings.tof.distB) / 1000.0;
      float out_left = 0.1 / (left * left);
      float out_right = 0.1 / (right * right);

      set_wheel_output(0.1 - out_left,  0.1 + out_right);
    }
    mode_changed = false;
  }
  
  void set_wheel_output(double left, double right){
    set_wheel_output(left, right, false);
  }

  // 1.0 = full forward, -1.0 = full backwards
  void set_wheel_output(double left, double right, bool brake) {
    right = right; // Make it go straight
    int deadband = 20;
    int remaining_band = 255 - deadband;
    int left_sign = left / abs(left);
    int right_sign = right / abs(right);

    int output_left = left_sign * deadband + left * remaining_band;
    int output_right = right_sign * deadband + right * remaining_band;

    if (left == 0.0) output_left = 0.0;
    if (right == 0.0) output_right = 0.0;

    if(brake){
      analogWrite(13, 1);
      analogWrite(14, 1);

      analogWrite(15, 1);
      analogWrite(12, 1);
    }else{
      analogWrite(13, max(output_left, 0));
      analogWrite(14, -min(output_left, 0));

      analogWrite(15, -min(output_right, 0));
      analogWrite(12, max(output_right, 0));
    }

    THREE_AXIS motor_speed;
    motor_speed.x = output_left;
    motor_speed.y = output_right;
    motor_speed.stamp = millis();

    float speed_avg = (output_left + output_right) / 2;
    
    distance_filter.update({speed_avg / 130}, {0}, false);

    long int last_write;
    if (data_buffers.motor_input.getLength() == 0) {
      last_write = 0;
    } else {
      last_write = data_buffers.motor_input.Last().stamp;
    }

    if (data_buffers.enabled[MOTOR] && millis() - last_write > 100) {
      data_buffers.motor_input.Append(motor_speed);
    }
  }

  void update_pid_controllers() {
    double a =  calculateDifferenceBetweenAngles(pid_controllers.setpoints[ROTATION], sensor_readings.gyro.z);
    pid_controllers.pid[ROTATION].step(a, 0.0);
    pid_controllers.pid[ROTATION_RATE].step(pid_controllers.setpoints[ROTATION_RATE], sensor_readings.gyro_delta.z);
  }

  void update_sensor_readings() {
    //Update TOF
    if (distanceSensorA.checkForDataReady() && distanceSensorB.checkForDataReady()) {
      sensor_readings.tof.distA = distanceSensorA.getDistance();
      distanceSensorA.clearInterrupt();

      sensor_readings.tof.distB = distanceSensorB.getDistance();
      distanceSensorB.clearInterrupt();

      sensor_readings.tof.stamp = millis();
      
      float avg_sensor_readings = (sensor_readings.tof.distA  + sensor_readings.tof.distB) / 2;
      distance_filter.update({0}, {avg_sensor_readings}, true);
      sensor_readings.tof.kalmanEst = distance_filter.mu(0);

      if (data_buffers.enabled[TOF]) {
        data_buffers.tof.Append(sensor_readings.tof);
      }
    }

    // Update IMU
    if (myICM.dataReady()) {
      myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
      sensor_readings.accel.x = myICM.accX();
      sensor_readings.accel.y = myICM.accY();
      sensor_readings.accel.z = myICM.accZ();

      sensor_readings.accel.stamp = millis();

      if (data_buffers.enabled[ACCEL]) {
        data_buffers.accel.Append(sensor_readings.accel);
      }

      sensor_readings.mag.x = myICM.magX();
      sensor_readings.mag.y = myICM.magY();
      sensor_readings.mag.z = myICM.magZ();

      sensor_readings.mag.stamp = millis();

      if (data_buffers.enabled[MAG]) {
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

      if (data_buffers.enabled[GYRO]) {
        data_buffers.gyro.Append(sensor_readings.gyro);
      }

      float roll = atan2(sensor_readings.accel.y, sensor_readings.accel.z) * (180.0 / 3.14);
      float pitch = atan2(sensor_readings.accel.x, sensor_readings.accel.z) * (180.0 / 3.14);

      // https://seanboe.me/blog/complementary-filters
      float gyro_favor = 0.98;
      pose.rot.x = (gyro_favor) * (pose.rot.x + myICM.gyrX() * dt) + (1.00 - gyro_favor) * (roll);
      pose.rot.y = (gyro_favor) * (pose.rot.y - myICM.gyrY() * dt) + (1.00 - gyro_favor) * (pitch);
      pose.rot.z = sensor_readings.gyro.z;

      pose.rot.stamp = millis();

      if (data_buffers.enabled[POSE] && (data_buffers.pose_rot.getLength() == 0 || millis() - data_buffers.pose_rot.Last().stamp > 100)) {
        data_buffers.pose_rot.Append(pose.rot);
      }
    }
  }

  BUFFER_TYPE string_to_buf_type(char *str) {
    if (strcmp(str, "GYRO") == 0) {
      return GYRO;
    }
    if (strcmp(str, "ACCEL") == 0) {
      return ACCEL;
    }
    if (strcmp(str, "MAG") == 0) {
      return MAG;
    }
    if (strcmp(str, "TOF") == 0) {
      return TOF;
    }
    if (strcmp(str, "POSE") == 0) {
      return POSE;
    }
    if (strcmp(str, "MOTOR") == 0) {
      return MOTOR;
    }
    return BUF_NA;
  }

  PID_CONTROLLER_TYPE string_to_PID_type(char *str) {
    if (strcmp(str, "ROTATION") == 0) {
      return ROTATION;
    }
    if (strcmp(str, "ROTATION_RATE") == 0) {
      return ROTATION_RATE;
    }
    return PID_NA;
  }

  void set_enabled(bool enable) {
    if (enable && !robot_enabled) {
      start_time = millis();
    }
    robot_enabled = enable;
  }

  void set_gyro(double val){
    sensor_readings.gyro.z = val;
  }
};

class BLE_HANDLER {
private:
  BLEService testService{ BLE_UUID_TEST_SERVICE };

  BLECStringCharacteristic rx_characteristic_string{ BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE };

  BLEFloatCharacteristic tx_characteristic_float{ BLE_UUID_TX_FLOAT, BLERead | BLENotify };
  BLECStringCharacteristic tx_characteristic_string{ BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE };

  // RX
  RobotCommand robot_cmd{ ":|" };

  // TX
  EString tx_estring_value;
  float tx_float_value{ 0.0 };

  enum CommandTypes {
    PING,
    ECHO,
    GET_TIME_MILLIS,
    ENABLE_ROBOT,
    DISABLE_ROBOT,
    ENABLE_BUFFER,
    RETRIEVE_BUFFER,
    DISABLE_BUFFER,
    SET_PID_GAINS,
    SET_MODE,
    SET_TARGET,
    SET_GYRO
  };

  int interval = 500;
  long int last_write;

  CAR *the_car;

  bool connected = false;

public:
  void setup(CAR *car) {
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

  void update() {
    BLEDevice central = BLE.central();

    if (central) {
      if (!connected) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
      }

      // While central is connected
      if (central.connected()) {
        connected = true;
        read_data();
      } else if (connected) {
        connected = false;
        the_car->set_enabled(false);
        Serial.println("Disconnected");
      }
    }
  }

  void handle_command() {
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

      case ENABLE_BUFFER:
        {
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
            return;
          CAR::BUFFER_TYPE buf = the_car->string_to_buf_type(char_arr);
          if (buf != CAR::BUF_NA) {
            Serial.print("Enabling buffer: ");
            Serial.println(char_arr);
          } else {
            Serial.print("BLE attempted to enable unknown buffer: ");
            Serial.println(char_arr);
          }
          the_car->data_buffers.enabled[buf] = true;
        }
        break;

      case DISABLE_BUFFER:
        {
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
            return;
          CAR::BUFFER_TYPE buf = the_car->string_to_buf_type(char_arr);
          if (buf != CAR::BUF_NA) {
            Serial.print("Disabling buffer: ");
            Serial.println(char_arr);
          } else {
            Serial.print("BLE attempted to disable unknown buffer: ");
            Serial.println(char_arr);
          }
          the_car->data_buffers.enabled[buf] = false;
        }
        break;

      case RETRIEVE_BUFFER:
        {
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
            return;
          CAR::BUFFER_TYPE buf = the_car->string_to_buf_type(char_arr);
          if (buf != CAR::BUF_NA) {
            the_car->data_buffers.enabled[buf] = false;

            Serial.print("Sending buffer: ");
            Serial.print(char_arr);
            Serial.print(" of length: ");
            tx_estring_value.clear();
            tx_estring_value.append("<START BUFFER ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(">");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            if (buf == CAR::ACCEL) {
              Serial.println(the_car->data_buffers.accel.getLength());
              send_data_buffer(&(the_car->data_buffers.accel));
              the_car->data_buffers.accel.Clear();
            }
            if (buf == CAR::GYRO) {
              Serial.println(the_car->data_buffers.gyro.getLength());
              send_data_buffer(&(the_car->data_buffers.gyro));
              the_car->data_buffers.gyro.Clear();
            }
            if (buf == CAR::MAG) {
              Serial.println(the_car->data_buffers.mag.getLength());
              send_data_buffer(&(the_car->data_buffers.mag));
              the_car->data_buffers.mag.Clear();
            }
            if (buf == CAR::TOF) {
              Serial.println(the_car->data_buffers.tof.getLength());
              send_data_buffer(&(the_car->data_buffers.tof));
              the_car->data_buffers.tof.Clear();
            }
            if (buf == CAR::POSE) {
              Serial.println(the_car->data_buffers.pose_rot.getLength());
              send_data_buffer(&(the_car->data_buffers.pose_rot));
              the_car->data_buffers.pose_rot.Clear();
            }
            if (buf == CAR::MOTOR) {
              Serial.println(the_car->data_buffers.motor_input.getLength());
              send_data_buffer(&(the_car->data_buffers.motor_input));
              the_car->data_buffers.motor_input.Clear();
            }

            tx_estring_value.clear();
            tx_estring_value.append("<END BUFFER ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(">");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          } else {
            Serial.print("BLE requested unknown buffer: ");
            Serial.println(char_arr);
            tx_estring_value.clear();
            tx_estring_value.append("Unknown buffer: ");
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
          }
        }
        break;

      case SET_PID_GAINS:
        {
          char resp[4][MAX_MSG_SIZE];
          for (int i = 0; i < 4; i++) {
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
              return;
            strcpy(resp[i], char_arr);
          }

          CAR::PID_CONTROLLER_TYPE pid_type = the_car->string_to_PID_type(resp[0]);
          if (pid_type != CAR::PID_NA) {
            the_car->pid_controllers.pid[pid_type].set_gains(strtod(resp[1], NULL), strtod(resp[2], NULL), strtod(resp[3], NULL));
            Serial.print("Setting PID gains for controller #");
            Serial.print(pid_type);
            Serial.print(" to P: ");
            Serial.print(strtod(resp[1], NULL)), 4;
            Serial.print(" to I: ");
            Serial.print(strtod(resp[2], NULL), 4);    
            Serial.print(" to D: ");
            Serial.println(strtod(resp[3], NULL), 4);    
          }
        }
        break;
      
      case SET_MODE:
        success = robot_cmd.get_next_value(char_arr);
        if (!success)
          return;
        the_car->current_mode = (CAR::MODE_TYPE)atoi(char_arr);
        the_car->mode_changed = true;
        Serial.print("Set mode to: ");
        Serial.println(the_car->current_mode);
        break;
      
      case SET_TARGET:
        success = robot_cmd.get_next_value(char_arr);
        if (!success)
          return;
        the_car->current_target = strtod(char_arr, NULL);
        Serial.print("Set target to: ");
        Serial.println(char_arr);
        
        break;

      case SET_GYRO:
        success = robot_cmd.get_next_value(char_arr);
        if (!success)
          return;
        the_car->set_gyro(strtod(char_arr, NULL));
        Serial.print("Set current gyro to: ");
        Serial.println(char_arr);
        break;

      default:
        Serial.print("Invalid Command Type: ");
        Serial.println(cmd_type);
        break;
    }
  }

  void send_data_buffer(LinkedList<THREE_AXIS> *buf) {
    if (!buf->moveToStart()) {
      Serial.println("No data to report. Make sure to enable buffer before requesting data.");
      return;
    }
    do {
      send_data_point(&(buf->getCurrent()));
    } while (buf->next());
  }

  void send_data_buffer(LinkedList<TOF_DATA> *buf) {
    if (!buf->moveToStart()) {
      Serial.println("No data to report. Make sure to enable buffer before requesting data.");
      return;
    }
    do {
      send_data_point(&(buf->getCurrent()));
    } while (buf->next());
  }

  void send_data_point(THREE_AXIS *send) {
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

  void send_data_point(TOF_DATA *send) {
    tx_estring_value.clear();

    tx_estring_value.append("Time: ");
    tx_estring_value.append((int)send->stamp);

    tx_estring_value.append(" | ");
    tx_estring_value.append("DistA: ");
    tx_estring_value.append(send->distA);


    tx_estring_value.append(" | ");
    tx_estring_value.append("DistB: ");
    tx_estring_value.append(send->distB);

    tx_estring_value.append(" | ");
    tx_estring_value.append("Kalman: ");
    tx_estring_value.append(send->kalmanEst);

    tx_characteristic_string.writeValue(tx_estring_value.c_str());
  }

  void read_data() {
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
      handle_command();
    }
  }
};

CAR my_car;
BLE_HANDLER my_ble_handler;

void setup() {
  my_ble_handler.setup(&my_car);
  my_car.setup();
}

void loop() {
  my_ble_handler.update();
  my_car.update();
}