#include "LinkedList.h"

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

SFEVL53L1X distanceSensorA(Wire, 7, -1);
SFEVL53L1X distanceSensorB(Wire, 8, -1);

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "f74736e0-f5ac-4541-959d-e6c1f1b3f55c"

#define BLE_UUID_RX_STRING "58482b00-4146-4122-be67-2d89016731a8"

#define BLE_UUID_TX_FLOAT "51eed2ce-3329-4232-b8d5-8f022aaa2d1a"
#define BLE_UUID_TX_STRING "aa71399e-0f1d-411d-ac23-7ace2936fd5e"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_TOF_DATA_5S
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            /*
             * Your code goes here.
             */
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());             
            
            break;
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        case GET_TIME_MILLIS:
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((float)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        case GET_TOF_DATA_5S: {
            int tof_readings[25][3];
            for(int i = 0; i < 25; i++){
                unsigned long start = millis();
                distanceSensorA.startRanging(); //Write configuration bytes to initiate measurement
                distanceSensorB.startRanging();
                while (!distanceSensorA.checkForDataReady() || !distanceSensorB.checkForDataReady())
                {
                  ;;
                }
                int distanceA = distanceSensorA.getDistance(); //Get the result of the measurement from the sensor
                int distanceB = distanceSensorB.getDistance();
                distanceSensorA.clearInterrupt();
                distanceSensorA.stopRanging();
                distanceSensorB.clearInterrupt();
                distanceSensorB.stopRanging();

                tof_readings[i][0] = (int)millis();
                tof_readings[i][1] = distanceA;
                tof_readings[i][2] = distanceB;

                delay(start + 250 - millis()); // Around 5 times per sec                    
            }

            for(int i = 0; i < 25; i++){
                tx_estring_value.clear();
                tx_estring_value.append(tof_readings[i][0]);
                tx_estring_value.append("|");
                tx_estring_value.append(tof_readings[i][1]);
                tx_estring_value.append("|");
                tx_estring_value.append(tof_readings[i][2]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
        }break;
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Wire.begin();

    Serial.begin(115200);

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

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
    bool initialized = false;
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
      distanceSensorA.startRanging();
      distanceSensorB.startRanging();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();
    if(distanceSensorA.checkForDataReady()){
        Serial.println(distanceSensorA.getDistance());
        distanceSensorA.clearInterrupt();
      }

    // If a central is connected to the peripheral
    // if (central) {
    //     Serial.print("Connected to: ");
    //     Serial.println(central.address());

    //     // While central is connected
    //     while (central.connected()) {
    //         // Send data
    //         write_data();

    //         // Read data
    //         read_data();
    //     }

    //     Serial.println("Disconnected");
    // }
}
