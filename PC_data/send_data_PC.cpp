#include <stdio.h>
#include "serialib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string.h>
#include <iomanip>

using namespace std;

#if defined (_WIN32) || defined( _WIN64)
#define         BEAGLEBONE              "COM2"                               // COM1 for windows
#endif

#ifdef __linux__
#define         BEAGLEBONE               "/dev/ttyUSB0"                         // ttyS0 for linux_

#endif

// STEARING - DUTY CYCLE VALUES
#define DUTY_STEARING_DX 1052
#define DUTY_STEARING_IDLE 1476
#define DUTY_STEARING_SX 1890

// Previous values in 16 bit
// Nominal Value = 6881
// Boundaries = +/- 27% nominal value
#define DUTY_SERVO_DX 5024//4915
#define DUTY_SERVO_MIDDLE 6881
#define DUTY_SERVO_SX 8738//8847

// MOTOR - DUTY CYCLE VALUES
#define DUTY_MOTOR_MAX 2032
#define DUTY_MOTOR_IDLE 1500
#define DUTY_MOTOR_IDLE_SAFE 1340
#define DUTY_MOTOR_MIN 1000

// Previous values in 16 bit
// Nominal Value = 7012
// Boundaries = +/- 20% nominal value
#define DUTY_ESC_MAX 8412//8064
#define DUTY_ESC_MAX_SAFE 7200
#define DUTY_ESC_IDLE 7010
#define DUTY_ESC_MIN 5608//5960

//MODE - DUTY CYCLE VALUES
#define DUTY_MODE_HIGH 2024
#define DUTY_MODE_MIDDLE 1504
#define DUTY_MODE_LOW 980

typedef struct Sensor{
    float encoder[4];
    float acc[3];
    float gyro[3];
    float orientation[3];
    short int gps[2];
}sensor_t;

typedef struct Actuator{
    short int traction_motor;
    short int steering_motor;
    short int instruction;
}actuator_t;

actuator_t actuator = {.traction_motor = DUTY_ESC_IDLE,.steering_motor = DUTY_SERVO_MIDDLE,.instruction = 100};

typedef union {
    sensor_t sensor;
    char sensor_data_byte[56];
}sensor_data_t;

typedef union {
    actuator_t actuator;
    char actuator_data_byte[6];
}actuator_data_t;

sensor_data_t sensor_data;


// Declare and Initialize for actuator data
actuator_data_t actuator_data = {actuator};

int main()
{

    serialib LS_BB;                                                                 // Object of the serialib class
    int Ret_BB;                                                                     // Used for return values
  
    // Open serial ports
    Ret_BB=LS_BB.Open(BEAGLEBONE,115200);                                           // Open serial link at 115200 bauds

    cout << "Make your choice" << endl;
    cout << "\t1 remote\n\t2 controlled"<<endl;
    
    while (1) {

        cin >> actuator_data.actuator.instruction;
        cout << actuator_data.actuator.instruction << endl;
        Ret_BB=LS_BB.Write(actuator_data.actuator_data_byte,sizeof(actuator_data));



    }
    // Close the connection with the devices

    LS_BB.Close();

}
