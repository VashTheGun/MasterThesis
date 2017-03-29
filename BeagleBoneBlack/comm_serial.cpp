#include <stdio.h>
#include "serialib.h"
#include "sensorfusion.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string.h>
#include <iomanip>
#include <math.h>       /* atan2 */
#include "sensor_type.h"
#define PI 3.14159265

using namespace std;

// SERIAL COMMUNICATION
    // WINDOWS
    #if defined (_WIN32) || defined( _WIN64)
    #define         ARDUINO_MEGA            "COM1"                               // COM1 for windows
    #define         COMPUTER              "COM2"                               // COM1 for windows
    #endif
    // LINUX
    #ifdef __linux__
    #define         ARDUINO_MEGA             "/dev/ttyS4"                         // ttyS0 for linux
    #define         COMPUTER               "/dev/ttyS2"                         // ttyS0 for linux
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

// MODE - DUTY CYCLE VALUES
#define DUTY_MODE_HIGH 2024
#define DUTY_MODE_MIDDLE 1504
#define DUTY_MODE_LOW 980

// SENSOR DATA STRUCTURE
sensor_data_t sensor_data;
// ACTUATOR DATA STRUCTURE
actuator_t actuator = {.traction_motor = DUTY_ESC_IDLE,.steering_motor = DUTY_SERVO_MIDDLE,.instruction = 50};

// Declare and Initialize for actuator data
actuator_data_t actuator_data = {actuator};


// MAIN CODE
int main()
{
    // SERIAL COMM w. ARDUINO MEGA
    serialib LS_AM;                                                                 // Object of the serialib class
    int Ret_AM;                                                                     // Used for return values
    // SERIAL COMM w. PC
    serialib LS_PC;                                                                 // Object of the serialib class
    int Ret_PC;                                                                     // Used for return values
    
    int Ret;
    char* val;
    char val_AM[1];
    float accel[3];
    float t_val;
    float time_new;

    // SENSOR FUSION DAT STRUCTURE
    fusion_output_data_t fusion_data;
    SensorFusion sensorfusion;

    // LOG FILE
        std::ofstream out;
        out.open("/home/debian/data/data.txt", std::ios::app);
        clock_t start, stop;
    
        // current date/time based on current system
        time_t now = time(0);
        // convert now to string form
        char* dt = ctime(&now);
        
        out << dt << endl << endl;    
        std::string str_ln1 = ("[millisec] \t[rad/sec] \t[rad/sec] \t[rad/sec] \t[rad/sec] \t[m/s^2] \t[m/s^2] \t[m/s^2] \t[rad/sec] \t[rad/sec] \t[rad/sec] \t[deg] \t[deg] \t[deg] \t[m] \t[m] \t[sec] \t[sec] \t[] \t[] \t[m] \t[m] \t[deg] \t[m/s] \t[m/s]");
        std::string str_ln2 = ("time \tenc_fl \tenc_fr \tenc_rl \tenc_rr \tacc_x \tacc_y \tacc_z \tgyro_x \tgyro_y \tgyro_z \tyaw \tpitch \troll \tgps_x \tgps_y \ttime_gps \ttime_imu \ttraction \tsteering \tfused_x \tfused_y \tfused_theta \tfused_vx \tfused_vy");
        out << str_ln1 << endl;
        out << str_ln2 << endl;


    // Open serial ports
    Ret_AM=LS_AM.Open(ARDUINO_MEGA,115200);                                         // Open serial link at 115200 bauds
    Ret_PC=LS_PC.Open(COMPUTER,115200);                                             // Open serial link at 115200 bauds
    
    time_new = 0;

    while (1) {

        start = std::clock();
        
        //send a string to Arduino Mega
        Ret_AM=LS_AM.WriteString("AT\n");
        //read sensors data from Arduino Mega
        Ret_AM=LS_AM.Read(sensor_data.sensor_data_byte,sizeof(sensor_data),5000);                  // Read a maximum of 128 characters with a timeout of 5 seconds

        
        //sensor fusion
        sensorfusion.ekf_sensor_fusion(sensor_data);
        fusion_data = sensorfusion.getFusionData();

        
        // TRACTION & STEERING %
        float traction = -(sensor_data.sensor.command_traction_motor -7010)/1402.0;
        float steering = -(sensor_data.sensor.command_steering_motor -6881)/1857.0;
	    float time_new = time_new+sensor_data.sensor.time_imu ;


        //save data on a txtfile
        out << fixed << setprecision(5) << time_new*1000   << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.encoder[0] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.encoder[1] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.encoder[3] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.encoder[2] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.acc[0] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.acc[1] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.acc[2] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.gyro[0] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.gyro[1] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.gyro[2] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.orientation[0] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.orientation[1] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.orientation[2] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.gps[0] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.gps[1] << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.time_gps << "\t";
        out << fixed << setprecision(10) << sensor_data.sensor.time_imu << "\t";
        out << fixed << setprecision(10) << traction << "\t";
        out << fixed << setprecision(10) << steering << "\t";
        out << fixed << setprecision(10) << fusion_data.fusion_output.pos[0] << "\t";
        out << fixed << setprecision(10) << fusion_data.fusion_output.pos[1] << "\t";
        out << fixed << setprecision(10) << fusion_data.fusion_output.vel[0] << "\t";
        out << fixed << setprecision(10) << fusion_data.fusion_output.vel[1] << "\t";
        out << fixed << setprecision(10) << fusion_data.fusion_output.ang[0] << "\t";
        out << fixed << setprecision(10) << fusion_data.fusion_output.ang[1] << endl;


    
        //print out data

/*        cout << fixed << setprecision(2) << time_new*1000 << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.encoder[0] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.encoder[1] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.encoder[3] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.encoder[2] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.acc[0] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.acc[1] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.acc[2] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.gyro[0] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.gyro[1] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.gyro[2] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.orientation[0] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.orientation[1] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.orientation[2] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.gps[0] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.gps[1] << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.time_gps << "\t";
        cout << fixed << setprecision(5) << sensor_data.sensor.time_imu << "\t";
        cout << fixed << setprecision(5) << traction << "\t";
        cout << fixed << setprecision(5) << steering << endl;
*/
/*
        cout << fixed << setprecision(10) << fusion_data.fusion_output.pos[0] << "\t";
        cout << fixed << setprecision(10) << fusion_data.fusion_output.pos[1] << "\t";
        cout << fixed << setprecision(10) << fusion_data.fusion_output.vel[0] << "\t";
        cout << fixed << setprecision(10) << fusion_data.fusion_output.vel[1] << "\t";
        cout << fixed << setprecision(10) << fusion_data.fusion_output.ang[0] << "\t";
        cout << fixed << setprecision(10) << fusion_data.fusion_output.ang[1] << endl;
*/


        //send the motors input to the Arduino Mega
        //Ret=LS_AM.Write(actuator_data.actuator_data_byte,sizeof(actuator_data));

        //send fused data to PC
        Ret=LS_PC.Write(fusion_data.fusion_output_data_byte,sizeof(fusion_data));
    }

    // Close the connection with the devices
    LS_AM.Close();
    LS_PC.Close();

}
