#include <stdio.h>
#include "serialib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string.h>
#include <iomanip>
#include "sensor_type.h"

using namespace std;

#if defined (_WIN32) || defined( _WIN64)
#define         BEAGLEBONE              "COM2"                               // COM1 for windows
#endif

#ifdef __linux__
#define         BEAGLEBONE               "/dev/ttyUSB1"                         // ttyS0 for linux_

#endif


#define precision 4

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

sensor_data_t sensor_data;
command_data_t command_data;
fusion_output_data_t fusion_data;


actuator_t actuator = {.traction_motor = DUTY_ESC_IDLE,.steering_motor = DUTY_SERVO_MIDDLE,.instruction = 50};

// Declare and Initialize for actuator data
actuator_data_t actuator_data = {actuator};


int main()
{

    serialib LS_BB;                                                                 // Object of the serialib class
    int Ret_BB;                                                                     // Used for return values
    char val;
    char val_AM[1];
    char value;

    std::ofstream out;
    out.open("data.txt", std::ios::app);
    
    clock_t start, stop;
    //std::fstream myfile("send_data.txt", std::ios_base::in);

    // current date/time based on current system
    time_t now = time(0);
    // convert now to string form
    char* dt = ctime(&now);
    out << dt << endl << endl;

    std::string str_ln1 = ("[millisec] \t[rad/sec] \t[rad/sec] \t[rad/sec] \t[rad/sec] \t[m/s^2] \t[m/s^2] \t[m/s^2] \t[rad/sec] \t[rad/sec] \t[rad/sec] \t[deg] \t[deg] \t[deg] \t[m] \t[m] \t[sec] \t[sec] \t[sec] \t[--] \t[--] \t[m] \t[m] \t[deg] \t[m/s] \t[m/s]");
    std::string str_ln2 = ("time \tenc_fl \tenc_fr \tenc_rl \tenc_rr \tacc_x \tacc_y \tacc_z \tgyro_x \tgyro_y \tgyro_z \tyaw \tpitch \troll \tgps_x \tgps_y \ttime_script \ttime_gps \ttime_imu \ttraction \tsteering \tfused_x \tfused_y \tfused_theta \tfused_vx \tfused_vy");
    out << str_ln1 << endl;
    out << str_ln2 << endl;
    //out.close();
    printf (dt);
  
    // Open serial ports
    Ret_BB=LS_BB.Open(BEAGLEBONE,115200);                                           // Open serial link at 115200 bauds

    float     time;
    float t = time/CLOCKS_PER_SEC;
//    value = 'a';
    int del;

    while (1) {
        
        start = std::clock();

        //usleep(100);
        //Ret_BB=LS_BB.Read(command_data.command_data_byte,sizeof(command_data),1);
        //usleep(100);
        //if ( command_data.command.com == 10){                                                              // If a string has been read from, print the string
        
            
            Ret_BB=LS_BB.WriteString("AT\n"); 
            Ret_BB=LS_BB.Read(fusion_data.fusion_output_data_byte,sizeof(fusion_data),5000);                  // Read a maximum of 128 characters with a timeout of 5 seconds
            //usleep(100);
            //cout << Ret_BB << endl;    
                
            //float traction = (sensor_data.sensor.command_traction_motor -7010)/1402.0;
            //float steering = (sensor_data.sensor.command_steering_motor -6881)/1857.0;
            //usleep(100);
            //save data on a txtfile
            
            /*
            out << fixed << setprecision(precision) << clock()<< "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.encoder[0] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.encoder[1] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.encoder[2] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.encoder[3] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.acc[0] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.acc[1] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.acc[2] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.gyro[0] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.gyro[1] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.gyro[2] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.orientation[0] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.orientation[1] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.orientation[2] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.gps[0] << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.gps[1] << "\t";
            out << fixed << setprecision(precision) << time<< "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.time_gps << "\t";
            out << fixed << setprecision(precision) << sensor_data.sensor.time_imu << "\t";        
            out << fixed << setprecision(precision) << traction << "\t";
            out << fixed << setprecision(precision) << steering << endl; */
            out << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.pos[0] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.pos[1] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.vel[0] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.vel[1] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.ang[0] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.ang[1] << endl;
            
            
            //usleep(1000);
            //print out
            /*
            cout << fixed << setprecision(10) << clock() << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.encoder[0] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.encoder[1] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.encoder[2] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.encoder[3] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.acc[0] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.acc[1] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.acc[2] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.gyro[0] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.gyro[1] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.gyro[2] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.orientation[0] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.orientation[1] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.orientation[2] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.gps[0] << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.gps[1] << "\t";
            cout << fixed << setprecision(6) << t<< "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.time_gps << "\t";
            cout << fixed << setprecision(4) << sensor_data.sensor.time_imu << "\t";        
            cout << fixed << setprecision(4) << traction << "\t";
            cout << fixed << setprecision(4) << steering << endl; */
            
             

            out << fixed << setprecision(10) << fusion_data.fusion_output.pos[0] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.pos[1] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.vel[0] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.vel[1] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.ang[0] << "\t";
            out << fixed << setprecision(10) << fusion_data.fusion_output.ang[1] << endl;
            
            
            stop = std::clock() ;
            //usleep(10000);
            
            time = (float)(stop-start) / (float)CLOCKS_PER_SEC*1000.0f;
         //del = 20000-t*1000000;
            
        //}
        
        
        //ristd::cout << setprecision(2) << time << " ms" << std::endl;
    }
    // Close the connection with the devices

    LS_BB.Close();

}
