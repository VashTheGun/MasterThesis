#ifndef SENSOR_TYPE_H
#define SENSOR_TYPE_H

typedef struct {
    float encoder[4];
    float acc[3];
    float gyro[3];
    float orientation[3];
    float gps[2];
    float time_gps;
    float time_imu;
    short int command_traction_motor;
    short int command_steering_motor;
}sensor_t;

typedef union {
    sensor_t sensor;
    char sensor_data_byte[sizeof(sensor)];
}sensor_data_t;

typedef struct {
    short int com;
}command_t;

typedef union {
    command_t command;
    char command_data_byte[sizeof(command)];
}command_data_t;


//typedef struct Fusion_Output{
//    float pos[2];
//    float ang[1];
//    float vel[2];
//}fusion_output_t;


typedef struct {
    short int traction_motor;
    short int steering_motor;
    short int instruction;
}actuator_t;

typedef union {
    actuator_t actuator;
    char actuator_data_byte[sizeof(actuator)];
}actuator_data_t;


//typedef union {
//    fusion_output_t fusion_output;
//    char fusion_output_data_byte[24];
//}fusion_output_data_t;

typedef struct {
    float pos[2]; /* x,y */
    float ang[3]; /* psi, omega, theta */
    float vel[2]; /* vx, vy */
    float acc[2]; /* ax, ay */
    
}fusion_output_t;

typedef union {
    fusion_output_t fusion_output;
    char fusion_output_data_byte[sizeof(fusion_output)];
}fusion_output_data_t;

#endif