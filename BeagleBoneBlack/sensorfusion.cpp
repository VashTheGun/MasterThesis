#include <iostream>
#define PI 3.14159265
#include "sensorfusion.h"
#include <cmath>       /* atan2 */

SensorFusion::SensorFusion(){
    f_c = sqrt(2);
    omega_cut = 2 * PI * f_c; // 2 * pi * f_c
	alpha     = 0.5;//(omega_cut * sensor_data.sensor.time_gps) / (1+ omega_cut * sensor_data.sensor.time_gps);
};

void SensorFusion::ekf_sensor_fusion(sensor_data_t fusion_input){
    
        float A[9][1] = {
             {fusion_output_prev.fusion_output.pos[0]+fusion_output_prev.fusion_output.vel[0]*fusion_input.sensor.time_imu*cos(fusion_output_prev.fusion_output.ang[2])-fusion_output_prev.fusion_output.vel[1]*fusion_input.sensor.time_imu*sin(fusion_output_prev.fusion_output.ang[2])},
             {fusion_output_prev.fusion_output.vel[0]+fusion_output_prev.fusion_output.acc[0]*fusion_input.sensor.time_imu+fusion_output_prev.fusion_output.ang[1]*fusion_output_prev.fusion_output.vel[1]*fusion_input.sensor.time_imu},
             {fusion_output_prev.fusion_output.acc[0]},
             {fusion_output_prev.fusion_output.pos[1]+fusion_output_prev.fusion_output.vel[0]*fusion_input.sensor.time_imu*sin(fusion_output_prev.fusion_output.ang[2])+fusion_output_prev.fusion_output.vel[1]*fusion_input.sensor.time_imu*cos(fusion_output_prev.fusion_output.ang[2])},
             {fusion_output_prev.fusion_output.vel[1]+fusion_output_prev.fusion_output.acc[1]*fusion_input.sensor.time_imu-fusion_output_prev.fusion_output.ang[1]*fusion_output_prev.fusion_output.vel[0]*fusion_input.sensor.time_imu},
             {fusion_output_prev.fusion_output.acc[1]},
             {fusion_output_prev.fusion_output.ang[0]+fusion_output_prev.fusion_output.ang[1]*fusion_input.sensor.time_imu},
             {fusion_output_prev.fusion_output.ang[1]},    
             {fusion_output_prev.fusion_output.ang[2]+fusion_output_prev.fusion_output.ang[1]*fusion_input.sensor.time_imu},
         };
         
         float F[9][9] = {
             {1, fusion_input.sensor.time_imu*cos(fusion_output_prev.fusion_output.ang[2]), 0, 0, fusion_input.sensor.time_imu*sin(fusion_output_prev.fusion_output.ang[2]), 0, 0, 0, -fusion_input.sensor.time_imu*(fusion_output_prev.fusion_output.vel[1]*cos(fusion_output_prev.fusion_output.ang[2])+fusion_output_prev.fusion_output.vel[0]*sin(fusion_output_prev.fusion_output.ang[2]))},
             {0, 1, fusion_input.sensor.time_imu, 0, fusion_input.sensor.time_imu*fusion_output_prev.fusion_output.ang[1], 0, 0, fusion_input.sensor.time_imu*fusion_output_prev.fusion_output.vel[1], 0},
             {0, 0, 1, 0, 0, 0, 0, 0, 0},
             {0, fusion_input.sensor.time_imu*sin(fusion_output_prev.fusion_output.ang[2]), 0, 1, fusion_input.sensor.time_imu*cos(fusion_output_prev.fusion_output.ang[2]), 0, 0, 0, fusion_input.sensor.time_imu*(fusion_output_prev.fusion_output.vel[0]*cos(fusion_output_prev.fusion_output.ang[2])-fusion_output_prev.fusion_output.vel[0]*sin(fusion_output_prev.fusion_output.ang[2]))},
             {0, -fusion_input.sensor.time_imu*fusion_output_prev.fusion_output.ang[1], 0, 0, 1, fusion_input.sensor.time_imu, 0, -fusion_input.sensor.time_imu*fusion_output_prev.fusion_output.vel[0], 0},
             {0, 0, 0, 0, 0, 1, 0, 0, 0},
             {0, 0, 0, 0, 0, 0, 1,fusion_input.sensor.time_imu, 0},
             {0, 0, 0, 0, 0, 0, 0, 1, 0},    
             {0, 0, 0, 0, 0, 0, 0,fusion_input.sensor.time_imu, 1},
         };
    
    
    float u_enc = (fusion_input.sensor.encoder[0]+fusion_input.sensor.encoder[1]+fusion_input.sensor.encoder[2]+fusion_input.sensor.encoder[3])/2*0.05;
    
    float z[7][1] = {
        {fusion_input.sensor.gps[0]},
        {u_enc},
        {fusion_input.sensor.acc[0]},
        {fusion_input.sensor.gps[1]},
        {fusion_input.sensor.acc[1]},
        {fusion_input.sensor.orientation[0]},    
        {fusion_input.sensor.gyro[2]},
    };

    // Finding transpose of matrix a[][] and storing it in array trans[][].
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
        {
            Ft[j][i]=F[i][j];
        }
        
    // Finding transpose of matrix a[][] and storing it in array trans[][].
    for(i = 0; i < 7; ++i)
        for(j = 0; j < 9; ++j)
        {
            Ht[j][i]=H[i][j];
        }




    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
            for(k = 0; k < 9; ++k)
            {
                FP[i][j] += F[i][k] * P[k][j];
            }

    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
            for(k = 0; k < 9; ++k)
            {
                FPFt[i][j] += FP[i][k] * Ft[k][j];
            }
            
    // Summing matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
            {
                p[i][j] = FPFt[i][j] + Q[i][j];
            }

            
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 7; ++i)
        for(j = 0; j < 9; ++j)
            for(k = 0; k < 9; ++k)
            {
                Hp[i][j] += H[i][k] * p[k][j];
            }

    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 7; ++i)
        for(j = 0; j < 7; ++j)
            for(k = 0; k < 9; ++k)
            {
                HpHt[i][j] += Hp[i][k] * Ht[k][j];
            }            
  
    // Summing matrix a and b and storing in array mult.
    for(i = 0; i < 7; ++i)
        for(j = 0; j < 7; ++j)
            {
                S[i][j] = HpHt[i][j] + R[i][j];
            }
            
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 7; ++j)
            for(k = 0; k < 9; ++k)
            {
                pHt[i][j] += p[i][k] * Ht[k][j];
            }            

    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 7; ++j)
            for(k = 0; k < 7; ++k)
            {
                K[i][j] += pHt[i][k] * S[k][j];
            }
            
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 7; ++i)
        for(j = 0; j < 1; ++j)
            for(k = 0; k < 9; ++k)
            {
                HA[i][j] += H[i][k] * A[k][j];
            }

    // Summing matrix a and b and storing in array mult.
    for(i = 0; i < 7; ++i)
        for(j = 0; j < 1; ++j)
            {
                z_HA[i][j] = z[i][j] - HA[i][j];
            }

    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 1; ++j)
            for(k = 0; k < 7; ++k)
            {
                Kz_HA[i][j] += K[i][k] * z_HA[k][j];
            }            

    // Summing matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 1; ++j)
            {
                xx[i][j] = A[i][j] + Kz_HA[i][j];
            }            
            
            
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 7; ++j)
            for(k = 0; k < 7; ++k)
            {
                KS[i][j] += K[i][k] * S[k][j];
            }  
            
    // Finding transpose of matrix a[][] and storing it in array trans[][].
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
        {
            Kt[j][i]=K[i][j];
        }           
            
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
            for(k = 0; k < 7; ++k)
            {
                KSKt[i][j] += KS[i][k] * Kt[k][j];
            }  
                        
    // Summing matrix a and b and storing in array mult.
    for(i = 0; i < 9; ++i)
        for(j = 0; j < 9; ++j)
            {
                P[i][j] = p[i][j] - KSKt[i][j];
            }          
            
 
 
 
   fusion_output.fusion_output.pos[0] = xx[1][1];
   fusion_output.fusion_output.vel[0] = xx[2][1];
   fusion_output.fusion_output.acc[0] = xx[3][1];
   fusion_output.fusion_output.pos[1] = xx[4][1];
   fusion_output.fusion_output.vel[1] = xx[5][1];
   fusion_output.fusion_output.acc[1] = xx[6][1]; 
   fusion_output.fusion_output.ang[0] = xx[7][1];
   fusion_output.fusion_output.ang[1] = xx[8][1];
   fusion_output.fusion_output.ang[2] = xx[9][1];
   
};

fusion_output_data_t SensorFusion::getFusionData(){
    fusion_output_prev = fusion_output;
    return fusion_output;

};

