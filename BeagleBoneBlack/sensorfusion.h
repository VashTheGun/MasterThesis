#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "sensor_type.h"


class SensorFusion

{
    private:
        fusion_output_data_t fusion_output ;
        fusion_output_data_t fusion_output_prev ;

        float f_c;
        float omega_cut;
        float alpha;
        
        int P[9][9] = {  
            {1000, 0, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
            {0, 1000, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 1 */
            {0, 0, 1000, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 2 */
            {0, 0, 0, 1000, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 3 */
            {0, 0, 0, 0, 1000, 0, 0, 0, 0} ,   /*  initializers for row indexed by 4 */
            {0, 0, 0, 0, 0, 1000, 0, 0, 0} ,   /*  initializers for row indexed by 5 */
            {0, 0, 0, 0, 0, 0, 1000, 0, 0} ,   /*  initializers for row indexed by 6 */
            {0, 0, 0, 0, 0, 0, 0, 1000, 0} ,   /*  initializers for row indexed by 7 */
            {0, 0, 0, 0, 0, 0, 0, 0, 1000} ,   /*  initializers for row indexed by 8 */
         };
         
         int H[7][9] = {  
            {1, 0, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
            {0, 1, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 1 */
            {0, 0, 1, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 2 */
            {0, 0, 0, 1, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 3 */
            {0, 0, 0, 0, 0, 1, 0, 0, 0} ,   /*  initializers for row indexed by 4 */
            {0, 0, 0, 0, 0, 0, 1, 0, 0} ,   /*  initializers for row indexed by 5 */
            {0, 0, 0, 0, 0, 0, 0, 1, 0} ,   /*  initializers for row indexed by 6 */
         };
         
         float Q[9][9] = {  
            {0.0055, 0, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
            {0, 0.0005, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 1 */
            {0, 0, 0.005, 0, 0, 0, 0, 0, 0} ,    /*  initializers for row indexed by 2 */
            {0, 0, 0, 0.0055, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 3 */
            {0, 0, 0, 0, 0.01, 0, 0, 0, 0} ,     /*  initializers for row indexed by 4 */
            {0, 0, 0, 0, 0, 0.001, 0, 0, 0} ,    /*  initializers for row indexed by 5 */
            {0, 0, 0, 0, 0, 0, 0.05, 0, 0} ,     /*  initializers for row indexed by 6 */
            {0, 0, 0, 0, 0, 0, 0, 0.0005, 0} ,   /*  initializers for row indexed by 7 */
            {0, 0, 0, 0, 0, 0, 0, 0, 0.001} ,    /*  initializers for row indexed by 8 */
         };
         
         float R[7][7] = {  
            {0.05, 0, 0, 0, 0, 0, 0} ,     /*  initializers for row indexed by 0 */
            {0, 0.1, 0, 0, 0, 0, 0} ,      /*  initializers for row indexed by 1 */
            {0, 0, 0.3, 0, 0, 0, 0} ,      /*  initializers for row indexed by 2 */
            {0, 0, 0, 0.05, 0, 0, 0} ,     /*  initializers for row indexed by 3 */
            {0, 0, 0, 0, 0.003, 0, 0} ,    /*  initializers for row indexed by 4 */
            {0, 0, 0, 0, 0, 0.1, 0} ,      /*  initializers for row indexed by 5 */
            {0, 0, 0, 0, 0, 0, 0.01} ,     /*  initializers for row indexed by 6 */
         };
         
         int I[9][9] = {  
            {1, 0, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
            {0, 1, 0, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 1 */
            {0, 0, 1, 0, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 2 */
            {0, 0, 0, 1, 0, 0, 0, 0, 0} ,   /*  initializers for row indexed by 3 */
            {0, 0, 0, 0, 1, 0, 0, 0, 0} ,   /*  initializers for row indexed by 4 */
            {0, 0, 0, 0, 0, 1, 0, 0, 0} ,   /*  initializers for row indexed by 5 */
            {0, 0, 0, 0, 0, 0, 1, 0, 0} ,   /*  initializers for row indexed by 6 */
            {0, 0, 0, 0, 0, 0, 0, 1, 0} ,   /*  initializers for row indexed by 7 */
            {0, 0, 0, 0, 0, 0, 0, 0, 1} ,   /*  initializers for row indexed by 8 */
         };
         

         
         float mult[10][10], FP[10][10], FPFt[10][10], Hp[10][10], HpHt[10][10];
         float trans[10][10], Ft[10][10], Ht[10][10], pHt[10][10], K[10][10], HA[10][10], z_HA[10][10];
         float summ[10][10], p[10][10], S[10][10], Kz_HA[10][10], xx[10][10], KS[10][10], Kt[10][10], KSKt[10][10];
         
         int  r1, c1, r2, c2, i, j, k;


    public:
        SensorFusion();
        void ekf_sensor_fusion(sensor_data_t fusion_input);
        fusion_output_data_t getFusionData();

    };

#endif