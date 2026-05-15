//
// Created by Roman on 2024/3/23.
//

#include "BiasEstimate.h"

float static_gx_bias_ = 0.0;
float static_gy_bias_ = 0.0;
float static_gz_bias_ = 0.0;
float est_temp_gxBias_ = 0.0;
float est_temp_gyBias_ = 0.0;
float est_temp_gzBias_ = 0.0;
float est_temp_axBias_ = 0.0;
float est_temp_ayBias_ = 0.0;
float est_temp_azBias_ = 0.0;
float real_gx_bias_ = 0.0;
float real_gy_bias_ = 0.0;
float real_gz_bias_ = 0.0;
float real_ax_bias_ = 0.0;
float real_ay_bias_ = 0.0;
float real_az_bias_ = 0.0;
float average_tempature_ = 0.0;
int static_buf_size_ = 300;
float gxBiasBuff_[23] = {0.0f};
float gyBiasBuff_[23] = {0.0f};
float gzBiasBuff_[23] = {0.0f};
float axBiasBuff_[23] = {0.0f};
float ayBiasBuff_[23] = {0.0f};
float azBiasBuff_[23] = {0.0f};
float tempatureBuff_[23] = {0.0f};

void calcSensorVar(const float * gyro, float *var){

    static float gyroNorm[9] = {0.0};
    float mean = 0.0;

    for (int i = 1; i < 9; ++i) {
        gyroNorm[i-1] = gyroNorm[i];
    }
    gyroNorm[8] = sqrt(gyro[0]*gyro[0]+gyro[1]*gyro[1]+gyro[2]*gyro[2]);

    if(gyroNorm[8] > ZERO_EPS){
        for(int i = 0; i < 9; i++){
            mean += gyroNorm[i];
        }
        mean /= 9.0;

        *var = 0.0;
        for(int j = 0; j < 9; j++){
            float tmp = gyroNorm[j] - mean;
            *var += pow(tmp,2.0);
        }
        *var /= 9;
    }
}

void calibrationImuData(ImuDataI imudata, float tempature){

    float gyro_norm = sqrt(imudata.gx*imudata.gx + imudata.gy*imudata.gy + imudata.gz*imudata.gz);
    float acc_norm = sqrt(imudata.ax*imudata.ax + imudata.ay*imudata.ay + imudata.az*imudata.az);

    static float static_gx_buffer[300] = {0.0};
    static float static_gy_buffer[300] = {0.0};
    static float static_gz_buffer[300] = {0.0};
    static float static_tempature_buffer[300] = {0.0};
    static float gyro_norm_buf[7] = {0.0};
    static float acc_norm_buf[7] = {0.0};
    static float static_gx_sum = 0.0;
    static float static_gy_sum = 0.0;
    static float static_gz_sum = 0.0;
    static int static_bias_count = 0;
    static float static_tempature_sum = 0.0;

    float gyro_var = 0.0;
    float acc_var = 0.0;
    int buf_size = 7;
    for(int i = 1; i < buf_size; i++){
        gyro_norm_buf[i-1] = gyro_norm_buf[i];
        acc_norm_buf[i-1] = acc_norm_buf[i];
    }
    gyro_norm_buf[buf_size-1] = gyro_norm;
    acc_norm_buf[buf_size-1] = acc_norm;

    if(fabs(gyro_norm_buf[0]) > ZERO_EPS){
        calcSensorVar(gyro_norm_buf, &gyro_var);
        calcSensorVar(acc_norm_buf, &acc_var);

        if(gyro_norm_buf[3] < 3.0 && acc_norm_buf[3] > 0.9 && acc_norm_buf[3] < 1.1 &&
           gyro_var < 3.0 && acc_var < 3.0 && gyro_var > 0.0 && acc_var > 0.0){
            for(int i = 1; i < static_buf_size_; i++){
                static_gx_buffer[i-1] = static_gx_buffer[i];
                static_gy_buffer[i-1] = static_gy_buffer[i];
                static_gz_buffer[i-1] = static_gz_buffer[i];
                static_tempature_buffer[i-1] = static_tempature_buffer[i];
            }
            static_gx_buffer[static_buf_size_-1] = imudata.gx;
            static_gy_buffer[static_buf_size_-1] = imudata.gy;
            static_gz_buffer[static_buf_size_-1] = imudata.gz;
            static_tempature_buffer[static_buf_size_-1] = tempature;
            static_bias_count += 1;
            static_gx_sum += imudata.gx;
            static_gy_sum += imudata.gy;
            static_gz_sum += imudata.gz;
            static_tempature_sum += tempature;

            if(static_bias_count >= static_buf_size_){
                average_tempature_ = static_tempature_sum / static_bias_count;
                static_bias_count -= 1;
                static_gx_sum -= static_gx_buffer[0];
                static_gy_sum -= static_gy_buffer[0];
                static_gz_sum -= static_gz_buffer[0];
                static_tempature_sum -= static_tempature_buffer[0];
            }

            static_gx_bias_ = static_gx_sum / static_bias_count;
            static_gy_bias_ = static_gy_sum / static_bias_count;
            static_gz_bias_ = static_gz_sum / static_bias_count;
            if(fabs(static_gx_bias_) > 5.0 || fabs(static_gy_bias_) > 5.0 || fabs(static_gz_bias_) > 5.0){
                static_gx_bias_ = 0.0;
                static_gy_bias_ = 0.0;
                static_gz_bias_ = 0.0;
            }
        }
    }
}

void imuBiasDataBase(int imuNumber){

    int index = 0;

    if(1 == imuNumber){
        tempatureBuff_[0] = -36.4615f;tempatureBuff_[1] = -33.5885f;tempatureBuff_[2] = -29.0505f;
        tempatureBuff_[3] = -24.588f;tempatureBuff_[4] = -17.7935f;tempatureBuff_[5] = -13.8991f;
        tempatureBuff_[6] = -9.179299;tempatureBuff_[7] = -2.598381;tempatureBuff_[8] = 1.396730;
        tempatureBuff_[9] = 5.607775;tempatureBuff_[10] = 10.258026;tempatureBuff_[11] = 15.593785;
        tempatureBuff_[12] = 20.322643;tempatureBuff_[13] = 24.962489;tempatureBuff_[14] = 30.310453;
        tempatureBuff_[15] = 35.642604;tempatureBuff_[16] = 39.432188;tempatureBuff_[17] = 45.035220;
        tempatureBuff_[18] = 50.481319;tempatureBuff_[19] = 55.244711;tempatureBuff_[20] = 60.346200;
        tempatureBuff_[21] = 66.445561;tempatureBuff_[22] = 70.408405;

        gxBiasBuff_[0] = -0.29262f;gxBiasBuff_[1] = -0.25537f;gxBiasBuff_[2] = -0.18703f;
        gxBiasBuff_[3] = -0.13266f;gxBiasBuff_[4] = -0.064985f;gxBiasBuff_[5] = -0.027476f;
        gxBiasBuff_[6] = 0.004526;gxBiasBuff_[7] = 0.034076;gxBiasBuff_[8] = 0.030745;
        gxBiasBuff_[9] = 0.024123;gxBiasBuff_[10] = 0.065044;gxBiasBuff_[11] = 0.084225;
        gxBiasBuff_[12] = 0.078307;gxBiasBuff_[13] = 0.099822;gxBiasBuff_[14] = 0.105582;
        gxBiasBuff_[15] = 0.098860;gxBiasBuff_[16] = 0.081768;gxBiasBuff_[17] = 0.033692;
        gxBiasBuff_[18] = -0.039709;gxBiasBuff_[19] = -0.125426;gxBiasBuff_[20] = -0.188256;
        gxBiasBuff_[21] = -0.331108;gxBiasBuff_[22] = -0.396135;

        gyBiasBuff_[0] = 2.0368f;gyBiasBuff_[1] = 1.935f;gyBiasBuff_[2] = 1.7876f;
        gyBiasBuff_[3] = 1.647f;gyBiasBuff_[4] = 1.4391f;gyBiasBuff_[5] = 1.3218f;
        gyBiasBuff_[6] = 1.199238;gyBiasBuff_[7] = 1.092486;gyBiasBuff_[8] = 1.021086;
        gyBiasBuff_[9] = 0.954189;gyBiasBuff_[10] = 0.879621;gyBiasBuff_[11] = 0.849481;
        gyBiasBuff_[12] = 0.775070;gyBiasBuff_[13] = 0.700292;gyBiasBuff_[14] = 0.610431;
        gyBiasBuff_[15] = 0.507433;gyBiasBuff_[16] = 0.451626;gyBiasBuff_[17] = 0.309099;
        gyBiasBuff_[18] = 0.173057;gyBiasBuff_[19] = 0.052195;gyBiasBuff_[20] = -0.056320;
        gyBiasBuff_[21] = -0.224979;gyBiasBuff_[22] = -0.330338;

        gzBiasBuff_[0] = -0.21294f;gzBiasBuff_[1] = -0.27808f;gzBiasBuff_[2] = -0.37179f;
        gzBiasBuff_[3] = -0.46263f;gzBiasBuff_[4] = -0.56579f;gzBiasBuff_[5] = -0.61196f;
        gzBiasBuff_[6] = -0.653613;gzBiasBuff_[7] = -0.684530;gzBiasBuff_[8] = -0.678930;
        gzBiasBuff_[9] = -0.679592;gzBiasBuff_[10] = -0.694130;gzBiasBuff_[11] = -0.675810;
        gzBiasBuff_[12] = -0.646996;gzBiasBuff_[13] = -0.676698;gzBiasBuff_[14] = -0.687537;
        gzBiasBuff_[15] = -0.663622;gzBiasBuff_[16] = -0.653216;gzBiasBuff_[17] = -0.627104;
        gzBiasBuff_[18] = -0.580061;gzBiasBuff_[19] = -0.494913;gzBiasBuff_[20] = -0.365378;
        gzBiasBuff_[21] = -0.162015;gzBiasBuff_[22] = -0.041704;

        axBiasBuff_[0] = -0.006458f;axBiasBuff_[1] = -0.0056057f;axBiasBuff_[2] = -0.0057639f;
        axBiasBuff_[3] = -0.0062098f;axBiasBuff_[4] = -0.0073693f;axBiasBuff_[5] = -0.0079156f;
        axBiasBuff_[6] = -0.008806;axBiasBuff_[7] = -0.010432;axBiasBuff_[8] = -0.011549;
        axBiasBuff_[9] = -0.010885;axBiasBuff_[10] = -0.011554;axBiasBuff_[11] = -0.012142;
        axBiasBuff_[12] = -0.012233;axBiasBuff_[13] = -0.012279;axBiasBuff_[14] = -0.012219;
        axBiasBuff_[15] = -0.012123;axBiasBuff_[16] = -0.012521;axBiasBuff_[17] = -0.012318;
        axBiasBuff_[18] = -0.012202;axBiasBuff_[19] = -0.012261;axBiasBuff_[20] = -0.012466;
        axBiasBuff_[21] = -0.012120;axBiasBuff_[22] = -0.012591;

        ayBiasBuff_[0] = 0.04582f;ayBiasBuff_[1] = 0.045058f;ayBiasBuff_[2] = 0.044438f;
        ayBiasBuff_[3] = 0.043626f;ayBiasBuff_[4] = 0.041381f;ayBiasBuff_[5] = 0.037243f;
        ayBiasBuff_[6] = 0.033051;ayBiasBuff_[7] = 0.023358;ayBiasBuff_[8] = 0.023384;
        ayBiasBuff_[9] = 0.021929;ayBiasBuff_[10] = 0.021518;ayBiasBuff_[11] = 0.021216;
        ayBiasBuff_[12] = 0.020378;ayBiasBuff_[13] = 0.019697;ayBiasBuff_[14] = 0.019050;
        ayBiasBuff_[15] = 0.018286;ayBiasBuff_[16] = 0.018608;ayBiasBuff_[17] = 0.017681;
        ayBiasBuff_[18] = 0.016964;ayBiasBuff_[19] = 0.016210;ayBiasBuff_[20] = 0.015497;
        ayBiasBuff_[21] = 0.014812;ayBiasBuff_[22] = 0.014076;

        azBiasBuff_[0] = -0.020638f;azBiasBuff_[1] = -0.022353f;azBiasBuff_[2] = -0.025008f;
        azBiasBuff_[3] = -0.027442f;azBiasBuff_[4] = -0.030509f;azBiasBuff_[5] = -0.032053f;
        azBiasBuff_[6] = -0.033662;azBiasBuff_[7] = -0.035230;azBiasBuff_[8] = -0.036034;
        azBiasBuff_[9] = -0.036450;azBiasBuff_[10] = -0.036700;azBiasBuff_[11] = -0.036567;
        azBiasBuff_[12] = -0.036252;azBiasBuff_[13] = -0.037109;azBiasBuff_[14] = -0.037872;
        azBiasBuff_[15] = -0.038406;azBiasBuff_[16] = -0.038748;azBiasBuff_[17] = -0.039111;
        azBiasBuff_[18] = -0.038691;azBiasBuff_[19] = -0.038090;azBiasBuff_[20] = -0.036717;
        azBiasBuff_[21] = -0.034160;azBiasBuff_[22] = -0.032775;
    }

    if(2 == imuNumber){
        tempatureBuff_[0] = -35.273067;tempatureBuff_[1] = -30.908729;tempatureBuff_[2] = -26.168018;
        gxBiasBuff_[0] = 0.155717;gxBiasBuff_[1] = 0.137441;gxBiasBuff_[2] = 0.147380;
        gyBiasBuff_[0] = 1.870185;gyBiasBuff_[1] = 1.632953;gyBiasBuff_[2] = 1.430250;
        gzBiasBuff_[0] = -0.286372;gzBiasBuff_[1] = -0.170346;gzBiasBuff_[2] = -0.079209;
        axBiasBuff_[0] = 0.006264;axBiasBuff_[1] = 0.004399;axBiasBuff_[2] = 0.002926;
        ayBiasBuff_[0] = 0.029989;ayBiasBuff_[1] = 0.030590;ayBiasBuff_[2] = 0.031373;
        azBiasBuff_[0] = -0.008543;azBiasBuff_[1] = -0.011553;azBiasBuff_[2] = -0.014350;

        tempatureBuff_[3] = -20.419030;tempatureBuff_[4] = -15.832435;tempatureBuff_[5] = -10.725203;
        gxBiasBuff_[3] = 0.146421;gxBiasBuff_[4] = 0.147281;gxBiasBuff_[5] = 0.146981;
        gyBiasBuff_[3] = 1.171486;gyBiasBuff_[4] = 0.989372;gyBiasBuff_[5] = 0.807317;
        gzBiasBuff_[3] = 0.061758;gzBiasBuff_[4] = 0.120987;gzBiasBuff_[5] = 0.164319;
        axBiasBuff_[3] = -0.000075;axBiasBuff_[4] = -0.001360;axBiasBuff_[5] = -0.003454;
        ayBiasBuff_[3] = 0.032006;ayBiasBuff_[4] = 0.032515;ayBiasBuff_[5] = 0.033325;
        azBiasBuff_[3] = -0.017765;azBiasBuff_[4] = -0.020131;azBiasBuff_[5] = -0.022647;

        tempatureBuff_[6] = -5.936474;tempatureBuff_[7] = -1.184992;tempatureBuff_[8] = 3.681225;
        gxBiasBuff_[6] = 0.156607;gxBiasBuff_[7] = 0.145158;gxBiasBuff_[8] = 0.118477;
        gyBiasBuff_[6] = 0.675814;gyBiasBuff_[7] = 0.569703;gyBiasBuff_[8] = 0.484027;
        gzBiasBuff_[6] = 0.153691;gzBiasBuff_[7] = 0.128175;gzBiasBuff_[8] = 0.107162;
        axBiasBuff_[6] = -0.003367;axBiasBuff_[7] = -0.003275;axBiasBuff_[8] = -0.003184;
        ayBiasBuff_[6] = 0.034709;ayBiasBuff_[7] = 0.036126;ayBiasBuff_[8] = 0.037268;
        azBiasBuff_[6] = -0.024624;azBiasBuff_[7] = -0.026222;azBiasBuff_[8] = -0.027218;

        tempatureBuff_[9] = 8.560110;tempatureBuff_[10] = 13.311310;tempatureBuff_[11] = 18.105043;
        gxBiasBuff_[9] = 0.081031;gxBiasBuff_[10] = 0.045691;gxBiasBuff_[11] = 0.008245;
        gyBiasBuff_[9] = 0.408607;gyBiasBuff_[10] = 0.343528;gyBiasBuff_[11] = 0.288155;
        gzBiasBuff_[9] = 0.082212;gzBiasBuff_[10] = 0.058539;gzBiasBuff_[11] = 0.046260;
        axBiasBuff_[9] = -0.002945;axBiasBuff_[10] = -0.000989;axBiasBuff_[11] = -0.000022;
        ayBiasBuff_[9] = 0.038418;ayBiasBuff_[10] = 0.040705;ayBiasBuff_[11] = 0.042714;
        azBiasBuff_[9] = -0.027979;azBiasBuff_[10] = -0.028082;azBiasBuff_[11] = -0.027918;

        tempatureBuff_[12] = 22.688629;tempatureBuff_[13] = 27.704665;tempatureBuff_[14] = 32.661820;
        gxBiasBuff_[12] = -0.019064;gxBiasBuff_[13] = -0.031920;gxBiasBuff_[14] = -0.048127;
        gyBiasBuff_[12] = 0.232983;gyBiasBuff_[13] = 0.179536;gyBiasBuff_[14] = 0.139650;
        gzBiasBuff_[12] = 0.022210;gzBiasBuff_[13] = -0.020556;gzBiasBuff_[14] = -0.038474;
        axBiasBuff_[12] = -0.000083;axBiasBuff_[13] = -0.000864;axBiasBuff_[14] = -0.001180;
        ayBiasBuff_[12] = 0.044911;ayBiasBuff_[13] = 0.046644;ayBiasBuff_[14] = 0.047870;
        azBiasBuff_[12] = -0.028297;azBiasBuff_[13] = -0.029088;azBiasBuff_[14] = -0.029450;

        tempatureBuff_[15] = 37.633846;tempatureBuff_[16] = 42.526494;tempatureBuff_[17] = 48.557051;
        gxBiasBuff_[15] = -0.064327;gxBiasBuff_[16] = -0.089864;gxBiasBuff_[17] = -0.133542;
        gyBiasBuff_[15] = 0.099727;gyBiasBuff_[16] = 0.067696;gyBiasBuff_[17] = 0.032021;
        gzBiasBuff_[15] = -0.041224;gzBiasBuff_[16] = -0.029431;gzBiasBuff_[17] = 0.007198;
        axBiasBuff_[15] = -0.001527;axBiasBuff_[16] = -0.001859;axBiasBuff_[17] = -0.002731;
        ayBiasBuff_[15] = 0.048509;ayBiasBuff_[16] = 0.049322;ayBiasBuff_[17] = 0.047366;
        azBiasBuff_[15] = -0.029391;azBiasBuff_[16] = -0.029047;azBiasBuff_[17] = -0.027470;

        tempatureBuff_[18] = 52.951052;tempatureBuff_[19] = 57.721297;tempatureBuff_[20] = 62.940683;
        gxBiasBuff_[18] = -0.178807;gxBiasBuff_[19] = -0.239394;gxBiasBuff_[20] = -0.291698;
        gyBiasBuff_[18] = -0.008409;gyBiasBuff_[19] = -0.039004;gyBiasBuff_[20] = -0.061317;
        gzBiasBuff_[18] = 0.054055;gzBiasBuff_[19] = 0.075127;gzBiasBuff_[20] = 0.058954;
        axBiasBuff_[18] = -0.003087;axBiasBuff_[19] = -0.003893;axBiasBuff_[20] = -0.004570;
        ayBiasBuff_[18] = 0.049262;ayBiasBuff_[19] = 0.049931;ayBiasBuff_[20] = 0.050458;
        azBiasBuff_[18] = -0.026835;azBiasBuff_[19] = -0.025707;azBiasBuff_[20] = -0.024109;

        tempatureBuff_[21] = 67.768491;tempatureBuff_[22] = 72.765808;
        gxBiasBuff_[21] = -0.378660;gxBiasBuff_[22] = -0.461144;
        gyBiasBuff_[21] = -0.100609;gyBiasBuff_[22] = -0.131951;
        gzBiasBuff_[21] = 0.068366;gzBiasBuff_[22] = 0.064826;
        axBiasBuff_[21] = -0.004905;axBiasBuff_[22] = -0.004420;
        ayBiasBuff_[21] = 0.050672;ayBiasBuff_[22] = 0.049733;
        azBiasBuff_[21] = -0.022432;azBiasBuff_[22] = -0.020277;
    }

    if(3 == imuNumber){
        index = 0;
        tempatureBuff_[index] = -36.055841;tempatureBuff_[index+1] = -31.543063;tempatureBuff_[index+2] = -26.339923;
        gxBiasBuff_[index] = 0.279627;gxBiasBuff_[index+1] = 0.287357;gxBiasBuff_[index+2] = 0.355399;
        gyBiasBuff_[index] = 6.300263;gyBiasBuff_[index+1] = 3.635572;gyBiasBuff_[index+2] = 1.749340;
        gzBiasBuff_[index] = 0.477800;gzBiasBuff_[index+1] = 0.376442;gzBiasBuff_[index+2] = 0.275504;
        axBiasBuff_[index] = -0.002110;axBiasBuff_[index+1] = -0.002273;axBiasBuff_[index+2] = 0.000338;
        ayBiasBuff_[index] = 0.018785;ayBiasBuff_[index+1] = 0.018802;ayBiasBuff_[index+2] = 0.022731;
        azBiasBuff_[index] = 0.011109;azBiasBuff_[index+1] = 0.007896;azBiasBuff_[index+2] = 0.004334;

        index = 3;
        tempatureBuff_[index] = -21.285125;tempatureBuff_[index+1] = -16.537641;tempatureBuff_[index+2] = -11.599386;
        gxBiasBuff_[index] = 0.375379;gxBiasBuff_[index+1] = 0.372739;gxBiasBuff_[index+2] = 0.354084;
        gyBiasBuff_[index] = 2.376545;gyBiasBuff_[index+1] = 1.880360;gyBiasBuff_[index+2] = 1.471682;
        gzBiasBuff_[index] = 0.146949;gzBiasBuff_[index+1] = 0.044953;gzBiasBuff_[index+2] = -0.049669;
        axBiasBuff_[index] = -0.000321;axBiasBuff_[index+1] = -0.000768;axBiasBuff_[index+2] = -0.001258;
        ayBiasBuff_[index] = 0.022299;ayBiasBuff_[index+1] = 0.022038;ayBiasBuff_[index+2] = 0.021606;
        azBiasBuff_[index] = 0.001469;azBiasBuff_[index+1] = -0.001094;azBiasBuff_[index+2] = -0.003598;

        index = 6;
        tempatureBuff_[index] = -6.681222;tempatureBuff_[index+1] = -3.101739;tempatureBuff_[index+2] = 1.988743;
        gxBiasBuff_[index] = 0.364267;gxBiasBuff_[index+1] = 0.341428;gxBiasBuff_[index+2] = 0.048859;
        gyBiasBuff_[index] = 1.165476;gyBiasBuff_[index+1] = 0.956977;gyBiasBuff_[index+2] = 1.035393;
        gzBiasBuff_[index] = -0.111839;gzBiasBuff_[index+1] = -0.162352;gzBiasBuff_[index+2] = -0.659105;
        axBiasBuff_[index] = -0.001633;axBiasBuff_[index+1] = -0.002847;axBiasBuff_[index+2] = -0.000193;
        ayBiasBuff_[index] = 0.021106;ayBiasBuff_[index+1] = 0.018965;ayBiasBuff_[index+2] = 0.022990;
        azBiasBuff_[index] = -0.005589;azBiasBuff_[index+1] = -0.007040;azBiasBuff_[index+2] = -0.035466;

        index = 9;
        tempatureBuff_[index] = 7.396333;tempatureBuff_[index+1] = 12.193009;tempatureBuff_[index+2] = 16.788698;
        gxBiasBuff_[index] = 0.213072;gxBiasBuff_[index+1] = 0.184783;gxBiasBuff_[index+2] = 0.173061;
        gyBiasBuff_[index] = 0.598240;gyBiasBuff_[index+1] = 0.490921;gyBiasBuff_[index+2] = 0.402865;
        gzBiasBuff_[index] = -0.252711;gzBiasBuff_[index+1] = -0.255743;gzBiasBuff_[index+2] = -0.228232;
        axBiasBuff_[index] = -0.022532;axBiasBuff_[index+1] = -0.022732;axBiasBuff_[index+2] = -0.021868;
        ayBiasBuff_[index] = 0.033037;ayBiasBuff_[index+1] = 0.035050;ayBiasBuff_[index+2] = 0.035782;
        azBiasBuff_[index] = -0.010094;azBiasBuff_[index+1] = -0.010651;azBiasBuff_[index+2] = -0.010769;

        index = 12;
        tempatureBuff_[index] = 22.313140;tempatureBuff_[index+1] = 26.504474;tempatureBuff_[index+2] = 31.527031;
        gxBiasBuff_[index] = 0.170251;gxBiasBuff_[index+1] = 0.206771;gxBiasBuff_[index+2] = 0.228176;
        gyBiasBuff_[index] = 0.275186;gyBiasBuff_[index+1] = 0.162478;gyBiasBuff_[index+2] = 0.061706;
        gzBiasBuff_[index] = -0.218956;gzBiasBuff_[index+1] = -0.245774;gzBiasBuff_[index+2] = -0.269604;
        axBiasBuff_[index] = -0.022860;axBiasBuff_[index+1] = -0.021538;axBiasBuff_[index+2] = -0.020881;
        ayBiasBuff_[index] = 0.035752;ayBiasBuff_[index+1] = 0.035725;ayBiasBuff_[index+2] = 0.035311;
        azBiasBuff_[index] = -0.011262;azBiasBuff_[index+1] = -0.012040;azBiasBuff_[index+2] = -0.012808;

        index = 15;
        tempatureBuff_[index] = 37.094591;tempatureBuff_[index+1] = 41.982731;tempatureBuff_[index+2] = 47.146901;
        gxBiasBuff_[index] = 0.214179;gxBiasBuff_[index+1] = 0.200716;gxBiasBuff_[index+2] = 0.168656;
        gyBiasBuff_[index] = -0.044991;gyBiasBuff_[index+1] = -0.100061;gyBiasBuff_[index+2] = -0.165117;
        gzBiasBuff_[index] = -0.283446;gzBiasBuff_[index+1] = -0.276980;gzBiasBuff_[index+2] = -0.278585;
        axBiasBuff_[index] = -0.022576;axBiasBuff_[index+1] = -0.022074;axBiasBuff_[index+2] = -0.021597;
        ayBiasBuff_[index] = 0.034902;ayBiasBuff_[index+1] = 0.034456;ayBiasBuff_[index+2] = 0.033883;
        azBiasBuff_[index] = -0.013271;azBiasBuff_[index+1] = -0.013302;azBiasBuff_[index+2] = -0.012451;

        index = 18;
        tempatureBuff_[index] = 51.715028;tempatureBuff_[index+1] = 56.511412;tempatureBuff_[index+2] = 62.583203;
        gxBiasBuff_[index] = 0.128968;gxBiasBuff_[index+1] = 0.059125;gxBiasBuff_[index+2] = -0.008788;
        gyBiasBuff_[index] = -0.206377;gyBiasBuff_[index+1] = -0.241696;gyBiasBuff_[index+2] = -0.232156;
        gzBiasBuff_[index] = -0.260859;gzBiasBuff_[index+1] = -0.223552;gzBiasBuff_[index+2] = -0.147895;
        axBiasBuff_[index] = -0.021185;axBiasBuff_[index+1] = -0.020878;axBiasBuff_[index+2] = -0.017923;
        ayBiasBuff_[index] = 0.033479;ayBiasBuff_[index+1] = 0.032340;ayBiasBuff_[index+2] = 0.026103;
        azBiasBuff_[index] = -0.012042;azBiasBuff_[index+1] = -0.011330;azBiasBuff_[index+2] = -0.009805;

        index = 21;
        tempatureBuff_[index] = 66.750765;tempatureBuff_[index+1] = 71.627541;
        gxBiasBuff_[index] = -0.092465;gxBiasBuff_[index+1] = -0.186112;
        gyBiasBuff_[index] = -0.239688;gyBiasBuff_[index+1] = -0.213645;
        gzBiasBuff_[index] = -0.083783;gzBiasBuff_[index+1] = 0.013216;
        axBiasBuff_[index] = -0.004555;axBiasBuff_[index+1] = -0.004698;
        ayBiasBuff_[index] = 0.027028;ayBiasBuff_[index+1] = 0.026038;
        azBiasBuff_[index] = -0.007916;azBiasBuff_[index+1] = -0.006427;
    }

    if(4 == imuNumber){
        tempatureBuff_[0] = -34.9653f;tempatureBuff_[1] = -29.8152f;tempatureBuff_[2] = -24.8691f;
        tempatureBuff_[3] = -21.786f;tempatureBuff_[4] = -16.8338f;tempatureBuff_[5] = -12.0755f;
        tempatureBuff_[6] = -7.0888f;tempatureBuff_[7] = -2.4066f;tempatureBuff_[8] = 2.4697f;
        tempatureBuff_[9] = 7.7462f;tempatureBuff_[10] = 12.4975f;tempatureBuff_[11] = 17.0436f;
        tempatureBuff_[12] = 22.5124f;tempatureBuff_[13] = 26.7963f;tempatureBuff_[14] = 31.7928f;
        tempatureBuff_[15] = 37.2542f;tempatureBuff_[16] = 42.1429f;tempatureBuff_[17] = 46.9551f;
        tempatureBuff_[18] = 51.8349f;tempatureBuff_[19] = 56.6799f;tempatureBuff_[20] = 62.6014f;
        tempatureBuff_[21] = 66.8957f;tempatureBuff_[22] = 71.6934f;

        gxBiasBuff_[0] = 0.5952f;gxBiasBuff_[1] = 0.5812f;gxBiasBuff_[2] = 0.5726f;
        gxBiasBuff_[3] = 0.5736f;gxBiasBuff_[4] = 0.5826f;gxBiasBuff_[5] = 0.55529f;
        gxBiasBuff_[6] = 0.55075f;gxBiasBuff_[7] = 0.5412f;gxBiasBuff_[8] = 0.52291f;
        gxBiasBuff_[9] = 0.4996f;gxBiasBuff_[10] = 0.47806f;gxBiasBuff_[11] = 0.55117f;
        gxBiasBuff_[12] = 0.52282f;gxBiasBuff_[13] = 0.54604f;gxBiasBuff_[14] = 0.56212f;
        gxBiasBuff_[15] = 0.56702f;gxBiasBuff_[16] = 0.55948f;gxBiasBuff_[17] = 0.54278f;
        gxBiasBuff_[18] = 0.51731f;gxBiasBuff_[19] = 0.48235f;gxBiasBuff_[20] = 0.45024f;
        gxBiasBuff_[21] = 0.3898f;gxBiasBuff_[22] = 0.30721f;

        gyBiasBuff_[0] = 0.9783f;gyBiasBuff_[1] = 0.8520f;gyBiasBuff_[2] = 0.7273f;
        gyBiasBuff_[3] = 0.6236f;gyBiasBuff_[4] = 0.4834f;gyBiasBuff_[5] = 0.40823f;
        gyBiasBuff_[6] = 0.34159f;gyBiasBuff_[7] = 0.2933f;gyBiasBuff_[8] = 0.2472f;
        gyBiasBuff_[9] = 0.19339f;gyBiasBuff_[10] = 0.16008f;gyBiasBuff_[11] = 0.12543f;
        gyBiasBuff_[12] = 0.089277f;gyBiasBuff_[13] = 0.064457f;gyBiasBuff_[14] = 0.041182f;
        gyBiasBuff_[15] = 0.0012983f;gyBiasBuff_[16] = -0.033096f;gyBiasBuff_[17] = -0.071671f;
        gyBiasBuff_[18] = -0.10835f;gyBiasBuff_[19] = -0.15013f;gyBiasBuff_[20] = -0.18346f;
        gyBiasBuff_[21] = -0.21058f;gyBiasBuff_[22] = -0.22878f;

        gzBiasBuff_[0] = 0.7339f;gzBiasBuff_[1] = 0.5457f;gzBiasBuff_[2] = 0.3937f;
        gzBiasBuff_[3] = 0.3103f;gzBiasBuff_[4] = 0.2196f;gzBiasBuff_[5] = 0.06813f;
        gzBiasBuff_[6] = -0.0343f;gzBiasBuff_[7] = -0.1077f;gzBiasBuff_[8] = -0.1847f;
        gzBiasBuff_[9] = -0.15722f;gzBiasBuff_[10] = -0.13859f;gzBiasBuff_[11] = -0.041425f;
        gzBiasBuff_[12] = -0.034829f;gzBiasBuff_[13] = -0.050702f;gzBiasBuff_[14] = -0.060166f;
        gzBiasBuff_[15] = -0.076434f;gzBiasBuff_[16] = -0.070024f;gzBiasBuff_[17] = -0.033924f;
        gzBiasBuff_[18] = 0.039934f;gzBiasBuff_[19] = 0.15046f;gzBiasBuff_[20] = 0.36165f;
        gzBiasBuff_[21] = 0.53644f;gzBiasBuff_[22] = 0.74522f;

        axBiasBuff_[0] = -0.0092695f;axBiasBuff_[1] = -0.0102f;axBiasBuff_[2] = -0.0103f;
        axBiasBuff_[3] = -0.0108f;axBiasBuff_[4] = -0.0114f;axBiasBuff_[5] = -0.0118f;
        axBiasBuff_[6] = -0.0121f;axBiasBuff_[7] = -0.0125f;axBiasBuff_[8] = -0.01873f;
        axBiasBuff_[9] = -0.01489f;axBiasBuff_[10] = -0.016477f;axBiasBuff_[11] = -0.017854f;
        axBiasBuff_[12] = -0.018954f;axBiasBuff_[13] = -0.020179f;axBiasBuff_[14] = -0.022399f;
        axBiasBuff_[15] = -0.022514f;axBiasBuff_[16] = -0.02432f;axBiasBuff_[17] = -0.025971f;
        axBiasBuff_[18] = -0.026602f;axBiasBuff_[19] = -0.027062f;axBiasBuff_[20] = -0.027711f;
        axBiasBuff_[21] = -0.011486f;axBiasBuff_[22] = -0.012084f;

        ayBiasBuff_[0] = 0.033443f;ayBiasBuff_[1] = 0.0322f;ayBiasBuff_[2] = 0.0313f;
        ayBiasBuff_[3] = 0.0312f;ayBiasBuff_[4] = 0.031f;ayBiasBuff_[5] = 0.0305f;
        ayBiasBuff_[6] = 0.0302f;ayBiasBuff_[7] = 0.03023f;ayBiasBuff_[8] = 0.02955f;
        ayBiasBuff_[9] = 0.04241f;ayBiasBuff_[10] = 0.045134f;ayBiasBuff_[11] = 0.047512f;
        ayBiasBuff_[12] = 0.046901f;ayBiasBuff_[13] = 0.048599f;ayBiasBuff_[14] = 0.049511f;
        ayBiasBuff_[15] = 0.047896f;ayBiasBuff_[16] = 0.048603f;ayBiasBuff_[17] = 0.049358f;
        ayBiasBuff_[18] = 0.049358f;ayBiasBuff_[19] = 0.048556f;ayBiasBuff_[20] = 0.036677f;
        ayBiasBuff_[21] = 0.032679f;ayBiasBuff_[22] = 0.030974f;

        azBiasBuff_[0] = 0.0029595f;azBiasBuff_[1] = -3e-04f;azBiasBuff_[2] = -0.0032f;
        azBiasBuff_[3] = -0.0038f;azBiasBuff_[4] = -0.0064f;azBiasBuff_[5] = -0.0087f;
        azBiasBuff_[6] = -0.0109f;azBiasBuff_[7] = -0.01262f;azBiasBuff_[8] = -0.01410f;
        azBiasBuff_[9] = -0.01569f;azBiasBuff_[10] = -0.016509f;azBiasBuff_[11] = -0.016867f;
        azBiasBuff_[12] = -0.017566f;azBiasBuff_[13] = -0.018693f;azBiasBuff_[14] = -0.019787f;
        azBiasBuff_[15] = -0.019854f;azBiasBuff_[16] = -0.020033f;azBiasBuff_[17] = -0.019991f;
        azBiasBuff_[18] = -0.019705f;azBiasBuff_[19] = -0.019165f;azBiasBuff_[20] = -0.017438f;
        azBiasBuff_[21] = -0.01543f;azBiasBuff_[22] = -0.013712f;
    }
		  if(5 == imuNumber){
        index = 0;
        tempatureBuff_[index] = -34.5130;tempatureBuff_[index+1] = -29.9853;tempatureBuff_[index+2] = -24.9637;
        gxBiasBuff_[index] = 0.3776 ;gxBiasBuff_[index+1] = 0.3375 ;gxBiasBuff_[index+2] = 0.2320 ;
        gyBiasBuff_[index] = 0.2744 ;gyBiasBuff_[index+1] = 0.2300 ;gyBiasBuff_[index+2] = 0.1352 ;
        gzBiasBuff_[index] = -0.1308;gzBiasBuff_[index+1] = -0.1449;gzBiasBuff_[index+2] = -0.1482 ;
        axBiasBuff_[index] = -0.0043;axBiasBuff_[index+1] = -0.0053;axBiasBuff_[index+2] = -0.0060 ;
        ayBiasBuff_[index] = 0.0559 ;ayBiasBuff_[index+1] = 0.0523 ;ayBiasBuff_[index+2] = 0.0496 ;
        azBiasBuff_[index] = 0.0139 ;azBiasBuff_[index+1] = 0.0110 ;azBiasBuff_[index+2] = 0.0085 ;

        index = 3;
        tempatureBuff_[index] = -19.9511;tempatureBuff_[index+1] = -14.9489;tempatureBuff_[index+2] = -9.9688;
        gxBiasBuff_[index] = 0.1758 ;gxBiasBuff_[index+1] = 0.1431 ;gxBiasBuff_[index+2] = 0.1130 ;
        gyBiasBuff_[index] = 0.0789 ;gyBiasBuff_[index+1] = 0.0591 ;gyBiasBuff_[index+2] = 0.0535 ;
        gzBiasBuff_[index] = -0.1589;gzBiasBuff_[index+1] = -0.1628;gzBiasBuff_[index+2] = -0.1742;
        axBiasBuff_[index] = -0.0067;axBiasBuff_[index+1] = -0.0075;axBiasBuff_[index+2] = -0.0081;
        ayBiasBuff_[index] = 0.0477 ;ayBiasBuff_[index+1] = 0.0459 ;ayBiasBuff_[index+2] = 0.0439 ;
        azBiasBuff_[index] = 0.0057 ;azBiasBuff_[index+1] = 0.0033 ;azBiasBuff_[index+2] = 0.0011 ;

        index = 6;
        tempatureBuff_[index] = -4.9842;tempatureBuff_[index+1] = -0.0301;tempatureBuff_[index+2] = 5.0617;
        gxBiasBuff_[index] = 0.0671 ;gxBiasBuff_[index+1] = 0.0522 ;gxBiasBuff_[index+2] = 0.0597 ;
        gyBiasBuff_[index] = 0.0406 ;gyBiasBuff_[index+1] = 0.0525 ;gyBiasBuff_[index+2] = 0.0329 ;
        gzBiasBuff_[index] = -0.1856;gzBiasBuff_[index+1] = -0.1736;gzBiasBuff_[index+2] = -0.1525;
        axBiasBuff_[index] = -0.0089;axBiasBuff_[index+1] = -0.0099;axBiasBuff_[index+2] = -0.0107;
        ayBiasBuff_[index] = 0.0419 ;ayBiasBuff_[index+1] = 0.0399 ;ayBiasBuff_[index+2] = 0.0381 ;
        azBiasBuff_[index] = -0.0008;azBiasBuff_[index+1] = -0.0020;azBiasBuff_[index+2] = -0.0025;

        index = 9;
        tempatureBuff_[index] = 10.0474;tempatureBuff_[index+1] = 15.0757;tempatureBuff_[index+2] = 20.0454;
        gxBiasBuff_[index] = 0.0229 ;gxBiasBuff_[index+1] = 0.0465 ;gxBiasBuff_[index+2] = 0.0371;
        gyBiasBuff_[index] = 0.0309 ;gyBiasBuff_[index+1] = 0.0170 ;gyBiasBuff_[index+2] = -0.0138;
        gzBiasBuff_[index] = -0.1517;gzBiasBuff_[index+1] = -0.0937;gzBiasBuff_[index+2] = -0.0490;
        axBiasBuff_[index] = -0.0120;axBiasBuff_[index+1] = -0.0132;axBiasBuff_[index+2] = -0.0142;
        ayBiasBuff_[index] = 0.0367 ;ayBiasBuff_[index+1] = 0.0353 ;ayBiasBuff_[index+2] = 0.0374;
        azBiasBuff_[index] = -0.0027;azBiasBuff_[index+1] = -0.0026;azBiasBuff_[index+2] = -0.0026;

        index = 12;
        tempatureBuff_[index] = 25.0450;tempatureBuff_[index+1] = 30.0779;tempatureBuff_[index+2] = 35.0045;
        gxBiasBuff_[index] = 0.0094 ;gxBiasBuff_[index+1] = -0.0271 ;gxBiasBuff_[index+2] = -0.0043 ;
        gyBiasBuff_[index] = -0.0224;gyBiasBuff_[index+1] = -0.0324 ;gyBiasBuff_[index+2] = -0.0261 ;
        gzBiasBuff_[index] = -0.0290;gzBiasBuff_[index+1] = 0.0016  ;gzBiasBuff_[index+2] = -0.0026 ;
        axBiasBuff_[index] = -0.0155;axBiasBuff_[index+1] = -0.0166 ;axBiasBuff_[index+2] = -0.0173 ;
        ayBiasBuff_[index] = 0.0354 ;ayBiasBuff_[index+1] = 0.0333  ;ayBiasBuff_[index+2] = 0.0311 ;
        azBiasBuff_[index] = -0.0037;azBiasBuff_[index+1] = -0.0043 ;azBiasBuff_[index+2] = -0.0050 ;

        index = 15;
        tempatureBuff_[index] = 40.0485;tempatureBuff_[index+1] = 45.0568;tempatureBuff_[index+2] = 50.0681;
        gxBiasBuff_[index] = 0.0756 ;gxBiasBuff_[index+1] = 0.1467 ;gxBiasBuff_[index+2] = 0.1920;
        gyBiasBuff_[index] = -0.0159;gyBiasBuff_[index+1] = -0.0165;gyBiasBuff_[index+2] = -0.0333;
        gzBiasBuff_[index] = -0.0284;gzBiasBuff_[index+1] = -0.0468;gzBiasBuff_[index+2] = -0.0631 ;
        axBiasBuff_[index] = -0.0177;axBiasBuff_[index+1] = -0.0182;axBiasBuff_[index+2] = -0.0189 ;
        ayBiasBuff_[index] = 0.0289 ;ayBiasBuff_[index+1] = 0.0274 ;ayBiasBuff_[index+2] = 0.0265 ;
        azBiasBuff_[index] = -0.0056;azBiasBuff_[index+1] = -0.0057;azBiasBuff_[index+2] = -0.0058 ;

        index = 18;
        tempatureBuff_[index] = 55.0526;tempatureBuff_[index+1] = 60.0546;tempatureBuff_[index+2] = 65.0569;
        gxBiasBuff_[index] = 0.3019 ;gxBiasBuff_[index+1] = 0.3416 ;gxBiasBuff_[index+2] = 0.4774;
        gyBiasBuff_[index] = -0.0467;gyBiasBuff_[index+1] = -0.1135;gyBiasBuff_[index+2] = -0.1775;
        gzBiasBuff_[index] = -0.0958;gzBiasBuff_[index+1] = -0.1125;gzBiasBuff_[index+2] = -0.1171;
        axBiasBuff_[index] = -0.0197;axBiasBuff_[index+1] = -0.0205;axBiasBuff_[index+2] = -0.0215;
        ayBiasBuff_[index] = 0.0259 ;ayBiasBuff_[index+1] = 0.0255 ;ayBiasBuff_[index+2] = 0.0249 ;
        azBiasBuff_[index] = -0.0058;azBiasBuff_[index+1] = -0.0051;azBiasBuff_[index+2] = -0.0044;

        index = 21;
        tempatureBuff_[index] = 69.5845;tempatureBuff_[index+1] = 70;
        gxBiasBuff_[index] = 0.5566 ;gxBiasBuff_[index+1] = 0.5567;
        gyBiasBuff_[index] = -0.2675;gyBiasBuff_[index+1] = -0.2677;
        gzBiasBuff_[index] = -0.1065;gzBiasBuff_[index+1] = -0.1070;
        axBiasBuff_[index] = -0.0230;axBiasBuff_[index+1] = -0.0235;
        ayBiasBuff_[index] = 0.0248 ;ayBiasBuff_[index+1] = 0.02481 ;
        azBiasBuff_[index] = -0.0016;azBiasBuff_[index+1] = -0.0017;
    }
		 if(7 == imuNumber){
        index = 0;
        tempatureBuff_[index] = -36.4599;tempatureBuff_[index+1] =-35.0819;tempatureBuff_[index+2] = -30.0436;
        gxBiasBuff_[index] = 0.2206 ;gxBiasBuff_[index+1] = 0.2247 ;gxBiasBuff_[index+2] = 0.2132 ;
        gyBiasBuff_[index] = 0.1185 ;gyBiasBuff_[index+1] = 0.1322 ;gyBiasBuff_[index+2] = 0.0971 ;
        gzBiasBuff_[index] = -0.6919;gzBiasBuff_[index+1] = -0.6484;gzBiasBuff_[index+2] = -0.5210  ;
        axBiasBuff_[index] = -0.0173;axBiasBuff_[index+1] = -0.0155;axBiasBuff_[index+2] = -0.0165  ;
        ayBiasBuff_[index] = 0.0203 ;ayBiasBuff_[index+1] = 0.0244 ;ayBiasBuff_[index+2] = 0.0278 ;
        azBiasBuff_[index] = 0.0366 ;azBiasBuff_[index+1] = 0.0357 ;azBiasBuff_[index+2] = 0.0321 ;

        index = 3;
        tempatureBuff_[index] = -25.0525;tempatureBuff_[index+1] = -20.0545;tempatureBuff_[index+2] = -15.0266;
        gxBiasBuff_[index] = 0.1929 ;gxBiasBuff_[index+1] = 0.2035 ;gxBiasBuff_[index+2] = 0.2069 ;
        gyBiasBuff_[index] = 0.0454 ;gyBiasBuff_[index+1] = 0.0202 ;gyBiasBuff_[index+2] = 0.0088 ;
        gzBiasBuff_[index] = -0.4034;gzBiasBuff_[index+1] = -0.3017;gzBiasBuff_[index+2] = -0.2025 ;
        axBiasBuff_[index] = -0.0169;axBiasBuff_[index+1] = -0.0173; axBiasBuff_[index+2] = -0.0177;
        ayBiasBuff_[index] = 0.0285 ;ayBiasBuff_[index+1] = 0.0290 ;ayBiasBuff_[index+2] = 0.0290 ;
        azBiasBuff_[index] = 0.0282 ;azBiasBuff_[index+1] = 0.0252 ;azBiasBuff_[index+2] = 0.0222 ;

        index = 6;
        tempatureBuff_[index] = -10.0594;tempatureBuff_[index+1] = -5.0842;tempatureBuff_[index+2] = -0.0959;
        gxBiasBuff_[index] = 0.2367 ;gxBiasBuff_[index+1] = 0.2336 ;gxBiasBuff_[index+2] = 0.2409 ;
        gyBiasBuff_[index] = -0.0047 ;gyBiasBuff_[index+1] = -0.0262 ;gyBiasBuff_[index+2] = -0.0465 ;
        gzBiasBuff_[index] = -0.0848 ;gzBiasBuff_[index+1] = -0.0150 ;gzBiasBuff_[index+2] = 0.0610 ;
        axBiasBuff_[index] = -0.0180 ;axBiasBuff_[index+1] = -0.0183 ;axBiasBuff_[index+2] = -0.0178 ;
        ayBiasBuff_[index] = 0.0290 ;ayBiasBuff_[index+1] = 0.0291 ;ayBiasBuff_[index+2] = 0.0283 ;
        azBiasBuff_[index] = 0.0195 ;azBiasBuff_[index+1] = 0.0175 ;azBiasBuff_[index+2] = 0.0152 ;

        index = 9;
        tempatureBuff_[index] = 4.9603;tempatureBuff_[index+1] = 9.9467;tempatureBuff_[index+2] = 14.9547;
        gxBiasBuff_[index] = 0.2282 ;gxBiasBuff_[index+1] = 0.2128 ;gxBiasBuff_[index+2] = 0.2039 ;
        gyBiasBuff_[index] = -0.0760 ;gyBiasBuff_[index+1] = -0.1103 ;gyBiasBuff_[index+2] = -0.1505 ;
        gzBiasBuff_[index] = 0.1295 ;gzBiasBuff_[index+1] = 0.1711 ;gzBiasBuff_[index+2] = 0.2107 ;
        axBiasBuff_[index] = -0.0167 ;axBiasBuff_[index+1] = -0.0156 ;axBiasBuff_[index+2] = -0.0143 ;
        ayBiasBuff_[index] = 0.0272 ;ayBiasBuff_[index+1] = 0.0260 ;ayBiasBuff_[index+2] = 0.0246 ;
        azBiasBuff_[index] = 0.0146 ;azBiasBuff_[index+1] = 0.0140 ;azBiasBuff_[index+2] = 0.0137 ;

        index = 12;
        tempatureBuff_[index] = 19.9511;tempatureBuff_[index+1] = 24.9352;tempatureBuff_[index+2] = 29.9546;
        gxBiasBuff_[index] = 0.1789 ;gxBiasBuff_[index+1] = 0.1244  ;gxBiasBuff_[index+2] = 0.0664  ;
        gyBiasBuff_[index] = -0.2068 ;gyBiasBuff_[index+1] = -0.1857  ;gyBiasBuff_[index+2] = -0.1509  ;
        gzBiasBuff_[index] = 0.2456 ;gzBiasBuff_[index+1] = 0.2684  ;gzBiasBuff_[index+2] = 0.2773  ;
        axBiasBuff_[index] = -0.0104 ;axBiasBuff_[index+1] = -0.0097  ;axBiasBuff_[index+2] = -0.0106  ;
        ayBiasBuff_[index] = 0.0220 ;ayBiasBuff_[index+1] = 0.0210  ;ayBiasBuff_[index+2] = 0.0203 ;
        azBiasBuff_[index] = 0.0138 ;azBiasBuff_[index+1] = 0.0127  ;azBiasBuff_[index+2] = 0.0116  ;

        index = 15;
        tempatureBuff_[index] = 34.8931;tempatureBuff_[index+1] = 39.9223;tempatureBuff_[index+2] = 44.9228;
        gxBiasBuff_[index] = 0.0398 ;gxBiasBuff_[index+1] = 0.0300 ;gxBiasBuff_[index+2] = 0.0187;
        gyBiasBuff_[index] = -0.1269 ;gyBiasBuff_[index+1] = -0.1238 ;gyBiasBuff_[index+2] = -0.1275 ;
        gzBiasBuff_[index] = 0.2720 ;gzBiasBuff_[index+1] = 0.2639 ;gzBiasBuff_[index+2] = 0.2551  ;
        axBiasBuff_[index] = -0.0122 ;axBiasBuff_[index+1] = -0.0151 ;axBiasBuff_[index+2] = -0.0182  ;
        ayBiasBuff_[index] = 0.0192 ;ayBiasBuff_[index+1] = 0.0176 ;ayBiasBuff_[index+2] = 0.0153 ;
        azBiasBuff_[index] = 0.0108 ;azBiasBuff_[index+1] = 0.0103 ;azBiasBuff_[index+2] = 0.0098  ;

        index = 18;
        tempatureBuff_[index] = 49.9396;tempatureBuff_[index+1] = 54.8935;tempatureBuff_[index+2] = 59.9371;
        gxBiasBuff_[index] = 0.0015 ;gxBiasBuff_[index+1] = 0.0048 ;gxBiasBuff_[index+2] = -0.0031;
        gyBiasBuff_[index] = -0.1414 ;gyBiasBuff_[index+1] = -0.1458 ;gyBiasBuff_[index+2] = -0.1608 ;
        gzBiasBuff_[index] = 0.2295 ;gzBiasBuff_[index+1] = 0.1881 ;gzBiasBuff_[index+2] = 0.1080 ;
        axBiasBuff_[index] = -0.0245 ;axBiasBuff_[index+1] = -0.0357 ;axBiasBuff_[index+2] = -0.0404 ;
        ayBiasBuff_[index] = 0.0145 ;ayBiasBuff_[index+1] = 0.0144 ;ayBiasBuff_[index+2] = 0.0140 ;
        azBiasBuff_[index] = 0.0096 ;azBiasBuff_[index+1] = 0.0092 ;azBiasBuff_[index+2] = 0.0094 ;

        index = 21;
        tempatureBuff_[index] = 64.9236;tempatureBuff_[index+1] = 71.1641;
        gxBiasBuff_[index] = 0.0517 ;gxBiasBuff_[index+1] = 0.0730;
        gyBiasBuff_[index] = -0.1381 ;gyBiasBuff_[index+1] = -0.1417 ;
        gzBiasBuff_[index] = 0.0403 ;gzBiasBuff_[index+1] = -0.0664 ;
        axBiasBuff_[index] = -0.0370 ;axBiasBuff_[index+1] = -0.0271 ;
        ayBiasBuff_[index] = 0.0092 ;ayBiasBuff_[index+1] = 0.0183  ;
        azBiasBuff_[index] = 0.0097 ;azBiasBuff_[index+1] = 0.0063 ;
    }

		
}

int findIndex(float curTempature){

    for(int i = 1; i < 23; i++){
        if(curTempature < tempatureBuff_[i]){
            return i;
        }
    }
}

void temperatureCompensateBias(float temperature, int imuNumber){
		
    imuBiasDataBase(imuNumber);
    float min_tempature = tempatureBuff_[0];
    float max_tempature = tempatureBuff_[22];
		
    float k_gx = 0.0, k_gy = 0.0, k_gz = 0.0,k_az = 0.0,k_ax = 0.0,k_ay = 0.0;
    float b_gx = 0.0, b_gy = 0.0, b_gz = 0.0, b_az = 0.0, b_ax = 0.0, b_ay = 0.0;
		
    if(temperature > max_tempature){
        est_temp_gxBias_ = gxBiasBuff_[22];
        est_temp_gyBias_ = gyBiasBuff_[22];
        est_temp_gzBias_ = gzBiasBuff_[22];
        est_temp_axBias_ = axBiasBuff_[22];
        est_temp_ayBias_ = ayBiasBuff_[22];
        est_temp_azBias_ = azBiasBuff_[22];
        return;
    }
		
    if(temperature < min_tempature){
        est_temp_gxBias_ = gxBiasBuff_[0];
        est_temp_gyBias_ = gyBiasBuff_[0];
        est_temp_gzBias_ = gzBiasBuff_[0];
        est_temp_axBias_ = axBiasBuff_[0];
        est_temp_ayBias_ = ayBiasBuff_[0];
        est_temp_azBias_ = azBiasBuff_[0];
        return;
    }
		
    int index = findIndex(temperature);
    k_gx = (gxBiasBuff_[index] - gxBiasBuff_[index-1]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    b_gx = (tempatureBuff_[index]*gxBiasBuff_[index-1] - tempatureBuff_[index-1]*gxBiasBuff_[index]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    k_gy = (gyBiasBuff_[index] - gyBiasBuff_[index-1]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    b_gy = (tempatureBuff_[index]*gyBiasBuff_[index-1] - tempatureBuff_[index-1]*gyBiasBuff_[index]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    k_gz = (gzBiasBuff_[index] - gzBiasBuff_[index-1]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    b_gz = (tempatureBuff_[index]*gzBiasBuff_[index-1] - tempatureBuff_[index-1]*gzBiasBuff_[index]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    k_ax = (axBiasBuff_[index] - axBiasBuff_[index-1]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    b_ax = (tempatureBuff_[index]*axBiasBuff_[index-1] - tempatureBuff_[index-1]*axBiasBuff_[index]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    k_ay = (ayBiasBuff_[index] - ayBiasBuff_[index-1]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    b_ay = (tempatureBuff_[index]*ayBiasBuff_[index-1] - tempatureBuff_[index-1]*ayBiasBuff_[index]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    k_az = (azBiasBuff_[index] - azBiasBuff_[index-1]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);
    b_az = (tempatureBuff_[index]*azBiasBuff_[index-1] - tempatureBuff_[index-1]*azBiasBuff_[index]) / (tempatureBuff_[index] - tempatureBuff_[index-1]);

    est_temp_gxBias_ = k_gx * temperature + b_gx;
    est_temp_gyBias_ = k_gy * temperature + b_gy;
    est_temp_gzBias_ = k_gz * temperature + b_gz;
    est_temp_axBias_ = k_ax * temperature + b_ax;
    est_temp_ayBias_ = k_ay * temperature + b_ay;
    est_temp_azBias_ = k_az * temperature + b_az;
		
    if(fabs(est_temp_gxBias_) > 3.0 || fabs(est_temp_gyBias_) > 3.0 || fabs(est_temp_gzBias_) > 3.0){
        est_temp_gxBias_ = 0.0;
        est_temp_gyBias_ = 0.0;
        est_temp_gzBias_ = 0.0;
    }
		
    if(fabs(est_temp_axBias_) > 0.06 || fabs(est_temp_ayBias_) > 0.06 || fabs(est_temp_azBias_) > 0.06){
        est_temp_axBias_ = 0.0;
        est_temp_ayBias_ = 0.0;
        est_temp_azBias_ = 0.0;
    }
}

void ImuBiasCompensate(ImuDataI imudata, float temperature, int imuNumber){

    calibrationImuData(imudata, temperature);
    temperatureCompensateBias(temperature, imuNumber);

    if(fabs(static_gx_bias_) < ZERO_EPS && fabs(static_gy_bias_) < ZERO_EPS && fabs(static_gz_bias_) < ZERO_EPS){
        real_gx_bias_ = est_temp_gxBias_ * 0.7f;
        real_gy_bias_ = est_temp_gyBias_ * 0.7f;
        real_gz_bias_ = est_temp_gzBias_ * 0.7f;
    } else{
        if(fabs(est_temp_gxBias_) < ZERO_EPS && fabs(est_temp_gyBias_) < ZERO_EPS && fabs(est_temp_gzBias_) < ZERO_EPS){
            real_gx_bias_ = static_gx_bias_;
            real_gy_bias_ = static_gy_bias_;
            real_gz_bias_ = static_gz_bias_;
        } else{ // 温度估计bias有值,看静止估计的bias的温度和当前温度是否差异过大，如果是，则采用温度估计出的bias
            float delta_tempature = fabs(temperature - average_tempature_);
            if(delta_tempature > 3.0 || fabs(static_gz_bias_) < ZERO_EPS){
                real_gx_bias_ = est_temp_gxBias_ * 0.7f;
                real_gy_bias_ = est_temp_gyBias_ * 0.7f;
                real_gz_bias_ = est_temp_gzBias_ * 0.7f;
            } else{
                real_gx_bias_ = static_gx_bias_;
                real_gy_bias_ = static_gy_bias_;
                real_gz_bias_ = static_gz_bias_;
            }
        }
    }

    // real_gx_bias_ = est_temp_gxBias_ * 0.7f;
    // real_gy_bias_ = est_temp_gyBias_ * 0.7f;
    // real_gz_bias_ = est_temp_gzBias_ * 0.7f;

    real_ax_bias_ = est_temp_axBias_ * 0.5f;
    real_ay_bias_ = est_temp_ayBias_ * 0.5f;
    real_az_bias_ = est_temp_azBias_ * 0.5f;

}
