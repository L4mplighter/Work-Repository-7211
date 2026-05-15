
#include "attitudeEstimator.h"
#include "matrix_ext.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	50.0			// sample frequency in Hz
#define twoKpDef	(2.0 * 5.8)	    // 2 * proportional gain
#define twoKiDef	(2.0 * 0.0)  	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

float twoKp_ = twoKpDef;											// 2 * proportional gain (Kp)
float twoKpz_ = twoKpDef;
float twoKi_ = twoKiDef;											// 2 * integral gain (Ki)
float q0_ = 1.0, q1_ = 0.0, q2_ = 0.0, q3_ = 0.0;					// quaternion of sensor frame relative to auxiliary frame
float integralFBx_ = 0.0,  integralFBy_ = 0.0, integralFBz_ = 0.0;	// integral error terms scaled by Ki
float euler_[3] = {0.0};

float roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;                //实时姿态角
int att_algo_init_flag_ = 0;
float pre_yaw_ = 0.0;
int static_count_ = 0;
int static_flag_count_ = 0;
float bias_sum_[3] = {0.0};
float gx_bias_ = 0.0;
float gy_bias_ = 0.0;
float gz_bias_ = 0.0;
float var_ = 0.0;

float x_[7] = {0.0};
float P_[49] = {0.0};
float predict_x_[7] = {0.0};
float dt_;

float Q_[9] = {0.0};
float R_[9] = {0.0};
float gravity_[3] = {0.0};
float tau_gyro_bias_;

int static_count_;
int static_flag_count_;
int mea_count_;
float bias_sum_[3];
int init_flag_;
int acc_valid_flag_;
int is_zupt_;
int count_;
int bias_count_ = 0;

float gx_bias_;
float gy_bias_;
float gz_bias_;
int algo_count_ = 0;
int algo_init_flag_ = 0;

float predict_x[7] = {0.0};
// Eigen::Matrix<float, 7, 7> predict_P;
float predict_P[49] = {0.0};
float version_ = 1.0;

//====================================================================================================
// Functions

 void getVersion(){
     version_ = 1.7;
 }

void initEKFParameters(){

    int i = 0;

    // P_ = Eigen::Matrix<float, 7, 7>::Identity() * 1.0e-1;
    for(i = 0; i < 7; i++){
        P_[8*i] = 1.0e-4;
    }
    // Q_  = Eigen::Matrix3d::Identity() * 0.08;
    for(i = 0; i < 3; i++){
        Q_[4*i] = 0.08;
    }
    // R_ = Eigen::Matrix3d::Identity() * 0.033;
    for(i = 0; i < 3; i++){
        R_[4*i] = 0.033;
    }
    // gravity_ = Eigen::Vector3d(0.0, 0.0, -9.79);
    gravity_[2] = -9.79;
    tau_gyro_bias_= 360000.0;

    /* x =[qw qx qy qz gyro_bias_x gyro_bias_y gyro_bias_z] */
    // x_ << 1, 0, 0, 0, 0, 0, 0;
    x_[0] = 1.0;
    // predict_x_ << 1, 0, 0, 0, 0, 0, 0;
    predict_x_[0] = 1.0;

//    sensor_var_file_ = fopen("/home/roman/work/data/602_debug/var.txt", "w");
//    sensor_filter_file_ = fopen("C:\\work\\rawdata\\20230331_160min_B305\\20230331_160min_B305\\1#\\fiterImu.txt", "w");

    static_count_ = 0;
    static_flag_count_ = 0;
    for(int i = 0; i < 3; i++){
        bias_sum_[i] = 0.0;
    }
    gx_bias_ = 0.0;
    gy_bias_ = 0.0;
    gz_bias_ = 0.0;

    acc_valid_flag_ = 0;
    init_flag_ = 0;
    is_zupt_ = 0;

//    pre_gx_output_ = 0.0;

    count_ = 0;
}

void initParameters(){

    roll_ = 0.0;pitch_ = 0.0;yaw_ = 0.0;
    q0_ = 1.0; q1_ = 0.0; q2_ = 0.0; q3_ = 0.0;
    integralFBx_ = 0.0;  integralFBy_ = 0.0; integralFBz_ = 0.0;
    for(int i = 0; i < 3; i++){
        euler_[i] = 0.0;
    }
}

void predictionUpdate(
        float * predict_x,
        float * predict_P,
        const float * gyro){

    int i = 0;
    float quat[4];
    // Eigen::Vector3d gyro_bias;
    float gyro_bias[3];

    if(1 == acc_valid_flag_){
        for(i = 0; i < 4; i++){
            quat[i] = x_[i];
        }
    }
    else{
        // quat = predict_x_.head<4>();
        for(i = 0; i < 4; i++){
            quat[i] = predict_x_[i];
        }
    }

    // gyro_bias = x_.tail<3>();
    for(i = 0; i < 3; i++){
        gyro_bias[i] = x_[4+i];
    }

    /* gyro2omega */
    // Eigen::Vector3d w = gyro - gyro_bias;
    float w[3] = {0.0};
    for(i = 0; i < 3; i++){
        w[i] = gyro[i] - gyro_bias[i];
    }
    // Eigen::Matrix4d omega;
    float omega[16];
    // omega <<
    //            0, -w[0], -w[1], -w[2],
    //         w[0],     0, w[2],  -w[1],
    //         w[1], -w[2],    0,   w[0],
    //         w[2], w[1], -w[0],    0;

    omega[0] = 0.0;
    omega[1] = w[0];
    omega[2] = w[1];
    omega[3] = w[2];
    omega[4] = -w[0];
    omega[5] = 0.0;
    omega[6] = -w[2];
    omega[7] = w[1];
    omega[8] = -w[1];
    omega[9] = w[2];
    omega[10] = 0.0;
    omega[11] = -w[0];
    omega[12] = -w[2];
    omega[13] = -w[1];
    omega[14] = w[0];
    omega[15] = 0.0;

    /* predict_x = f(x, w) */
    // Eigen::Vector4d predeted_quat;
    float predeted_quat[4], tmp_quat[4];
    // predeted_quat = quat + 0.5 * dt_ * omega * quat;
    matrixMul(1, 4, quat, 4, 4, omega, tmp_quat);
    matrixDelta(1, 4, 0.5 * dt_, tmp_quat, tmp_quat);
    matrixAdd(1, 4, quat, tmp_quat, predeted_quat);

    // predict_x.head<4>() = predeted_quat.normalized();
    float quat_norm = 0.0;
    quat_norm = 1.0 / sqrt(predeted_quat[0]*predeted_quat[0] + predeted_quat[1]*predeted_quat[1] + predeted_quat[2]*predeted_quat[2] + predeted_quat[3]*predeted_quat[3]);
    for(i = 0; i < 4; i++){
        predict_x[i] = predeted_quat[i] * quat_norm;
    }

    // predict_x.tail<3>() = gyro_bias;
    for(i = 0; i < 3; i++){
        predict_x[4+i] = gyro_bias[i];
    }

    // predict_x_ = predict_x;
    memcpy(predict_x_, predict_x, sizeof(predict_x_));
//    gx_bias_ = gyro_bias[0];
//    gy_bias_ = gyro_bias[1];
//    gz_bias_ = gyro_bias[2];

    /* predict_P = F P_ Ft + L Q_ Lt */
    /* F */
    // Eigen::Matrix<float, 7, 7> F;
    float F[49] = {0.0};
    // F = Eigen::Matrix<float, 7, 7>::Identity();
    for(i = 0; i < 7; i++){
        F[8*i] = 1.0;
    }
    // F.block<4, 4>(0, 0) += 0.5 * dt_ * omega;
    float tmp_omega[16] = {0.0};
    float tmp_F[16] = {0.0};
    float reu_F[16] = {0.0};
    matrixDelta(4, 4, 0.5*dt_, omega, tmp_omega);
    for(i = 0; i < 4; i++){
        tmp_F[5*i] = 1.0;
    }
    matrixAdd(4, 4, tmp_omega, tmp_F, reu_F);
    for(i = 0; i < 4; i++){
        F[i] = reu_F[i];
        F[i+7] = reu_F[4+i];
        F[i+14] = reu_F[8+i];
        F[i+21] = reu_F[12+i];
    }

    // F.block<4, 3>(0, 4) <<
    //         +quat[1], +quat[2], +quat[3],
    //         -quat[0], +quat[3], -quat[2],
    //         -quat[3], -quat[0], +quat[1],
    //         +quat[2], -quat[1], -quat[0];

    F[28] = +quat[1];
    F[29] = -quat[0];
    F[30] = -quat[3];
    F[31] = +quat[2];
    F[35] = +quat[2];
    F[36] = +quat[3];
    F[37] = -quat[0];
    F[38] = -quat[1];
    F[42] = +quat[3];
    F[43] = -quat[2];
    F[44] = +quat[1];
    F[45] = -quat[0];

    // F.block<4, 3>(0, 4) *= 0.5 * dt_;
    float tmp_F1[12] = {0.0};
    for(i = 0; i < 4; i++){
        tmp_F1[i] = F[28+i];
        tmp_F1[4+i] = F[35+i];
        tmp_F1[8+i] = F[42+i];
    }
    matrixDelta(4, 3, 0.5*dt_, tmp_F1, tmp_F1);
    for(i = 0; i < 4; i++){
        F[28+i] = tmp_F1[i];
        F[35+i] = tmp_F1[4+i];
        F[42+i] = tmp_F1[8+i];
    }
    // F.block<3, 3>(3, 3) -= Eigen::Matrix<float, 3, 3>::Identity() * dt_ / tau_gyro_bias_;
    float tmp_F2[9] = {0.0}, tmp1[9] = {0.0}, tmp2[9] = {0.0};
    for(i = 0; i < 3; i++){
        tmp1[4*i] = dt_ / tau_gyro_bias_;
    }
    for(i = 0; i < 3; i++){
        tmp_F2[i] = F[24+i];
        tmp_F2[3+i] = F[31+i];
        tmp_F2[6+i] = F[38+i];
    }
    matrixSub(3, 3, tmp_F2, tmp1, tmp2);
    for(i = 0; i < 3; i++){
        F[24+i] = tmp2[i];
        F[31+i] = tmp2[3+i];
        F[38+i] = tmp2[6+i];
    }

    // F(6,6) = 0; // gyro z轴bias没法估计
    F[48] = 0.0;
    /* L */
    // Eigen::Matrix<float, 7, 3> L;
    float L[21] = {0.0};
    // L <<
    //         -quat[1], -quat[2], -quat[3],
    //         +quat[0], -quat[3], +quat[2],
    //         +quat[3], +quat[0], -quat[1],
    //         -quat[2], +quat[1], +quat[0],
    //         0, 0, 0,
    //         0, 0, 0,
    //         0, 0, 0;
    L[0] = -quat[1];
    L[1] = +quat[0];
    L[2] = +quat[3];
    L[3] = -quat[2];
    L[7] = -quat[2];
    L[8] = -quat[3];
    L[9] = +quat[0];
    L[10] = +quat[1];
    L[14] = -quat[3];
    L[15] = +quat[2];
    L[16] = -quat[1];
    L[17] = +quat[0];

    // L *= 0.5 * dt_;
    matrixDelta(3, 7, 0.5*dt_, L, L);
    // predict_P = F * P_ * F.transpose() + L * Q_ * L.transpose();
    float tmp3[21] = {0.0}, tmp4[21] = {0.0};
    float tmp5[49] = {0.0};
    matrixTranspose(3, 7, L, tmp3);
    matrixMul(7, 3, tmp3, 3, 3, Q_, tmp4);
//    matrixMul(7, 3, L, 3, 7, tmp4, tmp5);
    matrixMul(7, 3, tmp4, 3, 7, L, tmp5);

    float tmp6[49] = {0.0}, tmp7[49] = {0.0};
    float tmp8[49] = {0.0};
    matrixTranspose(7, 7, F, tmp6);
//    matrixMul(7, 7, P_, 7, 7, tmp6, tmp7);
    matrixMul(7, 7, tmp6, 7, 7, P_, tmp7);
//    matrixMul(7, 7, F, 7, 7, tmp7, tmp8);
    matrixMul(7, 7, tmp7, 7, 7, F, tmp8);

    matrixAdd(7, 7, tmp8, tmp5, tmp6);
    memcpy(predict_P, tmp6, sizeof(tmp6));

}

void accObservationUpdate(
        const float * predict_x,
        const float * predict_P,
        const float * z){

    // Eigen::Vector4d predeted_quat = predict_x.head<4>();
    int i = 0;
    float predeted_quat[4] = {0.0};
    for(i = 0; i < 4; i++){
        predeted_quat[i] = predict_x[i];
    }

    /* y = z - h(x) */
    // Eigen::Vector3d acc;
    float acc[3] = {0.0};
    // Eigen::Quaternion<float> quat_tmp(predeted_quat[0], predeted_quat[1],
    //                                    predeted_quat[2], predeted_quat[3]);
    float quat_tmp[4];
    quat_tmp[0] = predeted_quat[0];
    quat_tmp[1] = -predeted_quat[1];
    quat_tmp[2] = -predeted_quat[2];
    quat_tmp[3] = -predeted_quat[3];

    // acc = quat_tmp.conjugate()._transformVector(gravity_);
    float mat[9];
    quaternion2RotationMatrix(quat_tmp, mat);
//    matrixMul(3, 3, mat, 3, 1, gravity_, acc);
    matrixMul(1, 3, gravity_, 3, 3, mat, acc);

    float accNorm = sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    float deltaAcc = fabs(accNorm - 9.79);

    // Eigen::Vector3d y;
    float y[3] = {0.0};
//    for(i = 0; i < 3; i++){
//        y[i] = z[i] - acc[i];
//    }
    if(deltaAcc < 5.0 || 0 == init_flag_){
        for(i = 0; i < 3; i++){
            y[i] = z[i] - acc[i];
        }
    }
//    y = z - acc;
    init_flag_ = 1;

    /* H */
    // Eigen::Matrix<float, 3, 7> H;
    float H[21] = {0.0};
    float qw = predeted_quat[0], qx = predeted_quat[1],
            qy = predeted_quat[2], qz = predeted_quat[3];
    // H <<
    //         -qy, +qz, -qw, +qx, 0, 0, 0,
    //         +qx, +qw, +qz, +qy, 0, 0, 0,
    //         +qw, -qx, -qy, +qz, 0, 0, 0;
    H[0] = -qy;
    H[1] = +qx;
    H[2] = +qw;
    H[3] = +qz;
    H[4] = +qw;
    H[5] = -qx;
    H[6] = -qw;
    H[7] = +qz;
    H[8] = -qy;
    H[9] = +qx;
    H[10] = +qy;
    H[11] = +qz;

    // H *= 2 * gravity_[2];
    matrixDelta(7, 3, 2.0*gravity_[2], H, H);

    /* x_ */
    // Eigen::Matrix<float, 7, 3> K = predict_P * H.transpose() *
    //                                 (H * predict_P * H.transpose() + R_).inverse();
    float K[21] = {0.0};
    float tmp1[21] = {0.0}, tmp2[21] = {0.0};
    float tmp3[9] = {0.0},tmp4[9] = {0.0};
    float tmp5[21] = {0.0};
    matrixTranspose(7, 3, H, tmp1);
//    matrixMul(7, 7, predict_P, 7, 3, tmp1, tmp2);
    matrixMul(3, 7, tmp1, 7, 7, predict_P, tmp2);
//    matrixMul(3, 7, H, 7, 3, tmp2, tmp3);
    matrixMul(3, 7, tmp2, 7, 3, H, tmp3);
    matrixAdd(3, 3, tmp3, R_, tmp3);
    matrixInverse(3, tmp3, tmp4);
//    matrixMul(7, 3, tmp1, 3, 3, tmp4, tmp5);
    matrixMul(3, 3, tmp4, 3, 7, tmp1, tmp5);
//    matrixMul(7, 7, predict_P, 7, 3, tmp5, K);
    matrixMul(3, 7, tmp5, 7, 7, predict_P, K);

    // Eigen::Matrix<float, 7, 1> x_tmp = predict_x + K * y;
    float x_tmp[7] = {0.0};
//    matrixMul(7, 3, K, 3, 1, y, x_tmp);
    matrixMul(1, 3, y, 3, 7, K, x_tmp);
    matrixAdd(7, 1, predict_x, x_tmp, x_tmp);

    // x_.head<4>() = x_tmp.head<4>().normalized();
    float quat_norm = 0.0;
    quat_norm = 1.0 / sqrt(x_tmp[0]*x_tmp[0] + x_tmp[1]*x_tmp[1] + x_tmp[2]*x_tmp[2] + x_tmp[3]*x_tmp[3]);
//    for(i = 0; i < 4; i++){
//        quat_norm += sqrt(x_tmp[i]*x_tmp[i]);
//    }
    for(i = 0; i < 4; i++){
        x_[i] = x_tmp[i] * quat_norm;
    }

    x_[4] = x_tmp[4];
    x_[5] = x_tmp[5];
//    x_.tail<3>() = x_tmp.tail<3>();

    /* P_ */
    // P_ = (Eigen::Matrix<float, 7, 7>::Identity() - K * H) * predict_P;
    float tmp6[49] = {0.0}, I[49] = {0.0};
    for(i = 0; i < 7; i++){
        I[8*i] = 1.0;
    }
//    matrixMul(7, 3, K, 3, 7, H, tmp6);
    matrixMul(7, 3, H, 3, 7, K, tmp6);
    matrixSub(7, 7, I, tmp6, tmp6);
//    matrixMul(7, 7, tmp6, 7, 7, predict_P, P_);
    matrixMul(7, 7, predict_P, 7, 7, tmp6, P_);
}

void attitude(
        float * quat,
        const float dt,
        const float * acc,
        const float * gyro,
        float * euler){

    // Eigen::Vector3d  tmp_gyro, tmp_acc;
    float tmp_gyro[3], tmp_acc[3];
    int i = 0;
    count_ ++;

    memset(predict_x, 0, sizeof(predict_x));
    memset(predict_P, 0, sizeof(predict_P));

    if(0 == algo_init_flag_){
        initEKFParameters();
        algo_init_flag_ = 1;
    }
    algo_count_ ++;
    float gyroNorm = sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

    tmp_gyro[0] = -gyro[1];
    tmp_gyro[1] = -gyro[0];
    tmp_gyro[2] = -gyro[2];
    tmp_acc[0] = -acc[1];
    tmp_acc[1] = -acc[0];
    tmp_acc[2] = -acc[2];

    float tmp_euler[3] = {0.0};
    float tmp_quat[4] = {0.0};
    float Cb2n[9] = {0.0};
    float Cn2b[9] = {0.0};
    float n_gyro[3] = {0.0};
    float cal_gyro[3] = {0.0};

    tmp_euler[0] = roll_;
    tmp_euler[1] = pitch_;
    euler2Quaternion(tmp_euler, tmp_quat);

    quaternion2RotationMatrix(tmp_quat, Cb2n);
    matrixMul(1, 3, tmp_gyro, 3, 3, Cb2n, n_gyro);
    n_gyro[1] -= 0.0041667 * DEG2RAD * cos(39.79636*DEG2RAD);
    n_gyro[2] -= 0.0041667 * DEG2RAD * sin(39.79636*DEG2RAD);
    matrixTranspose(3, 3, Cb2n, Cn2b);
    matrixMul(1, 3, n_gyro, 3, 3, Cn2b, cal_gyro);
    tmp_gyro[0] = cal_gyro[0];
    tmp_gyro[1] = cal_gyro[1];
    tmp_gyro[2] = cal_gyro[2];

    dt_ = dt;

    predictionUpdate(predict_x, predict_P, tmp_gyro);

    float accNorm = sqrt(tmp_acc[0]*tmp_acc[0] + tmp_acc[1]*tmp_acc[1] + tmp_acc[2]*tmp_acc[2]);
    float deltaAcc = fabs(accNorm - 9.79);

    float acc_th = 0.061;
    if(deltaAcc < acc_th && gyroNorm < 1.0){
        mea_count_ ++;
    }
    else{
        mea_count_ = 0;
    }
    if(count_ <= 200){
        setObservationNoize(0.01);
    }
    else{
        setObservationNoize(0.7);
    }

    accObservationUpdate(predict_x, predict_P,tmp_acc);
    acc_valid_flag_ = 1;

    float quat_tmp[4];
    if(1 == acc_valid_flag_){
        // Eigen::Quaternion<float> quat_tmp(x_[0], x_[1], x_[2], x_[3]);
        for(i = 0; i < 4; i++){
            quat_tmp[i] = x_[i];
            quat[i] = quat_tmp[i];
        }
        // quat = quat_tmp;
    }
    else{
        // Eigen::Quaternion<float> quat_tmp(predict_x_[0], predict_x_[1], predict_x_[2], predict_x_[3]);
        // quat = quat_tmp;
        for(i = 0; i < 4; i++){
            quat_tmp[i] = predict_x_[i];
            quat[i] = quat_tmp[i];
        }
    }

    quaternion2Euler(quat,euler);
    roll_ = euler[0] * RAD2DEG;
    pitch_ = euler[1] * RAD2DEG;

}

void setProcessNoize(const float process_noize){
    // Q_ = Eigen::Matrix3d::Identity() * process_noize;
    for(int i = 0; i < 3; i++){
        Q_[4*i] = process_noize;
    }
}

void setObservationNoize(const float observation_noize){
    // R_ = Eigen::Matrix3d::Identity() * observation_noize;
    for(int i = 0; i < 3; i++){
        R_[4*i] = observation_noize;
    }
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

float euler_ekf_[3] = {0.0};
void attitudeEKF(float gx, float gy, float gz, float ax, float ay, float az, float dt, float gz_scale) {

    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    if(0 == att_algo_init_flag_){
        initParameters();
        att_algo_init_flag_ = 1;
    }

    float tmp_gyro[3] = {0.0};
    float tmp_acc[3] = {0.0};
    tmp_gyro[0] = gx;
    tmp_gyro[1] = gy;
    tmp_gyro[2] = gz;
    tmp_acc[0] = ax * 9.89;
    tmp_acc[1] = ay * 9.89;
    tmp_acc[2] = az * 9.89;

    calcSensorVar(tmp_gyro, &var_);
    var_ *= 1e3;
    calcGyroBias(tmp_gyro,tmp_acc, var_);

//    gx -= gx_bias_;
//    gy -= gy_bias_;
    gz += gz_bias_ * 2.5;

//    gz *= gz_scale;

    float quat[4] = {0.0};

    attitude(quat,dt,tmp_acc,tmp_gyro,euler_ekf_);
    mahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
    return;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void mahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float tmp_quat[4] = {0.0};
    float tmp_euler[3] = {0.0};
    float Cb2n[9] = {0.0};
    float Cn2b[9] = {0.0};
    float gyro[3] = {0.0};
    float n_gyro[3] = {0.0};

    gyro[0] = gx;
    gyro[1] = gy;
    gyro[2] = gz;

    tmp_euler[0] = roll_;
    tmp_euler[1] = pitch_;
    tmp_euler[2] = 0;
    euler2Quaternion(tmp_euler, tmp_quat);
    quaternion2RotationMatrix(tmp_quat, Cb2n);
    matrixMul(1, 3, gyro, 3, 3, Cb2n, n_gyro);
    n_gyro[1] -= 0.0041667 * DEG2RAD * cos(39.79636*DEG2RAD);
    n_gyro[2] -= 0.0041667 * DEG2RAD * sin(39.79636*DEG2RAD);
    matrixTranspose(3, 3, Cb2n, Cn2b);
    matrixMul(1, 3, n_gyro, 3, 3, Cn2b, gyro);
    gx = gyro[0];
    gy = gyro[1];
    gz = gyro[2];

    float gyro_norm = 0.0;
    gyro_norm = sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
//    if(gyro_norm < 1.0 * DEG2RAD){
//        return;
//    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

        // Normalise accelerometer measurement
        recipNorm = sqrt(ax * ax + ay * ay + az * az);
        ax *= 1.0/recipNorm;
        ay *= 1.0/recipNorm;
        az *= 1.0/recipNorm;

        if(recipNorm > 11.0 || recipNorm < 8.5){
            twoKp_ = 20.0;
        } else{
            twoKp_ = 2.0 * 1.415;
        }

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1_ * q3_ - q0_ * q2_;
        halfvy = q0_ * q1_ + q2_ * q3_;
        halfvz = q0_ * q0_ - 0.5 + q3_ * q3_;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi_ > 0.0) {
            integralFBx_ += twoKi_ * halfex * (dt);	// integral error scaled by Ki
            integralFBy_ += twoKi_ * halfey * (dt);
            integralFBz_ += twoKi_ * halfez * (dt);
            gx += integralFBx_;	// apply integral feedback
            gy += integralFBy_;
            gz += integralFBz_;
        }
        else {
            integralFBx_ = 0.0;	// prevent integral windup
            integralFBy_ = 0.0;
            integralFBz_ = 0.0;
        }

        // Apply proportional feedback
        gx += twoKp_ * halfex;
        gy += twoKp_ * halfey;
        gz += twoKp_ * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5 * (dt));		// pre-multiply common factors
    gy *= (0.5 * (dt));
    gz *= (0.5 * (dt));
    qa = q0_;
    qb = q1_;
    qc = q2_;
    q0_ += (-qb * gx - qc * gy - q3_ * gz);
    q1_ += (qa * gx + qc * gz - q3_ * gy);
    q2_ += (qa * gy - qb * gz + q3_ * gx);
    q3_ += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= 1.0/recipNorm;
    q1_ *= 1.0/recipNorm;
    q2_ *= 1.0/recipNorm;
    q3_ *= 1.0/recipNorm;

    float quat[4] = {0.0};
    quat[0] = q0_;
    quat[1] = q1_;
    quat[2] = q2_;
    quat[3] = q3_;

    quaternion2Euler(quat, euler_);
//    roll_ = euler_[0] * RAD2DEG;
//    pitch_ = euler_[1] * RAD2DEG;
    yaw_ = euler_[2] * RAD2DEG;
}

//---------------------------------------------------------------------------------------------------
void quaternion2Euler(float *quat, float *euler){

    float q0q1 = quat[0] * quat[1];
    float q2q3 = quat[2] * quat[3];
    float q0q2 = quat[0] * quat[2];
    float q1q3 = quat[1] * quat[3];
    float q1q2 = quat[1] * quat[2];
    float q0q3 = quat[0] * quat[3];
    float q1q1 = quat[1] * quat[1];
    float q2q2 = quat[2] * quat[2];
    float q3q3 = quat[3] * quat[3];

    euler[0] = atan2(2.0*(q1q3-q0q2),(1.0-2.0*(q1q1+q2q2))); // roll [-pi, pi]
    euler[1] = -asin(2.0*(q2q3+q0q1)); // pitch [-pi/2, pi/2]
    euler[2] = -atan2(2.0*(q1q2-q0q3), 1.0-2.0*(q1q1+q3q3)); // yaw [-pi, pi]

//    if(euler[2] < ZERO_EPS){
//        euler[2] += 2*PI;
//    }
}

void euler2Quaternion(float *euler, float *quat){

    float roll = (euler[0]*DEG2RAD) / 2.0;
    float pitch = (euler[1]*DEG2RAD) / 2.0;
    float yaw = (euler[2]*DEG2RAD) / 2.0;

    quat[0] = cos(pitch) * cos(roll) * cos(yaw)   - sin(pitch) * sin(roll)  * sin(yaw);
    quat[1] = cos(roll)  * cos(yaw)  * sin(pitch) - cos(pitch) * sin(roll)  * sin(yaw);
    quat[2] = cos(pitch) * cos(yaw)  * sin(roll)  + cos(roll)  * sin(pitch) * sin(yaw);
    quat[3] = cos(pitch) * cos(roll) * sin(yaw)   + cos(yaw)   * sin(pitch) * sin(roll);
}

void quaternion2RotationMatrix(float *quat, float *mat){
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];

    mat[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    mat[3] = 2.0 * (q1*q2 - q0*q3);
    mat[6] = 2.0 * (q1*q3 + q0*q2);
    mat[1] = 2.0 * (q1*q2 + q0*q3);
    mat[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    mat[7] = 2.0 * (q2*q3 - q0*q1);
    mat[2] = 2.0 * (q1*q3 - q0*q2);
    mat[5] = 2.0 * (q2*q3 + q0*q1);
    mat[8] = q0*q0 - q1*q1 -q2*q2 + q3*q3;
}

void calcSensorVar(const float * data, float *var){

    static float gyroNorm[9] = {0.0};
    float mean = 0.0;

    for (int i = 1; i < 9; ++i) {
        gyroNorm[i-1] = gyroNorm[i];
    }
    gyroNorm[8] = sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]);

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

void calcGyroBias(const float * gyro,const float * acc, const float var){

    float gyro_norm = sqrt(gyro[0]*gyro[0] +
                           gyro[1]*gyro[1] +
                           gyro[2]*gyro[2]);
    if(var < BIASVAR && gyro_norm < GYROSTATIC){
        static_count_ ++;
        static_flag_count_ ++;
        bias_sum_[0] += gyro[0];
        bias_sum_[1] += gyro[1];
        bias_sum_[2] += gyro[2];

    }
		
    if(static_count_ >= STATICOUNT){
        gx_bias_ = bias_sum_[0] / static_count_;
        gy_bias_ = bias_sum_[1] / static_count_;
        gz_bias_ = bias_sum_[2] / static_count_;
//        static_count_ = 0;
    }
}

//====================================================================================================
// END OF CODE
//====================================================================================================
