#include "errorcor.h"
#include "math.h"
#include "adxl375.h"

#define ADXL375   0
#define asm330_1    1
#define asm330_2    2
#define asm330_3    3
#define asm330_4    4
#define asm330_m    5

#define f200g_1_2 1
#define f200g_3 0

float gx_cail = 0.0, gy_cail = 0.0, gz_cail = 0.0, ax_cail = 0.0, ay_cail = 0.0, az_cail = 0.0;
float acc_614[3] = {0.0}, gyro_[3] = {0.0}, acc_375[3] = {0.0};

/*******************************************************************************
*函数名:SixPosition_cor
*描述:加速度计安装误差补偿
*******************************************************************************/
void SixPosition_cor(float Ax,float Ay,float Az,float* ax,float* ay,float* az, int imu_num)
{
      float x,y,z;
if(imu_num == ADXL375) {
//    float acc_bias[3]={-0.219835816628611,	0.549652514071702,	1.08953186855679};  
//        float acc_scale[3] = {1.04090940960102,	1.07185090160774,	0.931854511596282};        
//          float NJZ[3][3]={
//              0.996957699506098,	0.00392382847549733,	-0.0455039786041953,
//-0.00935842075157238,	1.00051250918482,	0.0257070482566256,
//0.0655014481539445,	0.0220001250006562,	0.997559670103512
//        };
              float acc_bias[3]={-0.623683449278830,	-0.778472656218733,	0.243508444671498};   
            float acc_scale[3] = {1.0,	1.0,	1.0};
          float NJZ[3][3]={
             0.999425924976543,	-0.0281572957682858,	-0.0190469818518404,
             0.0211829978738351,	0.998894011533139,	-0.0144295895829922,
            -0.000414717702581355,	0.0362967830050903,	0.999498713734516
        };
        x = Ax/acc_scale[0] + acc_bias[0]*(-1);  
		y = Ay/acc_scale[1] + acc_bias[1]*(-1);
		z = Az/acc_scale[2] + acc_bias[2]*(-1);                                                                                                                                                                                                                                                                                                                                                                                                                     

  	 *ax = (NJZ[0][0] * x + NJZ[0][1] * y + NJZ[0][2]* z);//这九个数就是NJZ
	   *ay = (NJZ[1][0] * x + NJZ[1][1] * y + NJZ[1][2]* z);
	   *az = (NJZ[2][0] * x + NJZ[2][1] * y + NJZ[2][2]* z);
} else if(imu_num == asm330_1) {
	 float acc_bias[3] = {0.015189976000000,  0.004682116000000, -0.013843096000000};
	 float acc_scale[3] = {1.0,	1.0,	1.0};
	 float NJZ[3][3] = {1.004378070845542,-0.005153189359113, 0.008464900626479,
	0.004813053514455,  1.006530628357841,  -0.010912493397620,
	-0.006995234191916, 0.004944510495800, 0.999601415843009};

		x = Ax/acc_scale[0] + acc_bias[0]*(-1);  
		y = Ay/acc_scale[1] + acc_bias[1]*(-1);
		z = Az/acc_scale[2] + acc_bias[2]*(-1);                                                                                                                                                                                                                                                                                                                                                                                                                        

  	   *ax = (NJZ[0][0] * x + NJZ[0][1] * y + NJZ[0][2]* z);
	   *ay = (NJZ[1][0] * x + NJZ[1][1] * y + NJZ[1][2]* z);
	   *az = (NJZ[2][0] * x + NJZ[2][1] * y + NJZ[2][2]* z);
} else if (imu_num == asm330_2) {
	 float acc_bias[3] = {0.020922512000000,  0.000700036000000, -0.014289616000000};
	 float acc_scale[3] = {1.0,	1.0,	1.0};
	 float NJZ[3][3] = {0.998030063780578,-0.009952286056183, 0.001351558787115,
	0.010212746562426,  1.002005070096326,  0.001752117095849,
	-0.000924256913844, -0.007583272028035, 0.998741910807430};
 
		x = Ax/acc_scale[0] + acc_bias[0]*(-1);  
		y = Ay/acc_scale[1] + acc_bias[1]*(-1);
		z = Az/acc_scale[2] + acc_bias[2]*(-1);                                                                                                                                                                                                                                                                                                                                                                                                                        

  	 *ax =   (NJZ[0][0] * x + NJZ[0][1] * y + NJZ[0][2]* z);
	   *ay = (NJZ[1][0] * x + NJZ[1][1] * y + NJZ[1][2]* z);
	   *az = (NJZ[2][0] * x + NJZ[2][1] * y + NJZ[2][2]* z);
}  else if (imu_num == asm330_3) {

	 float acc_bias[3] = {0.015682124000000,  0.003081232000000, -0.022189360000000};
	  float acc_scale[3] = {1.0,	1.0,	1.0};
	 float NJZ[3][3] = {1.003708900663294,-0.008690122583058, -0.000828089062883,
	0.006790532532692,  1.003838989728755,  -0.017315935525998,
	0.003934356192309, 0.013025822086991, 1.001054364298530};

		x = Ax/acc_scale[0] + acc_bias[0]*(-1);  
		y = Ay/acc_scale[1] + acc_bias[1]*(-1);
		z = Az/acc_scale[2] + acc_bias[2]*(-1);                                                                                                                                                                                                                                                                                                                                                                                                                        

  	 *ax = (NJZ[0][0] * x + NJZ[0][1] * y + NJZ[0][2]* z);
	   *ay = (NJZ[1][0] * x + NJZ[1][1] * y + NJZ[1][2]* z);
	   *az = (NJZ[2][0] * x + NJZ[2][1] * y + NJZ[2][2]* z);
}  else if (imu_num == asm330_4) {
	 float acc_bias[3] = {0.017509440000000,  0.003976712000000, -0.016003960000000};
	 float acc_scale[3] = {1.0,	1.0,	1.0};
	 float NJZ[3][3] = {1.005401938943528,-0.005951795469679, 0.001402148265289,
	0.006905692780736,  1.010197087241986,  -0.004142806910396,
	-0.000322064802064, -0.006083613700723, 1.004659661501102};

		x = Ax/acc_scale[0] + acc_bias[0]*(-1);  
		y = Ay/acc_scale[1] + acc_bias[1]*(-1);
		z = Az/acc_scale[2] + acc_bias[2]*(-1);                                                                                                                                                                                                                                                                                                                                                                                                                        

  	 *ax = (NJZ[0][0] * x + NJZ[0][1] * y + NJZ[0][2]* z);
	   *ay = (NJZ[1][0] * x + NJZ[1][1] * y + NJZ[1][2]* z);
	   *az = (NJZ[2][0] * x + NJZ[2][1] * y + NJZ[2][2]* z);
}  
else if (imu_num == asm330_m) {
        float acc_bias[3]={0.017262807000, 0.002658605000, -0.016508888500};   
    float acc_scale[3] = {1,		 1,	1};
          float NJZ[3][3]={
          1.001913823197340,  -0.007159016396059,   0.005896017604202,
         0.006787809916864  , 1.005173089144834 , -0.005416709178512,
          -0.004223074450107,  -0.000979761953872 ,  1.000859460155896
        };

		x = Ax/acc_scale[0] + acc_bias[0]*(-1);  
		y = Ay/acc_scale[1] + acc_bias[1]*(-1);
		z = Az/acc_scale[2] + acc_bias[2]*(-1);                                                                                                                                                                                                                                                                                                                                                                                                                        

  	 *ax = (NJZ[0][0] * x + NJZ[0][1] * y + NJZ[0][2]* z);
	   *ay = (NJZ[1][0] * x + NJZ[1][1] * y + NJZ[1][2]* z);
	   *az = (NJZ[2][0] * x + NJZ[2][1] * y + NJZ[2][2]* z);
}

//*****************7号***********************

//	float lingpian[1][3]={-0.000602185482274316,	0.000636198094469414,	-0.00228596454777292
//};   
//  float NJZ[3][3]={1.00015784898685,	0.00173094441341829,	0.0423457742599735,
//-0.00198147821954412,	1.00038293338430,	0.0105888880810028,
//-0.0423460171359806,	-0.0108736399483623,	1.00150061433763
//};


}

  /******************************************************************************
*函数名:gyro_calibration
*描述:陀螺安装误差补偿
*******************************************************************************/
void gyro_calibration(float gyr_x, float gyr_y, float gyr_z, float *Gx, float *Gy, float *Gz, int imu_num)
{	
	float gyr_x0,gyr_y0,gyr_z0;
	float gyr_mx,gyr_my,gyr_mz;
    if(imu_num == asm330_1) {
	 float gyro_bias[3] = {1.449662892803043,  -0.697051518787883, 0.147170995303031};
	 float gyro_scale[3] ={ 1.0, 1.0,  1.0};
	 float B[3][3] = {0.999693493670233,-0.005616986279555, 0.005331230510739,
	0.005623292037710,  0.997855494870362,  -0.012194204309593,
	-0.008123299490098, 0.007579095215735, 0.997798982263081};


           gyr_x0 = gyr_x/gyro_scale[0] + gyro_bias[0]*(-1);			
            gyr_y0 = gyr_y/gyro_scale[1] + gyro_bias[1]*(-1);
            gyr_z0 = gyr_z/gyro_scale[2] + gyro_bias[2]*(-1);  

			gyr_mx = B[0][0] * gyr_x0 + B[0][1] * gyr_y0 + B[0][2]* gyr_z0 ;
			gyr_my = B[1][0] * gyr_x0 + B[1][1] * gyr_y0 + B[1][2]* gyr_z0 ;		 
			gyr_mz = B[2][0] * gyr_x0 + B[2][1] * gyr_y0 + B[2][2]* gyr_z0 ;

			*Gx = gyr_mx ;
			*Gy = gyr_my ;
			*Gz = gyr_mz ;
    } else if(imu_num == asm330_2 ) {
	 float gyro_bias[3] = {1.508376702878788,  -1.162421568333334, 0.277881440378785};
	 float gyro_scale[3] ={ 1.0, 1.0,  1.0};
	 float B[3][3] = {0.998274993714094,-0.009771644728815, 0.001172315235591,
	0.014956793356655,  0.998711825952320,  0.001685455623966,
	-0.002296492074005, -0.002869830799020, 0.995328425682072};


           gyr_x0 = gyr_x/gyro_scale[0] + gyro_bias[0]*(-1);			
            gyr_y0 = gyr_y/gyro_scale[1] + gyro_bias[1]*(-1);
            gyr_z0 = gyr_z/gyro_scale[2] + gyro_bias[2]*(-1);  

			gyr_mx = B[0][0] * gyr_x0 + B[0][1] * gyr_y0 + B[0][2]* gyr_z0 ;
			gyr_my = B[1][0] * gyr_x0 + B[1][1] * gyr_y0 + B[1][2]* gyr_z0 ;		 
			gyr_mz = B[2][0] * gyr_x0 + B[2][1] * gyr_y0 + B[2][2]* gyr_z0 ;

			*Gx = gyr_mx ;
			*Gy = gyr_my ;
			*Gz = gyr_mz ;
    } else if(imu_num == asm330_3 ) {

	 float gyro_bias[3] = {1.263199045075750,  -1.070940342272736, -0.196440146136369};
	float gyro_scale[3] ={ 1.0, 1.0,  1.0};
	 float B[3][3] = {0.998982271623604,-0.010835896122559, -0.003047336420389,
	0.010501738624183,  1.000100919945607,  -0.014015348893490,
	0.001050680758699, 0.014775931338171, 0.997216508920129};

           gyr_x0 = gyr_x/gyro_scale[0] + gyro_bias[0]*(-1);			
            gyr_y0 = gyr_y/gyro_scale[1] + gyro_bias[1]*(-1);
            gyr_z0 = gyr_z/gyro_scale[2] + gyro_bias[2]*(-1);  

			gyr_mx = B[0][0] * gyr_x0 + B[0][1] * gyr_y0 + B[0][2]* gyr_z0 ;
			gyr_my = B[1][0] * gyr_x0 + B[1][1] * gyr_y0 + B[1][2]* gyr_z0 ;		 
			gyr_mz = B[2][0] * gyr_x0 + B[2][1] * gyr_y0 + B[2][2]* gyr_z0 ;

			*Gx = gyr_mx ;
			*Gy = gyr_my ;
			*Gz = gyr_mz ;
    } else if(imu_num == asm330_4 ) {
	 float gyro_bias[3] = {1.038013868787876,  -0.626160606060614, 0.154288994090904};
	float gyro_scale[3] ={ 1.0, 1.0,  1.0};
	 float B[3][3] = {1.001863664839598,-0.006551929254504, 0.004228152081962,
	0.005573049636511,  0.999731897354547,  -0.007411188712131,
	-0.000039435239861, 0.000523032063423, 0.997907791550423};

           gyr_x0 = gyr_x/gyro_scale[0] + gyro_bias[0]*(-1);			
            gyr_y0 = gyr_y/gyro_scale[1] + gyro_bias[1]*(-1);
            gyr_z0 = gyr_z/gyro_scale[2] + gyro_bias[2]*(-1);  

			gyr_mx = B[0][0] * gyr_x0 + B[0][1] * gyr_y0 + B[0][2]* gyr_z0 ;
			gyr_my = B[1][0] * gyr_x0 + B[1][1] * gyr_y0 + B[1][2]* gyr_z0 ;		 
			gyr_mz = B[2][0] * gyr_x0 + B[2][1] * gyr_y0 + B[2][2]* gyr_z0 ;

			*Gx = gyr_mx ;
			*Gy = gyr_my ;
			*Gz = gyr_mz ;
    } 
	 else if(imu_num == asm330_m ) {
    float gyro_bias[3]={
           1.32558532920634,-0.924229775650793, 0.050620291206343
        };
        float gyro_scale[3] = {
      1,		1,		1
        };
	float B[3][3]={ 
   0.999972606368723,   -0.007668119500770,   0.005438960696805,
   0.008941113764550,   1.007596475428799,   -0.006802485371980,
   -0.005865139234438,  0.004159984103454 ,  0.997279059428313
                 };

           gyr_x0 = gyr_x/gyro_scale[0] + gyro_bias[0]*(-1);			
            gyr_y0 = gyr_y/gyro_scale[1] + gyro_bias[1]*(-1);
            gyr_z0 = gyr_z/gyro_scale[2] + gyro_bias[2]*(-1);  

			gyr_mx = B[0][0] * gyr_x0 + B[0][1] * gyr_y0 + B[0][2]* gyr_z0 ;
			gyr_my = B[1][0] * gyr_x0 + B[1][1] * gyr_y0 + B[1][2]* gyr_z0 ;		 
			gyr_mz = B[2][0] * gyr_x0 + B[2][1] * gyr_y0 + B[2][2]* gyr_z0 ;

			*Gx = gyr_mx ;
			*Gy = gyr_my ;
			*Gz = gyr_mz ;
    }

}

//*********************************转台标定总函数****************************************************//
void errorcor_1()
{
  SixPosition_cor(asm330_data_mean_.acc[0], asm330_data_mean_.acc[1], asm330_data_mean_.acc[2], 
                       &asm330_data_mean_.acc[0], &asm330_data_mean_.acc[1], &asm330_data_mean_.acc[2], asm330_m);
  gyro_calibration(asm330_data_mean_.gyro[0], asm330_data_mean_.gyro[1], asm330_data_mean_.gyro[2], &asm330_data_mean_.gyro[0], 
                        &asm330_data_mean_.gyro[1], &asm330_data_mean_.gyro[2], asm330_m);
}
void errorcor()
{ 
        float temp = 0.0f;
        // z = -z
     
        
//        SixPosition_cor(adxl375_regs_data.accel[0], adxl375_regs_data.accel[1], adxl375_regs_data.accel[2], 
//                        &adxl375_regs_data.accel[0], &adxl375_regs_data.accel[1], &adxl375_regs_data.accel[2], ADXL375);
//        
//        SixPosition_cor(asm330_data1_.acc[0], asm330_data1_.acc[1], asm330_data1_.acc[2], 
//                        &asm330_data1_.acc[0], &asm330_data1_.acc[1], &asm330_data1_.acc[2], asm330_1);
//        SixPosition_cor(asm330_data2_.acc[0], asm330_data2_.acc[1], asm330_data2_.acc[2], 
//                        &asm330_data2_.acc[0], &asm330_data2_.acc[1], &asm330_data2_.acc[2], asm330_2);
//        SixPosition_cor(asm330_data3_.acc[0], asm330_data3_.acc[1], asm330_data3_.acc[2], 
//                        &asm330_data3_.acc[0], &asm330_data3_.acc[1], &asm330_data3_.acc[2], asm330_3);
//        SixPosition_cor(asm330_data4_.acc[0], asm330_data4_.acc[1], asm330_data4_.acc[2], 
//                        &asm330_data4_.acc[0], &asm330_data4_.acc[1], &asm330_data4_.acc[2], asm330_4);
//        gyro_calibration(asm330_data1_.gyro[0], asm330_data1_.gyro[1], asm330_data1_.gyro[2], &asm330_data1_.gyro[0], 
//                        &asm330_data1_.gyro[1], &asm330_data1_.gyro[2], asm330_1);
//        gyro_calibration(asm330_data2_.gyro[0], asm330_data2_.gyro[1], asm330_data2_.gyro[2], &asm330_data2_.gyro[0], 
//                        &asm330_data2_.gyro[1], &asm330_data2_.gyro[2], asm330_2);
//        gyro_calibration(asm330_data3_.gyro[0], asm330_data3_.gyro[1], asm330_data3_.gyro[2], &asm330_data3_.gyro[0], 
//                        &asm330_data3_.gyro[1], &asm330_data3_.gyro[2], asm330_3);
//        gyro_calibration(asm330_data4_.gyro[0], asm330_data4_.gyro[1], asm330_data4_.gyro[2], &asm330_data4_.gyro[0], 
//                        &asm330_data4_.gyro[1], &asm330_data4_.gyro[2], asm330_4);
								
								
		  asm330_data1_.acc[2] = -asm330_data1_.acc[2]; asm330_data2_.acc[2] = -asm330_data2_.acc[2]; 
        asm330_data3_.acc[2] = -asm330_data3_.acc[2]; asm330_data4_.acc[2] = -asm330_data4_.acc[2];
        
        asm330_data1_.gyro[2] = -asm330_data1_.gyro[2]; asm330_data2_.gyro[2] = -asm330_data2_.gyro[2]; 
        asm330_data3_.gyro[2] = -asm330_data3_.gyro[2]; asm330_data4_.gyro[2] = -asm330_data4_.gyro[2];
        
        adxl375_regs_data.accel[2] = -adxl375_regs_data.accel[2];
        //x = -y, y = -x;
        temp = asm330_data1_.acc[0]; asm330_data1_.acc[0] = -asm330_data1_.acc[1]; asm330_data1_.acc[1] = -temp;
        temp = asm330_data2_.acc[0]; asm330_data2_.acc[0] = -asm330_data2_.acc[1]; asm330_data2_.acc[1] = -temp;
        temp = asm330_data3_.acc[0]; asm330_data3_.acc[0] = -asm330_data3_.acc[1]; asm330_data3_.acc[1] = -temp;
        temp = asm330_data4_.acc[0]; asm330_data4_.acc[0] = -asm330_data4_.acc[1]; asm330_data4_.acc[1] = -temp;
        temp = asm330_data1_.gyro[0]; asm330_data1_.gyro[0] = -asm330_data1_.gyro[1]; asm330_data1_.gyro[1] = -temp;
        temp = asm330_data2_.gyro[0]; asm330_data2_.gyro[0] = -asm330_data2_.gyro[1]; asm330_data2_.gyro[1] = -temp;
        temp = asm330_data3_.gyro[0]; asm330_data3_.gyro[0] = -asm330_data3_.gyro[1]; asm330_data3_.gyro[1] = -temp;
        temp = asm330_data4_.gyro[0]; asm330_data4_.gyro[0] = -asm330_data4_.gyro[1]; asm330_data4_.gyro[1] = -temp;
        temp = adxl375_regs_data.accel[0]; adxl375_regs_data.accel[0] = -adxl375_regs_data.accel[1]; adxl375_regs_data.accel[1] = -temp;		
         
        SixPosition_cor(asm330_data1_.acc[0], asm330_data1_.acc[1], asm330_data1_.acc[2], 
                        &asm330_data1_.acc[0], &asm330_data1_.acc[1], &asm330_data1_.acc[2], asm330_1);
        SixPosition_cor(asm330_data2_.acc[0], asm330_data2_.acc[1], asm330_data2_.acc[2], 
                        &asm330_data2_.acc[0], &asm330_data2_.acc[1], &asm330_data2_.acc[2], asm330_2);
        SixPosition_cor(asm330_data3_.acc[0], asm330_data3_.acc[1], asm330_data3_.acc[2], 
                        &asm330_data3_.acc[0], &asm330_data3_.acc[1], &asm330_data3_.acc[2], asm330_3);
        SixPosition_cor(asm330_data4_.acc[0], asm330_data4_.acc[1], asm330_data4_.acc[2], 
                        &asm330_data4_.acc[0], &asm330_data4_.acc[1], &asm330_data4_.acc[2], asm330_4);
        gyro_calibration(asm330_data1_.gyro[0], asm330_data1_.gyro[1], asm330_data1_.gyro[2], &asm330_data1_.gyro[0], 
                        &asm330_data1_.gyro[1], &asm330_data1_.gyro[2], asm330_1);
        gyro_calibration(asm330_data2_.gyro[0], asm330_data2_.gyro[1], asm330_data2_.gyro[2], &asm330_data2_.gyro[0], 
                        &asm330_data2_.gyro[1], &asm330_data2_.gyro[2], asm330_2);
        gyro_calibration(asm330_data3_.gyro[0], asm330_data3_.gyro[1], asm330_data3_.gyro[2], &asm330_data3_.gyro[0], 
                        &asm330_data3_.gyro[1], &asm330_data3_.gyro[2], asm330_3);
        gyro_calibration(asm330_data4_.gyro[0], asm330_data4_.gyro[1], asm330_data4_.gyro[2], &asm330_data4_.gyro[0], 
                        &asm330_data4_.gyro[1], &asm330_data4_.gyro[2], asm330_4);
       
		  asm330_data1_.acc[2] = -asm330_data1_.acc[2]; asm330_data2_.acc[2] = -asm330_data2_.acc[2]; 
        asm330_data3_.acc[2] = -asm330_data3_.acc[2]; asm330_data4_.acc[2] = -asm330_data4_.acc[2];
        
        asm330_data1_.gyro[2] = -asm330_data1_.gyro[2]; asm330_data2_.gyro[2] = -asm330_data2_.gyro[2]; 
        asm330_data3_.gyro[2] = -asm330_data3_.gyro[2]; asm330_data4_.gyro[2] = -asm330_data4_.gyro[2]; 
		  
		  asm330_data1_.acc[0] = -asm330_data1_.acc[0]; asm330_data2_.acc[0] = -asm330_data2_.acc[0]; 
        asm330_data3_.acc[0] = -asm330_data3_.acc[0]; asm330_data4_.acc[0] = -asm330_data4_.acc[0];
        
        asm330_data1_.gyro[0] = -asm330_data1_.gyro[0]; asm330_data2_.gyro[0] = -asm330_data2_.gyro[0]; 
        asm330_data3_.gyro[0] = -asm330_data3_.gyro[0]; asm330_data4_.gyro[0] = -asm330_data4_.gyro[0]; 
        
}