#include "cal_vector3.h"
#include "AttitudeEKF.h"
#include "imu.h"
#include "cal_math.h"
//×ËÌ¬ÆÀ¹Àº¯Êý--ekf
void  EKF (Vector3f gyr,Vector3f acc,Vector3f mag,uint8_t update_vect[3]	,float dt )
{
    /* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
	float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
//	float x_aposteriori_k[12];		/**< states */
//	float P_aposteriori_k[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//				     0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//				     0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//				     0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
//				     0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
//				     0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
//				     0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
//				     0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
//				     0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
//				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
//				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
//				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f,
//				    }; /**< init: diagonal matrix with big values */

	static float x_aposteriori[12];
	static float P_aposteriori[144];

	/* output euler angles */
	float euler[3] = {0.0f, 0.0f, 0.0f};

  static 	float Rot_matrix[9] = {1.f,  0,  0,
			                           0,  1.f,  0,
			                           0,  0,  1.f
			                                      };		/**< init: identity matrix */
float debugOutput[4] = { 0.0f };


float q_rotSpeed=1e-4f;  //¹ý³ÌÔëÉù
float q_rotAcc=0.08f;
float q_acc=0.009f;	
float q_mag=0.005;//0.005f;
float r_gyro=0.0008f;   //²âÁ¿ÔëÉù
float r_accel=10000.0f;
float r_mag=100.f;  //100.0f

static u8 flag=0;
if(flag==0)
	/* Initialize filter */
 //	AttitudeEKF_initialize();
flag=1;



		z_k[0] =  gyr.x ; //²âÁ¿¾ØÕó
		z_k[1] =  gyr.y;
		z_k[2] =  gyr.z;


		z_k[3] = acc.x *9.81/4096;
		z_k[4] = acc.y *9.81/4096;
		z_k[5] = acc.z *9.81/4096;
		

float mag_nor=invSqrt(mag.x*mag.x+mag.y*mag.y+mag.z*mag.z)*100;
		z_k[6] = mag.x*mag_nor;
		z_k[7] = mag.y*mag_nor ;
		z_k[8] = mag.z*mag_nor ;
		
float J[9]={0};
			J[0]=0.0018;
			J[4]=0.0018;
			J[8]=0.0037;


		AttitudeEKF(false, // approx_prediction
				1,
				update_vect,
				dt,
				z_k,
				q_rotSpeed, // q_rotSpeed,
				q_rotAcc,   // q_rotAcc
			  q_acc,      // q_acc
				q_mag,      // q_mag
				r_gyro,     // r_gyro
				r_accel,    // r_accel
				r_mag,      // r_mag
				J,          //J
				x_aposteriori,
				P_aposteriori,
				Rot_matrix,
				euler,
				debugOutput);
				
			  
		    imu.angle.x= To_180_degrees(euler[0]* 57.29577951f -180);//½Ç¶È¹éµ½Õý¸º180
				imu.angle.y=euler[1]* 57.29577951f;
				imu.angle.z=euler[2]* 57.29577951f;
				
}