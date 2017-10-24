#include "cal_quaternion.h"


// 返回该四元数的等效旋转矩阵
void Quaternion::rotation_matrix(Matrix3f &m)
{
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q4q4 = q4 * q4;

    m.a.x = 1-2*(q3q3 + q4q4);
    m.a.y =   2*(q2q3 - q1q4);
    m.a.z =   2*(q2q4 + q1q3);
    m.b.x =   2*(q2q3 + q1q4);
    m.b.y = 1-2*(q2q2 + q4q4);
    m.b.z =   2*(q3q4 - q1q2);
    m.c.x =   2*(q2q4 - q1q3);
    m.c.y =   2*(q3q4 + q1q2);
    m.c.z = 1-2*(q2q2 + q3q3);
}

// 返回该四元数的等效旋转矩阵中的重力分量
void Quaternion::vector_gravity(Vector3f &v)
{
  v.x = 2*(q2*q4 - q1*q3);								
  v.y = 2*(q1*q2 + q3*q4);						  
  v.z = 1-2*(q2*q2 + q3*q3);
}

// 将一个向量从地理坐标系转换到机体坐标系
void Quaternion::earth_to_body(Vector3f &v)
{
    Matrix3f m;
    v.z = -v.z;
    rotation_matrix(m);
    v = m * v;
    v.z = -v.z;
}

//四元数归一化
void Quaternion::normalize(void)
{
	float length; 
	length= pythagorous4(q1, q2, q3, q4);
	q1 /= length;
	q2 /= length;
	q3 /= length;
	q4 /= length;
}

//一阶龙格库塔法更新四元数
void Quaternion::Runge_Kutta_1st(Vector3f &g, float deltaT)
{
  q1 += 0.5 * (-q2*g.x - q3*g.y - q4*g.z)* deltaT;
  q2 += 0.5 * (q1*g.x + q3*g.z - q4*g.y)* deltaT;
  q3 += 0.5 * (q1*g.y - q2*g.z + q4*g.x)* deltaT;
  q4 += 0.5 * (q1*g.z + q2*g.y - q3*g.x)* deltaT;	
}

//欧拉角转四元数
void Quaternion::from_euler(float roll, float pitch, float yaw)
{
    float cr2 = cosf(roll*0.5f);
    float cp2 = cosf(pitch*0.5f);
    float cy2 = cosf(yaw*0.5f);
    float sr2 = sinf(roll*0.5f);
    float sp2 = sinf(pitch*0.5f);
    float sy2 = sinf(yaw*0.5f);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

//四元数转欧拉角
void Quaternion::to_euler(float *roll, float *pitch, float *yaw)
{
    if (roll) {
        *roll = degrees(atan2f(2.0f*(q1*q2 + q3*q4),1 - 2.0f*(q2*q2 + q3*q3)));
    }
    if (pitch) {
        // 使用safe_asin()来处理pitch接近90/-90时的奇点
        *pitch = degrees(asin(2.0f*(q1*q3 - q2*q4)));
    }
    if (yaw) {
        *yaw = degrees(atan2f(2.0f*(q2*q3 - q1*q4), 2.0f*(q1*q1 + q2*q2) - 1));
    }
}











