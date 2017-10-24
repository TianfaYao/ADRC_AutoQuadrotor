
#ifndef CAL_QUATERNION_H
#define CAL_QUATERNION_H
#include "cal_math.h"
#include <math.h>

class Quaternion
{
public:
    float        q1, q2, q3, q4;

    // 构造函数
    // roll=0, pitch=0, yaw=0
    Quaternion() {
        q1 = 1; q2 = q3 = q4 = 0;
    }

    //赋值构造函数
    Quaternion(const float _q1, const float _q2, const float _q3, const float _q4) :
        q1(_q1), q2(_q2), q3(_q3), q4(_q4) {
    }

    //函数调用操作符
    void operator        ()(const float _q1, const float _q2, const float _q3, const float _q4)
    {
        q1 = _q1; q2 = _q2; q3 = _q3; q4 = _q4;
    }
		
		//四元数归一化
		void normalize(void);
		
    // 返回该四元数的等效旋转矩阵
    void rotation_matrix(Matrix3f &m);

    // 返回该四元数的等效旋转矩阵中的重力分量
    void vector_gravity(Vector3f &v);
		
    // 将一个向量从地理坐标系转换到机体坐标系
    void earth_to_body(Vector3f &v);

		//一阶龙格库塔法更新四元数
		void Runge_Kutta_1st(Vector3f &g, float deltaT);
		
    //欧拉角转四元数
    void from_euler(float roll, float pitch, float yaw);

    //四元数转欧拉角
    void to_euler(float *roll, float *pitch, float *yaw);
};
#endif // QUATERNION_H
