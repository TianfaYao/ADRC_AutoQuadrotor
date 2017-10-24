#ifndef 	DEV_MAVLINK_H
#define   DEV_MAVLINK_H
#include "stm32f4xx.h"
typedef struct
{
   u16 ch1;
	 u16 ch2;
	 u16 ch3;
	 u16 ch4;
	 u16 aux1;
	 u16 aux2;
	 u16 aux3;
	 u16 aux4;
	 u16 aux5;
	 u16 aux6;
}_Debug;

extern _Debug debug;


class _MAVLINK
{
	
public:
	
	
	void Data_Receive_Anl(u8 *data_buf,u8 num);
	//检查是否有接收到无线数据
	void Check_Event(void);
	//数据发送
	void Data_Exchange(void);
	//失控保护检查
	void Failsafe_Check(void);

	class flag{
		public:
		u8 send_version;
		u8 send_status;
		u8 send_senser;
		u8 send_senser2;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_rcdata;
		u8 send_offset;
		u8 send_motopwm;
		u8 send_user;
		u8 send_power;
	}f;
	
private:
		
	u8 data_to_send[50];

	void Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
	void Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
	void Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
	void Send_Senser2(s32 alt_bar,u16 alt_csb);
	void Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
	void Send_Power(u16 votage, u16 current);
	void Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
	void Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
	void Send_Check(u8 head, u8 check_sum);

	void Send_Data(u8 *dataToSend , u8 length);

};

extern _MAVLINK Mavlink;

#endif









