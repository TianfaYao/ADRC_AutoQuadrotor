#include "dev_gps.h"
#include <stdlib.h>

#define GPSUSART_MAX_RECV_LEN		800					//最大接收缓冲字节数
#define GPSUSART_RX_BUF_old_LEN 800
#define GPSUSART_MAX_SEND_LEN		300					//最大发送缓冲字节数
#define GPSUSART_RX_EN 					1						//0,不接收;1,接收.

nmea_msg gpsx; 											         //GPS信息

u8 GPSUSART_RX_ready =0;
u8 GPSUSART_RX_BUF[GPSUSART_MAX_RECV_LEN];
u8 GPSUSART_RX_BUF_old[GPSUSART_RX_BUF_old_LEN];
u8 GPSUSART_TX_BUF[GPSUSART_MAX_SEND_LEN]; 		//接收缓冲区，最大GPSUSART_MAX_RECV_LEN字节
uint16_t GPSUSART_RX_STA=0;

//不产生代码只是代码的搬运工
/*====================================================================================================*
**函数原型 :u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
**功    能 : 
**输    入 : None
**输    出 : None
**备    注 :  //从buf里面得到第cx个逗号所在的位置
             //返回值:0~0XFE,代表逗号所在位置的偏移.
            //       0XFF,代表不存在第cx个逗号			
**====================================================================================================*/
/*====================================================================================================*/				  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}


/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : u32 NMEA_Pow(u8 m,u8 n)
**功    能 : m^n次方.		
**输    入 : None
**输    出 : None
**备    注 :  //m^n函数
              //返回值:m^n次方.		
**====================================================================================================*/
/*====================================================================================================*/	

u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}



/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : int NMEA_Str2num(u8 *buf,u8*dx)
**功    能 : LED初始化
**输    入 : None
**输    出 : None
**备    注 :  //str转换为数字,以','或者'*'结束
             //buf:数字存储区
            //dx:小数点位数,返回给调用函数
           //返回值:转换后的数值	
**====================================================================================================*/
/*====================================================================================================*/	

int NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}




/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
**功    能 : GSV
**输    入 : None
**输    出 : None
**备    注 :  //分析GPGSV信息
             //gpsx:nmea信息结构体
            //buf:接收到的GPS数据缓冲区首地址
**====================================================================================================*/
/*====================================================================================================*/	

void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;   	 
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(u8*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}


/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
**功    能 : GGA
**输    入 : None
**输    出 : None
**备    注 :  //分析GPGGA信息
             //gpsx:nmea信息结构体
            //buf:接收到的GPS数据缓冲区首地址
**====================================================================================================*/
/*====================================================================================================*/	

void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GPGGA");
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}





/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
**功    能 : GSA
**输    入 : None
**输    出 : None
**备    注 :  //分析GPGSA信息
             //gpsx:nmea信息结构体
            //buf:接收到的GPS数据缓冲区首地址
**====================================================================================================*/
/*====================================================================================================*/	

void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx; 
	u8 i;   
	p1=(u8*)strstr((const char *)buf,"$GPGSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										      //得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}



/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
**功    能 : GPRMC
**输    入 : None
**输    出 : None
**备    注 :  //分析GPRMC信息
             //gpsx:nmea信息结构体
            //buf:接收到的GPS数据缓冲区首地址
**====================================================================================================*/
/*====================================================================================================*/	

void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;	   
	float rs;  
	p1=(u8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}



/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
**功    能 : GPVTG
**输    入 : None
**输    出 : None
**备    注 :  //分析GPVTG信息
             //gpsx:nmea信息结构体
            //buf:接收到的GPS数据缓冲区首地址
**====================================================================================================*/
/*====================================================================================================*/	

void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}
}  


/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
**功    能 : GPVTG
**输    入 : None
**输    出 : None
**备    注 :  
**====================================================================================================*/
/*====================================================================================================*/

void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
}


/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void Call_GPS(void)
**功    能 : GPS 数据解析函数
**输    入 : None
**输    出 : None
**备    注 :  
**====================================================================================================*/
/*====================================================================================================*/

void GPS_Call(void)
{
	  u16 i,rxlen;
    if(GPSUSART_RX_STA>400)//禁止连续进入
    {
    rxlen=GPSUSART_RX_STA;	//得到数据长度
    for(i=0;i<rxlen;i++)GPSUSART_RX_BUF_old[i]=GPSUSART_RX_BUF[i];
    GPSUSART_RX_STA=0;		   	//启动下一次接收
    GPSUSART_RX_BUF_old[i]=0;	//自动添加结束符
    GPS_Analysis(&gpsx,(u8*)GPSUSART_RX_BUF_old);//分析字符串
    }
	
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void GPS_DataCacheCall(u8 data)
**功    能 : GPS 数据缓存回调函数
**输    入 : 串口接收数据
**输    出 : None
**备    注 :  
**====================================================================================================*/
/*====================================================================================================*/
void GPS_DataCacheCall(u8 data)
{
		if(GPSUSART_RX_STA<GPSUSART_MAX_RECV_LEN)		//还可以接收数据
		{
			GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;		//记录接收到的值
		}
		else
		{
			GPSUSART_RX_STA=0;
			GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;		//记录接收到的值
		}
}


