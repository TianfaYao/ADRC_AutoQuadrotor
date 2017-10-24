#ifndef CAL_ADRC_H
#define CAL_ADRC_H
//TD 
typedef struct 
{
	float v1 ;
	float v2 ;
	float r;
	float h;
}Tracking_Diff;//Tracking  Differentiator 
//ESO
typedef struct 
{
  float z1;
	float z2;
	float z3;
	float h;
	float r;
	float c1;
	float u;
	float B1;
	float B2;
	float B3;
}Extend_St_Obs; //Extend State Observer


typedef struct 
{
  float z1;
	float z2;
	float h;
	float e;
	float u;
	float B;
}Extend_St_Obs2; //Extend State Observer


typedef struct 
{
	float bo;
	float b1;
	float b2;
  float e0;
	float e1;
	float e2;
	float u;
}lpid;

void TrackingDiffInit(Tracking_Diff &td,float r);
void TrackingDiffCal(Tracking_Diff &td,float vt,float dt);
void ExtendStObsInit(Extend_St_Obs &eso,float r,float c1);
void ExtendStObsInit2(Extend_St_Obs2 &eso,float h);
void ExtendStObsCal(Extend_St_Obs &eso,float v1,float v2,float x,float dt);
void dt2dtalp(float i_r,float i_h,float &i,float o_r,float o_h,float &o,lpid &lp );
void ExtendStObsCal2( Extend_St_Obs2 &eso, float i,float u,float B1,float B2,float drt,float eso_b,float eso_h);
#endif