#ifndef UAV_ADRC_H
#define UAV_ADRC_H


class _ADRC
{
	public:
  void Init(void);
	void Angle(float dt );
	void Angular(float dt);
	
	
	private:
};

extern _ADRC  Adrc_;

#endif