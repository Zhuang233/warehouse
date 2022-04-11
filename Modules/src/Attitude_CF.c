#include "Attitude_CF.h"
#include "math.h"

#define Kp 0.4f        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f       // integral gain governs rate of convergence of gyroscope biases

float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz,float dt)
{
  float norm;
	float halfT;
	float	hx, hy, hz, bx, bz;
  float vx, vy, vz;
	float wx, wy, wz;
  float ex, ey, ez;
	float t11,t12,t13,t23,t33;
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	halfT=dt*0.5f;
	
	if(ax==0&&ay==0&&az==0)
 		return;
	if(mx==0&&my==0&&mz==0)
 		return;
	 //���ݹ�һ��	
  norm = sqrt(ax*ax + ay*ay + az*az);      
  ax = ax/norm;
  ay = ay/norm;
  az = az/norm;
	
	norm = sqrt(mx*mx + my*my + mz*mz);      
  mx = mx/norm;
  my = my/norm;
  mz = mz/norm;
	
  // estimated direction of gravity and flux (v and w)   �����������������/��Ǩ
	//����������ת����������ϵ
  vx = 2.0f*(q1q3 - q0q2);																		//��Ԫ����xyz�ı�ʾ
  vy = 2.0f*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);  
		
	arm_sqrt_f32(hx*hx+hy*hy,&bx);
	bz = hz; 
	
	wx = 2.0f*bx*(0.5 - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5 - q1q1 - q2q2);  

  // error is sum of cross product between reference direction of fields and direction measured by sensors
//  ex = (ay*vz - az*vy) + (my*wz - mz*wy);                           					 //�������������õ���־������
//  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
//	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	ex = (ay*vz - az*vy) ;                           					 //�������������õ���־������
  ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
  exInt = exInt + ex * Ki;								  							//�������л���
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;
	
  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   										//�����PI�󲹳��������ǣ����������Ư��
  gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
		
  // integrate quaternion rate and normalise						  //��Ԫ�ص�΢�ַ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  arm_sqrt_f32(q0*q0+ q1*q1 + q2*q2 + q3*q3,&norm);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

	q[0]=q0;
	q[1]=q1;
	q[2]=q2;
	q[3]=q3;
	
	GetRotationMatrix(q);
	
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2*(q1*q2+q0*q3);
	t13=2*(q1*q3-q0*q2);
	t23=2*(q2*q3+q0*q1);
	t33=q0*q0+q3*q3-q1*q1-q2*q2;
	
//	Roll_cf=atan2(t23,t33);
//	Pitch_cf=asin(-t13);
	angles[0]=atan2(t23,t33);
	angles[1]=asin(-t13);
	angles[2]=atan2(t12,t11);
	Roll=angles[0]* 57.3;
	Pitch=angles[1]* 57.3;
	Yaw=angles[2]* 57.3;
	global_params.AttitudeActual.roll=Roll;
	global_params.AttitudeActual.pitch=Pitch;
	global_params.AttitudeActual.yaw=Yaw;
}

void AttitudeCF(float dt)
{
		
}
