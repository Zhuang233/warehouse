//#include "sensor.h"
//#include "math.h"

//float alt_update_dt=0.01;
//void GetSensorsData(void)
//{
////	static uint32_t time_now=0,time_last=0;
////	static uint8_t fun_first_run=1;
////	float dt;
//////	float velocity=0;
////	get_dt_in_seconds(&time_now,&time_last,&dt);
////	if(fun_first_run)
////	{
////		fun_first_run=0;
////		dt=0.01;
////	}
////	//get altitude from barometer.
////	alt_update_dt=dt;
////	GetNEDAcceleration();
////	if(baro_available)
////	{
////		altitude_baro=MS5611_get_height()-altitude_baro_ground;
////		global_params.AltitudeActual=baro_altitude_kalman_filter(ned_acc.z,altitude_baro,dt);
////	}
////	//update GPS meassage
////	GPS_Update();
////	position_estimate_EKF(ned_acc.x,ned_acc.y,gps.pos_x,gps.pos_y,dt);
////	RotateVelToBody(body_vel,EST_X[1],EST_Y[1]);
//}
