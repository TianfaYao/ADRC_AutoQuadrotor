
/**
 * @file ekf_att_pos_estimator_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */


#include "AttitudePositionEstimatorEKF.h"
#include "estimator_22states.h"
#include <string.h>
#include "stdio.h"
#include "math.h"
#include "dev_mpu6050.h"
#include "dev_ak8975.h"
AttitudePositionEstimatorEKF  att_ekf;
#define   M_PI  3.14159
static uint64_t IMUusec = 0;
extern uint32_t GetSysTime_us(void) ;
//extern float radians(float deg);

#include "cal_math.h"
//Constants 
static const float rc = 10.0f;	// RC time constant of 1st order LPF in seconds
static const uint64_t FILTER_INIT_DELAY = 1 * 1000 * 1000;	///< units: microseconds
static const float POS_RESET_THRESHOLD = 5.0f;				///< Seconds before we signal a total GPS failure

// These are unused
#if 0
static const unsigned MAG_SWITCH_HYSTERESIS =
	10;	///< Ignore the first few mag failures (which amounts to a few milliseconds)
static const unsigned GYRO_SWITCH_HYSTERESIS =
	5;	///< Ignore the first few gyro failures (which amounts to a few milliseconds)
static const unsigned ACCEL_SWITCH_HYSTERESIS =
	5;	///< Ignore the first few accel failures (which amounts to a few milliseconds)
#endif

static const float EPH_LARGE_VALUE = 1000.0f;
static const float EPV_LARGE_VALUE = 1000.0f;

/**
 * estimator app start / stop handling function
 *
 * @ingroup apps
 */

AttPosEKF att_pos_ekf;

uint32_t millis();
uint64_t getMicros();
uint32_t getMillis();

uint32_t millis()
{
	return getMillis();
}
uint32_t getMillis()
{
	return getMicros() / 1000;
}

uint64_t getMicros()
{
	return IMUusec;
}
AttitudePositionEstimatorEKF::AttitudePositionEstimatorEKF() :

	_task_should_exit(false),
	_task_running(false),
	_estimator_task(-1),

	_prediction_steps(0),
	_prediction_last(0),

	_filter_ref_offset(0.0f),
	_baro_gps_offset(0.0f),

	 //states 
	_gps_alt_filt(0.0f),
	_baro_alt_filt(0.0f),
	_covariancePredictionDt(0.0f),
	_gpsIsGood(false),
	_previousGPSTimestamp(0),
	_baro_init(false),
	_gps_initialized(false),
	_filter_start_time(0),
	_last_sensor_timestamp(GetSysTime_us()),
	_distance_last_valid(0),
	_data_good(false),
	_ekf_logging(true),
	_debug(0),
	_was_landed(true),

	_newHgtData(false),
	_newAdsData(false),
	_newDataMag(false),
	_newRangeData(false)

	//_terrain_estimator(nullptr),
{
	//_terrain_estimator = new TerrainEstimator();

	// indicate consumers that the current position data is not valid 
	_gps.eph = 10000.0f;
	_gps.epv = 10000.0f;
	//fetch initial parameter values 
	parameters_update();
}
//参数初始化
int AttitudePositionEstimatorEKF::parameters_update()
{
  	if (1) {
		// yawVarScale = 1.0f;
		// windVelSigma = 0.1f;
	dAngBiasSigma = 1.0e-6;
	dVelBiasSigma = 0.0002f;
	magEarthSigma =  0.0003f;
	magBodySigma  =  0.0003f;
		// gndHgtSigma  = 0.02f;
	vneSigma = 		0.2f;
	vdSigma = 		0.3f;
	posNeSigma =  2.0f;
	posDSigma = 	2.0f;
		
	magMeasurementSigma = 			0.05;
	airspeedMeasurementSigma =  1.4f;
	gyroProcessNoise = 					1.4544411e-2f;
	accelProcessNoise = 				0.5f;

	rngFinderPitch = 0.0f; // XXX base on SENS_BOARD_Y_OFF
			
#if 0
		// Initially disable loading until
		// convergence is flight-test proven
	magBias.x = _mag_offset_x.get();
	magBias.y = _mag_offset_y.get();
	magBias.z = _mag_offset_z.get();
#endif
	}
	return 1;
}

//使能状态记录
int AttitudePositionEstimatorEKF::enable_logging(bool logging)
{
	_ekf_logging = logging;
	return 0;
}
//飞机起飞
void AttitudePositionEstimatorEKF::vehicle_status_poll()
{
	
	bool updated;
	if (updated) {
		// Tell EKF that the vehicle is a fixed wing or multi-rotor
	setIsFixedWing(!0);
	}
	
}
//飞机落地
void AttitudePositionEstimatorEKF::vehicle_land_detected_poll()
{
	
	bool updated=1;
	if (updated) { 
		// Save params on landed and previously not landed
		if (_vehicle_land_detected.landed && !_was_landed) {
			magBias.x=0;
			magBias.y=0;
			magBias.z=0;
		}
		_was_landed = _vehicle_land_detected.landed;
	}
	
}
//检查ekf 状态
int AttitudePositionEstimatorEKF::check_filter_state()
{
	/*
	 *    CHECK IF THE INPUT DATA IS SANE
	 */

	struct ekf_status_report ekf_report;

	int check = CheckAndBound(&ekf_report);

	const char *const feedback[] = { NULL,
					 "NaN in states, resetting",
					 "stale sensor data, resetting",
					 "got initial position lock",
					 "excessive gyro offsets",
					 "velocity diverted, check accel config",
					 "excessive covariances",
					 "unknown condition, resetting"
				       };

	// Print out error condition
	if (check) {
		unsigned warn_index = static_cast<unsigned>(check);
		unsigned max_warn_index = (sizeof(feedback) / sizeof(feedback[0]));
		if (max_warn_index < warn_index) {
			warn_index = max_warn_index;
		}
	}
	struct estimator_status_s rep;
	memset(&rep, 0, sizeof(rep));
	// If error flag is set, we got a filter reset
	if (check && ekf_report.error) {
		// Count the reset condition
		// GPS is in scaled integers, convert
		double lat = _gps.lat / 1.0e7;
		double lon = _gps.lon / 1.0e7;
		float gps_alt = _gps.alt / 1e3f;

		 // Set up height correctly
     //_baro.altitude 气压高度？？？
	   float  _baro_altitude=Ms5611.BaroAlt;
		initReferencePosition(_gps.timestamp, _gpsIsGood, lat, lon, gps_alt, _baro_altitude);

	} else if (_ekf_logging) {
	GetFilterState(&ekf_report);
	}

	if (_ekf_logging || check) {
		///?????????????????????
		rep.timestamp = GetSysTime_us();

		rep.nan_flags |= (((uint8_t)ekf_report.angNaN)		<< 0);
		rep.nan_flags |= (((uint8_t)ekf_report.summedDelVelNaN)	<< 1);
		rep.nan_flags |= (((uint8_t)ekf_report.KHNaN)		<< 2);
		rep.nan_flags |= (((uint8_t)ekf_report.KHPNaN)		<< 3);
		rep.nan_flags |= (((uint8_t)ekf_report.PNaN)		<< 4);
		rep.nan_flags |= (((uint8_t)ekf_report.covarianceNaN)	<< 5);
		rep.nan_flags |= (((uint8_t)ekf_report.kalmanGainsNaN)	<< 6);
		rep.nan_flags |= (((uint8_t)ekf_report.statesNaN)	<< 7);

		rep.health_flags |= (((uint8_t)ekf_report.velHealth)	<< 0);
		rep.health_flags |= (((uint8_t)ekf_report.posHealth)	<< 1);
		rep.health_flags |= (((uint8_t)ekf_report.hgtHealth)	<< 2);
		rep.health_flags |= (((uint8_t)!ekf_report.gyroOffsetsExcessive)	<< 3);
		rep.health_flags |= (((uint8_t)ekf_report.onGround)	<< 4);
		rep.health_flags |= (((uint8_t)ekf_report.staticMode)	<< 5);
		rep.health_flags |= (((uint8_t)ekf_report.useCompass)	<< 6);
		rep.health_flags |= (((uint8_t)ekf_report.useAirspeed)	<< 7);

		rep.timeout_flags |= (((uint8_t)ekf_report.velTimeout)	<< 0);
		rep.timeout_flags |= (((uint8_t)ekf_report.posTimeout)	<< 1);
		rep.timeout_flags |= (((uint8_t)ekf_report.hgtTimeout)	<< 2);
		rep.timeout_flags |= (((uint8_t)ekf_report.imuTimeout)	<< 3);

		if (_debug > 10) {

			if (rep.health_flags < ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))) {


			if (rep.timeout_flags) {

		}

		// Copy all states or at least all that we can fit
		size_t ekf_n_states = ekf_report.n_states;
		size_t max_states = (sizeof(rep.states) / sizeof(rep.states[0]));
		rep.n_states = (ekf_n_states < max_states) ? ekf_n_states : max_states;

		// Copy diagonal elemnts of covariance matrix
		float covariances[28];
	 get_covariance(covariances);

		for (size_t i = 0; i < rep.n_states; i++) {
			rep.states[i] = ekf_report.states[i];
			rep.covariances[i] = covariances[i];
		}
	}
 	return check;
  }
 }
}


void AttitudePositionEstimatorEKF::loop_up(void){
	

	//得到时间戳 暂时注视
	_filter_start_time = GetSysTime_us();
	/*
	 * do subscriptions
	 */
	/* sets also parameters in the EKF object */
	 parameters_update();  //初始化的时候参数已经确定如果不是动态改变的话
	_gps.vel_n_m_s = 0.0f;
	_gps.vel_e_m_s = 0.0f;
	_gps.vel_d_m_s = 0.0f;

	_task_running = true;
//卡在这儿
//	while (!_task_should_exit) {
//    //参数加载 
//			/* update parameters from storage */
//			//parameters_update();
//		}

		/* only run estimator if gyro updated */
		if (1) {

	/* check vehicle status for changes to publication state */
			vehicle_status_poll();  //起飞
			vehicle_land_detected_poll();  //磁罗盘
			_baro_init = false;
			_gps_initialized = false;
				_last_sensor_timestamp = GetSysTime_us();
			ZeroVariables();
		  dtIMU = 0.01f;
				_filter_start_time = _last_sensor_timestamp;

				/* now skip this loop and get data on the next one, which will also re-init the filter */
			
			}

			/**
			 *    PART ONE: COLLECT ALL DATA
			 **/
			pollData();   //传感器数据

			/*
			 *    CHECK IF ITS THE RIGHT TIME TO RUN THINGS ALREADY
			 */

			/**
			 *    PART TWO: EXECUTE THE FILTER
			 *
			 *    We run the filter only once all data has been fetched
			 **/
//所有数据就位
			if (1) {
				if (_gpsIsGood) {
					_baro_gps_offset = _baro_alt_filt - _gps_alt_filt;
				}
				/* Initialize the filter first */
				if (!statesInitialised) {
					// North, East Down position (m)
					float initVelNED[3] = {0.0f, 0.0f, 0.0f};
				posNE[0] = 0.0f;
				posNE[1] = 0.0f;
					_baro_gps_offset = 0.0f;
					//_baro_alt_filt = _baro.altitude;
				InitialiseFilter(initVelNED, 0.0, 0.0, 0.0f, 0.0f);
					//_filter_ref_offset = -_baro.altitude;

				} else {

					if (!_gps_initialized && _gpsIsGood) {
						initializeGPS();
						//continue;
					}

					// Check if on ground - status is used by covariance prediction
				setOnGround(_vehicle_land_detected.landed);

					// We're apparently initialized in this case now
					// check (and reset the filter as needed)
					int check = check_filter_state();

					if (check) {
						// Let the system re-initialize itself
						//continue;
					}
					// Run EKF data fusion steps
					updateSensorFusion(_gpsIsGood, _newDataMag, _newRangeData, _newHgtData, _newAdsData);
					// Run separate terrain estimator
				//	_terrain_estimator->predict(dtIMU, &_att, &_sensor_combined, &_distance);
				//	_terrain_estimator->measurement_update(GetSysTime_us(), &_gps, &_distance, &_att);
					}
				}
	_task_running = false;

	//_estimator_task = -1;
}


void AttitudePositionEstimatorEKF::initReferencePosition(hrt_abstime timestamp,
		bool gps_valid, double lat, double lon, float gps_alt, float baro_alt)
{
	// Reference altitude
	if (isfinite(states[9])) {
		_filter_ref_offset = states[9];

	} else if (isfinite(-hgtMea)) {
		_filter_ref_offset = -hgtMea;

	} else {
		//第一次来
		_filter_ref_offset = -Ms5611.BaroAlt;
	}

	// init filtered gps and baro altitudes
	_baro_alt_filt = Ms5611.BaroAlt;

	if (gps_valid) {
		_gps_alt_filt = gps_alt;

		// Initialize projection
		//自己注视
		//map_projection_init(&_pos_ref, lat, lon);
	}
}


void AttitudePositionEstimatorEKF::initializeGPS()
{
	// GPS is in scaled integers, convert
	double lat = _gps.lat / 1.0e7;
	double lon = _gps.lon / 1.0e7;
	float gps_alt = _gps.alt / 1e3f;

	 // Set up height correctly
   //气压高度？？？？
    baroHgt = Ms5611.BaroAlt;
		hgtMea = baroHgt;

			// Set up position variables correctly
		GPSstatus = _gps.fix_type;

		gpsLat = radians(lat);
		gpsLon = radians(lon) - M_PI;
		gpsHgt = gps_alt;

	// Look up mag declination based on current position
	//float declination = radians(get_mag_declination(lat, lon));
  float declination ;
	
	float initVelNED[3];
	initVelNED[0] = _gps.vel_n_m_s;
	initVelNED[1] = _gps.vel_e_m_s;
	initVelNED[2] = _gps.vel_d_m_s;

InitialiseFilter(initVelNED, radians(lat),radians(lon) - M_PI, gps_alt, declination);

	//initReferencePosition(_gps.timestamp, _gpsIsGood, lat, lon, gps_alt, _baro.altitude);


	_gps_initialized = true;
}



void AttitudePositionEstimatorEKF::updateSensorFusion(const bool fuseGPS, const bool fuseMag,
		const bool fuseRangeSensor, const bool fuseBaro, const bool fuseAirSpeed)
{
	// Run the strapdown INS equations every IMU update
UpdateStrapdownEquationsNED();

	// store the predicted states for subsequent use by measurement fusion
StoreStates(getMillis());

	// sum delta angles and time used by covariance prediction
summedDelAng = summedDelAng + correctedDelAng;
summedDelVel = summedDelVel + dVelIMU;
	_covariancePredictionDt += dtIMU;

	// only fuse every few steps
	if (_prediction_steps < MAX_PREDICITION_STEPS && ((GetSysTime_us() - _prediction_last) < 20 * 1000)) {
		_prediction_steps++;
		return;

	} else {
		_prediction_steps = 0;
		_prediction_last = GetSysTime_us();
	}

	// perform a covariance prediction if the total delta angle has exceeded the limit
	// or the time limit will be exceeded at the next IMU update
	if ((_covariancePredictionDt >= (covTimeStepMax - dtIMU))
	    || (summedDelAng.length() > covDelAngMax)) {
	CovariancePrediction(_covariancePredictionDt);
	summedDelAng.zero();
	summedDelVel.zero();
		_covariancePredictionDt = 0.0f;
	}

	// Fuse GPS Measurements
	if (fuseGPS && _gps_initialized) {
		// Convert GPS measurements to Pos NE, hgt and Vel NED

		// set fusion flags
	fuseVelData = _gps.vel_ned_valid;
	fusePosData = true;

		// recall states stored at time of measurement after adjusting for delays
	RecallStates(statesAtVelTime, (getMillis() - _parameters.vel_delay_ms));
	RecallStates(statesAtPosTime, (getMillis() - _parameters.pos_delay_ms));

		// run the fusion step
	FuseVelposNED();

	} else if (!_gps_initialized) {

		// force static mode
	staticMode = true;

		// Convert GPS measurements to Pos NE, hgt and Vel NED
	velNED[0] = 0.0f;
	velNED[1] = 0.0f;
	velNED[2] = 0.0f;

	posNE[0] = 0.0f;
	posNE[1] = 0.0f;

		// set fusion flags
	fuseVelData = true;
	fusePosData = true;

		// recall states stored at time of measurement after adjusting for delays
	RecallStates(statesAtVelTime, (getMillis() - _parameters.vel_delay_ms));
	RecallStates(statesAtPosTime, (getMillis() - _parameters.pos_delay_ms));

		// run the fusion step
	FuseVelposNED();

	} else {
	fuseVelData = false;
	fusePosData = false;
	}

	if (fuseBaro) {
		// Could use a blend of GPS and baro alt data if desired
	hgtMea = baroHgt;
	fuseHgtData = true;

		// recall states stored at time of measurement after adjusting for delays
	RecallStates(statesAtHgtTime, (getMillis() - _parameters.height_delay_ms));

		// run the fusion step
	FuseVelposNED();

	} else {
	fuseHgtData = false;
	}

	// Fuse Magnetometer Measurements
	if (fuseMag) {
	fuseMagData = true;
	RecallStates(statesAtMagMeasTime,
				   (getMillis() - _parameters.mag_delay_ms)); // Assume 50 msec avg delay for magnetometer data

	magstate.obsIndex = 0;
	FuseMagnetometer();
	FuseMagnetometer();
	FuseMagnetometer();

	} else {
	fuseMagData = false;
	}

	// Fuse Airspeed Measurements
//	if (fuseAirSpeed && _airspeed.true_airspeed_m_s > 5.0f) {
//	fuseVtasData = true;
//	RecallStates(statesAtVtasMeasTime,
//				   (getMillis() - _parameters.tas_delay_ms)); // assume 100 msec avg delay for airspeed data
//	FuseAirspeed();

//	} 
//	else {
//	fuseVtasData = false;
//	}

	// Fuse Rangefinder Measurements
	if (fuseRangeSensor) {
		if (Tnb.z.z > 0.9f) {
			// rngMea is set in sensor readout already
		fuseRngData = true;
		fuseOptFlowData = false;
		RecallStates(statesAtRngTime, (getMillis() - 100.0f));
		OpticalFlowEKF();
		fuseRngData = false;
		}
	}
}



int AttitudePositionEstimatorEKF::start()
{
	return 1;
}




void AttitudePositionEstimatorEKF::pollData()
{

	

	/* set time fields */
	IMUusec = GetSysTime_us();
	float deltaT = ( GetSysTime_us() - _last_sensor_timestamp) / 1e6f;

	/* guard against too large deltaT's */
	if (!isfinite(deltaT) || deltaT > 1.0f || deltaT < 0.0001f) {

		if (isfinite(dtIMUfilt) && dtIMUfilt < 0.5f && dtIMUfilt > 0.0001f) {
			deltaT = dtIMUfilt;

		} else {
			deltaT = 0.01f;
		}
	}

	/* fill in last data set */
dtIMU = deltaT;
if(1){
angRate.x = Mpu6050.gyrdeg.x;
angRate.y = Mpu6050.gyrdeg.y;
angRate.z = Mpu6050.gyrdeg.z;
dAngIMU = angRate * dtIMU/1.e6;
	
	 //float gyro_dt = _sensor_combined.gyro_integral_dt / 1.e6f;
	//dAngIMU = angRate * gyro_dt;
}

if(1)
{
	//if (_last_accel != _sensor_combined.timestamp + _sensor_combined.accelerometer_timestamp_relative) {
	accel.x =  Mpu6050.acc.x;
	accel.y =  Mpu6050.acc.y;
	accel.z =  Mpu6050.acc.z;
    dVelIMU = accel * dtIMU/1.e6;
	
		 //float accel_dt = _sensor_combined.accelerometer_integral_dt / 1.e6f;
		//dVelIMU = accel * accel_dt;
		//_last_accel = _sensor_combined.timestamp + _sensor_combined.accelerometer_timestamp_relative;
	}

	Vector3f_px4 mag(Ak8975.mag.x ,Ak8975.mag.y,Ak8975.mag.z);
    if(1){
	 // if (mag.length() > 0.1f && _last_mag != _sensor_combined.timestamp + _sensor_combined.magnetometer_timestamp_relative) {
	magData.x = mag.x;
	magData.y = mag.y;
	magData.z = mag.z;
		_newDataMag = true;
		//_last_mag = _sensor_combined.timestamp + _sensor_combined.magnetometer_timestamp_relative;
		
	}

	//_last_sensor_timestamp = _sensor_combined.timestamp;
	_last_sensor_timestamp = IMUusec;

	// XXX this is for assessing the filter performance
	// leave this in as long as larger improvements are still being made.


	_data_good = true;

	//Update AirSpeed


	if (0) {// 空速 
	//VtasMeas = _airspeed.true_airspeed_unfiltered_m_s;
VtasMeas = 0;
	}

	bool gps_update=true;
	if (gps_update) {
		//We are more strict for our first fix
		float requiredAccuracy = _parameters.pos_stddev_threshold;

		if (_gpsIsGood) {
			requiredAccuracy = _parameters.pos_stddev_threshold * 2.0f;
		}

		//Check if the GPS fix is good enough for us to use  //eph 水平定位精度 epv 垂直定位精度
		if ((_gps.fix_type >= 3) && (_gps.eph < requiredAccuracy) && (_gps.epv < requiredAccuracy)) {
			_gpsIsGood = true;

		} else {
			_gpsIsGood = false;
		}

		if (_gpsIsGood) {

			//Calculate time since last good GPS fix
			 float dtLastGoodGPS = static_cast<float>(_gps.timestamp - _previousGPSTimestamp) / 1e6f;


			//？？？？？？？？？？？？

			//_global_pos.dead_reckoning = false;

			//Fetch new GPS data
		GPSstatus = _gps.fix_type;
		velNED[0] = _gps.vel_n_m_s;
		velNED[1] = _gps.vel_e_m_s;
		velNED[2] = _gps.vel_d_m_s;

		gpsLat = radians(_gps.lat / (double)1e7);
		gpsLon = radians(_gps.lon / (double)1e7) - M_PI;
		gpsHgt = _gps.alt / 1e3f;

			if (_previousGPSTimestamp != 0) {
				//Calculate average time between GPS updates
				
				VAL_LIMIT(dtLastGoodGPS, 0.01f, POS_RESET_THRESHOLD);
				
			updateDtGpsFilt(dtLastGoodGPS);
				// update LPF
				float filter_step = (dtLastGoodGPS / (rc + dtLastGoodGPS)) * (gpsHgt - _gps_alt_filt);

				if (isfinite(filter_step)) {
					_gps_alt_filt += filter_step;
				}
			}

			//check if we had a GPS outage for a long time
			if (_gps_initialized) {

				//？？？？？？？？？？？？？
				//Convert from global frame to local frame
				//map_projection_project(&_pos_ref, (_gps.lat / 1.0e7), (_gps.lon / 1.0e7), &posNE[0], &posNE[1]);

				if (dtLastGoodGPS > POS_RESET_THRESHOLD) {
				ResetPosition();
				ResetVelocity();
				}
			}

			//PX4_INFO("gps alt: %6.1f, interval: %6.3f", (double)gpsHgt, (double)dtGoodGPS);

			// if (_gps.s_variance_m_s > 0.25f && _gps.s_variance_m_s < 100.0f * 100.0f) {
			//vneSigma = sqrtf(_gps.s_variance_m_s);
			// } else {
			//vneSigma = _parameters.velne_noise;
			// }

			// if (_gps.p_variance_m > 0.25f && _gps.p_variance_m < 100.0f * 100.0f) {
			//posNeSigma = sqrtf(_gps.p_variance_m);
			// } else {
			//posNeSigma = _parameters.posne_noise;
			// }

			// PX4_INFO("vel: %8.4f pos: %8.4f", _gps.s_variance_m_s, _gps.p_variance_m);

			_previousGPSTimestamp = _gps.timestamp;

		}
	}

	// If it has gone more than POS_RESET_THRESHOLD amount of seconds since we received a GPS update,
	// then something is very wrong with the GPS (possibly a hardware failure or comlink error)
	const float dtLastGoodGPS = static_cast<float>(GetSysTime_us() - _previousGPSTimestamp) / 1e6f;

	if (dtLastGoodGPS >= POS_RESET_THRESHOLD) {


		_gpsIsGood = false;
		_global_pos.dead_reckoning = false;
	}

	//If we have no good GPS fix for half a second, then enable dead-reckoning mode while armed (for up to POS_RESET_THRESHOLD seconds)
	else if (dtLastGoodGPS >= 0.5f) {


			_global_pos.dead_reckoning = true;

		} else {
			_global_pos.dead_reckoning = false;
		}
	

	//Update barometer
  bool  _newHgtData=true;

	if(_newHgtData) {
		static hrt_abstime baro_last = 0;
		// init lowpass filters for baro and gps altitude
		float baro_elapsed;
		if (baro_last == 0) {
			baro_elapsed = 0.0f;
		} else {
			baro_elapsed = (GetSysTime_us() - baro_last) / 1e6f;
		}
		baro_last = GetSysTime_us();

		if (!_baro_init) {
			_baro_init = true;
			_baro_alt_filt = Ms5611.BaroAlt;
		}
      VAL_LIMIT(baro_elapsed, 0.001f, 0.1f);
	updateDtHgtFilt(baro_elapsed);

	baroHgt =  Ms5611.BaroAlt;
		float filter_step = (baro_elapsed / (rc + baro_elapsed)) * ( Ms5611.BaroAlt - _baro_alt_filt);

		if (isfinite(filter_step)) {
			_baro_alt_filt += filter_step;
		}
	}
	//Update range data
	if (_newRangeData) {
//高度计
//		if ((_distance.current_distance > _distance.min_distance)
//		    && (_distance.current_distance < _distance.max_distance)) {
//		rngMea = _distance.current_distance;
//			_distance_last_valid = _distance.timestamp;

//		} else {
//			_newRangeData = false;
//		}
	}
}













