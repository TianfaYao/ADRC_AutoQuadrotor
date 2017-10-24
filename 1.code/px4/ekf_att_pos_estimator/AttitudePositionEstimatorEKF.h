

#pragma once


#include "stm32f4xx.h"
typedef   unsigned int size_t;
typedef   uint64_t	hrt_abstime;

#include "estimator_22states.h"
#include "dev_gps.h"

//Forward declaration
class AttPosEKF;
typedef uint32_t	param_t;
typedef uint64_t	hrt_abstime;


struct vehicle_land_detected_s {
	uint64_t timestamp; // required for logger
	float alt_max;
	bool landed;
	bool freefall;
	bool ground_contact;
	bool maybe_landed;
};

struct sensor_combined_s {

	uint64_t timestamp; // required for logger
	float gyro_rad[3];
	uint32_t gyro_integral_dt;
	int32_t accelerometer_timestamp_relative;
	float accelerometer_m_s2[3];
	uint32_t accelerometer_integral_dt;
	int32_t magnetometer_timestamp_relative;
	float magnetometer_ga[3];
	int32_t baro_timestamp_relative;
	float baro_alt_meter;
	float baro_temp_celcius;
	//static const int32_t RELATIVE_TIMESTAMP_INVALID = 2147483647;

};

struct vehicle_global_position_s {

	uint64_t timestamp; // required for logger
	uint64_t time_utc_usec;
	double lat;
	double lon;
	double delta_lat_lon[2];
	float alt;
	float delta_alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float pos_d_deriv;
	float yaw;
	float eph;
	float epv;
	float evh;
	float evv;
	float terrain_alt;
	float pressure_alt;
	uint8_t lat_lon_reset_counter;
	uint8_t alt_reset_counter;
	bool terrain_alt_valid;
	bool dead_reckoning;

};



	class AttitudePositionEstimatorEKF:public AttPosEKF
{
public:
	  //AttPosEKF                   *_ekf;
    /**
     * Constructor
     */
    AttitudePositionEstimatorEKF();

    /* we do not want people ever copying this class */
   // AttitudePositionEstimatorEKF(const AttitudePositionEstimatorEKF& that) = delete;
   // AttitudePositionEstimatorEKF operator=(const AttitudePositionEstimatorEKF&) = delete;

    /**
     * Destructor, also kills the sensors task.
     */
    //~AttitudePositionEstimatorEKF();

    /**
     * Start the sensors task.
     *
     * @return  OK on success.
     */
    int start();
    void loop_up();
    /**
     * Task status
     *
     * @return  true if the mainloop is running
     */
    bool task_running() { return _task_running; }

    /**
     * Print the current status.
     */
    void print_status();

    /**
     * Trip the filter by feeding it NaN values.
     */
    int trip_nan();

    /**
     * Enable logging.
     *
     * @param   enable Set to true to enable logging, false to disable
     */
    int enable_logging(bool enable);

    /**
     * Set debug level.
     *
     * @param   debug Desired debug level - 0 to disable.
     */
    int set_debuglevel(unsigned debug) { _debug = debug; return 0; }

    static const unsigned MAX_PREDICITION_STEPS = 3; /**< maximum number of prediction steps between updates */

private:
    bool        _task_should_exit;      /**< if true, sensor task should exit */
    bool        _task_running;          /**< if true, task is running in its mainloop */
    int     _estimator_task;        /**< task handle for sensor task */

    int     _sensor_combined_sub;
    int     _distance_sub;          /**< distance measurement */
    int     _airspeed_sub;          /**< airspeed subscription */
    int     _baro_sub;          /**< barometer subscription */
    int     _gps_sub;           /**< GPS subscription */
    int     _vehicle_status_sub;
    int     _vehicle_land_detected_sub;
    int     _params_sub;            /**< notification of parameter updates */
    int     _manual_control_sub;        /**< notification of manual control updates */
    int     _home_sub;          /**< home position as defined by commander / user */
    int     _landDetectorSub;
    int     _armedSub;

//    struct vehicle_attitude_s           _att;           /**< vehicle attitude */
//    struct control_state_s              _ctrl_state;    /**< control state */
//    struct gyro_report                  _gyro;
//    struct accel_report                 _accel;
//    struct mag_report                   _mag;
//    struct airspeed_s                   _airspeed;      /**< airspeed */
//    struct baro_report                  _baro;          /**< baro readings */
//    struct vehicle_status_s		_vehicle_status;
//    struct vehicle_land_detected_s      _vehicle_land_detected;
//    struct vehicle_global_position_s    _global_pos;        /**< global vehicle position */
//    struct vehicle_local_position_s     _local_pos;     /**< local vehicle position */
//    struct vehicle_gps_position_s       _gps;           /**< GPS position */
//    struct wind_estimate_s              _wind;          /**< wind estimate */
//    struct distance_sensor_s            _distance;      /**< distance estimate */
//    struct actuator_armed_s             _armed;
			struct vehicle_land_detected_s      _vehicle_land_detected;
			struct vehicle_global_position_s    _global_pos; 
			struct vehicle_gps_position_s       _gps;


    float           _gps_alt_filt;
    float           _baro_alt_filt;
    float           _covariancePredictionDt;  ///< time lapsed since last covariance prediction
    bool            _gpsIsGood;               ///< True if the current GPS fix is good enough for us to use
    uint64_t        _previousGPSTimestamp;    ///< Timestamp of last good GPS fix we have received
    bool            _baro_init;
    bool            _gps_initialized;
		
		hrt_abstime     _filter_start_time;
    hrt_abstime     _last_sensor_timestamp;
    hrt_abstime     _distance_last_valid;

		
    bool            _data_good;         ///< all required filter data is ok
    bool            _ekf_logging;       ///< log EKF state
    unsigned        _debug;             ///< debug level - default 0
    bool            _was_landed;        ///< indicates if was landed in previous iteration

    bool            _newHgtData;
    bool            _newAdsData;
    bool            _newDataMag;
    bool            _newRangeData;



    struct {
        int32_t vel_delay_ms;
        int32_t pos_delay_ms;
        int32_t height_delay_ms;
        int32_t mag_delay_ms;
        int32_t tas_delay_ms;
        float velne_noise;
        float veld_noise;
        float posne_noise;
        float posd_noise;
        float mag_noise;
        float gyro_pnoise;
        float acc_pnoise;
        float gbias_pnoise;
        float abias_pnoise;
        float mage_pnoise;
        float magb_pnoise;
        float eas_noise;
        float pos_stddev_threshold;
        int32_t airspeed_mode;
    }       _parameters;            /**< local copies of interesting parameters */

    struct {
        param_t vel_delay_ms;
        param_t pos_delay_ms;
        param_t height_delay_ms;
        param_t mag_delay_ms;
        param_t tas_delay_ms;
        param_t velne_noise;
        param_t veld_noise;
        param_t posne_noise;
        param_t posd_noise;
        param_t mag_noise;
        param_t gyro_pnoise;
        param_t acc_pnoise;
        param_t gbias_pnoise;
        param_t abias_pnoise;
        param_t mage_pnoise;
        param_t magb_pnoise;
        param_t eas_noise;
        param_t pos_stddev_threshold;
        param_t airspeed_mode;
    }       _parameter_handles;     /**< handles for interesting parameters */

   // TerrainEstimator            *_terrain_estimator;
    float                       _filter_ref_offset;   /**< offset between initial baro reference and GPS init baro altitude */
    float                       _baro_gps_offset;   /**< offset between baro altitude (at GPS init time) and GPS altitude */
		unsigned _prediction_steps;
    uint64_t _prediction_last;
		
private:
    /**
     * Update our local parameter cache.
     */
    int     parameters_update();

    /**
     * Update control outputs
     *
     */
    void        control_update();

    /**
     * Check for changes in land detected.
     */
    void        vehicle_status_poll();

    /**
     * Check for changes in land detected.
     */
    void        vehicle_land_detected_poll();

    /**
     * Shim for calling task_main from task_create.
     */
    static void task_main_trampoline(int argc, char *argv[]);

    /**
     * Main filter task.
     */
    void        task_main();

    /**
     * Check filter sanity state
     *
     * @return zero if ok, non-zero for a filter error condition.
     */
    int     check_filter_state();

    /**
    * @brief
    *   Publish the euler and quaternions for attitude estimation
    **/


    /**
    * @brief
    *   Runs the sensor fusion step of the filter. The parameters determine which of the sensors
    *   are fused with each other
    **/
    void updateSensorFusion(const bool fuseGPS, const bool fuseMag, const bool fuseRangeSensor,
            const bool fuseBaro, const bool fuseAirSpeed);

    /**
    * @brief
    *   Initialize first time good GPS fix so we can get a reference point to calculate local position from
    *   Should only be required to call once
    **/
    void initializeGPS();

    /**
     * Initialize the reference position for the local coordinate frame
     */
		
    void initReferencePosition(hrt_abstime timestamp, bool gps_valid,
            double lat, double lon, float gps_alt, float baro_alt);

    /**
    * @brief
    *   Polls all uORB subscriptions if any new sensor data has been publish and sets the appropriate
    *   flags to true (e.g newDataGps)
    **/
    void pollData();
};


extern AttitudePositionEstimatorEKF  att_ekf;