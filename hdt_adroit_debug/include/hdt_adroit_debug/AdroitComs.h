/**
  ******************************************************************************
  ******************************************************************************
  * @file    AdroitComs.h
  * @author  HDT Robotics, Inc.
  ******************************************************************************
  ******************************************************************************
  */

#pragma once

#ifndef AdroitComs_h
#define AdroitComs_h

#define _USE_MATH_DEFINES

#include <stdint.h>
#include <math.h>
#include <fstream>
//#include <semaphore.h>
#include <pthread.h>

#include <hdt_adroit_debug/AdroitCrc.h>

#define PARAM_WAIT_SHORT		1
#define PARAM_WAIT_LONG			5

#define SEGMENT_WAIT_TIME		10
#define SEGMENT_BUFFER_SIZE		128
#define	SEGMENT_MAX_BYTES		7
#define BLOCK_BUFFER_SIZE		512
#define BLOCK_MAX_BYTES			504			// multiple of 4 and 7
#define BLOCK_MAX_SEGMENTS		72			// 72 ~= 504/7

#define MAX_DRIVES				(uint8_t)128
#define BROADCAST_ADDR			(uint8_t)0
#define POSITION_CONV			(2.0f * (float)M_PI)
#define VELOCITY_CONV			(8.0f)
#define EFFORT_CONV				(256.0f)
#define CURRENT_CONV			(64.0f)
#define VOLTAGE_CONV			(64.0f)
#define TEMPERATURE_CONV		(128.0f)
#define INERTIA_CONV			(32.0f)
#define DAMPING_CONV			(128.0f)
#define STIFFNESS_CONV			(1024.0f)

#define SV_GAIN					0.866025403784439		// sqrt(3)/2, space vector gain for <= 100% duty
#define PC_GAIN					0.816496580927726		// sqrt(2/3), park/clarke transform power invariant gain
#define SP_GAIN					1.4142135623731			// sqrt(2), single phase equivalent gain


class AdroitComs {
public:
	typedef enum MessageType {
		STATE_CHANGE_CMD = 		(uint8_t)0,
		ERROR_TELEM = 			(uint8_t)1,
		HIGH_SPEED_TELEM = 		(uint8_t)3,
		CONTROL_CMD = 			(uint8_t)4,
		MEDIUM_SPEED_TELEM =	(uint8_t)5,
		IMPEDANCE_CMD = 		(uint8_t)6,
		LOW_SPEED_TELEM =		(uint8_t)7,
		DEBUG_TELEM =			(uint8_t)9,
		CURRENT_CMD =			(uint8_t)10,
		PARAMETER_TELEM =		(uint8_t)11,
		PARAMETER_CMD =			(uint8_t)12,
		STATUS_CMD =			(uint8_t)14,
		STATUS_TELEM =			(uint8_t)15
	} MessageType;

	typedef enum StateType {
		NO_STATE			= (int8_t)-1,
		INIT_STATE			= (int8_t)0,
		PROGRAM_STATE		= (int8_t)1,
		STARTUP_STATE		= (int8_t)2,
		RUN_STATE			= (int8_t)3,
		HOLD_STATE			= (int8_t)4,
		SLEEP_STATE			= (int8_t)5,
		FAULT_STATE			= (int8_t)6,
		NUM_STATES			= (int8_t)7
	} StateType;

	typedef enum ParamaterType {
		APPLICATION_PARAMETER =		(uint16_t)0x2000,
		LUT_PARAMETER =				(uint16_t)0x3000,
		APPLICATION_IMAGE =			(uint16_t)0x4000,
		SPECIAL_PARAMETER =			(uint16_t)0x5000
	} ParameterType;

	typedef enum CCSType {
		SEGMENT_WRITE_REQ =		(uint8_t)0,
		INITIATE_WRITE_REQ =	(uint8_t)1,
		INITIATE_READ_REQ =		(uint8_t)2,
		SEGMENT_READ_REQ =		(uint8_t)3,
		ABORT_TRANSFER_REQ =	(uint8_t)4,
		BLOCK_READ_REQ =		(uint8_t)5,
		BLOCK_WRITE_REQ =		(uint8_t)6
	} CCSType;

	typedef enum SCSType {
		SEGMENT_READ_RES =		(uint8_t)0,
		SEGMENT_WRITE_RES =		(uint8_t)1,
		INITIATE_READ_RES =		(uint8_t)2,
		INITIATE_WRITE_RES =	(uint8_t)3,
		ABORT_TRANSFER_RES =	(uint8_t)4,
		BLOCK_WRITE_RES =		(uint8_t)5,
		BLOCK_READ_RES =		(uint8_t)6
	} SCSType;

	// Client/Server Subcommand
	typedef enum SubcmdType {
		BLOCK_INITIATE =		(uint8_t)0,
		BLOCK_END =				(uint8_t)1,
		BLOCK_ACKNOWLEDGE =		(uint8_t)2
	} SubcmdType;

	typedef struct InitWriteReqHeader {
		unsigned s		: 1;
		unsigned e		: 1;
		unsigned n		: 2;
		unsigned x		: 1;
		unsigned ccs	: 3;
	} InitWriteReqHeader;

	typedef struct InitWriteResHeader {
		unsigned x		: 5;
		unsigned scs	: 3;
	} InitWriteResHeader;

	typedef struct SegWriteReqHeader {
		unsigned c		: 1;
		unsigned n		: 3;
		unsigned t		: 1;
		unsigned ccs	: 3;
	} SegWriteReqHeader;

	typedef struct SegWriteResHeader {
		unsigned x		: 4;
		unsigned t		: 1;
		unsigned scs	: 3;
	} SegWriteResHeader;

	typedef struct InitReadReqHeader {
		unsigned x		: 5;
		unsigned ccs	: 3;
	} InitReadReqHeader;

	typedef struct InitReadResHeader {
		unsigned s		: 1;
		unsigned e		: 1;
		unsigned n		: 2;
		unsigned x		: 1;
		unsigned scs	: 3;
	} InitReadResHeader;

	typedef struct SegReadReqHeader {
		unsigned x		: 4;
		unsigned t		: 1;
		unsigned ccs	: 3;
	} SegReadReqHeader;

	typedef struct SegReadResHeader {
		unsigned c		: 1;
		unsigned n		: 3;
		unsigned t		: 1;
		unsigned scs	: 3;
	} SegReadResHeader;

	typedef struct BlkInitReqHeader {
		unsigned cs		: 1;
		unsigned s		: 1;
		unsigned cc		: 1;
		unsigned x		: 2;
		unsigned ccs	: 3;
	} BlkInitReqHeader;

	typedef struct BlkInitResHeader {
		unsigned ss		: 2;
		unsigned sc		: 1;
		unsigned x		: 2;
		unsigned scs	: 3;
	} BlkInitResHeader;

	typedef struct BlkWriteReqHeader {
		unsigned seqno	: 7;
		unsigned c		: 1;
	} BlkWriteReqHeader;

	typedef struct BlkWriteResHeader {
		unsigned ss		: 2;
		unsigned x		: 3;
		unsigned scs	: 3;
	} BlkWriteResHeader;

	typedef struct BlkEndReqHeader {
		unsigned cs		: 1;
		unsigned x		: 1;
		unsigned n		: 3;
		unsigned ccs	: 3;
	} BlkEndReqHeader;

	typedef struct BlkEndResHeader {
		unsigned ss		: 2;
		unsigned x		: 3;
		unsigned scs	: 3;
	} BlkEndResHeader;

	struct StateCmd {
		uint8_t state;
	};

	struct BITTelem {
		uint8_t error_code;
		uint8_t error_severity;
		uint32_t error_value;
		float error_value_float;
	};

	struct HSTelem {
		float position;
		float velocity;
		float effort;
		float motor_current;
	};

	struct ControlCmd {
		float position;
		float velocity;
		float effort;
		float motor_current;
	};

	struct MSTelem {
		int16_t var1;
		int16_t var2;
		int16_t var3;
		int16_t var4;
	};

	struct ImpedanceCmd {
		float inertia;
		float damping;
		float stiffness;
	};

	struct ImpedanceParams {
		float inertia;
		float kmin;
		float kmax;
	};

	struct LSTelem {
		float bus_voltage;
		float bus_current;
		float temperature;
	};

	struct DebugTelem {
		uint8_t		index;
		uint8_t		hs_samples;
		int16_t		ls_var1;
		int16_t		ls_var2;
		int16_t		ls_var3;
		int16_t		hs_var1[16];
		int16_t		hs_var2[16];
		uint32_t	crc;
	};

	struct CurrentCmd {
		float motor_current;
	};

	struct ParameterMsg {
		uint8_t head;
		uint16_t index;
		uint8_t sub_index;
		uint32_t data;
	};

	struct SegmentedMsg {
		uint8_t head;
		uint8_t data[7];
	};

	struct StatusTelem {
		int16_t state;
	};

	struct ParameterData {
		float pos_max;
		float pos_min;
		float vel_limit;
		float current_limit;
		float effort_max;
		float vel_hold;
		float accel_limit;
		float motor_rp;
		float motor_ke;
		float gear_ratio;
		uint8_t num_poles;
		uint8_t ctl_mode;
		uint8_t continuous_rot;
		uint32_t dev_type;
	};

	enum ParameterIndex {
		PARAMETER_CHECKSUM =					0,
		IMAGE_CHECKSUM =							1,
		IMAGE_LENGTH =								2,
		SERIAL_NUMBER =								3,
		DEVICE_TYPE_ARRAY =						4,
		HOME_POSITION =								5,
		STORAGE_POSITION =						6,
		INPUT_XAXIS_DISP =						7,
		INPUT_YAXIS_DISP =						8,
		INPUT_ZAXIS_DISP =						9,
		INPUT_XAXIS_ROT =							10,
		INPUT_YAXIS_ROT =							11,
		INPUT_ZAXIS_ROT =							12,
		OUTPUT_XAXIS_ROT =						13,
		OUTPUT_YAXIS_ROT =						14,
		OUTPUT_ZAXIS_ROT =						15,
		OUTPUT_XAXIS_DISP =						16,
		OUTPUT_YAXIS_DISP =						17,
		OUTPUT_ZAXIS_DISP =						18,
		DEVICE_ADDRESS_ARRAY =				19,
		DEVICE_TELEM_ARRAY =					20,
		MS_TELEM_INDEX_ARRAY =				21,
		DEBUG_LS_TELEM_INDEX_ARRAY =	22,
		DEBUG_HS_TELEM_INDEX_ARRAY =	23,
		SLEEP_TIMEOUT	=								24,
		MAJOR_VERSION_INDEX =					25,
		MINOR_VERSION_INDEX =					26,
		VARIANT_INDEX =								27,
		SVN_REVISION_INDEX =					28,
		BUILD_DATE_INDEX =						29,
		BUILD_TIME_INDEX =						30,
		ERROR_SELECT_ARRAY =					32,
		OVER_TEMP_ERROR_WARNING	=			33,
		OVER_TEMP_ERROR_LIMIT =				34,
		OVER_VOLTAGE_ERROR_LIMIT =		35,
		UNDER_VOLTAGE_ERROR_LIMIT =		36,
		OVER_CURRENT_ERROR_LIMIT =		37,
		WRONG_DIR_VEL_ERROR_LIMIT =		38,
		POS_CAL_ERROR_LIMIT =					39,
		POS_SENSOR_SEL_ARRAY =				64,
		POS_SENSOR_SLOPE =						65,
		POS_SENSOR_OFFSET =						66,
		POS_SENSOR_ADJUST =						67,
		POS_SENSOR_LOOKUP_OFFSET =		68,
		EFFORT_SENSOR_SLOPE =					69,
		EFFORT_SENSOR_OFFSET =				70,
		BUS_CURRENT_SENSOR_SLOPE =		71,
		BUS_CURRENT_SENSOR_OFFSET =		72,
		BUS_VOLTAGE_SENSOR_SLOPE =		73,
		BUS_VOLTAGE_SENSOR_OFFSET =		74,
		ANALOG_HALLA_SENSOR_SLOPE =		75,
		ANALOG_HALLA_SENSOR_OFFSET =	76,
		ANALOG_HALLB_SENSOR_SLOPE =		77,
		ANALOG_HALLB_SENSOR_OFFSET =	78,
		COIL_TEMP_SENSOR_SLOPE =			79,
		COIL_TEMP_SENSOR_OFFSET =			80,
		PCB_TEMP_SENSOR_SLOPE =				81,
		PCB_TEMP_SENSOR_OFFSET =			82,
		POS_SENSOR_FILTER_WC =				100,
		EFFORT_SENSOR_FILTER_WC =			101,
		BUS_CURRENT_SENSOR_FILTER_WC =102,
		BUS_VOLTAGE_SENSOR_FILTER_WC =103,
		VELOCITY_SENSOR_FILTER_WC =		104,
		MOTOR_ALIGNMENT_OFFSET =			160,
		MOTOR_ALIGNMENT_DELAY =				161,
		MOTOR_ALIGNMENT_DURATION =		162,
		MOTOR_ALIGNMENT_CURRENT =			163,
		MOTOR_ALIGNMENT_PHASE =				164,
		MOTOR_PARAM_ARRAY =						165,
		MOTOR_BACK_EMF_CONSTANT =			166,
		MOTOR_TERMINAL_RESISTANCE =		167,
		GEAR_RATIO =									168,
		POSITION_PID_KP = 						169,
		POSITION_PID_KI	=							170,
		POSITION_PID_KD	=							171,
		VELOCITY_PID_KP =							172,
		VELOCITY_PID_KI =							173,
		VELOCITY_PID_KD =							174,
		EFFORT_PID_KP =								175,
		EFFORT_PID_KI =								176,
		EFFORT_PID_KD =								177,
		POSITION_LIMIT_MAX =					178,
		POSITION_LIMIT_MIN =					179,
		VELOCITY_LIMIT =							180,
		CURRENT_LIMIT =								181,
		IMPEDANCE_EFFORT_MAX =				182,
		HOLD_TIMEOUT =								183,
		HOLD_VELOCITY =								184,
		HOLD_CURRENT =								185,
		HOLD_POSITION_ERROR =					186,
		HOLD_EFFORT =									187,
		DRIFT_ALPHA =									188,
		DRIFT_MAX =										189,
		ACCELERATION_LIMIT =					190,
		MOTOR_INDUCTANCE =						191,
		WIPER_POSITION =							192,
		POSITION_ERROR_DEADBAND =			194,
		EFFORT_SENSOR_DEADBAND =			195,
		HALL_1_ALIGNMENT_ARRAY =			224,
		HALL_2_ALIGNMENT_ARRAY =			225,
		HALL_3_ALIGNMENT_ARRAY =			226,
		HALL_4_ALIGNMENT_ARRAY =			227,
		HALL_5_ALIGNMENT_ARRAY =			228,
		HALL_6_ALIGNMENT_ARRAY =			229,
		HALL_FORWARD_ARRAY =					230,
		HALL_REVERSE_ARRAY =					231,
		HALL_MIDPOINT =								232,
		LOOKUP_TABLE_1_INDEX =				256,
		LOOKUP_TABLE_2_INDEX =				288,
		LOOKUP_TABLE_3_INDEX =				320,
		LOOKUP_TABLE_4_INDEX =				352,
		LOOKUP_TABLE_5_INDEX =				384
	};

	struct AdroitDrive {
		uint8_t port;
		StateCmd state_cmd;
		BITTelem error_telem;
		HSTelem hs_telem;
		ControlCmd control_cmd;
		MSTelem ms_telem;
		ImpedanceCmd impedance_cmd;
		LSTelem ls_telem;
		DebugTelem debug_telem;
		CurrentCmd current_cmd;
		ParameterMsg parameter_telem;
		ParameterMsg parameter_cmd;
		StatusTelem status_telem;
		bool msg_update[16];
		bool control;
		bool commit_needed;
		//sem_t semaphore;
		pthread_mutex_t mutex;
		pthread_cond_t cond_var;
		ImpedanceParams impedance_params;
		ParameterData parameter_data;
	};

	struct SegmentData {
		uint16_t 	index;
		uint8_t 	sub_index;
		uint8_t		toggle;
		uint8_t		size;
		uint32_t	bytes;
		uint8_t		buffer[SEGMENT_BUFFER_SIZE];
	};	

	struct AdroitMsg {
		uint32_t id;
		uint8_t ff;
		uint8_t dlc;
		uint8_t data[8];
		uint32_t time;
	};

	enum ErrorSeverity {
		ERROR_INFO = 		(uint8_t)0,
		ERROR_WARNING = 	(uint8_t)1,
		ERROR_CRITICAL =	(uint8_t)2
	};
	
	enum ErrorCodes {
		ERROR_NONE		= (uint8_t)0,
		ERROR_OVER_TEMP		= (uint8_t)1,
		ERROR_OVER_VOLT		= (uint8_t)2,
		ERROR_UNDER_VOLT	= (uint8_t)3,
		ERROR_OVER_CURRENT 	= (uint8_t)4,
		ERROR_INIT 		= (uint8_t)5,
		ERROR_SENSOR 		= (uint8_t)6,
		ERROR_WRONG_DIR 	= (uint8_t)7,
		ERROR_SLAVE_DRIVE 	= (uint8_t)8,
		ERROR_POSITION_CAL 	= (uint8_t)9,	
		ERROR_LIMIT_SW		= (uint8_t)10,
		ERROR_COMS_TIMEOUT	= (uint8_t)11,
		ERROR_COMS_SEND		= (uint8_t)12,
		ERROR_COMS_RECEIVE	= (uint8_t)13,
		ERROR_NUM 		= (uint8_t)14
	};

	AdroitComs(uint8_t source);
	~AdroitComs();
	void reset_drive(AdroitDrive *drive);

	void send_state_cmd(uint8_t addr, StateCmd *cmd);
	void send_control_cmd(uint8_t addr, ControlCmd *cmd);
	void send_impedance_cmd(uint8_t addr, ImpedanceCmd *cmd);
	void send_current_cmd(uint8_t addr, CurrentCmd *cmd);
	void send_parameter_cmd(uint8_t addr, ParameterMsg *cmd);
	void send_status_cmd(uint8_t addr);

	int read_parameter(int addr, int index, AdroitDrive *drive);
	int write_parameter(int addr, int index, AdroitDrive *drive, uint32_t *data);
	int commit_parameters(int addr, AdroitDrive *drive);
	int load_application_image(uint8_t addr, std::vector<unsigned char> data, AdroitDrive *drive);

	bool process_error_telem(BITTelem *telem, AdroitMsg *msg);
	bool process_hs_telem(HSTelem *telem, AdroitMsg *msg);
	bool process_ms_telem(MSTelem *telem, AdroitMsg *msg);
	bool process_ls_telem(LSTelem *telem, AdroitMsg *msg);
	bool process_debug_telem(DebugTelem *telem, AdroitMsg *msg);
	bool process_parameter_telem(ParameterMsg *telem, AdroitMsg *msg);
	bool process_status_telem(StatusTelem *telem, AdroitMsg *msg);
	bool process_control_cmd(ControlCmd *cmd, AdroitMsg *msg);

	void set_impedance_cmd(AdroitDrive *drive, double damping_ratio, double stiffness_ratio);
protected:
	//uint8_t addr;
	virtual int SendMsg(AdroitMsg *msg) = 0;
	virtual int Write(void) = 0;
	SegmentData seg_data;
	AdroitCrc *seg_crc;
	int timed_wait(AdroitDrive *drive, int sec);
};

#endif // AdroitComs_h
