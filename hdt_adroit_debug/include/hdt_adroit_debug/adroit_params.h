/*----------------------------------------------------------------------------
 * Name:    adroit_params.h
 * Purpose: information about parameters used by adroit actuators
 * Note(s): this file intended to be used as the primary source of parameter information for drives and console
 *----------------------------------------------------------------------------*/
 
#ifndef adroit_params_h
#define adroit_params_h

#define NUM_PARAMETER_PAGES							4	//define number of flash memory pages used for parameters, putting it here so the git branch is easier to organize

// this enum is used by drive code to address the parameter space
// when updating this enum make sure to update the ParameterIndexList, ParameterDataTypeList, and ParameterNameList
//   arrays below so that console has the correcte definitions for reading/writing parameters
typedef enum ParameterIndex {
	PARAMETER_CHECKSUM =					0,
	IMAGE_CHECKSUM =							1,
	IMAGE_LENGTH =								2,
	SERIAL_NUMBER =								3,
	DEVICE_TYPE_ARRAY =						4,
	POSITION_CONVERSION =					7,
	VELOCITY_CONVERSION =					8,
	EFFORT_CONVERSION =						9,
	CURRENT_CONVERSION =					10,
	VOLTAGE_CONVERSION =					11,
	TEMPERATURE_CONVERSION =			12,
	INERTIA_CONVERSION =					13,
	DAMPING_CONVERSION =					14,
	STIFFNESS_CONVERSION =				15,
	ROTATION_COUNTER =						18,
	DEVICE_ADDRESS_ARRAY =				19,
	DEVICE_TELEM_ARRAY =					20,
	MS_TELEM_INDEX_ARRAY =				21,
	DEBUG_LS_TELEM_INDEX_ARRAY =	22,
	DEBUG_HS_TELEM_INDEX_ARRAY =	23,
	SLEEP_TIMEOUT	=								24,
	MAJOR_VERSION_INDEX =					25,
	MINOR_VERSION_INDEX =					26,
	VARIANT_INDEX =								27,
	REPOSITORY_REVISION_INDEX =		28,
	BUILD_DATE_INDEX =						29,
	BUILD_TIME_INDEX =						30,
	MOTOR_CONTROLLER_SN =					31,
	ERROR_SELECT_ARRAY =					32,
	OVER_TEMP_ERROR_WARNING	=			33,
	OVER_TEMP_ERROR_LIMIT =				34,
	OVER_VOLTAGE_ERROR_LIMIT =		35,
	UNDER_VOLTAGE_ERROR_LIMIT =		36,
	OVER_CURRENT_ERROR_LIMIT =		37,
	POS_CAL_ERROR_LIMIT =					39,
	POS_SENSOR_STARTUP_CYCLES =				64, //prototyping
	POS_SENSOR_DEFAULT_ADC =						65, //prototyping
	POS_SENSOR_MAX_STEP =						66, //prototyping
	POS_SENSOR_LOOKUP_OFFSET =		68,
	EFFORT_SENSOR_SLOPE =					69,
	EFFORT_SENSOR_OFFSET =				70,
	BUS_CURRENT_SENSOR_SLOPE =		71,
	BUS_CURRENT_SENSOR_OFFSET =		72,
	BUS_VOLTAGE_SENSOR_SLOPE =		73,
	BUS_VOLTAGE_SENSOR_OFFSET =		74,
	COIL_TEMP_SENSOR_SLOPE =			79,
	COIL_TEMP_SENSOR_OFFSET =			80,
	PCB_TEMP_SENSOR_SLOPE =				81,
	PCB_TEMP_SENSOR_OFFSET =			82,
	POS_SENSOR_FILTER_WC =				100, // used to setup filter, but filter is only being used to store previous value
	EFFORT_SENSOR_FILTER_WC =			101,
	VELOCITY_EMF_FILTER_WC =			102,
	BUS_VOLTAGE_SENSOR_FILTER_WC =103,
	VELOCITY_SENSOR_FILTER_WC =		104,
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
	DRIFT_MAX =										189, //unused
	ACCELERATION_LIMIT =					190,
	MOTOR_INDUCTANCE =						191,
	WIPER_POSITION =							192,
	DELAY_SAMPLES =								193, //unused
	POSITION_ERROR_DEADBAND =			194, //passed into PositionPid.Deadband but not used
	EFFORT_SENSOR_DEADBAND =			195,
	BUS_VOLTAGE_NOMINAL =					196,
	BUS_VOLTAGE_REGEN_EPS =				197,
	REGEN_DIRECT_CURRENT =				198,
	BRAKE_RELEASE_VOLTAGE = 			199,
	BRAKE_RUN_VOLTAGE = 					200,
	BRAKE_TANSITION_TIME =				201,
	HALL_MIDPOINT =								232,
	HALL_FLAG_A =									233,
	HALL_FLAG_B =									234,
	LOOKUP_TABLE_1_INDEX =				256,
	LOOKUP_TABLE_2_INDEX =				288,
	LOOKUP_TABLE_3_INDEX =				320,
	LOOKUP_TABLE_4_INDEX =				352,
	LOOKUP_TABLE_5_INDEX =				384,
	LOOKUP_TABLE_6_INDEX =				512
} ParameterIndex;

typedef enum ParameterDataTypes {
	ARRAY_PARAMETER = 0,
	UINT32_PARAMETER = 1,
	INT32_PARAMETER = 2,
	FLOAT32_PARAMETER = 3,
	UINT16_PARAMETER = 4,
	UINT8_PARAMETER = 5
} ParameterDataTypes;


// these arrays are not used by drive code, they are used by console to correctly encode/decode parameters
// make sure these match the ParameterIndex enum
#define ParameterCount 1024
#define ParameterListLength 100
#define ParameterIndexList {0,\
	1,\
	2,\
	3,\
	4,\
	7,\
	8,\
	9,\
	10,\
	11,\
	12,\
	13,\
	14,\
	15,\
	18,\
	19,\
	20,\
	21,\
	22,\
	23,\
	24,\
	25,\
	26,\
	27,\
	28,\
	29,\
	30,\
	31,\
	32,\
	33,\
	34,\
	35,\
	36,\
	37,\
	39,\
	64,\
	65,\
	66,\
	68,\
	69,\
	70,\
	71,\
	72,\
	73,\
	74,\
	79,\
	80,\
	81,\
	82,\
	100,\
	101,\
	102,\
	103,\
	104,\
	165,\
	166,\
	167,\
	168,\
	169,\
	170,\
	171,\
	172,\
	173,\
	174,\
	175,\
	176,\
	177,\
	178,\
	179,\
	180,\
	181,\
	182,\
	183,\
	184,\
	185,\
	186,\
	187,\
	188,\
	189,\
	190,\
	191,\
	192,\
	193,\
	194,\
	195,\
	196,\
	197,\
	198,\
	199,\
	200,\
	201,\
	232,\
	233,\
	234,\
	256,\
	288,\
	320,\
	352,\
	384,\
	512}

#define ParameterNameList {"PARAMETER_CHECKSUM",\
	"IMAGE_CHECKSUM",\
	"IMAGE_LENGTH",\
	"SERIAL_NUMBER",\
	"DEVICE_TYPE_ARRAY",\
	"POSITION_CONVERSION",\
	"VELOCITY_CONVERSION",\
	"EFFORT_CONVERSION",\
	"CURRENT_CONVERSION",\
	"VOLTAGE_CONVERSION",\
	"TEMPERATURE_CONVERSION",\
	"INERTIA_CONVERSION",\
	"DAMPING_CONVERSION",\
	"STIFFNESS_CONVERSION",\
	"ROTATION_COUNTER",\
	"DEVICE_ADDRESS_ARRAY",\
	"DEVICE_TELEM_ARRAY",\
	"MS_TELEM_INDEX_ARRAY",\
	"DEBUG_LS_TELEM_INDEX_ARRAY",\
	"DEBUG_HS_TELEM_INDEX_ARRAY",\
	"SLEEP_TIMEOUT",\
	"MAJOR_VERSION_INDEX",\
	"MINOR_VERSION_INDEX",\
	"VARIANT_INDEX",\
	"REPOSITORY_REVISION_INDEX",\
	"BUILD_DATE_INDEX",\
	"BUILD_TIME_INDEX",\
	"MOTOR_CONTROLLER_SN",\
	"ERROR_SELECT_ARRAY",\
	"OVER_TEMP_ERROR_WARNING",\
	"OVER_TEMP_ERROR_LIMIT",\
	"OVER_VOLTAGE_ERROR_LIMIT",\
	"UNDER_VOLTAGE_ERROR_LIMIT",\
	"OVER_CURRENT_ERROR_LIMIT",\
	"POS_CAL_ERROR_LIMIT",\
	"POS_SENSOR_STARTUP_CYCLES",\
	"POS_SENSOR_DEFAULT_ADC",\
	"POS_SENSOR_MAX_STEP",\
	"POS_SENSOR_LOOKUP_OFFSET",\
	"EFFORT_SENSOR_SLOPE",\
	"EFFORT_SENSOR_OFFSET",\
	"BUS_CURRENT_SENSOR_SLOPE",\
	"BUS_CURRENT_SENSOR_OFFSET",\
	"BUS_VOLTAGE_SENSOR_SLOPE",\
	"BUS_VOLTAGE_SENSOR_OFFSET",\
	"COIL_TEMP_SENSOR_SLOPE",\
	"COIL_TEMP_SENSOR_OFFSET",\
	"PCB_TEMP_SENSOR_SLOPE",\
	"PCB_TEMP_SENSOR_OFFSET",\
	"POS_SENSOR_FILTER_WC",\
	"EFFORT_SENSOR_FILTER_WC",\
	"VELOCITY_EMF_FILTER_WC",\
	"BUS_VOLTAGE_SENSOR_FILTER_WC",\
	"VELOCITY_SENSOR_FILTER_WC",\
	"MOTOR_PARAM_ARRAY",\
	"MOTOR_BACK_EMF_CONSTANT",\
	"MOTOR_TERMINAL_RESISTANCE",\
	"GEAR_RATIO",\
	"POSITION_PID_KP",\
	"POSITION_PID_KI",\
	"POSITION_PID_KD",\
	"VELOCITY_PID_KP",\
	"VELOCITY_PID_KI",\
	"VELOCITY_PID_KD",\
	"EFFORT_PID_KP",\
	"EFFORT_PID_KI",\
	"EFFORT_PID_KD",\
	"POSITION_LIMIT_MAX",\
	"POSITION_LIMIT_MIN",\
	"VELOCITY_LIMIT",\
	"CURRENT_LIMIT",\
	"IMPEDANCE_EFFORT_MAX",\
	"HOLD_TIMEOUT",\
	"HOLD_VELOCITY",\
	"HOLD_CURRENT",\
	"HOLD_POSITION_ERROR",\
	"HOLD_EFFORT",\
	"DRIFT_ALPHA",\
	"DRIFT_MAX",\
	"ACCELERATION_LIMIT",\
	"MOTOR_INDUCTANCE",\
	"WIPER_POSITION",\
	"DELAY_SAMPLES",\
	"POSITION_ERROR_DEADBAND",\
	"EFFORT_SENSOR_DEADBAND",\
	"BUS_VOLTAGE_NOMINAL",\
	"BUS_VOLTAGE_REGEN_EPS",\
	"REGEN_DIRECT_CURRENT",\
	"BRAKE_RELEASE_VOLTAGE",\
	"BRAKE_RUN_VOLTAGE",\
	"BRAKE_TANSITION_TIME",\
	"HALL_MIDPOINT",\
	"HALL_FLAG_A",\
	"HALL_FLAG_B",\
	"LOOKUP_TABLE_1_INDEX",\
	"LOOKUP_TABLE_2_INDEX",\
	"LOOKUP_TABLE_3_INDEX",\
	"LOOKUP_TABLE_4_INDEX",\
	"LOOKUP_TABLE_5_INDEX",\
	"LOOKUP_TABLE_6_INDEX"}
	
#define ParameterDataTypeList {UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	ARRAY_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	ARRAY_PARAMETER,\
	ARRAY_PARAMETER,\
	UINT8_PARAMETER,\
	UINT8_PARAMETER,\
	UINT8_PARAMETER,\
	FLOAT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	ARRAY_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	ARRAY_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	UINT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	UINT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	FLOAT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT32_PARAMETER,\
	UINT16_PARAMETER}

typedef struct DevAddrArray {
	unsigned addr			: 8;
	unsigned master		: 8;
	unsigned slave		: 8;
	unsigned spare		: 8;
} DevAddrArray;

typedef struct DevTelemArray {
	unsigned hs				: 1;
	unsigned ms				: 1;
	unsigned ls				: 1;
	unsigned debug		: 1;
	unsigned slave		: 1;
	unsigned spare		: 3;
	unsigned hs_rate	: 8;
	unsigned ms_rate	: 8;
	unsigned ls_rate	: 8;
} DevTelemArray;

typedef struct MSTelemSelArray {
	unsigned ind0		: 8;
	unsigned ind1		: 8;
	unsigned ind2		: 8;
	unsigned ind3		:	8;
} MSTelemSelArray;

typedef struct DebugHSTelemSelArray {
	unsigned ind0		: 8;
	unsigned ind1		: 8;
	unsigned spare	: 16;
} DebugHSTelemSelArray;

typedef struct DebugLSTelemSelArray {
	unsigned ind0		: 8;
	unsigned ind1		: 8;
	unsigned ind2		: 8;
	unsigned spare	: 8;
} DebugLSTelemSelArray;

typedef struct ErrorSelectArray {
	unsigned over_temp		: 1;
	unsigned over_volt		: 1;
	unsigned under_volt		: 1;
	unsigned over_current	: 1;
	unsigned init_error		: 1;
	unsigned sensor_error	: 1;
	unsigned wrong_dir		: 1;
	unsigned slave_error	: 1;
	unsigned pos_cal			: 1;
	unsigned spare				: 23;
} ErrorSelectArray;

typedef struct PosSensorSelArray {
	unsigned lookup_table	: 1;
	unsigned mech_est			: 1;
	unsigned drift_corr		: 1;
	unsigned spare				: 29;
} PosSensorSelArray;

typedef struct MotorParamArray {
	unsigned joint_sign_rev	: 1;
	unsigned multi_cycle_cal: 1;
	unsigned spare2					: 1;
	unsigned spare3					: 1;
	unsigned spare4					: 1;
	unsigned spare5					: 1;
	unsigned spare6					: 1;
	unsigned spare7					: 1;
	unsigned motor_poles		: 8;
	unsigned ol_period			: 8;
	unsigned spare24				: 8;
} MotorParamArray;

typedef struct DevTypeArray {
	unsigned end_effector			: 1;
	unsigned linear_drive			: 1;
	unsigned continuous_rot		: 1;
	unsigned temp_limit				: 1;
	unsigned temp_res_adj			: 1;
	unsigned coms_master			: 1;
	unsigned regen_en					: 1;
	unsigned spare7						: 1;
	unsigned control_mode			: 8;
	unsigned spare16					: 8;
	unsigned spare24					: 8;
} DevTypeArray;

typedef struct HallAlignmentArray {
	unsigned pos						: 16;
	unsigned width					: 16;
} HallAlignmentArray;

typedef struct HallDirectionArray {
	unsigned ind0						: 4;
	unsigned ind1						: 4;
	unsigned ind2						: 4;
	unsigned ind3						: 4;
	unsigned ind4						: 4;
	unsigned ind5						: 4;
	unsigned ind6						: 4;
	unsigned ind7						: 4;
} HallDirectionArray;


#endif // adroit_params_h
