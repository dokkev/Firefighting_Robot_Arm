/**
  ******************************************************************************
  ******************************************************************************
  * @file    AdroitComs.cpp
  * @author  HDT Robotics, Inc.
  ******************************************************************************
  ******************************************************************************
  */

#include <string.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <algorithm>

#include <hdt_adroit_debug/AdroitComs.h>
#include <hdt_adroit_debug/AdroitCrc.h>

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
AdroitComs::AdroitComs(uint8_t source) {
	//addr = source;
	memset(&seg_data, 0, sizeof(SegmentData));

	seg_crc = new AdroitCrc();
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
AdroitComs::~AdroitComs() {
}

/*----------------------------------------------------------------------------
  reset drive
 *----------------------------------------------------------------------------*/
void AdroitComs::reset_drive(AdroitDrive *drive) {
	// reset whole drive struct
	memset(drive, 0, sizeof(AdroitDrive));

	drive->status_telem.state = AdroitComs::NO_STATE;
	for(int i = 0; i < 16; i++) {
		drive->msg_update[i] = false;
	}
	drive->control = false;
	drive->commit_needed = false;

	// initialize semaphore
	//sem_init(&drive->semaphore, 1, 0);
	pthread_mutex_init(&drive->mutex, NULL);
	pthread_cond_init(&drive->cond_var, NULL);
}

/*----------------------------------------------------------------------------
  send state command
 *----------------------------------------------------------------------------*/
void AdroitComs::set_impedance_cmd(AdroitDrive *drive, double damping_ratio, double stiffness_ratio) {
	// set drive parmeters
	drive->impedance_cmd.inertia = drive->impedance_params.inertia;
	drive->impedance_cmd.stiffness = (float)stiffness_ratio*(drive->impedance_params.kmax - drive->impedance_params.kmin) + drive->impedance_params.kmin;
	drive->impedance_cmd.damping = (float)damping_ratio*2.0f*sqrt(drive->impedance_cmd.inertia*drive->impedance_cmd.stiffness);
}

/*----------------------------------------------------------------------------
  send state command
 *----------------------------------------------------------------------------*/
void AdroitComs::send_state_cmd(uint8_t addr, StateCmd *cmd) {
	AdroitMsg msg;
	// build header
	msg.id = (STATE_CHANGE_CMD << 7) | (addr & 0x7F);
	msg.dlc = 1;

	// set data
	msg.data[0] = cmd->state;

	// send msg
	SendMsg(&msg);

	// write immedately (non synchronous)
	Write();
}

/*----------------------------------------------------------------------------
  send control command
 *----------------------------------------------------------------------------*/
void AdroitComs::send_control_cmd(uint8_t addr, ControlCmd *cmd) {
	AdroitMsg msg;

	// build header
	msg.id = (CONTROL_CMD << 7) | (addr & 0x7F);
	msg.dlc = 8;

	// set data
	int16_t position_fp = (int16_t)(cmd->position * (float)INT16_MAX / POSITION_CONV);
	int16_t velocity_fp = (int16_t)(cmd->velocity * (float)INT16_MAX / VELOCITY_CONV);
	int16_t effort_fp = (int16_t)(cmd->effort * (float)INT16_MAX / EFFORT_CONV);
	uint16_t motor_current_fp = (int16_t)(cmd->motor_current * (float)INT16_MAX / CURRENT_CONV);

	msg.data[0] = position_fp & 0xFF;
	msg.data[1] = (position_fp >> 8) & 0xFF;
	msg.data[2] = velocity_fp & 0xFF;
	msg.data[3] = (velocity_fp >> 8) & 0xFF;
	msg.data[4] = effort_fp & 0xFF;
	msg.data[5] = (effort_fp >> 8) & 0xFF;
	msg.data[6] = motor_current_fp & 0xFF;
	msg.data[7] = (motor_current_fp >> 8) & 0xFF;

	// send msg
	SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  send impedance command
 *----------------------------------------------------------------------------*/
void AdroitComs::send_impedance_cmd(uint8_t addr, ImpedanceCmd *cmd) {
	AdroitMsg msg;

	// build header
	msg.id = (IMPEDANCE_CMD << 7) | (addr & 0x7F);
	msg.dlc = 6;

	// set data
	int16_t inertia_fp = (int16_t)(cmd->inertia * (float)INT16_MAX / INERTIA_CONV);
	int16_t damping_fp = (int16_t)(cmd->damping * (float)INT16_MAX / DAMPING_CONV);
	int16_t stiffness_fp = (int16_t)(cmd->stiffness * (float)INT16_MAX / STIFFNESS_CONV);

	msg.data[0] = inertia_fp & 0xFF;
	msg.data[1] = (inertia_fp >> 8) & 0xFF;
	msg.data[2] = damping_fp & 0xFF;
	msg.data[3] = (damping_fp >> 8) & 0xFF;
	msg.data[4] = stiffness_fp & 0xFF;
	msg.data[5] = (stiffness_fp >> 8) & 0xFF;

	// send msg
	SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  send current command
 *----------------------------------------------------------------------------*/
void AdroitComs::send_current_cmd(uint8_t addr, CurrentCmd *cmd) {
	AdroitMsg msg;

	// build header
	msg.id = (CURRENT_CMD << 7) | (addr & 0x7F);
	msg.dlc = 2;

	// set data
	int16_t current_fp = (int16_t)(cmd->motor_current * (float)INT16_MAX / CURRENT_CONV);

	msg.data[0] = current_fp & 0xFF;
	msg.data[1] = (current_fp >> 8) & 0xFF;

	// send msg
	SendMsg(&msg);
}

/*----------------------------------------------------------------------------
  send parameter command
 *----------------------------------------------------------------------------*/
void AdroitComs::send_parameter_cmd(uint8_t addr, ParameterMsg *cmd) {
	AdroitMsg msg;

	// build header
	msg.id = (PARAMETER_CMD << 7) | (addr & 0x7F);
	msg.dlc = 8;

	// set data
	//msg.data[0] = cmd->s & 0x01;
	//msg.data[0] |= (cmd->exp & 0x01) << 1;
	//msg.data[0] |= ((4 - cmd->num) & 0x03) << 2;
	//msg.data[0] |= (cmd->ccs & 0x07) << 5;

	msg.data[0] = cmd->head;
	msg.data[1] = cmd->index & 0xFF;
	msg.data[2] = (cmd->index >> 8) & 0xFF;
	msg.data[3] = cmd->sub_index & 0xFF;
	msg.data[4] = cmd->data & 0xFF;
	msg.data[5] = (cmd->data >> 8) & 0xFF;
	msg.data[6] = (cmd->data >> 16) & 0xFF;
	msg.data[7] = (cmd->data >> 24) & 0xFF;

	// send msg
	SendMsg(&msg);

	// write immedately (non synchronous)
	Write();
}

/*----------------------------------------------------------------------------
  send status command
 *----------------------------------------------------------------------------*/
void AdroitComs::send_status_cmd(uint8_t addr) {
	AdroitMsg msg;

	// build header
	msg.id = (STATUS_CMD << 7) | (addr & 0x7F);
	msg.dlc = 0;
	
	// send msg
	SendMsg(&msg);

	// write immedately (non synchronous)
	Write();
}

/*----------------------------------------------------------------------------
  process bit error telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_error_telem(BITTelem *telem, AdroitMsg *msg) {
	uint8_t value_type;
	uint32_t value;

	// check message length
	if(msg->dlc != 6) {
		return false;
	}

	// set values
	telem->error_code = msg->data[0];
	telem->error_severity = msg->data[1] & 0x0F;
	value_type = (msg->data[1] & 0xF0) >>4;
	value = msg->data[2] + (msg->data[3] << 8) + (msg->data[4] << 16) + (msg->data[5] << 24);
	switch(value_type){
		case 0:
			// error value is uint32
			telem->error_value = value;
			telem->error_value_float = 0;
			break;
		case 1:
			// error value is float32
			memcpy(&telem->error_value_float, &value, 4);
			telem->error_value = 0;
			break;
		default:
			// error value is unknown type
			telem->error_value_float = NAN;
			break;
	}
	return true;
}

/*----------------------------------------------------------------------------
  process high speed telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_hs_telem(HSTelem *telem, AdroitMsg *msg) {
	// check message length
	if(msg->dlc != 8) {
		return false;
	}

	// set values
	int16_t position_fp = msg->data[0] | (msg->data[1] << 8);
	int16_t velocity_fp = msg->data[2] | (msg->data[3] << 8);
	int16_t effort_fp = msg->data[4] | (msg->data[5] << 8);
	int16_t motor_current_fp = msg->data[6] | (msg->data[7] << 8);

	telem->position = (float)position_fp / (float)INT16_MAX * POSITION_CONV;
	telem->velocity = (float)velocity_fp / (float)INT16_MAX * VELOCITY_CONV;
	telem->effort = (float)effort_fp / (float)INT16_MAX * EFFORT_CONV;
	telem->motor_current = (float)motor_current_fp / (float)INT16_MAX * CURRENT_CONV;

	return true;
}

/*----------------------------------------------------------------------------
  process medium speed telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_ms_telem(MSTelem *telem, AdroitMsg *msg) {
	// check message length
	if(msg->dlc != 8) {
		return false;
	}

	// set values
	telem->var1 = msg->data[0] | (msg->data[1] << 8);
	telem->var2 = msg->data[2] | (msg->data[3] << 8);
	telem->var3 = msg->data[4] | (msg->data[5] << 8);
	telem->var4 = msg->data[6] | (msg->data[7] << 8);

	return true;
}

/*----------------------------------------------------------------------------
  process low speed telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_ls_telem(LSTelem *telem, AdroitMsg *msg) {
	// check message length
	if(msg->dlc != 6) {
		return false;
	}

	// set values
	int16_t bus_voltage_fp = msg->data[0] | (msg->data[1] << 8);
	int16_t bus_current_fp = msg->data[2] | (msg->data[3] << 8);
	int16_t temperature_fp = msg->data[4] | (msg->data[5] << 8);

	telem->bus_voltage = (float)bus_voltage_fp / (float)INT16_MAX * VOLTAGE_CONV;
	telem->bus_current = (float)bus_current_fp / (float)INT16_MAX * CURRENT_CONV;
	telem->temperature = (float)temperature_fp / (float)INT16_MAX * TEMPERATURE_CONV;

	return true;
}

/*----------------------------------------------------------------------------
  process debug telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_debug_telem(DebugTelem *telem, AdroitMsg *msg) {
	//uint8_t index = msg->data[0];
	SegReadResHeader telem_head;
	bool ret = false;
	//static int good;
	//static int bad;

	// process header
	memcpy(&telem_head, &msg->data[0], 1);
	uint8_t telem_size = SEGMENT_MAX_BYTES - telem_head.n;

	// save data to buffer
	if((seg_data.bytes + telem_size) < SEGMENT_BUFFER_SIZE) {
		memcpy(&seg_data.buffer[seg_data.bytes], &msg->data[1], telem_size);
		seg_data.bytes += telem_size;

		// check for last message
		if(telem_head.c == 1) {
			telem->index = seg_data.buffer[0];
			telem->hs_samples = seg_data.buffer[1];
			telem->ls_var1 = seg_data.buffer[2] | (seg_data.buffer[3] << 8);
			telem->ls_var2 = seg_data.buffer[4] | (seg_data.buffer[5] << 8);
			telem->ls_var3 = seg_data.buffer[6] | (seg_data.buffer[7] << 8);
			memcpy(&telem->hs_var1[0], &seg_data.buffer[8], 32);
			memcpy(&telem->hs_var2[0], &seg_data.buffer[40], 32);
			memcpy(&telem->crc, &seg_data.buffer[72], sizeof(uint32_t));

			// calculate 32 bit checksum, check for match
			uint32_t crc = seg_crc->crc32(&seg_data.buffer[0], (sizeof(DebugTelem) - sizeof(uint32_t)));
			if(crc == telem->crc) {
				ret = true;
				//good++;
			}
			else {
				ret = false;
				//bad++;
			}

			memset(&seg_data, 0, sizeof(SegmentData));
		}
	}
	// segment transfer failed
	else {
		memset(&seg_data, 0, sizeof(SegmentData));
	}

	return ret;
}

/*----------------------------------------------------------------------------
  process parameter telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_parameter_telem(ParameterMsg *telem, AdroitMsg *msg) {
	uint8_t scs;
	bool ret = false;

	// check message length
	if(msg->dlc != 8) {
		return false;
	}

	// parse scs
	scs = (msg->data[0] & 0xE0) >> 5;

	// take scs-dependent action
	switch(scs) {
	case(INITIATE_READ_RES):
		telem->index = ((msg->data[1] | (msg->data[2] << 8)) & (~APPLICATION_PARAMETER));
		memcpy(&telem->data, &msg->data[4], 4);
		ret = true;
		break;
	case(INITIATE_WRITE_RES):
		ret = true;
		break;
	case(SEGMENT_READ_RES):
		break;
	case(BLOCK_WRITE_RES):
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}

/*----------------------------------------------------------------------------
  process status telemetry
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_status_telem(StatusTelem *telem, AdroitMsg *msg) {
	// check message length
	if(msg->dlc != 1) {
		return false;
	}

	// set values
	telem->state = msg->data[0];

	return true;
}

/*----------------------------------------------------------------------------
  process control command
 *----------------------------------------------------------------------------*/
bool AdroitComs::process_control_cmd(ControlCmd *cmd, AdroitMsg *msg) {
	// check message length
	if(msg->dlc != 8) {
		return false;
	}

	// set values
	int16_t position_fp = msg->data[0] | (msg->data[1] << 8);
	int16_t velocity_fp = msg->data[2] | (msg->data[3] << 8);
	int16_t effort_fp = msg->data[4] | (msg->data[5] << 8);
	int16_t motor_current_fp = msg->data[6] | (msg->data[7] << 8);

	cmd->position = (float)position_fp / (float)INT16_MAX * POSITION_CONV;
	cmd->velocity = (float)velocity_fp / (float)INT16_MAX * VELOCITY_CONV;
	cmd->effort = (float)effort_fp / (float)INT16_MAX * EFFORT_CONV;
	cmd->motor_current = (float)motor_current_fp / (float)INT16_MAX * CURRENT_CONV;

	return true;
}

/*----------------------------------------------------------------------------
  read parameter
 *----------------------------------------------------------------------------*/
int AdroitComs::read_parameter(int addr, int index, AdroitDrive *drive) {
	AdroitComs::ParameterMsg cmd;
	AdroitComs::InitReadReqHeader head;
//	printf("read drive %d param %d\n", addr, index);

	// consume semaphore if already released
	//int ret = sem_trywait(&drive->semaphore);
	int ret;

	// set parameter cmd data
	cmd.index = AdroitComs::APPLICATION_PARAMETER | index;
	cmd.sub_index = 0;
	head.ccs = AdroitComs::INITIATE_READ_REQ;
	head.x = 0;
	memcpy(&cmd.head, &head, sizeof(cmd.head));
	memset(&cmd.data, 0, sizeof(cmd.data));

	// send parameter read request
	send_parameter_cmd(addr, &cmd);

	// wait for response
	drive->msg_update[PARAMETER_TELEM] = false;
	ret = timed_wait(drive, PARAM_WAIT_SHORT);
	if((ret == -1) || (drive->msg_update[PARAMETER_TELEM] == false)) {
//		printf("read failed %d, msg_update = %d\n", addr, drive->msg_update[PARAMETER_TELEM]);
		return 0;
	}
	else {
		return 1;
	}
}

/*----------------------------------------------------------------------------
  write parameter
 *----------------------------------------------------------------------------*/
int AdroitComs::write_parameter(int addr, int index, AdroitDrive *drive, uint32_t *data) {
	AdroitComs::ParameterMsg cmd;
	AdroitComs::InitWriteReqHeader head;
//	printf("write drive %d param %d\n", addr, index);

	// consume semaphore if already released
	//int ret = sem_trywait(&drive->semaphore);
	int ret;

	// set parameter cmd data
	cmd.index = AdroitComs::APPLICATION_PARAMETER | index;
	cmd.sub_index = 0;
	head.ccs = AdroitComs::INITIATE_WRITE_REQ;
	head.x = 0;
	head.e = 1;
	head.n = 0;
	head.s = 1;
	memcpy(&cmd.head, &head, sizeof(cmd.head));
	memcpy(&cmd.data, data, sizeof(cmd.data));

	// send parameter read request
	send_parameter_cmd(addr, &cmd);

	// wait for response
	drive->msg_update[PARAMETER_TELEM] = false;
	ret = timed_wait(drive, PARAM_WAIT_SHORT);
	if((ret == -1) || (drive->msg_update[PARAMETER_TELEM] == false)) {
//		printf("write failed %d, msg_update = %d\n", addr, drive->msg_update[PARAMETER_TELEM]);
		return 0;
	}
	else {
		// commit needed
		drive->commit_needed = true;

		return 1;
	}
}

/*----------------------------------------------------------------------------
  commit parameters
 *----------------------------------------------------------------------------*/
int AdroitComs::commit_parameters(int addr, AdroitDrive *drive) {
	AdroitComs::ParameterMsg cmd;
	AdroitComs::InitWriteReqHeader head;

	// consume semaphore if already released
	//int ret = sem_trywait(&drive->semaphore);
	int ret;

	// set parameter cmd data
	cmd.index = AdroitComs::APPLICATION_PARAMETER;
	cmd.sub_index = 0;
	head.ccs = AdroitComs::INITIATE_WRITE_REQ;
	head.x = 0;
	head.e = 1;
	head.n = 0;
	head.s = 1;
	memcpy(&cmd.head, &head, sizeof(cmd.head));
	memset(&cmd.data, 0, sizeof(cmd.data));

	// send parameter read request
	send_parameter_cmd(addr, &cmd);

	// wait for response
	drive->msg_update[PARAMETER_TELEM] = false;
	ret = timed_wait(drive, PARAM_WAIT_SHORT);
	if((ret == -1) || (drive->msg_update[PARAMETER_TELEM] == false)) {
	  //printf("commit failed %d, GetLastError() = %d, msg_update = %d\n", ret, GetLastError(), drive->msg_update[PARAMETER_TELEM]);
	  return 0;
	}
	else {
	  // committed
	  drive->commit_needed = false;
	  return 1;
	}
}

/*----------------------------------------------------------------------------
  load application image
 *----------------------------------------------------------------------------*/
int AdroitComs::load_application_image(uint8_t addr, std::vector<unsigned char> data, AdroitDrive *drive) {
	AdroitMsg msg;
	AdroitComs::ParameterMsg param_msg;
	AdroitComs::BlkInitReqHeader init_head;
	AdroitComs::BlkWriteReqHeader write_head;
	AdroitComs::BlkEndReqHeader end_head;

	// consume semaphore if already released
	//int ret = sem_trywait(&drive->semaphore);
	int ret;

	// check application length
	if(data.size() == 0) {
		printf("invalid application image E0...\n");
		return 0;
	}

	printf("loading application image...\n");
	printf("\n|");

	// init msg
	msg.id = (AdroitComs::PARAMETER_CMD << 7) | (addr & 0x7F);
	msg.dlc = 8;

	// initiate block transfer
	// set header
	init_head.ccs = AdroitComs::BLOCK_WRITE_REQ;
	init_head.x = 0;
	init_head.cc = 1;
	init_head.s = 1;
	init_head.cs = AdroitComs::BLOCK_INITIATE;
	memcpy(&param_msg.head, &init_head, 1);

	// set indicies
	param_msg.index = 0x4000;
	param_msg.sub_index = 0x00;

	// set data size
	param_msg.data = data.size();

	// send first msg
	send_parameter_cmd(addr, &param_msg);

	// check for response
	ret = timed_wait(drive, PARAM_WAIT_LONG);
	if(ret == -1) {
		printf("\napplication load unsuccessful E1...\n");
		return 0;
	}

	// calculate number of blocks to send
	int num_blocks = data.size()/BLOCK_MAX_BYTES;
	if(data.size() % BLOCK_MAX_BYTES) num_blocks += 1;
	int bytes_sent = 0;
	int block_size = 0;
	int bytes_rem = 0;

	// iterate through blocks
	for(int i = 0; i < num_blocks; i++) {

		// test for missing last block
		if(i == (num_blocks - 1)) {
			i = i;
		}

		bytes_rem = data.size() - bytes_sent;

		// get block size
		if(bytes_rem >= BLOCK_MAX_BYTES) {
			block_size = BLOCK_MAX_BYTES;
		}
		else {
			block_size = bytes_rem;
		}

		// calculate number of segments in this block
		int num_segs = block_size/SEGMENT_MAX_BYTES;
		if(block_size % SEGMENT_MAX_BYTES) num_segs += 1;

		// send most of the data segments
		for(int j = 0; j < (num_segs - 1); j++) {
			// set header
			write_head.seqno = j + 1;
			write_head.c = 0;
			memcpy(&msg.data[0], &write_head, 1);

			// copy data
			memset(&msg.data[1], 0, SEGMENT_MAX_BYTES);
			for(int k = 0; k < SEGMENT_MAX_BYTES; k++) {
				msg.data[k + 1] = data.at(i*BLOCK_MAX_BYTES + j*SEGMENT_MAX_BYTES + k);
				//printf("%d\r\n", i*BLOCK_MAX_BYTES + j*SEGMENT_MAX_BYTES + k);
			}
			bytes_sent += SEGMENT_MAX_BYTES;

			// send msg
			SendMsg(&msg);
			Write();

			// wait to send next message
			//Sleep(2);
			usleep(2000);
		}

		// send last data segment
		// set header
		write_head.seqno = num_segs;
		write_head.c = 1;
		memcpy(&msg.data[0], &write_head, 1);

		// update remaining bytes
		memset(&msg.data[1], 0, SEGMENT_MAX_BYTES);
		bytes_rem = data.size() - bytes_sent;
		for(int j = 0; j < std::min(bytes_rem, SEGMENT_MAX_BYTES); j++) {
			msg.data[j + 1] = data.at(i*BLOCK_MAX_BYTES + (num_segs - 1)*SEGMENT_MAX_BYTES + j);
			//printf("%d\r\n", i*BLOCK_MAX_BYTES + (num_segs - 1)*SEGMENT_MAX_BYTES + j);
			bytes_sent++;
		}

		// send msg
		SendMsg(&msg);
		Write();

		// check for response
		ret = timed_wait(drive, PARAM_WAIT_SHORT);
		if(ret == -1) {
			printf("\napplication load unsuccessful E2...\n");
			return 0;
		}

		// update display
		if(i % (num_blocks/10) == 0) {
			//printf("-*-");
		}
	}

	// end block transfer
	// set header
	end_head.ccs = AdroitComs::BLOCK_WRITE_REQ;
	end_head.x = 0;
	end_head.n = bytes_rem;
	end_head.cs = AdroitComs::BLOCK_END;
	memcpy(&msg.data[0], &end_head, 1);

	// calculate 32 bit checksum and set as bytes 1-4
	AdroitCrc adroit_crc;
	uint32_t crc = adroit_crc.crc32(data);
	memcpy(&msg.data[1], &crc, 4);

	// send msg
	SendMsg(&msg);
	Write();

	// check for response
	ret = timed_wait(drive, PARAM_WAIT_SHORT);
	if(ret == -1) {
		printf("\napplication load unsuccessful E3...\n");
		return 0;
	}

	return 1;

	// finish display
	//printf("|\n");
	//printf("\napplication load complete!\n");
}

/*----------------------------------------------------------------------------
  timed wait
 *----------------------------------------------------------------------------*/
int AdroitComs::timed_wait(AdroitDrive *drive, int sec) {
	int ret;
	
	// create timespec
	timespec tm;
	clock_gettime(CLOCK_REALTIME, &tm);
	tm.tv_sec += sec;

	// wait for cond var
	pthread_mutex_lock(&drive->mutex);
	ret = pthread_cond_timedwait(&drive->cond_var, &drive->mutex, &tm);
	pthread_mutex_unlock(&drive->mutex);

	return ret;
}
