#ifndef _FOC_MOTOR_OD_H_
#define _FOC_MOTOR_OD_H_

#include <stdint.h>


//
// Object Dictionary Definitions
//

// Atributes of memory address
#define ATTR_READ   1
#define ATTR_WRITE  2
#define ATTR_RW     3

// Size of memory address
#define MEM_SZ_8  1
#define MEM_SZ_16 2
#define MEM_SZ_32 4
#define MEM_SZ_64 8
#define MEM_SZ_FLOAT 4

// Object Dictionary Entry
typedef struct {
	uint8_t RdWrAtt;   // RdWrAtt: (0) 0x00 Reserved, (1) 0b01 Only Read, (2) 0x10 Only Write, (3) 0x11 Read and Write
	uint8_t MemSize;   // Number of bytes: 1,2,4,8. Floats are 4 bytes long.
	void   *ptrData;   // Pointer to MemDirection
} Motor_OD_Entry_t;

// Commands for accesing objects
#define COM_MOTOR_CMD_READ                   0x0A
#define COM_MOTOR_CMD_WRITE                  0x0B

// Errors while accessing objects
#define ERR_MOTOR_OD_NOT_READ                0x01
#define ERR_MOTOR_OD_NOT_WRITE               0x02
#define ERR_MOTOR_OD_WRONG_SIZE              0x03
#define ERR_MOTOR_OD_WRONG_ADD               0x04
#define ERR_MOTOR_OD_WRONG_CRC               0x05


//
// Object Dictionry - Addresses
//

// Node ID, the unique ID for this motor, 1 byte, RW
#define OD_NODEID                            0x000
// Status Word, actual motor status, 2 bytes, RO
#define OD_STATUS_WORD                       0x001
// Command Word, when we want the motor to do something, 2 bytes, WO
#define OD_COMMAND_WORD                      0x002

// Motor properties

// Actual applied voltage to the board VIN, float, RO
#define OD_VIN_VOLTAGE                       0x003
// Maximun allowed motor voltage, float, RW
#define OD_MOTOR_MAXVOLT                     0x004
// Voltage used by internal identification algorithm, float, RW
#define OD_MOTOR_VOLT_ID                     0x005
// Maximun allowed motor current, float, RW
#define OD_MOTOR_MAXCURR                     0x006
// Maximun allowed motor velocity, float, RW
#define OD_MOTOR_MAXVEL                      0x007
// Number of poles pairs, 1 byte, RW
#define OD_MOTOR_PP                          0x008
// Motor Zero Electrical Angle, float, RW
#define OD_MOTOR_ZERO_ELECT                  0x009
// Motor resistance, float, RW
#define OD_MOTOR_R                           0x00A
// Motor Inductance D, float, RW
#define OD_MOTOR_LD                          0x00B
// Motor Inductance Q, float, RW
#define OD_MOTOR_LQ                          0x00C
// Motor Encoder PPR, float, RW
#define OD_MOTOR_ENC_PPR                     0x00D
// Motor Encoder direction, int, RO
#define OD_MOTOR_ENC_DIR                     0x00E
// Motor Desired SetPoint for Actual Control Loop, 8 bytes, RW
#define OD_MOTOR_SETPOINT                    0x00F
// Motor Maximun velocity for Profile Position in RPM, 2 bytes, RW
#define OD_MOTOR_PROFILE_VEL                 0x010
// Motor aceleration time for Profile Position in ms, 2 bytes, RW
#define OD_MOTOR_PROFILE_TACEL               0x011
// Motor phases polarity
#define OD_MOTOR_PHASES_POL                  0x012

// Actual motor position in pulses, int64, 8 bytes, RO
#define OD_MOTOR_POS_PULSES                  0x013
// Actual velocity in RPM, int16, 2 bytes, RO
#define OD_MOTOR_VEL_RPM                     0x014
// Actual Id current in A, float, 4 bytes, RO
#define OD_MOTOR_CURRENT_ID_MA               0x015
// Actual Iq current in A, float, 4 bytes RO
#define OD_MOTOR_CURRENT_IQ_MA               0x016
// Motor PhaseA inverted, int16, 2 bytes, RO
#define OD_MOTOR_PHASEA_INVERTED             0x017
// Motor PhaseB inverted, int16, 2 bytes, RO
#define OD_MOTOR_PHASEB_INVERTED             0x018
// Motor PhasesA and B swapped, int16, 2 bytes, RO
#define OD_MOTOR_PHASES_SWAPPED              0x019

// Motor should be stop and disabled when PID or LPF properties are changed
// Ohterwise imprevisible motion may ocurr
// When Enabling Motor, automatically PID and LPF are updated

// Adjust control loop prescaler, 1 byte, RW
#define OD_MOTOR_CONTROL_LOOP_PS             0x01A
// Adjust Position PID, float, RW
#define OD_MOTOR_PID_POS_KP                  0x01B
#define OD_MOTOR_PID_POS_KI                  0x01C
#define OD_MOTOR_PID_POS_KD                  0x01D
#define OD_MOTOR_PID_POS_FF                  0x01E
#define OD_MOTOR_PID_POS_RAMP_LIM            0X01F
#define OD_MOTOR_PID_POS_LIMIT               0X020
// Adjust Velocity PID, float, RW
#define OD_MOTOR_PID_VEL_KP                  0x021
#define OD_MOTOR_PID_VEL_KI                  0x022
#define OD_MOTOR_PID_VEL_KD                  0x023
#define OD_MOTOR_PID_VEL_FF                  0x024
#define OD_MOTOR_PID_VEL_RAMP_LIM            0X025
#define OD_MOTOR_PID_VEL_LIMIT               0X026
// Adjust Current Id PID, float, RW
#define OD_MOTOR_PID_ID_KP                   0x027
#define OD_MOTOR_PID_ID_KI                   0x028
#define OD_MOTOR_PID_ID_KD                   0x029
#define OD_MOTOR_PID_ID_FF                   0x02A
#define OD_MOTOR_PID_ID_RAMP_LIM             0X02B
#define OD_MOTOR_PID_ID_LIMIT                0X02C
// Adjust Current Iq PID, float, RW
#define OD_MOTOR_PID_IQ_KP                   0x02D
#define OD_MOTOR_PID_IQ_KI                   0x02E
#define OD_MOTOR_PID_IQ_KD                   0x02F
#define OD_MOTOR_PID_IQ_FF                   0x030
#define OD_MOTOR_PID_IQ_RAMP_LIM             0X031
#define OD_MOTOR_PID_IQ_LIMIT                0X032

// Filters

// Tf in LPF velocity, float, RW
#define OD_MOTOR_LPFVEL_ALPHA                0x033
// Tf in LPF current Id, float, RW
#define OD_MOTOR_LPFID_ALPHA                 0x034
// Tf in LPF current Iq, float, RW
#define OD_MOTOR_LPFIQ_ALPHA                 0x035

// WIndow for controller 

// current window, float, RW
#define OD_MOTOR_CURRENT_WINDOW              0x036
// velocity window, float, RW
#define OD_MOTOR_VELOCITY_WINDOW             0x037
// position window, float, RW
#define OD_MOTOR_POSITION_WINDOW             0x038

// OUTPUT FUNCTIONS

// Firt output, float, RO
#define OD_MOTOR_OUPUT_1                     0x039
// Second output, float, RO
#define OD_MOTOR_OUPUT_2                     0x03A
// Third output, float, RO
#define OD_MOTOR_OUPUT_3                     0x03B
// Fourth output, float, 
#define OD_MOTOR_OUPUT_4                     0x03C

//
// TOTAL NUMBER OF REGISTER
//
#define OD_NUM_REGS                          0x03D


//
// MOTOR COMMANDS
//

// STOP MOTOR
#define MOT_COMMAND_STOP                     0xFF
// Clear Error bit
#define MOT_COMMAND_CLEAR                    0xFE
// Clear Error bit
#define MOT_COMMAND_RESET                    0xFD
// Enabling Motor
#define MOT_COMMAND_ENABLE                   0xFC
// Disable Motor
#define MOT_COMMAND_DISABLE                  0xFB

// Change motor control to Current
#define MOT_COMMAND_CURRENT_CONTROL          0x01
// Change mtor control to velocity  
#define MOT_COMMAND_VELOCITY_CONTROL         0x02
// Change motor control to Position
#define MOT_COMMAND_POSITION_CONTROL         0x03
// Change Motor Control to Porfile Position
#define MOT_COMMAND_PROFILE_POSITION_CONTROL 0x04
// Start Homming using current 
#define MOT_COMMAND_HOME_CURRENT             0x05
// Start Homming using velocity and microswitch
#define MOT_COMMAND_HOME_VELOCITY            0x06
// Home actual position
#define MOT_COMMAND_HOME_NOW                 0x07
// Start Profile Position Motion
#define MOT_COMMAND_START_PP_MOTION          0x10

// Align Current Sense
#define MOT_COMMAND_ALIGN_CURRENT            0x20
// Align Encoder an Find Zero Electrical Angle
#define MOT_COMMAND_ALIGN_ENCODER            0x21
// Identify Motor: R, Lq, Ld, Pole Pairs.
#define MOT_COMMAND_IDENTIFY                 0x22
// Tune PID in Current
#define MOT_COMMAND_TUNE_CURRENT             0x23
// Autotune PID in Velocity
#define MOT_COMMAND_TUNE_VEL                 0x24
// Autotune PID in Position
#define MOT_COMMAND_TUNE_POS                 0x25
// Measure time
#define MOT_COMMAND_CHECK_LOOP               0x26

// Save Data to Flash
#define MOT_COMMAND_SAVE_DATA                0x30
// Restore Data from Flash
#define MOT_COMMAND_RESTORE_DATA             0x31
// After updating Motor Params
#define MOT_COMMAND_PARAMS_CHANGED           0x32

//
// List of Errors
// 

// ****** Error while updating Encoder
// No magnetic field detected
#define MOT_ERROR_ENC_MAGNET                 0x01
// Error while computing CRC
#define MOT_ERROR_ENC_CRC                    0x02
// ****** Error in control-loop
#define MOT_ERROR_OVERCURRENT                0x03
// ****** General Error, Wrong Params
#define MOT_ERROR_WRONG_CONFIG               0x04
// ****** Error in Align Motor
// Delta motion too low
#define MOT_ERROR_ALIGN_DELTA                0x05
// Compute Pole-Pairs too high
#define MOT_ERROR_ALIGN_PP                   0x05
// ****** Error in Current Align
// No current detected in phase A
#define MOT_ERROR_CURRALIGN_NOPHA            0x07
// No current detected in phase B
#define MOT_ERROR_CURRALIGN_NOPHB            0x08

// Errors in Motor Identification
#define MOT_ERROR_IDENTIFY_I_NEG             0x10
#define MOT_ERROR_IDENTIFY_I_LOWER           0x11
#define MOT_ERROR_IDENTIFY_R                 0x12
#define MOT_ERROR_IDDENTIFY_LD_LQ            0x13
#define MOT_ERROR_IDDENTIFY_LQ_2LD           0x14
// Error in home
#define MOT_ERROR_TIMEOUT                    0x15
// Error cannot enable, invalid state
#define MOT_ERROR_CANNOT_ENABLE              0x16

// Error in Tunning Control Loops
#define MOT_ERROR_TUNE_VEL                   0x1A
#define MOT_ERROR_TUNE_POS                   0x1B
// Error when trying to Profile Position
#define MOT_ERROR_NOT_PROF_POS               0x1C
// Error trying to home when not posible
#define MOT_ERROR_CANNOT_HOME                0x1D
// Not Known Command
#define MOT_ERROR_UNKNOWN_COMMAND            0x1F

#endif
