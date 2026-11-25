#include <stdio.h>
#include <string.h>

#include "foc_time.h"
#include "foc_motor_od.h"
#include "foc_comm.h"

#include "n32l40x.h"

// DEBUG:
// which USART to use
#define USART485 USART1
#define USART485_IRQHandler USART1_IRQHandler

// PD15 as INPUT / OUTPUT Selection
// PD15 as LOW  <- INPUT (RECEIVING)
// PD15 as HIGH -> OUTPUT (SENDING)
#define Start_Receiving() GPIOD->PBC = GPIO_PIN_15
#define Start_Sending() GPIOD->PBSC = GPIO_PIN_15

// Local pointer to External Object Dictionary
__IO Motor_OD_Entry_t *ptr_motor_OD = 0;


//
// SERIAL COMMAND PROTOCOL 
//

// BYTE00: (ID Code 0b101) + (NodeID: 0x00001 to 0x1-1111) maximun 30 slaves
// BYTE01: (Read 'A', Write 'B') + (R/W: Size max 8 byte) Ex. 0xA4, 0xB8
// BYTE02: (Memory Dir from 0x00 to 0xFF) 
// BYTE03: UserByte0
// BYTE04: UserByte1
// BYTE05: UserByte2
// BYTE06: UserByte3
// BYTE07: CRC-8

//
// Local data
//

// Incoming Buffer
#define BUFFER_SIZE 32
__IO uint8_t RxBufferCOMM[BUFFER_SIZE];
__IO uint8_t TxBufferCOMM[BUFFER_SIZE];
// Buffer Index
// Index for writing incoming byte to buffers
__IO uint8_t RxIndex = 0x00;
// If the message has a different NodeID identifier, then discard the whole message
__IO uint8_t RxDiscardBytes = 0x00;
// Index for sending bytes from buffer to output
__IO uint8_t TxIndex = 0x00;
// Maximun number of bytes to send
__IO uint8_t TxBytesToSend = 0x00;
// store local time each 100ms
__IO int16_t comm_starttime  = 0x00;
__IO int16_t comm_actualtime = 0x00;


// Read a value from the object dictionary
// input:  memory address
//         pointer to store the data from memory address
// output: size of memory address
//         copy of memory address
// return value: Error if set
int com485_Read_OD(uint8_t memadr, uint8_t *size, void *data) {
	// check atribute
	uint8_t att = ptr_motor_OD[memadr].RdWrAtt;
	// ATT = bit1 (1-Write) - bit0 (1-Read)
	// Return error is not readable (bit0)
	if ( (att & 1) != 1 ) return ERR_MOTOR_OD_NOT_READ;

	// Set size
	*size = ptr_motor_OD[memadr].MemSize;
	// copy data
	memcpy( data, ptr_motor_OD[memadr].ptrData, *size);
	
	return 0;	
}

// Write a value in the object dictionary
// input:  memory address
//         pointer to the data to be written
//         number of bytes in the pointer
// return value: Error if set, otherwise the data is copied
int com485_Write_OD(uint8_t memdir, uint8_t size, void *data) {
	// check atribute
	uint8_t att = ptr_motor_OD[memdir].RdWrAtt;
	// ATT = bit1 (1-Write) - bit0 (1-Read)
	// return an error if not writable (bit1)
	if ( (att >> 1) != 1 ) return ERR_MOTOR_OD_NOT_WRITE;

	// check size
	uint8_t sz = ptr_motor_OD[memdir].MemSize;
	// return an error if different size
	if (size != sz ) return ERR_MOTOR_OD_WRONG_SIZE;

	// copy variable value to dictonary
	memcpy( ptr_motor_OD[memdir].ptrData, data, size);
	
	return 0;	
}

// Compute CRC-8
uint8_t com485_CRC8(uint8_t *buffer, int length) {
	uint8_t crc = 0x00;
	for (int i=0; i<length; i++) {
		uint8_t extract = buffer[i];
		for (int bit=0; bit<8; bit++) {
			uint8_t sum = (crc ^ extract) & 0x01;
			crc = crc >> 1;
			if (sum > 0) crc = crc ^ 0x8C;
			extract = extract >> 1;
		}
	}
	return crc;
}

// Enable interrupt for sending bytes
void com485_Send_Bytes( uint8_t bytes ) {
	// Reset incoming buffer
	RxIndex = 0;
	RxDiscardBytes = 0;
	// Set bytes to send
	TxIndex = 0;
	TxBytesToSend = bytes;
	// No timeout
	comm_starttime = 0;
	// PD15 to HIGH, Start sending bytes
	Start_Sending();
	// update int flags
	USART_ConfigInt(USART485, USART_INT_TXDE, ENABLE);
	USART_ConfigInt(USART485, USART_INT_TXC, ENABLE);
	USART_ConfigInt(USART485, USART_INT_RXDNE, DISABLE);
}

// Set ErrorCode in Msg
void com485_Set_Error( uint8_t ErrorCode ) {
	// update Msg Code and Msg length
	// Command to Error Msg (0xC0) + 0x01 byte, the errorcode
	TxBufferCOMM[1] = 0xC1;
	TxBufferCOMM[3] = ErrorCode;
	// Compute Message CRC
	TxBufferCOMM[4] = com485_CRC8( (uint8_t *)TxBufferCOMM, 4 );
	// Send 5 bytes
	com485_Send_Bytes( 5 );
}

// USART Interrupt Handler
void USART485_IRQHandler( void ) {
	//
	// Receive interrupt, when a byte is received
	//
	if ( USART_GetIntStatus( USART485, USART_INT_RXDNE ) != RESET )  {
		// set actual time at 100 ms multipliers
		comm_actualtime = time_get_us() / 100000;
		// check time out at 500 ms
		if ( comm_starttime != 0 && (comm_actualtime - comm_starttime) > 5 ) {
			// Time-out, discard the previous data
			RxIndex = 0;
			RxDiscardBytes = 0;
		}
		// store incoming byte
		RxBufferCOMM[RxIndex++] = USART_ReceiveData( USART485 );
		// restart timeout after each received byte
		comm_starttime = comm_actualtime;
		// Security, check limits, maybe never happens
		if ( RxIndex >= BUFFER_SIZE ) RxIndex = 0;
		// Bytes to discard ?
		// Discard the bytes after the whole message arrive
		if ( (RxDiscardBytes) && (RxIndex >= RxDiscardBytes) ) {
			for (int i = 0; i < RxIndex - RxDiscardBytes; i++ ) {
				// copy buffer after discarted message
				RxBufferCOMM[i] = RxBufferCOMM[i + RxDiscardBytes];
			}
			// update Index
			RxIndex -= RxDiscardBytes;
			RxDiscardBytes = 0;
		}
			
		// Process incoming bytes ....
		// Wait for at least 2 incoming bytes
		if (RxIndex < 2) goto rx_leave;

		// Get actual motor NodeID
		uint8_t NodeID = *((uint8_t*)ptr_motor_OD[OD_NODEID].ptrData);
		// Message FrameID
		uint8_t FrameID = RxBufferCOMM[0] >> 5;
		// Message NodeID
		// From 1 to 30
		uint8_t msgNodeID = RxBufferCOMM[0] & 0x1F;
		// Message Command
		uint8_t CommRW = RxBufferCOMM[1] >> 4;
		// Message Size
		uint8_t MsgSize = RxBufferCOMM[1] & 0x0F;
		// Message Size with 3 bytes in Header and 1 bytes in CRC
		uint8_t len = MsgSize + 4;

		// Is this a valid FrameID ?
		if (FrameID != 0b101 ) {
			// Error in communication
			// Discard the first byte
			RxDiscardBytes = 1;
			goto rx_leave;
		}
			
		// Is this the slave which has to answer ?
		if ( msgNodeID != NodeID ) {
			// Discard the whole message
			RxDiscardBytes = len;
			goto rx_leave;
		}

		// Wait for the whole incoming message
		if ( RxIndex < len ) goto rx_leave;

		// -------- Send answer -------------------------------------

		// Echo answer
		TxBufferCOMM[0] = RxBufferCOMM[0];        // Frame + NodeID
		TxBufferCOMM[1] = RxBufferCOMM[1] & 0xF0; // Command + 0 bytes in Size
		TxBufferCOMM[2] = RxBufferCOMM[2];        // Mem Address

		// Get Incoming CRC
		uint8_t CRCInc = RxBufferCOMM[len-1];
		// Check incoming CRC-16
		uint8_t CRCval = com485_CRC8( (uint8_t *)RxBufferCOMM, len-1 );
		// Check invalid incoming CRC
		if (CRCval != CRCInc) {
			// Communication error ...
			com485_Set_Error( ERR_MOTOR_OD_WRONG_CRC );
			goto rx_leave;
		}
			
		// Mem Address
		uint8_t memdir = RxBufferCOMM[2];
		
		// No valid Address
		if (memdir >= OD_NUM_REGS) {
			// set error to invalid address
			com485_Set_Error( ERR_MOTOR_OD_WRONG_ADD );
			goto rx_leave;
		}

		// WRITE command
		// Len is 0 in answer to the write command, no userbytes in answer
		len = 0;
		// Bytes to write in mem adress are in MsgSize
		// If error then len=1
		if ( CommRW == COM_MOTOR_CMD_WRITE ) {
			// Data Addres
			void *data = (void*) &RxBufferCOMM[3];
			// write 'MsgSize' userbytes to mem address
			int Err = com485_Write_OD(memdir, MsgSize, data);
			if ( Err ) {
				// set answer to send
				com485_Set_Error( Err );
				goto rx_leave;
			}
		}

		// READ command
		// UserBytes read are written in len
		// Except for error where len=1
		if ( CommRW == COM_MOTOR_CMD_READ ) {
			// Data Addres
			void *data = (void*) &TxBufferCOMM[3];
			// copy 'len' bytes to userbytes from mem address
			int Err = com485_Read_OD(memdir, &len, data);
			if ( Err ) {
				// set answer to send
				com485_Set_Error( Err );
				goto rx_leave;
			}
		}
			
		// Normal reply if no errors happens 
		// len was already set 0 for Write, a nBytes for Read
		// 3 bytes required for message header: byte0, byte1, byte2.
		// 1 bytes required at end: CRC-8
		TxBufferCOMM[1]    |= len;
		// Compute CRC in ANSWER
		TxBufferCOMM[len+3] = com485_CRC8( (uint8_t *)TxBufferCOMM, len+3 );
		// Send answer
		com485_Send_Bytes( len+4 );
	}
	
	// 
	// Transmit interrupt, byte to send when buffer is empty
	//
	if ( USART_GetIntStatus( USART485, USART_INT_TXDE ) != RESET )  {
		// send next byte
		if ( TxIndex < TxBytesToSend ) {
			// Send from Tx Buffer
			USART_SendData( USART485, TxBufferCOMM[TxIndex++] );
		};
	}
	
	//
	// Transmit end interrupt, after sending the answer
	//
	if ( USART_GetIntStatus( USART485, USART_INT_TXC ) != RESET )  {
		// Clear bit
		USART_ClrIntPendingBit(USART485, USART_INT_TXC);
		// The whole message was sent, no more bytes to send
		TxBytesToSend = 0;
		TxIndex = 0;
		// PD15 to LOW, start receiving bytes
		Start_Receiving();
		// Start Receiving
		RxIndex = 0;
		// update int flags
		USART_ConfigInt(USART485, USART_INT_RXDNE, ENABLE);
		USART_ConfigInt(USART485, USART_INT_TXDE, DISABLE);
		USART_ConfigInt(USART485, USART_INT_TXC, DISABLE);
	}

// point to resume 
rx_leave:
	
	/* Determine if an error flag still exists and clear the error flag */
	if ((USART_GetFlagStatus(USART485, USART_FLAG_OREF) != RESET)  || \
			(USART_GetFlagStatus(USART485, USART_FLAG_NEF) != RESET)  || \
			(USART_GetFlagStatus(USART485, USART_FLAG_PEF) != RESET)  || \
			(USART_GetFlagStatus(USART485, USART_FLAG_FEF) != RESET) ){
			/*Read the sts register first,and the read the DAT register to clear the all error flag*/
			(void)USART485->STS;
			(void)USART485->DAT;
	}

}


// Register the pointer to the Object Dictionary
void com485_init(Motor_OD_Entry_t *ptrOD) {
	// local pointer to OD
	ptr_motor_OD = ptrOD;
	
	// Init module
	Start_Receiving();
	// Set index
	RxIndex = 0;
	TxIndex = 0;
	RxDiscardBytes = 0;
	TxBytesToSend = 0;
	// No timeout
	comm_starttime = 0;
	// Clear flags
	USART_ClrFlag(USART485, USART_FLAG_RXDNE | USART_FLAG_TXC );
	// Update int flags
	USART_ConfigInt(USART485, USART_INT_TXDE, DISABLE);
	USART_ConfigInt(USART485, USART_INT_TXC, DISABLE);
	USART_ConfigInt(USART485, USART_INT_RXDNE, ENABLE);
}
