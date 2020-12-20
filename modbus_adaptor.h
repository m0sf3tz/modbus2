/**
 * December 15, 2020, Sam Hoshyar
 * 
 * Copyright 2020 by PlanB eStorage Ltd.
 * All right reserved 
 *  
 * Semaphor helper functions
 */

#pragma once

#include <modbus.h>
#include <semaphore.h>
#include <stdbool.h>


//#define MODBUS_TCP 
#define MODBUS_BROADCAST_ID_RTU ( 0 )
#define MAX_BBUM2_COUNT ( 12 ) //TODO: don't hardcode
#define MODBUS_TCP_PORT (1501)

// ------------------------------------------------------------------ Includes
// ------------------------------------------------------------------ Definitions
#define MAX_IP4_LEN ( 16 ) //need 1 extra for null
#define MAX_MODBUS_TIMEOUT (1)
#define BROADCAST_BITS (0)
#define BROADCAST_HOLDING_REGISTER (1)
// ------------------------------------------------------------------ Type Definitions
typedef struct{
    modbus_t  *ctx;
    int       addr;
    int       count;
    void      *src;
    int       workeType;
} modbusWriteHoldingArg_t;

// ------------------------------------------------------------------ Function Prototypes

int      modbusSystemInit( sem_t * sem, const int bbu2Count, uint32_t ip, char *stty );
void     setModbusContext( int id );
int      getModbusContext( );

// Adaptors (will funnel into TCP/RTU)

// Direct communication
int modbusWriteHoldingRegistersAdaptor( int addr, int count, uint16_t *src  );
int modbusReadHoldingRegistersAdaptor ( int addr, int count, uint16_t *dest );
int modbusReadInputRegistersAdaptor   ( int addr, int count, uint16_t *dest );
int modbusReadBitsAdaptor             ( int addr, int count, uint8_t *dest  );
int modbusWriteBitsAdaptor            ( int addr, int count, uint8_t *src   );
int modbusReadInputBitsAdaptor        ( int addr, int count, uint8_t *dest  );

// Broadcast Adaptor
int modbusBroadCastHoldingRegistersAdaptor ( int addr, int count, uint16_t *src);
int modbusBroadCastBitsAdaptor             ( int addr, int count, uint8_t *src );

