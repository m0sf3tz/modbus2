#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>
#include <semaphore.h>
#include <stdbool.h>
#include <pthread.h>
#include <arpa/inet.h>

#include "tracelog.h"
#include "sem.h"
#include "debug.h"
#include "modbus_adaptor.h"

// --------------------------------------------------------- Type Definitions
// ----------------------------------------------------- Forward Declarations
static int modbusWriteHoldingRegisters( modbus_t *ctx, int addr, int count, uint16_t *src  );
static int modbusWriteBits            ( modbus_t *ctx, int addr, int count, uint8_t  *dest );

// ---------------------------------------------------------------- Constants
// --------------------------------------------------------- Static Variables
#ifdef MODBUS_TCP
static modbus_t *ctx_arr_tcp[12];
#else
static           modbus_t *ctx_rtu;
#endif
static int       cbbumId = -1;
static sem_t     *modbus_sem = NULL;
static int       num_bbus;

// ----------------------------------------------------------- Implementation

// not production, used to fake a TCP "broadcast"
// ----------------------------------------------------------- sendFakeTcpBroadcastWorker
#ifdef MODBUS_TCP
static void *sendFakeTcpBroadcastWorker
(
    void *worker_args
)
{
    // can't return stack variables from pthread
    int *rc = malloc( sizeof(int) );

    modbusWriteHoldingArg_t *args = ( modbusWriteHoldingArg_t* )worker_args;
    if( args->workeType == BROADCAST_HOLDING_REGISTER )
    {
        *rc = modbusWriteHoldingRegisters( args->ctx, args->addr, args->count, (uint16_t*)args->src );
    }
    else if( args->workeType == BROADCAST_BITS )
    {
        *rc = modbusWriteBits( args->ctx, args->addr, args->count, (uint8_t*)args->src );
    }
    else
    {
        TLE( "Unknown worker type" );
        *rc = -1;
    }

    return (void *)rc;
}
#endif
// ----------------------------------------------------------- configureModbusContext
static int configureModbusContext
(
    modbus_t *ctx;
)
{
  // for TCP, we need to set a sensible timeout
  // default .5 Seconds, might leave it as is, tbd

  // for RTU, we need to set a sensible timeout 
  // and set MODBUS_RTU_RS485, we need to see if
  // the hardware has a RS482 <-> RS232 or what,
  // TBD

  return 0;
}

// ----------------------------------------------------------- modbusInit
static int modbusInit
(
    const char *ip,
    const int  bbu_2_count,
    const char *stty,
    const int  baud
)
{
    int rc;

    if ( 0 > bbu_2_count || bbu_2_count > MAX_BBUM2_COUNT )
    {
      TLE( "bbu_2_count set wrong = %d", bbu_2_count );
      return -1;
    }

    num_bbus = bbu_2_count;

#ifdef MODBUS_TCP
    if ( ip == NULL )
    {
       TLE( "IP set wrong!" );
       return -1;
    }
    TLV( "Setting up TCP MODBUS with ip = %s, bbu_2_count = %d", ip, bbu_2_count);

    modbus_t *tmp_ctx;
    int i;

    for ( i = 0; i < bbu_2_count ; i++ )
    {
        tmp_ctx = modbus_new_tcp( ip, MODBUS_TCP_PORT + i );
        if ( tmp_ctx == NULL )
        {
            TLE( "Unable to create the libmodbus context, error = %s, errno = %d, context = %d",
              modbus_strerror( errno ), errno, getModbusContext() );
            return -1;
        }

        if ( modbus_connect( tmp_ctx ) == -1 )
        {
              TLE( "Connection failed: %s, ERRNO = %d, PORT = %d", modbus_strerror( errno ), errno, MODBUS_TCP_PORT + i );
              modbus_free( tmp_ctx );
              return -1;
        }


        rc = configureModbusContext();
        if ( rc != 0 )
        {
            TLE( "Failed to set the properties of the modbus driver!" );
            return -1;
        }

        ctx_arr_tcp[i] = tmp_ctx;
    }
#else
    if ( stty == NULL )
    {
        TLE( "STTY == NULL!" );
        ABORT_ALWAYS();
    }

    // Update with valid BAUD rates(tbd)
    if ( baud != 9600 )
    {
        TLE( "Baud rate not supported = %d! ", baud );
        ABORT_ALWAYS();
    }

    ctx_rtu = modbus_new_rtu( stty, baud, 'N', 8, 1 );
    if ( ctx_rtu == NULL )
    {
        TLE( "Unable to create the libmodbus context, error = %s, errno = %d, context = %d",
            modbus_strerror( errno ), errno, getModbusContext() );
        return -1;
    }

    if ( modbus_connect( ctx_rtu ) == -1 )
    {
        TLE( "Connection failed: %s, ERRNO = %d", modbus_strerror( errno ), errno );
        modbus_free( ctx_rtu );
        return -1;
    }

    rc = configureModbusContext();
    if ( rc != 0 )
    {
        TLE( "Failed to set the properties of the modbus driver!" );
        return -1;
    }
#endif
    return 0;
}

// ----------------------------------------------------------- modbusWriteHoldingRegisters
static int modbusWriteHoldingRegisters
(
    modbus_t  *ctx,
    int       addr,
    int       count,
    uint16_t  *src
)
{
    if( ctx == NULL )
    {
        TLE( "cxt == NULL! \n" );
        ABORT_ALWAYS();
    }

    int rc = modbus_write_registers( ctx, addr, count, src );
    if ( rc == -1 ) {
        TLE ( "MODBUS ERROR ON WRITE, ERRNO HUMAN = %s ERRONO = \n, CONTEXT = %d",
            modbus_strerror( errno ), errno, getModbusContext() );
        return -1;
    }

    return ( 0 );
}

// ----------------------------------------------------------- modbusReadHoldingRegisters
static int modbusReadHoldingRegisters
(
    modbus_t *ctx,
    int addr,
    int count,
    uint16_t *val
)
{
    if( ctx == NULL )
    {
        TLE ( "cxt == NULL!\n" );
        ABORT_ALWAYS();
    }

    int rc = modbus_read_registers( ctx, addr, count, val );
    if ( rc == -1 )
    {
        TLE ( "MODBUS ERROR ON READ %s, errno = %d", modbus_strerror(errno), errno );
        return -1;
    }

    return( rc );
}

// ----------------------------------------------------------- modbusReadBits
static int modbusReadBits
(
    modbus_t *ctx,
    int       addr,
    int       count,
    uint8_t   *dest
)
{
    if( ctx == NULL )
    {
        TLE ( "cxt == NULL!\n" );
        ABORT_ALWAYS();
    }

    int rc = modbus_read_bits( ctx, addr, count, dest );
    if ( rc == -1 )
    {
        TLE ( "MODBUS ERROR ON READ BITS %s, errno = %d", modbus_strerror(errno), errno );
        return -1;
    }

    return( rc );
}

// ----------------------------------------------------------- modbusReadInputBits
static int modbusReadInputBits
(
    modbus_t *ctx,
    int       addr,
    int       count,
    uint8_t   *dest
)
{
    if( ctx == NULL )
    {
        TLE ( "cxt == NULL!\n" );
        ABORT_ALWAYS();
    }

    int rc = modbus_read_input_bits( ctx, addr, count, dest );
    if ( rc == -1 )
    {
        TLE ( "MODBUS ERROR ON READ INPUT BITS %s, errno = %d", modbus_strerror(errno), errno );
        return -1;
    }

    return( rc );
}

// ----------------------------------------------------------- modbusWriteBits
static int modbusWriteBits
(
    modbus_t *ctx,
    int       addr,
    int       count,
    uint8_t   *src
)
{
    if( ctx == NULL )
    {
        TLE ( "cxt == NULL!\n" );
        ABORT_ALWAYS();
    }

    int rc = modbus_write_bits( ctx, addr, count, src );
    if ( rc == -1 )
    {
        TLE ( "MODBUS ERROR ON WRITE BITS %s, errno = %d", modbus_strerror(errno), errno );
        return -1;
    }

    return( rc );
}


// -------------------------------------------------------------- modbusReadHoldingRegistersAdaptor 
int modbusReadHoldingRegistersAdaptor
(
    int      addr,
    int      count,
    uint16_t *dest
)
{
    int rc;
    const int id = getModbusContext();

    if ( 0 > count || count > MODBUS_MAX_WR_READ_REGISTERS )
    {
        TLE ( "Register Read count incorrect = %d", count );
        ABORT_ALWAYS();
    }

    if( dest == NULL )
    {
        TLE ( "dest is null!" );
        ABORT_ALWAYS();
    }

    rc = sem_timedwait_helper( MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL );
    if( rc != 0 )
    {
        TLE ( "Failed to get mutex to write registers with id = %d", id );
        ABORT_ALWAYS();
    }

#ifdef MODBUS_TCP
   rc =  modbusReadHoldingRegisters ( ctx_arr_tcp[id], addr, count, dest );
   goto cleanup;
#else
    rc = modbus_set_slave( ctx_rtu, id );
    if ( rc != 0 )
    {
      TLE ("Failed to set the ID for context %d", id );
      goto cleanup_abort;
    }
    rc =  modbusReadHoldingRegisters ( ctx_rtu, addr, count, dest );
    goto cleanup;
#endif

cleanup:
   sem_post( modbus_sem );
   return rc;

#ifndef MODBUS_TCP
cleanup_abort:
   sem_post ( modbus_sem );
   ABORT_ALWAYS();
   return -1; // will not get here, to stop compiler from complaining
#endif
}

// -------------------------------------------------------------- modbusBroadCastHoldingRegistersAdaptor
int modbusBroadCastHoldingRegistersAdaptor
(
    int       addr,
    int       count,
    uint16_t  *src
)
{
    int rc;
    int ret = 0;

    if ( 0 > count || count > MODBUS_MAX_WR_READ_REGISTERS )
    {
        TLE (" Broadcast holding register write has incorrect write count = %d", count );
        ABORT_ALWAYS();
    }

    if( src == NULL )
    {
        TLE ( "src is null!" );
        ABORT_ALWAYS();
    }

    rc = sem_timedwait_helper( MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL );
    if( rc != 0 )
    {
        TLE ("Failed to get mutext to broadcast in modbusBroadCastHoldingRegistersAdaptor");
        ABORT_ALWAYS();
    }
#ifdef MODBUS_TCP
    //          ***IMPORTANT***
    // TCP IS A ONE-TO-ONE PROTOCOL
    // IT DOES NOT SUPPORT BROADCAST, THIS IS A
    // POOR HACK TO ALLOW SOME TESTING, ONLY
    // RTU/UDP SUPPORTS BROADCAST, WHEN A CLIENT
    // BROADCASTS IT DOES NOT EXPECT A RESPONSE,
    // HOWEVER WE NEED TO SIT AND WAIT FOR A
    // RESPONSE SINCE WE ARE USING TCP, TO BETTER
    // SERIALIZE THIS PROCESS WE WILL SEND THE
    // BROADCAST FROM IT'S OWN THREAD
    //
    // For simplicity, this function does not do advanced
    // (or in cases, any) error recovery - it's only
    // run during simulations

    pthread_t                threads[num_bbus];
    modbusWriteHoldingArg_t  worker_args[num_bbus];

    int i;
    for( i = 0; i < num_bbus; i++ )
    {
        // Note: worker_args is not copied when pthread_create is called
        worker_args[i].ctx       = ctx_arr_tcp[i];
        worker_args[i].addr      = addr;
        worker_args[i].count     = count;
        worker_args[i].src       = src;
        worker_args[i].workeType = BROADCAST_HOLDING_REGISTER;

        rc = pthread_create( &threads[i], NULL, sendFakeTcpBroadcastWorker, (void *)&worker_args[i] );
        if ( rc )
        {
            TLE( "Failed to create worker thread for broadcast");
            goto cleanup_abort;
        }
    }

    // wait for all the threads to join
    for ( i = 0; i < num_bbus; i++ )
    {
        int *returnVal = NULL;
        pthread_join( threads[i], (void**)&returnVal );

        if( returnVal == NULL )
        {
            TLE( "pthread returned NULL!" );
            goto cleanup_abort;
        }

        if( *returnVal == -1 )
        {
            ret = -1;
        }

        free( returnVal );
    }
    goto cleanup;
#else
    // set the slave ID to BROADCAST
    rc = modbus_set_slave( ctx_rtu, MODBUS_BROADCAST_ID_RTU );
    if ( rc != 0 )
    {
      TLE ("Failed to set the ID for broadcast!" );
      goto cleanup_abort;
    }

    ret = modbusWriteHoldingRegisters(ctx_rtu, addr, count, src);
    goto cleanup;
#endif

cleanup:
   sem_post( modbus_sem );
   return ret;

cleanup_abort:
   sem_post ( modbus_sem );
   ABORT_ALWAYS ();
   return -1;
}

// -------------------------------------------------------------- modbusBroadCastBitsAdaptor
int modbusBroadCastBitsAdaptor
(
    int       addr,
    int       count,
    uint8_t  *src
)
{
    int rc;
    int ret = 0;

    if ( 0 > count || count > MODBUS_MAX_WRITE_BITS )
    {
        TLE (" Broadcast bits write has incorrect write count = %d", count );
        ABORT_ALWAYS();
    }

    if( src == NULL )
    {
        TLE ( "src is null!" );
        ABORT_ALWAYS();
    }

    rc = sem_timedwait_helper( MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL );
    if( rc != 0 )
    {
        TLE ("Failed to get mutext to broadcast in modbusBroadCastBitsAdaptor");
        ABORT_ALWAYS();
    }
#ifdef MODBUS_TCP
    //          ***IMPORTANT***
    // TCP IS A ONE-TO-ONE PROTOCOL
    // IT DOES NOT SUPPORT BROADCAST, THIS IS A
    // POOR HACK TO ALLOW SOME TESTING, ONLY
    // RTU/UDP SUPPORTS BROADCAST, WHEN A CLIENT
    // BROADCASTS IT DOES NOT EXPECT A RESPONSE,
    // HOWEVER WE NEED TO SIT AND WAIT FOR A
    // RESPONSE SINCE WE ARE USING TCP, TO BETTER
    // SERIALIZE THIS PROCESS WE WILL SEND THE
    // BROADCAST FROM IT'S OWN THREAD
    //
    // For simplicity, this function does not do advanced
    // (or in cases, any) error recovery - it's only
    // run during simulations

    pthread_t                threads[num_bbus];
    modbusWriteHoldingArg_t  worker_args[num_bbus];

    int i;
    for( i = 0; i < num_bbus; i++ )
    {
        // Note: worker_args is not copied when pthread_create is called
        worker_args[i].ctx       = ctx_arr_tcp[i];
        worker_args[i].addr      = addr;
        worker_args[i].count     = count;
        worker_args[i].src       = src;
        worker_args[i].workeType = BROADCAST_BITS;

        rc = pthread_create( &threads[i], NULL, sendFakeTcpBroadcastWorker, (void *)&worker_args[i] );
        if ( rc )
        {
            TLE( "Failed to create worker thread for broadcast");
            goto cleanup_abort;
        }
    }

    // wait for all the threads to join
    for ( i = 0; i < num_bbus; i++ )
    {
        int *returnVal = NULL;
        pthread_join( threads[i], (void**)&returnVal );

        if( returnVal == NULL )
        {
            TLE( "pthread returned NULL!" );
            goto cleanup_abort;
        }

        if( *returnVal == -1 )
        {
            ret = -1;
        }

        free( returnVal );
    }
    goto cleanup;
#else
    // set the slave ID to BROADCAST
    rc = modbus_set_slave( ctx_rtu, MODBUS_BROADCAST_ID_RTU );
    if ( rc != 0 )
    {
      TLE ("Failed to set the ID for broadcast!" );
      goto cleanup_abort;
    }

    ret = modbusWriteHoldingRegisters(ctx_rtu, addr, count, src);
    goto cleanup;
#endif

cleanup:
   sem_post( modbus_sem );
   return ret;

cleanup_abort:
   sem_post ( modbus_sem );
   ABORT_ALWAYS ();
   return -1;
}

// Write coil (rw)
// -------------------------------------------------------------- modbusWriteBitsAdaptor
int modbusWriteBitsAdaptor
(
    int addr, 
    int count,
    uint8_t *src 
)
{
    int rc;
    const int id = getModbusContext();

    if ( 0 > count || count > MODBUS_MAX_WRITE_BITS )
    {
        TLE ("write coil bit count incorrect = %d", count );
        ABORT_ALWAYS();  
    }

    if( src == NULL )
    {
        TLE ("src is null!");
        ABORT_ALWAYS();  
    }

    rc = sem_timedwait_helper(MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL);
    if ( rc != 0 )
    {
        TLE ("Could not get modbus mutex, context == %d for write coil", id);
        ABORT_ALWAYS();
    }

#ifdef MODBUS_TCP
    rc = modbusWriteBits( ctx_arr_tcp[id], addr, count, src );
    goto cleanup;
#else
    rc = modbus_set_slave( ctx_rtu, id );
    if ( rc != 0 ) 
    {
        TLE ("Failed to set the ID for context %d", getModbusContext() );
        goto cleanup_abort;
    }

    rc = modbusWriteBits ( ctx_rtu, addr, count, src );
    goto cleanup;
#endif

cleanup:
    sem_post( modbus_sem );
    return rc;

#ifndef MODBUS_TCP
cleanup_abort:
    sem_post ( modbus_sem );
    ABORT_ALWAYS ();
    return -1;
#endif
}

// Read input-bit (ro)
// -------------------------------------------------------------- modbusWriteBitsAdaptor
int modbusReadInputBitsAdaptor
(
    int addr,
    int count, 
    uint8_t *dest
)
{
    int rc;
    const int id = getModbusContext();

    if ( 0 > count || count > MODBUS_MAX_READ_BITS )
    {
        TLE ("Read input-bit count incorrect = %d", count );
        ABORT_ALWAYS();
    }

    if( dest == NULL )
    {
        TLE ("dest is null!");
        ABORT_ALWAYS();
    }

    rc = sem_timedwait_helper (MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL );
    if ( rc != 0 )
    {
        TLE ( "Could not get modbus mutex, context == %d for read-input-bit", id );
        ABORT_ALWAYS();
    }

#ifdef MODBUS_TCP
    rc = modbusReadInputBits( ctx_arr_tcp[id], addr, count, dest );
    goto cleanup;
#else
    rc = modbus_set_slave( ctx_rtu, id );
    if ( rc != 0 ) 
    {
        TLE ("Failed to set the ID for context %d", getModbusContext() );
        goto cleanup_abort;
    }

    rc = modbusReadInputBits( ctx_rtu, addr, count, dest );
    goto cleanup;
#endif

cleanup:
    sem_post( modbus_sem );
    return rc;

#ifndef MODBUS_TCP
cleanup_abort:
    sem_post ( modbus_sem );
    ABORT_ALWAYS ();
    return -1;
#endif

}

// Read coil (rw)
// -------------------------------------------------------------- modbusReadBitsAdaptor
int modbusReadBitsAdaptor 
(
    int       addr,
    int       count,
    uint8_t   *dest
)
{
    int rc;
    const int id = getModbusContext();

    if ( 0 > count || count > MODBUS_MAX_READ_BITS )
    {
        TLE ("Read coil bit count incorrect = %d", count );
        ABORT_ALWAYS();  
    }

    if( dest == NULL )
    {
        TLE ("dest is null!");
        ABORT_ALWAYS();  
    }

    rc = sem_timedwait_helper(MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL);
    if ( rc != 0 )
    {
        TLE ("Could not get modbus mutex, context == %d", getModbusContext);
        ABORT_ALWAYS();
    }

#ifdef MODBUS_TCP
    rc = modbusReadBits( ctx_arr_tcp[id], addr, count, dest );
    goto cleanup;
#else
    rc = modbus_set_slave( ctx_rtu, id );
    if ( rc != 0 )
    {
      TLE ("Failed to set the ID for context %d", getModbusContext() );
      goto cleanup_abort;
    }

    rc = modbusReadBits ( ctx_rtu, addr, count, dest );
    goto cleanup;
#endif

cleanup:
    sem_post( modbus_sem );
    return rc;
#ifndef MODBUS_TCP
cleanup_abort:
    sem_post ( modbus_sem );
    ABORT_ALWAYS ();
    return -1;
#endif
}

// -------------------------------------------------------------- modbusReadInputRegistersAdaptor 
int modbusWriteHoldingRegistersAdaptor
(
    int       addr,
    int       count,
    uint16_t  *src
)
{
    int rc;
    const int id = getModbusContext();

    if ( 0 > count || count > MODBUS_MAX_WR_WRITE_REGISTERS )
    {
        TLE ("Register Read count incorrect = %d", count );
        ABORT_ALWAYS();
    }

    if( src == NULL )
    {
        TLE ("src is null!");
        ABORT_ALWAYS();
    }

    rc = sem_timedwait_helper(MAX_MODBUS_TIMEOUT, modbus_sem, TRY_TO_RECOVER_ON_FAIL);
    if ( rc != 0 )
    {
        TLE ("Could not get modbus mutex, context == %d", id);
        ABORT_ALWAYS();
    }

#ifdef MODBUS_TCP
    rc = modbusWriteHoldingRegisters ( ctx_arr_tcp[id], addr, count, src);
    goto cleanup;
#else
    rc = modbus_set_slave( ctx_rtu, id );
    if ( rc != 0 ) 
    {
      TLE ("Failed to set the ID for context %d", id );
      goto cleanup_abort;
    }

    rc = modbusWriteHoldingRegisters ( ctx_rtu, addr, count, src );
    goto cleanup;
#endif

cleanup:
    sem_post( modbus_sem );
    return rc;
#ifndef MODBUS_TCP
cleanup_abort:
    sem_post ( modbus_sem );
    ABORT_ALWAYS ();
    return -1;
#endif
}

void setModbusContext
(
    int    id
)
{
    TLV( "Set modbus context to %d", id);
    if ( 0 > id || id > MAX_BBUM2_COUNT )
    {
        ABORT_ALWAYS();
    }
#ifdef MODBUS_TCP
    cbbumId = id;
#else
    // +1 is to make sure that cbbum does not use BROADCAST ID (0)
    cbbumId = id + 1;
#endif
}

// This is called PER CBBUM to set up the context 
// -------------------------------------------------------------- modbusSystemInit
int getModbusContext
(
)
{
    if ( cbbumId == -1 )
    {
        ABORT_ALWAYS();
    }

    return cbbumId;
}


// This is called ONCE by BBUM which will do all the top level
// initializations as required by modbus
// -------------------------------------------------------------- modbusSystemInit
int modbusSystemInit
(
    sem_t *sem,
    const int bbu2Count,
    uint32_t ip,
    char *stty
)
{
    static int init = - 1;
    if( init == -1 )
    {
      init = 1;
    }
    else
    {
        TLE ( "multiple calls to modbusSystemInit" );
        ABORT_ALWAYS();
    }
#ifdef MODBUS_TCP
    struct in_addr ip_addr;
    ip_addr.s_addr = ip;

    char * ip_str = inet_ntoa( ip_addr );
    if ( ip_str == (in_addr_t)(-1))
    {
        TLE ( "Failed to convert int to string for IP" );
        ABORT_ALWAYS();
    }

    TLV( "Will try to communicate with BBUS on %s", ip_str);
#else
    if ( stty == NULL )
    {
        TLE( "STTY == NULL!" );
        ABORT_ALWAYS();
    }
#endif

    if ( 0 > bbu2Count || bbu2Count > MAX_BBUM2_COUNT )
    {
        TLE ( "bbu2Count set wrong" );
        ABORT_ALWAYS();
    }

    TLV( "Setting up MODBUS" );

    if ( sem == NULL ) {
        TLE( "modbus_sem == NULL!" );
        ABORT_ALWAYS();
    }

    modbus_sem = sem;
#ifdef MODBUS_TCP
    return modbusInit( ip_str, bbu2Count, NULL, 0 );
#else
    return modbusInit( NULL, bbu2Count, stty , 9600 );
#endif
}
