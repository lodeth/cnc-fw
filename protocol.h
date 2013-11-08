#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#ifndef __AVR__
#define CLIENT_INCLUDE
#endif

#ifdef __GNUC__
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif


// ---------------------------------------------------------------------------------------------------
//
//  Notifications pushed from the device to host
//      Fields are little endian
//
// ---------------------------------------------------------------------------------------------------

// Machine position update push
#define MSG_POSITION        0xF1
    #define STATE_BIT_BUFFER_EMPTY 0  /* 0x01 Motion buffer empty */
    #define STATE_BIT_RUNNING      1  /* 0x02 Motion stream is being processed */
    #define STATE_BIT_MOVING       2  /* 0x04 Machine is moving */
    #define STATE_BIT_RES1         3  /* 0x08 */

    #define STATE_BIT_RES2         4  /* 0x10 */
    #define STATE_BIT_RES3         5  /* 0x20 */
    #define STATE_BIT_RES4         6  /* 0x40 */
    #define STATE_BIT_ESTOP        7  /* 0x80 Machine is halted because ESTOP was triggered */
   
    enum stop_reason_enum {
        STOP_REASON_NONE,
        STOP_REASON_RESET,
        STOP_REASON_COMMAND,
        STOP_REASON_UNDERRUN,
        STOP_REASON_LIMIT,
        STOP_REASON_PROBE,
        STOP_REASON_ESTOP,
        STOP_REASON_MAX,
    };

#pragma pack(push, 1)
    struct status_block
    {
#ifdef CLIENT_INCLUDE
        uint8_t state;  // saved in GPIOR0 on AVR
        uint8_t outputs;
#endif
        uint8_t inputs; // already inverted as per input mask
        uint8_t free_slots; // free slots in motion queue
        uint8_t tag;    // tag of active movement
        uint8_t stop_reason;
        int32_t X;
        int32_t Y;
        int32_t Z;
    } PACKED_STRUCT;
#pragma pack(pop)

// Internal error, system in panic mode
#define MSG_FIRMWARE_PANIC  0xEE

// ---------------------------------------------------------------------------------------------------
//
//  Generic responses to commands
//
// ---------------------------------------------------------------------------------------------------

#define RES_ACK             0x01    /* Command accepted */
#define RES_NAK             0x02    /* Command rejected */
#define RES_UNKNOWN         0x03    /* Unknown command */
#define RES_ESTOP           0x04    /* Command not allowed in ESTOP state */

// ---------------------------------------------------------------------------------------------------
//
//  Generic commands allowed always
//
// ---------------------------------------------------------------------------------------------------

#define CMD_NOP             0x00    /* Do nothing. Won't even send an ACK */
#define CMD_PING1           0x01    /* Do nothing but send an RES_PING1 */
    #define RES_PING1           0x0F
#define CMD_PING2           0x02    /* Do nothing but send an RES_PING2 */
    #define RES_PING2           0x0E
#define CMD_SET_ESTOP       0x03    /* Set ESTOP condition */
#define CMD_CLEAR_ESTOP     0x04    /* Attempt to clear ESTOP condition. */
#define CMD_GET_STATE       0x05    /* Request status from machine */

#define CMD_DEBUG           0x06    /* Do random stuff */
#define CMD_DEBUG2          0x07

// ---------------------------------------------------------------------------------------------------
//
//  Generic commands not allowed in ESTOP
//
// ---------------------------------------------------------------------------------------------------

#define CMD_MOTORS_ON       0x10    /* Assert STEPPER_ENABLE */
#define CMD_MOTORS_OFF      0x11    /* Deassert STEPPER_ENABLE */
#define CMD_SPINDLE_ON      0x12    /* Assert STEPPER_SPINDLE */
#define CMD_SPINDLE_OFF     0x13    /* Deassert STEPPER_SPINDLE */
#define CMD_SET_ZERO_POS    0x14    /* Reset machine coordinates to zero immediately */

// ---------------------------------------------------------------------------------------------------
//
//  Movement commands
//      Parameters are little endian
//
// ---------------------------------------------------------------------------------------------------

#pragma pack(push, 1)
struct movement_block {
    int8_t  X;
    int8_t  Y;
    int8_t  Z;
} PACKED_STRUCT; 
#pragma pack(pop)

#define CMD_MOVE_STOP       0x20    /* Start processing queued movement commands */
#define CMD_MOVE_START      0x21    /* Stop movement */

// Jog at constant speed
// Parameters:
//  struct movement_block
#define CMD_MOVE_JOG        0x22    /* Set motion to desired speeds. Buggy. */

// Queue movement for execution, response is followed by MSG_POSITION
//  Parameters:
//      uint8_t count
//      count * (struct movement_block)
//  Response:
//      RES_QUEUED
//          uint8_t count - count of moves queued, may be zero
//          uint8_t tag   - tag of first accepted move,
//                          wraps around at MOVEMENT_QUEUE_SIZE
//                          undefined value if count is zero

#define CMD_MOVE_QUEUE      0x23
    #define RES_QUEUED          0x20

// ---------------------------------------------------------------------------------------------------
//
//  Commands allowed only in ESTOP, mostly configuration
//
// ---------------------------------------------------------------------------------------------------

#define CMD_SET_ACTIVE_LOW  0x30
    #define ACTIVE_LOW_BIT_ESTOP   1
    #define ACTIVE_LOW_BIT_LIMITS  2
    #define ACTIVE_LOW_BIT_PROBE   3
    #define ACTIVE_LOW_BIT_STEP    4
    #define ACTIVE_LOW_BIT_DIR     5  
    #define ACTIVE_LOW_BIT_ENABLE  6  
    #define ACTIVE_LOW_BIT_SPINDLE 7  


#endif

