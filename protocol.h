#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Notifications pushed from the device to host

    // Sent upon device reset
    #define MSG_RESET           0xF0

    // Machine position update push
    #define MSG_POSITION        0xF1
        #define STATE_BIT_BUFFER_EMPTY 0  /* 0x01 Motion buffer empty */
        #define STATE_BIT_BUFFER_FULL  1  /* 0x02 Motion buffer full */
        #define STATE_BIT_MOVING       2  /* 0x04 Machine is moving */
        #define STATE_BIT_RUNNING      3  /* 0x08 Motion stream is being processed */
        #define STATE_BIT_LOST         4  /* 0x10 Machine was moving while motion was stopped */
        #define STATE_BIT_RES1         5  /* 0x20 */
        #define STATE_BIT_RES2         6  /* 0x40 */
        #define STATE_BIT_RES3         7  /* 0x80 */
       
        enum stop_reason_enum {
            STOP_REASON_NONE,
            STOP_REASON_RESET,
            STOP_REASON_COMMAND,
            STOP_REASON_UNDERRUN,
            STOP_REASON_LIMIT,
            STOP_REASON_PROBE,
            STOP_REASON_MAX,
        };

        struct position_block
        {
            //uint8_t state - actually saved in GPIOR0
            uint8_t inputs;
            uint8_t stop_reason;
            int32_t X;
            int32_t Y;
            int32_t Z;
        } __attribute__((packed));

    // Limit switch tripped, motion halted
    #define MSG_LIMIT_TRIPPED   0xF3

    // Probe switch tripped, motion halted
    #define MSG_PROBE_TRIPPED   0xF4

    // Internal error, system in panic mode
    #define MSG_FIRMWARE_PANIC  0xEE

// Generic responses
#define RES_ACK 0x01
#define RES_NAK 0x02

// Generic commands
#define CMD_NOP         0x00
#define CMD_ECHO        0x01

#define CMD_MOVE_STOP   0x02
#define CMD_MOVE_START  0x03

#define CMD_MOTORS_ON   0x04
#define CMD_MOTORS_OFF  0x05

#define CMD_SPINDLE_ON  0x06
#define CMD_SPINDLE_OFF 0x07

// Queue movement for execution
//  Parameters:
//      uint8_t : sequence number of movement mod 256
//      struct movement_block
//  Response:
//      RES_QUEUED   - the command was accepted to queue
//      ERR_FULL     - not queued, buffer is full
//      ERR_SEQUENCE - not queued, sequence number mismatch
//  all responses are followed by uint8_t sequence number of the NEXT EXPECTED movement

struct movement_block
{
    int8_t  nX;
    int8_t  nY;
    int8_t  nZ;
} __attribute__((packed));

#define CMD_MOVE_QUEUE  0x08
    #define RES_QUEUED      0x10
    #define ERR_FULL        0x11
    #define ERR_SEQUENCE    0x12

// Jog at constant speed
// Parameters:
//  struct movement_block - see CMD_MOVE_QUEUE
#define CMD_MOVE_JOG    0x09

// Reset machine coordinates to zero
#define CMD_ZERO_POSITION 0x0a

// Defined here because this is a common file
#define FIRMWARE_PANIC() for (;;) { cli(); UDR1 = MSG_FIRMWARE_PANIC; UDR0 = MSG_FIRMWARE_PANIC; }
#endif

