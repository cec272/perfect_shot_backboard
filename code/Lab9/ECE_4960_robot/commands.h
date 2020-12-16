#ifndef COMMANDS_H
#define COMMANDS_H
#include "BLE_example.h"

typedef enum
{
  NOT_A_COMMAND,
  SET_MOTORS,
  GET_MOTORS,
  RES_MOTORS,
  SER_TX,
  SER_RX,
  REQ_FLOAT,
  GIVE_FLOAT,
  PING,
  PONG,
  START_BYTESTREAM_TX,
  STOP_BYTESTREAM_TX,
  BYTESTREAM_TX,
  BYTESTREAM_RX
} __attribute__((__packed__)) cmd_type_e;

#pragma pack(push, 1)
typedef struct
{
  cmd_type_e command_type;
  uint8_t length;
  uint8_t data[97];
} cmd_t;
#pragma pack(pop)

typedef struct bt_debug_msg
{
  char *message;
  struct bt_debug_msg *next;
} bt_debug_msg_t;

extern bt_debug_msg_t *bt_debug_head;
extern bt_debug_msg_t *bt_debug_tail;

typedef struct
{
  int motorDriver;
  int ToF_sensor;
  int prox_sensor;
  int IMU;
} present_t;

void printOverBluetooth(String s);

/**
 * pushMessage(msg, len) is 0 if len characters of msg are able to be copied
 * to a fresh array. Returns -1 on failure for:
 * -malloc() failing
 * -msg is not null-terminated.
 */
int pushMessage(char *msg, int len);

/**
 * pullMessage() is a null-terminated string that is the oldest unread
 * message.
 */
char *pullMessage();

/**
 * availableMessage() 0 if no new messages are available, and nonzero otherwise.
 */
int availableMessage();

#endif