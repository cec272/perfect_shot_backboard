#include "commands.h"

void printOverBluetooth(String s)
{
  cmd_t cmd = {
      SER_TX, 1};
  cmd.length = s.length() + 2;
  if (cmd.length > 97)
    cmd.length = 97;
  s.toCharArray((char *)cmd.data, 97);
  amdtpsSendData((uint8_t *)&cmd, cmd.length);
}

int pushMessage(char *msg, int len)
{
  if (msg[len - 1] != 0)
    return -1;
  bt_debug_msg_t *tmp_msg = (bt_debug_msg_t *)malloc(sizeof(bt_debug_msg_t));
  if (!tmp_msg)
    return -1;
  char *tmp_str = (char *)malloc(len * sizeof(char));
  if (!tmp_str)
  {
    free(tmp_msg);
    return -1;
  }
  tmp_msg->message = tmp_str;
  memcpy(tmp_msg->message, msg, len);
  tmp_msg->next = NULL;
  if (!bt_debug_tail)
  {
    bt_debug_head = tmp_msg;
    bt_debug_tail = tmp_msg;
  }
  else
  {
    bt_debug_tail->next = tmp_msg;
  }
  return 0;
}

char *pullMessage()
{
  if (!bt_debug_head)
    return NULL;
  bt_debug_msg_t *tmp_msg = bt_debug_head;
  char *tmp_str = bt_debug_head->message;
  bt_debug_head = bt_debug_head->next;
  if (bt_debug_head == NULL)
    bt_debug_tail = NULL;
  free(tmp_msg);
  return tmp_str;
}

int availableMessage()
{
  return (int)bt_debug_head;
}
