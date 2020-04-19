#ifndef SOCKETCAN_INTERFACE_H
#define SOCKETCAN_INTERFACE_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <fcntl.h>

#include "ros2_odrive_can/odrive_can.hpp"

#define DEFAULT_READ_TIMOUT 500 // Microseconds
#define DEFAULT_CANID_MASK 0b11111 // Mask first 5 bits

class SocketcanInterface
{
public:
  SocketcanInterface(canid_t msg_id);
  SocketcanInterface(canid_t msg_id, uint16_t mask);
  SocketcanInterface(canid_t msg_id, uint16_t mask, uint32_t timeout_microseconds);
  ~SocketcanInterface();
  int readFrame(can_frame *frame);
  int writeFrame(can_frame frame);

private:
  int s;
  struct ifreq ifr;
  struct sockaddr_can addr;
  int openSocket(canid_t msg_id, uint16_t mask, uint32_t timeout_microseconds);

};

#endif
