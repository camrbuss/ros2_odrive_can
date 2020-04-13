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

#include "ros2_odrive_can/odrive_can.hpp"

class SocketcanInterface
{
public:
  SocketcanInterface(canid_t msg_id);
  ~SocketcanInterface();
  can_frame readFrame();
  int writeFrame(can_frame frame);


private:
  int s;
  struct ifreq ifr;
  struct sockaddr_can addr;
  int openSocket(canid_t msg_id);

};

#endif
