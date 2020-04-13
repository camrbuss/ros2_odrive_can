
#include "ros2_odrive_can/socketcan_interface.hpp"

SocketcanInterface::SocketcanInterface(canid_t msg_id)
{
    // Constructor
    this->openSocket(msg_id);
}

SocketcanInterface::~SocketcanInterface()
{
    // This might not ever be run
    close(this->s);
}

int SocketcanInterface::openSocket(canid_t msg_id)
{
    // Open socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
        return -1;
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    can_filter filt;
    filt.can_id = msg_id;
    filt.can_mask = 0b11111 & CAN_SFF_MASK; // Mask to first 5 bits of 11 bit identifier
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt));

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int err = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0)
        return -1;

    return s;
}

can_frame SocketcanInterface::readFrame()
{
    // Initialize new can frame to store data in
    can_frame frame;

    read(this->s, &frame, sizeof(frame));

    return frame;
}

int SocketcanInterface::writeFrame(can_frame frame)
{
    return write(this->s, &frame, sizeof(frame));
}
