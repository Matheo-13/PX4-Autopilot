#include "mavlink_link.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

namespace move_platform_pad
{

MavlinkLink::MavlinkLink(uint16_t localPort, std::string px4Ip, uint16_t px4Port)
    : localPort(localPort), px4Ip(std::move(px4Ip)), px4Port(px4Port)
{
}

MavlinkLink::~MavlinkLink()
{
    this->stop();
}

void MavlinkLink::start()
{
    this->sockFd = socket(AF_INET, SOCK_DGRAM, 0);
    // TODO: check sockFd >= 0, handle error

    sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons(this->localPort);
    bind(this->sockFd, reinterpret_cast<sockaddr *>(&localAddr), sizeof(localAddr));
    // TODO: check bind() return value

    this->running = true;
    this->rxThread = std::thread(&MavlinkLink::recvLoop, this);
}

void MavlinkLink::stop()
{
    this->running = false;
    if (this->sockFd >= 0)
    {
        shutdown(this->sockFd, SHUT_RDWR);
        close(this->sockFd);
        this->sockFd = -1;
    }
    if (this->rxThread.joinable())
    {
        this->rxThread.join();
    }
}

void MavlinkLink::recvLoop()
{
    uint8_t buffer[2048];
    mavlink_message_t msg;
    mavlink_status_t status;

    sockaddr_in srcAddr{};
    socklen_t srcLen = sizeof(srcAddr);

    while (this->running)
    {
        const ssize_t n = recvfrom(this->sockFd, buffer, sizeof(buffer), 0,
                                    reinterpret_cast<sockaddr *>(&srcAddr), &srcLen);
        if (n <= 0)
        {
            continue;  // TODO: distinguish timeout/EINTR vs real errors
        }

        for (ssize_t i = 0; i < n; ++i)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
            {
                this->handleMessage(msg);
            }
        }
    }
}

void MavlinkLink::handleMessage(const mavlink_message_t &msg)
{
    if (!this->haveTarget && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        // Learn PX4's sysid/compid from its first heartbeat so we can
        // address it directly for COMMAND_LONG etc.
        this->targetSysId = msg.sysid;
        this->targetCompId = msg.compid;
        this->haveTarget = true;
    }

    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            if (this->onHeartbeat)
            {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                this->onHeartbeat(hb);
            }
            break;
        }
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        {
            if (this->onExtendedSysState)
            {
                mavlink_extended_sys_state_t ext;
                mavlink_msg_extended_sys_state_decode(&msg, &ext);
                this->onExtendedSysState(ext);
            }
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            if (this->onLocalPosition)
            {
                mavlink_local_position_ned_t pos;
                mavlink_msg_local_position_ned_decode(&msg, &pos);
                this->onLocalPosition(pos);
            }
            break;
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            if (this->onPositionTarget)
            {
                mavlink_position_target_local_ned_t tgt;
                mavlink_msg_position_target_local_ned_decode(&msg, &tgt);
                this->onPositionTarget(tgt);
            }
            break;
        }
        case MAVLINK_MSG_ID_EVENT:
        {
            if (this->onEvent)
            {
                mavlink_event_t ev;
                mavlink_msg_event_decode(&msg, &ev);
                this->onEvent(ev);
            }
            break;
        }
        default:
            break;
    }
}

void MavlinkLink::requestMessageInterval(uint32_t mavlinkMsgId, float rateHz)
{
    if (!this->haveTarget)
    {
        return;  // wait for first heartbeat before addressing PX4 directly
    }

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        /*system_id=*/255, /*component_id=*/1, &msg,
        this->targetSysId, this->targetCompId,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(mavlinkMsgId),
        rateHz > 0 ? 1e6f / rateHz : -1.0f,
        0, 0, 0, 0, 0);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buf, &msg);

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(this->px4Port);
    inet_pton(AF_INET, this->px4Ip.c_str(), &dest.sin_addr);

    sendto(this->sockFd, buf, len, 0, reinterpret_cast<sockaddr *>(&dest), sizeof(dest));
}

}  // namespace move_platform_pad
