#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <mavlink.h>

namespace custom
{

class MavlinkLink
{
public:
    using ExtendedSysStateCb = std::function<void(const mavlink_extended_sys_state_t &)>;
    using HeartbeatCb        = std::function<void(const mavlink_heartbeat_t &)>;
    using LocalPositionCb    = std::function<void(const mavlink_local_position_ned_t &)>;
    using PositionTargetCb   = std::function<void(const mavlink_position_target_local_ned_t &)>;
    using EventCb            = std::function<void(const mavlink_event_t &)>;

    // localPort: where we listen. px4Ip/px4Port: PX4 SITL's mavlink UDP
    // endpoint (whatever you configure with `mavlink start -u <port> -r ...`
    // or the default onboard link, commonly 14540/14580 depending on setup).
    MavlinkLink(uint16_t localPort, std::string px4Ip, uint16_t px4Port);
    ~MavlinkLink();

    void start();
    void stop();

    void setExtendedSysStateCb(ExtendedSysStateCb cb) { this->_onExtendedSysState = std::move(cb); }
    void setHeartbeatCb(HeartbeatCb cb) { this->_onHeartbeat = std::move(cb); }
    void setLocalPositionCb(LocalPositionCb cb) { this->_onLocalPosition = std::move(cb); }
    void setPositionTargetCb(PositionTargetCb cb) { this->_onPositionTarget = std::move(cb); }
    void setEventCb(EventCb cb) { this->_onEvent = std::move(cb); }

    // Ask PX4 to stream a message at a given rate (Hz). Needed for
    // POSITION_TARGET_LOCAL_NED, which isn't always streamed by default.
    void requestMessageInterval(uint32_t mavlinkMsgId, float rateHz);

private:
    void recvLoop();
    void heartbeatLoop();
    void handleMessage(const mavlink_message_t &msg);
    void sendHeartbeat();

    int sockFd{-1};
    uint16_t _localPort;
    std::string _px4Ip;
    uint16_t _px4Port;

    uint8_t _targetSysId{1};
    uint8_t _targetCompId{1};
    bool _haveTarget{false};

    std::atomic<bool> _running{false};
    std::thread _rxThread;
    std::thread txHeartbeatThread;

    ExtendedSysStateCb _onExtendedSysState;
    HeartbeatCb _onHeartbeat;
    LocalPositionCb _onLocalPosition;
    PositionTargetCb _onPositionTarget;
    EventCb _onEvent;
};

}  // namespace move_platform_pad
