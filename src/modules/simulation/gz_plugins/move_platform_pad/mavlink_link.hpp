#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
	using MissionListCb      = std::function<void(const std::vector<mavlink_mission_item_int_t> &)>;

	// px4Ip/px4Port: PX4 SITL's MAVLink UDP endpoint (see `mavlink start -u <port> -r ...`).
	MavlinkLink(uint16_t localPort, std::string px4Ip, uint16_t px4Port);
	~MavlinkLink();

	bool start();
	void stop();

	void setExtendedSysStateCb(ExtendedSysStateCb cb) { this->_onExtendedSysState = std::move(cb); }
	void setHeartbeatCb(HeartbeatCb cb) { this->_onHeartbeat = std::move(cb); }
	void setLocalPositionCb(LocalPositionCb cb) { this->_onLocalPosition = std::move(cb); }
	void setPositionTargetCb(PositionTargetCb cb) { this->_onPositionTarget = std::move(cb); }
	void setEventCb(EventCb cb) { this->_onEvent = std::move(cb); }
	void setMissionListCb(MissionListCb cb) { this->_onMissionList = std::move(cb); }

	void requestMessageInterval(uint32_t mavlinkMsgId, float rateHz);

	void requestMissionList();

private:
	void recvLoop();
	void heartbeatLoop();
	void handleMessage(const mavlink_message_t &msg);
	void sendHeartbeat();
	void sendToPx4(const mavlink_message_t &msg);

	void handleMissionCount(const mavlink_mission_count_t &countMsg);
	void handleMissionItemInt(const mavlink_mission_item_int_t &itemMsg);
	void requestMissionItem(uint16_t seq);
	void sendMissionAck(uint8_t type);

	int sockFd{-1};
	uint16_t _localPort;
	std::string _px4Ip;
	uint16_t _px4Port;

	uint8_t _targetSysId{1};
	uint8_t _targetCompId{1};
	std::atomic<bool> _haveTarget{false};

	std::atomic<bool> _running{false};
	std::thread _rxThread;
	std::thread txHeartbeatThread;

	ExtendedSysStateCb _onExtendedSysState;
	HeartbeatCb _onHeartbeat;
	LocalPositionCb _onLocalPosition;
	PositionTargetCb _onPositionTarget;
	EventCb _onEvent;
	MissionListCb _onMissionList;

	std::mutex _missionMutex;
	bool _missionDownloadInProgress{false};
	uint16_t _missionExpectedCount{0};
	std::vector<mavlink_mission_item_int_t> _missionItems;

	bool _haveCachedMission{false};
	uint32_t _cachedMissionOpaqueId{0};
	uint32_t _pendingOpaqueId{0};  // opaque_id of the download currently in flight
};

}  // namespace custom
