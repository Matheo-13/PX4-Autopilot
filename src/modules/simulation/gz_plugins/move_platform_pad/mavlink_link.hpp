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
	void setMissionListCb(MissionListCb cb) { this->_onMissionList = std::move(cb); }

	// Ask PX4 to stream a message at a given rate (Hz). Needed for
	// POSITION_TARGET_LOCAL_NED, which isn't always streamed by default.
	void requestMessageInterval(uint32_t mavlinkMsgId, float rateHz);

	// Kick off a MISSION_REQUEST_LIST / MISSION_REQUEST_INT download of the
	// mission currently stored on PX4. Safe to call again while a download
	// is already in flight (it will just restart it). Requires that we've
	// already learned PX4's sysid/compid from a heartbeat.
	void requestMissionList();

private:
	void recvLoop();
	void heartbeatLoop();
	void handleMessage(const mavlink_message_t &msg);
	void sendHeartbeat();
	void sendToPx4(const mavlink_message_t &msg);

	// Mission download handshake (MISSION_COUNT -> N x MISSION_REQUEST_INT/
	// MISSION_ITEM_INT -> MISSION_ACK). See MAVLink "mission protocol".
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
	bool _haveTarget{false};

	std::atomic<bool> _running{false};
	std::thread _rxThread;
	std::thread txHeartbeatThread;

	ExtendedSysStateCb _onExtendedSysState;
	HeartbeatCb _onHeartbeat;
	LocalPositionCb _onLocalPosition;
	PositionTargetCb _onPositionTarget;
	EventCb _onEvent;
	MissionListCb _onMissionList;

	// Mission download state. Touched from the rx thread (on incoming
	// MISSION_COUNT / MISSION_ITEM_INT) and from requestMissionList()
	// (called from the sim thread via MovePlatformPadSystem, and from the
	// heartbeat thread for periodic re-polling), so it's mutex-protected.
	std::mutex _missionMutex;
	bool _missionDownloadInProgress{false};
	uint16_t _missionExpectedCount{0};
	std::vector<mavlink_mission_item_int_t> _missionItems;

	// Mission-cache extension (MAVLink's opaque_id on MISSION_COUNT /
	// MISSION_ACK): opaque_id changes whenever the mission stored on PX4
	// changes. If a MISSION_COUNT comes back with the same opaque_id as our
	// last completed download, the mission hasn't changed and we can reuse
	// _missionItems as-is instead of re-walking MISSION_REQUEST_INT /
	// MISSION_ITEM_INT for every item.
	bool _haveCachedMission{false};
	uint32_t _cachedMissionOpaqueId{0};
	uint32_t _pendingOpaqueId{0};  // opaque_id of the download currently in flight
};

}  // namespace custom
