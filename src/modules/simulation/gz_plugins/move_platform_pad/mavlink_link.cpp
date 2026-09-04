#include "mavlink_link.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

using namespace custom;

namespace
{
constexpr int kMissionRepollEveryTicks = 5;
}  // namespace

MavlinkLink::MavlinkLink(uint16_t localPort, std::string px4Ip, uint16_t px4Port)
	: _localPort(localPort), _px4Ip(std::move(px4Ip)), _px4Port(px4Port)
{
}

MavlinkLink::~MavlinkLink()
{
	this->stop();
}

bool MavlinkLink::start()
{
	this->sockFd = socket(AF_INET, SOCK_DGRAM, 0);

	if (this->sockFd < 0) {
		std::cerr << "[MavlinkLink] Failed to create socket: " << strerror(errno) << std::endl;
		return false;
	}

	sockaddr_in localAddr{};
	localAddr.sin_family = AF_INET;
	localAddr.sin_addr.s_addr = INADDR_ANY;
	localAddr.sin_port = htons(this->_localPort);

	if (bind(this->sockFd, reinterpret_cast<sockaddr *>(&localAddr), sizeof(localAddr)) < 0) {
		std::cerr << "[MavlinkLink] Failed to bind to port " << this->_localPort << ": "
			  << strerror(errno) << std::endl;
		close(this->sockFd);
		this->sockFd = -1;
		return false;
	}

	this->_running = true;
	this->_rxThread = std::thread(&MavlinkLink::recvLoop, this);
	this->txHeartbeatThread = std::thread(&MavlinkLink::heartbeatLoop, this);
	return true;
}

void MavlinkLink::stop()
{
	this->_running = false;

	if (this->sockFd >= 0) {
		shutdown(this->sockFd, SHUT_RDWR);
		close(this->sockFd);
		this->sockFd = -1;
	}

	if (this->_rxThread.joinable()) {
		this->_rxThread.join();
	}

	if (this->txHeartbeatThread.joinable()) {
		this->txHeartbeatThread.join();
	}
}

void MavlinkLink::recvLoop()
{
	uint8_t buffer[2048];
	mavlink_message_t msg;
	mavlink_status_t status;

	sockaddr_in srcAddr{};
	socklen_t srcLen = sizeof(srcAddr);

	while (this->_running) {
		// TODO: distinguish a real recv error from EINTR/timeout instead of
		// just retrying either way.
		const ssize_t n = recvfrom(this->sockFd, buffer, sizeof(buffer), 0,
					   reinterpret_cast<sockaddr *>(&srcAddr), &srcLen);

		if (n <= 0) {
			continue;
		}

		for (ssize_t i = 0; i < n; ++i) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
				this->handleMessage(msg);
			}
		}
	}
}

void MavlinkLink::handleMessage(const mavlink_message_t &msg)
{
	if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(&msg, &hb);

		const bool isAutopilotHeartbeat =
			msg.compid == MAV_COMP_ID_AUTOPILOT1 && hb.autopilot != MAV_AUTOPILOT_INVALID;

		if (!isAutopilotHeartbeat) {
			return;
		}

		if (!this->_haveTarget) {
			this->_targetSysId = msg.sysid;
			this->_targetCompId = msg.compid;
			this->_haveTarget = true;

			std::cerr << "[MavlinkLink] Learned autopilot target sysid="
				  << static_cast<int>(msg.sysid) << " compid="
				  << static_cast<int>(msg.compid) << std::endl;

			this->requestMissionList();
		}

		if (this->_onHeartbeat) {
			this->_onHeartbeat(hb);
		}

		return;
	}

	switch (msg.msgid) {
	case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
			if (this->_onExtendedSysState) {
				mavlink_extended_sys_state_t ext;
				mavlink_msg_extended_sys_state_decode(&msg, &ext);
				this->_onExtendedSysState(ext);
			}

			break;
		}

	case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
			if (this->_onLocalPosition) {
				mavlink_local_position_ned_t pos;
				mavlink_msg_local_position_ned_decode(&msg, &pos);
				this->_onLocalPosition(pos);
			}

			break;
		}

	case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
			if (this->_onPositionTarget) {
				mavlink_position_target_local_ned_t tgt;
				mavlink_msg_position_target_local_ned_decode(&msg, &tgt);
				this->_onPositionTarget(tgt);
			}

			break;
		}

	case MAVLINK_MSG_ID_EVENT: {
			if (this->_onEvent) {
				mavlink_event_t ev;
				mavlink_msg_event_decode(&msg, &ev);
				this->_onEvent(ev);
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_COUNT: {
			mavlink_mission_count_t count;
			mavlink_msg_mission_count_decode(&msg, &count);
			this->handleMissionCount(count);
			break;
		}

	case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
			mavlink_mission_item_int_t item;
			mavlink_msg_mission_item_int_decode(&msg, &item);
			this->handleMissionItemInt(item);
			break;
		}

	default:
		break;
	}
}

void MavlinkLink::heartbeatLoop()
{
	int tick = 0;

	while (this->_running) {
		this->sendHeartbeat();

		if (this->_haveTarget && (++tick % kMissionRepollEveryTicks == 0)) {
			this->requestMissionList();
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

void MavlinkLink::sendToPx4(const mavlink_message_t &msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	const int len = mavlink_msg_to_send_buffer(buf, &msg);

	sockaddr_in dest{};
	dest.sin_family = AF_INET;
	dest.sin_port = htons(this->_px4Port);
	inet_pton(AF_INET, this->_px4Ip.c_str(), &dest.sin_addr);

	sendto(this->sockFd, buf, len, 0, reinterpret_cast<sockaddr *>(&dest), sizeof(dest));
}

void MavlinkLink::sendHeartbeat()
{
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(
		/*system_id=*/255, /*component_id=*/MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
		MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
		/*base_mode=*/0, /*custom_mode=*/0, MAV_STATE_ACTIVE);

	this->sendToPx4(msg);
}

void MavlinkLink::requestMessageInterval(uint32_t mavlinkMsgId, float rateHz)
{
	if (!this->_haveTarget) {
		return;  // wait for first heartbeat before addressing PX4 directly
	}

	mavlink_message_t msg;
	mavlink_msg_command_long_pack(
		/*system_id=*/255, /*component_id=*/MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
		this->_targetSysId, this->_targetCompId,
		MAV_CMD_SET_MESSAGE_INTERVAL, 0,
		static_cast<float>(mavlinkMsgId),
		rateHz > 0 ? 1e6f / rateHz : -1.0f,
		0, 0, 0, 0, 0);

	this->sendToPx4(msg);
}

void MavlinkLink::requestMissionList()
{
	if (!this->_haveTarget) {
		return;  // wait for first heartbeat before addressing PX4 directly
	}

	mavlink_message_t msg;
	mavlink_msg_mission_request_list_pack(
		/*system_id=*/255, /*component_id=*/MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
		this->_targetSysId, this->_targetCompId,
		MAV_MISSION_TYPE_MISSION);

	this->sendToPx4(msg);
}

void MavlinkLink::requestMissionItem(uint16_t seq)
{
	if (!this->_haveTarget) {
		return;  // wait for first heartbeat before addressing PX4 directly
	}

	mavlink_message_t msg;
	mavlink_msg_mission_request_int_pack(
		/*system_id=*/255, /*component_id=*/MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
		this->_targetSysId, this->_targetCompId,
		seq, MAV_MISSION_TYPE_MISSION);

	this->sendToPx4(msg);
}

void MavlinkLink::sendMissionAck(uint8_t type)
{
	mavlink_message_t msg;
	mavlink_msg_mission_ack_pack(
		/*system_id=*/255, /*component_id=*/MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
		this->_targetSysId, this->_targetCompId,
		type, MAV_MISSION_TYPE_MISSION, /*opaque_id=*/0);

	this->sendToPx4(msg);
}

void MavlinkLink::handleMissionCount(const mavlink_mission_count_t &countMsg)
{
	bool cacheHit = false;
	bool needFirstItem = false;

	{
		std::lock_guard<std::mutex> lock(this->_missionMutex);

		if (this->_haveCachedMission && countMsg.opaque_id != 0
		    && countMsg.opaque_id == this->_cachedMissionOpaqueId
		    && countMsg.count == this->_missionItems.size()) {
			cacheHit = true;

		} else {
			this->_missionDownloadInProgress = true;
			this->_missionExpectedCount = countMsg.count;
			this->_pendingOpaqueId = countMsg.opaque_id;
			this->_missionItems.assign(countMsg.count, mavlink_mission_item_int_t{});
			needFirstItem = countMsg.count > 0;
		}
	}

	if (cacheHit) {
		this->sendMissionAck(MAV_MISSION_ACCEPTED);
		return;
	}

	if (needFirstItem) {
		this->requestMissionItem(0);

	} else {
		this->sendMissionAck(MAV_MISSION_ACCEPTED);
		{
			std::lock_guard<std::mutex> lock(this->_missionMutex);
			this->_missionDownloadInProgress = false;
			this->_haveCachedMission = true;
			this->_cachedMissionOpaqueId = countMsg.opaque_id;
		}

		if (this->_onMissionList) {
			this->_onMissionList({});
		}
	}
}

void MavlinkLink::handleMissionItemInt(const mavlink_mission_item_int_t &itemMsg)
{
	bool downloadComplete = false;
	std::vector<mavlink_mission_item_int_t> itemsCopy;

	{
		std::lock_guard<std::mutex> lock(this->_missionMutex);

		if (!this->_missionDownloadInProgress || itemMsg.seq >= this->_missionExpectedCount) {
			return;  // unexpected/stale item (e.g. leftover from a restarted download)
		}

		this->_missionItems[itemMsg.seq] = itemMsg;

		const bool haveAllItems = itemMsg.seq + 1 >= this->_missionExpectedCount;

		if (haveAllItems) {
			this->_missionDownloadInProgress = false;
			this->_haveCachedMission = true;
			this->_cachedMissionOpaqueId = this->_pendingOpaqueId;
			itemsCopy = this->_missionItems;
			downloadComplete = true;
		}
	}

	if (downloadComplete) {
		this->sendMissionAck(MAV_MISSION_ACCEPTED);

		if (this->_onMissionList) {
			this->_onMissionList(itemsCopy);
		}

	} else {
		this->requestMissionItem(itemMsg.seq + 1);
	}
}
