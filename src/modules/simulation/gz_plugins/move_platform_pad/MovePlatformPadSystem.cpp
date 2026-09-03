/**
 * @file move_platform_pad_system.cpp
 * Ported from move_platform_pad_node.cpp (Tiphaine CALVIER).
 */

#include "MovePlatformPadSystem.hpp"

#include <cstdlib>
#include <cstring>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>

#include <sdf/Box.hh>

using namespace custom;

// PX4 packs main_mode into byte 2 and sub_mode into byte 3 of
// HEARTBEAT.custom_mode (see px4_custom_mode.h). This reproduces the
// same strings mavros_msgs::msg::State::mode used to carry, so
// prepareForLanding()'s string comparisons don't need to change.
namespace
{
std::string decodePx4ModeString(uint32_t customMode)
{
    const uint8_t mainMode = (customMode >> 16) & 0xFF;
    const uint8_t subMode = (customMode >> 24) & 0xFF;
    constexpr uint8_t kMainAuto = 4;
    constexpr uint8_t kSubAutoMission = 4;
    constexpr uint8_t kSubAutoRtl = 5;

    if (mainMode == kMainAuto && subMode == kSubAutoMission) return "AUTO.MISSION";
    if (mainMode == kMainAuto && subMode == kSubAutoRtl) return "AUTO.RTL";
    return "OTHER";
}

gz::sim::Entity resolveEntityByName(gz::sim::EntityComponentManager &ecm, const std::string &name)
{
    gz::sim::Entity found{gz::sim::kNullEntity};
    ecm.Each<gz::sim::components::Name>(
        [&](const gz::sim::Entity &e, const gz::sim::components::Name *nameComp) -> bool
        {
            if (nameComp->Data().find(name) != std::string::npos)
            {
                found = e;
                return false;  // stop iterating
            }
            return true;
        });
    return found;
}
}  // namespace

MovePlatformPadSystem::~MovePlatformPadSystem()
{
    if (this->link)
    {
        this->link->stop();
    }
}

void MovePlatformPadSystem::Configure(
    const gz::sim::Entity & /*entity*/,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager & /*eventMgr*/)
{
    // Random generator used to shift the location of the platform
    std::random_device rd;
    this->rng = std::mt19937(rd());

    this->readEnvVariables();
    this->configureEntities(ecm);

    // --- Interface: was 5 mavros/gazebo subscriptions + 1 service client ---
    const uint16_t localPort = sdf->Get<int>("local_port", 14541).first;
    const std::string px4Ip = sdf->Get<std::string>("px4_ip", "127.0.0.1").first;
    const uint16_t px4Port = sdf->Get<int>("px4_port", 14580).first;

    this->link = std::make_unique<MavlinkLink>(localPort, px4Ip, px4Port);
    this->link->setExtendedSysStateCb(std::bind(&MovePlatformPadSystem::subCbExtendedState, this, std::placeholders::_1));
    this->link->setHeartbeatCb(std::bind(&MovePlatformPadSystem::subCbState, this, std::placeholders::_1));
    this->link->setEventCb(std::bind(&MovePlatformPadSystem::subCbStatusEvent, this, std::placeholders::_1));
    // TODO: wire subCbWaypointList() up to a MISSION_REQUEST_LIST /
    // MISSION_ITEM_INT download once implemented (see mavlink_link.hpp).
    this->link->start();

    gzmsg << "move_platform_pad_system configured for " << this->droneModelName
          << ", MAVLink link on port " << localPort << std::endl;
}

void MovePlatformPadSystem::subCbExtendedState(const mavlink_extended_sys_state_t &state_msg)
{
    std::lock_guard<std::mutex> lock(this->stateMutex);

    // If the drone is about to land and is in MC mode
    if (state_msg.landed_state == MAV_LANDED_STATE_LANDING && !this->landing
        && state_msg.vtol_state == MAV_VTOL_STATE_MC)
    {
        gzmsg << "Landing and MC detected." << std::endl;
        this->landing = true;

        this->prepareForLanding();
    }
    // reset landed = false if we are no more in landed state landing to allow the platform to be moved again
    else if (state_msg.landed_state != MAV_LANDED_STATE_LANDING && this->landing)
    {
        this->landing = false;
    }

    // If the drone is about to drop and is in MC mode
    if (this->dropping && state_msg.vtol_state == MAV_VTOL_STATE_MC)
    {
        gzmsg << "Dropping and MC detected." << std::endl;
        this->dropping = false;

        this->prepareForDropping();
    }
}

void MovePlatformPadSystem::subCbState(const mavlink_heartbeat_t &state_msg)
{
    std::lock_guard<std::mutex> lock(this->stateMutex);
    this->droneMode = decodePx4ModeString(state_msg.custom_mode);
}

void MovePlatformPadSystem::subCbStatusEvent(const mavlink_event_t &msg)
{
    if (msg.id == kSmartSrpClosestRpEventId)
    {
        // We need to read an int16_t after the offset
        const size_t offset = sizeof(uint16_t) + sizeof(float) + sizeof(float) + sizeof(int16_t) + sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(float) + sizeof(float);
        const size_t required_size = offset + sizeof(int16_t);

        // Ensure the buffer is big enough
        if (sizeof(msg.arguments) < required_size)
        {
            gzerr << "StatusEvent payload too small! Expected " << required_size
                  << ", got " << sizeof(msg.arguments) << std::endl;
            return;  // Stop processing to prevent crash
        }

        int16_t rallyPointAltInt;
        std::memcpy(&rallyPointAltInt, msg.arguments + offset, sizeof(int16_t));

        std::lock_guard<std::mutex> lock(this->stateMutex);
        this->rallyPointAlt = static_cast<double>(rallyPointAltInt);
        gzmsg << "rallyPointAlt: " << this->rallyPointAlt << std::endl;
    }
}

void MovePlatformPadSystem::subCbWaypointList(const std::vector<mavlink_mission_item_int_t> &wpList)
{
    if (wpList.empty())
    {
        gzerr << "Received null or empty waypoint list. Waiting for mission to be uploaded." << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(this->stateMutex);
    for (const auto &wpMsg : wpList)
    {
        // land wp
        if (wpMsg.command == MAV_CMD_NAV_LAND)
        {
            this->landingAlt = wpMsg.z;
        }
        // drop wp
        else if (wpMsg.command == MAV_CMD_DO_CHANGE_ALTITUDE)
        {
            this->dropping = wpMsg.current != 0;
            this->dropAlt = wpMsg.z;
        }
    }
}

void MovePlatformPadSystem::subCbModelStates(gz::sim::EntityComponentManager &ecm)
{
    std::lock_guard<std::mutex> lock(this->stateMutex);

    // Get Gazebo drone pose
    if (this->droneEntity != gz::sim::kNullEntity)
    {
        if (auto *pose = ecm.Component<gz::sim::components::Pose>(this->droneEntity))
        {
            this->gazeboDronePosX = pose->Data().Pos().X();
            this->gazeboDronePosY = pose->Data().Pos().Y();
            this->gazeboDronePosZ = pose->Data().Pos().Z();
        }
    }

    // Get the pose of the platform in the world
    if (this->platformEntity != gz::sim::kNullEntity)
    {
        if (auto *pose = ecm.Component<gz::sim::components::Pose>(this->platformEntity))
        {
            this->gazeboPlatformX = pose->Data().Pos().X();
            this->gazeboPlatformY = pose->Data().Pos().Y();
            this->gazeboPlatformZ = pose->Data().Pos().Z();
        }
    }
}

void MovePlatformPadSystem::readEnvVariables()
{
    gzmsg << "Reading env vars" << std::endl;
    const char *droneModelNameEnv = std::getenv("PX4_SIM_MODEL");
    if (droneModelNameEnv == nullptr || std::string(droneModelNameEnv).empty()) {
        gzerr << "PX4_SIM_MODEL needs to be set, this plugin does not support attaching after simulation started" << std::endl;
        throw;
    }

    std::string modelStr(droneModelNameEnv);
    if (modelStr.rfind("gz_", 0) == 0) {
        modelStr = modelStr.substr(3);
    }

    this->droneModelName = modelStr;

    gzmsg << "Found PX4_SIM_MODEL: " << this->droneModelName << std::endl;

    const char *droneInitialPoseEnv = std::getenv("PX4_GZ_MODEL_POSE");
    if (droneInitialPoseEnv == nullptr || std::string(droneInitialPoseEnv).empty()) {
        gzmsg << "PX4_GZ_MODEL_POSE needs to be set, this plugin does not support attaching after simulation started" << std::endl;
        return;
    }

    gzmsg << "Found PX4_GZ_MODEL_POSE: " << droneInitialPoseEnv << std::endl;

    std::stringstream ss(droneInitialPoseEnv);
    std::string token;
    double* fields[] = {&this->droneInitialPose.x,
                        &this->droneInitialPose.y,
                        &this->droneInitialPose.z};

    size_t index = 0;
    while (index < 3 && std::getline(ss, token, ',')) {
        try {
            if (!token.empty()) {
                *fields[index] = std::stod(token);
            }
        } catch (const std::exception&) {
            // If parsing fails for a token, keep the 0.0 default
            *fields[index] = 0.0;
            break;
        }
        index++;
    }
}

void MovePlatformPadSystem::configureEntities(gz::sim::EntityComponentManager &ecm) {
    this->platformEntity = resolveEntityByName(ecm, "platform");
    this->droneEntity = resolveEntityByName(ecm, droneModelName);

    if (this->platformEntity == gz::sim::kNullEntity) {
        gzerr << "You need to have a platform model named 'platform' to use this plugin" << std::endl;
        return;
    }

    if (this->droneEntity == gz::sim::kNullEntity) {
        gzwarn << "Entity not spawned yet or name is wrong" << std::endl;
        return;
    }

    auto links = ecm.EntitiesByComponents(gz::sim::components::ParentEntity(this->platformEntity));
    bool boxFound = false;
    for (const auto &linkEntity : links) {
        auto collisions = ecm.EntitiesByComponents(gz::sim::components::ParentEntity(linkEntity));

        for (const auto &collisionEntity : collisions) {
            auto geomComp = ecm.Component<gz::sim::components::Geometry>(collisionEntity);
            if (geomComp) {
                const sdf::Geometry &geom = geomComp->Data();

                if (geom.Type() == sdf::GeometryType::BOX) {
                    const sdf::Box *box = geom.BoxShape();
                    if (box) {
                        gz::math::Vector3d size = box->Size();

                        this->PlatformLengthM = size.X();
                        this->PlatformWidthM  = size.Y();
                        this->PlatformHeightM = size.Z();
                        boxFound = true;
                        break;
                    }
                }
            }
        }
        if (boxFound) break;
    }

    if (!boxFound) {
        gzerr << "Platform collision box has not been found" << std::endl;
        return;
    }

    this->reqX = droneInitialPose.x;
    this->reqY = droneInitialPose.y;
    this->reqZ = droneInitialPose.z;

    this->applyPendingMove(ecm, false);
    this->move_drone(ecm);

    this->configured = true;

}

void MovePlatformPadSystem::move_drone(gz::sim::EntityComponentManager &ecm)
{

    const double x = droneInitialPose.x;
    const double y = droneInitialPose.y;
    const double z = droneInitialPose.z + 0.20;

    if (this->droneEntity != gz::sim::kNullEntity)
    {
        gz::sim::Model(this->droneEntity).SetWorldPoseCmd(
            ecm, gz::math::Pose3d(x, y, z, 0, 0, 0));
        gzmsg << "Drone moved successfully in Gazebo." << std::endl;
    }
    else
    {
        gzerr << "Failed to move drone." << std::endl;
    }

}

void MovePlatformPadSystem::prepareForDropping() const
{
    double zDrop = this->dropAlt;
    if (std::abs(zDrop) < 1e-6)
    {
        gzerr << "dropAlt is 0.0 as if OC did not received the padAltAmsl. Moving the platform 15 m under the drone." << std::endl;
        zDrop = this->gazeboDronePosZ - 15.0;  // -15.0m because with a drop, the approach altitude is close to the pad
    }
    else if (this->gazeboDronePosZ - zDrop < 0.5)  // the height of the drone is below 0.5m
    {
        gzerr << "Drone is already below drop pose. Moving the platform 15 m under the drone." << std::endl;
        zDrop = this->gazeboDronePosZ - 15.0;
    }

    this->movePlatformPad(this->gazeboDronePosX, this->gazeboDronePosY, zDrop);
}

void MovePlatformPadSystem::prepareForLanding() const
{

    gzmsg << "Prepare for landing" << this->droneMode << std::endl;
    // Depending on the autoMode, select the altitude.
    double zAltitude = 0.0;
    if (this->droneMode == "AUTO.MISSION")
    {
        gzmsg << "Normal mission. Drone mode is " << this->droneMode << std::endl;
        zAltitude = this->landingAlt;
    }
    else if (this->droneMode == "AUTO.RTL")
    {
        gzmsg << "Straight to Rally Point. Drone mode is " << this->droneMode << std::endl;
        zAltitude = !std::isnan(this->rallyPointAlt) ? this->rallyPointAlt : this->gazeboDronePosZ - 40.0;
    }
    else
    {
        gzmsg << "Emergency landing or else. Moving the platform 40 m under the drone. Drone mode is : " << this->droneMode << std::endl;
        zAltitude = this->gazeboDronePosZ - 40.0;  // -40.0m so to the drone has time to switch to MC
    }
    gzmsg << "Drone pose is x = " << gazeboDronePosX << ", y = " << gazeboDronePosY
          << ", z = " << gazeboDronePosZ << ". Landing altitude is z = " << zAltitude << std::endl;

    if (this->gazeboDronePosZ - zAltitude < 0.5)  // the height of the drone is below 0.5m
    {
        gzerr << "Drone is already below land pose. Moving the platform 40 m under the drone." << std::endl;
        zAltitude = this->gazeboDronePosZ - 40.0;
    }

    this->movePlatformPad(this->gazeboDronePosX, this->gazeboDronePosY, zAltitude);
}

bool MovePlatformPadSystem::isPlatformBelow(const double x, const double y) const
{
    // Check if there is a platform below the drone to not move higher a platform, mainly if there is a emergency landing above takeoff position.
    return (std::abs(this->gazeboPlatformX - x) <= PlatformLengthM/2) &&
           (std::abs(this->gazeboPlatformY - y) <= PlatformWidthM/2) &&
           (this->gazeboPlatformZ + PlatformHeightM/2 <= this->gazeboDronePosZ);
}

void MovePlatformPadSystem::movePlatformPad(const double x, const double y, const double z) const
{
    // Check if a platform is already below this position
    if (this->isPlatformBelow(x, y))
    {
        return;
    }

    this->reqX = x;
    this->reqY = y;
    this->reqZ = z;
    this->moveRequested = true;
}

void MovePlatformPadSystem::applyPendingMove(gz::sim::EntityComponentManager &ecm, bool add_randomness)
{
    if (!this->moveRequested.exchange(false))
    {
        return;
    }

    std::uniform_real_distribution<> distr(-1.0, 1.0);

    double x, y, zPlatform;
    {
        std::lock_guard<std::mutex> lock(this->stateMutex);
        const double jitterX = add_randomness ? distr(this->rng) * (this->PlatformLengthM / 4) : 0.0;
        const double jitterY = add_randomness ? distr(this->rng) * (this->PlatformWidthM / 4) : 0.0;
        x = this->reqX + jitterX;
        y = this->reqY + jitterY;
        zPlatform = this->reqZ - PlatformHeightM / 2;

        if (this->platformEntity != gz::sim::kNullEntity)
        {
            this->gazeboPlatformX = x;
            this->gazeboPlatformY = y;
            this->gazeboPlatformZ = zPlatform;
        }
    }

    if (this->platformEntity != gz::sim::kNullEntity)
    {
         gz::sim::Model(this->platformEntity).SetWorldPoseCmd(
            ecm, gz::math::Pose3d(x, y, zPlatform, 0, 0, 0));
        gzmsg << "Platform moved successfully in Gazebo." << std::endl;
    }
    else
    {
        gzerr << "Failed to move platform." << std::endl;
    }
}


void MovePlatformPadSystem::PreUpdate(
    const gz::sim::UpdateInfo & /*info*/,
    gz::sim::EntityComponentManager &ecm)
{
    if (!configured) {
        readEnvVariables();
        configureEntities(ecm);
        return;
    }
    this->subCbModelStates(ecm);
    this->applyPendingMove(ecm);
}

GZ_ADD_PLUGIN(
    custom::MovePlatformPadSystem,
    gz::sim::System,
    custom::MovePlatformPadSystem::ISystemConfigure,
    custom::MovePlatformPadSystem::ISystemPreUpdate)
