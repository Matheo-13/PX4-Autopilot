/**
 * @file move_platform_pad_system.hpp
 * Ported from move_platform_pad_node.hpp (Tiphaine CALVIER).
 * Same class shape, same variable/function names, same logic.
 * Interface changed: mavros -> raw MAVLink (MavlinkLink), gazebo_msgs
 * service -> direct Gazebo Sim ECM writes.
 */

#ifndef MOVE_PLATFORM_PAD_SYSTEM_HPP
#define MOVE_PLATFORM_PAD_SYSTEM_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "mavlink_link.hpp"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

namespace move_platform_pad
{


struct Pose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class MovePlatformPadSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    MovePlatformPadSystem() = default;
    ~MovePlatformPadSystem() override;

    /**
     * @brief Gazebo System entry point.
     */
    void Configure(const gz::sim::Entity &entity,
                    const std::shared_ptr<const sdf::Element> &sdf,
                    gz::sim::EntityComponentManager &ecm,
                    gz::sim::EventManager &eventMgr) override;

    /**
     * @brief Runs every sim step.
     */
    void PreUpdate(const gz::sim::UpdateInfo &info,
                    gz::sim::EntityComponentManager &ecm) override;

private:
    // --- interface: was 5 mavros/gazebo subscriptions + 1 service client ---
    std::unique_ptr<MavlinkLink> link;
    gz::sim::Entity droneEntity{gz::sim::kNullEntity};
    gz::sim::Entity platformEntity{gz::sim::kNullEntity};

    // callbacks
    /** @brief Callback to extended state */
    void subCbExtendedState(const mavlink_extended_sys_state_t &state_msg);

    /** @brief Callback to state */
    void subCbState(const mavlink_heartbeat_t &state_msg);

    /** @brief Callback to status event to get the rallypoint altitude */
    void subCbStatusEvent(const mavlink_event_t &msg);

    /** @brief Callback to waypoint list to get the landing altitude from the landing waypoint and drop informations */
    void subCbWaypointList(const std::vector<mavlink_mission_item_int_t> &wpList);

    /** @brief Refreshes the drone and platform poses */
    void subCbModelStates(gz::sim::EntityComponentManager &ecm);

    // other functions

    /** @brief Function to read env vars, notably drone initial pose and drone model */
    void readEnvVariables();

    /** @brief Function to configure entities and set the configured state */
    void configureEntities(gz::sim::EntityComponentManager &ecm);

    /** @brief Move drone to desired position */
    void move_drone(gz::sim::EntityComponentManager &ecm);

    /** @brief Function to prepare the drop */
    void prepareForDropping() const;

    /** @brief Function to prepare the land */
    void prepareForLanding() const;

    /** @brief Function to check if there is a platform near this pose and below the drone */
    bool isPlatformBelow(const double x, const double y) const;

    /** @brief Function to move the plaftorm (and the pad) */
    void movePlatformPad(const double x, const double y, const double z) const;

    /** @brief Writes a move staged by movePlatformPad() into the ECM.
     * Only ever called from PreUpdate, on the sim thread. */
    void applyPendingMove(gz::sim::EntityComponentManager &ecm, bool add_randomness = true);

    // variables
    bool landing = false;
    bool dropping = false;

    std::string droneModelName;
    Pose droneInitialPose;
    bool configured = false;

    double gazeboDronePosX = 0.0;
    double gazeboDronePosY = 0.0;
    double gazeboDronePosZ = 0.0;
    double gazeboPlatformX = 0.0;
    double gazeboPlatformY = 0.0;
    double gazeboPlatformZ = 0.0;

    double landingAlt = 0.0;
    double dropAlt = 0.0;
    double rallyPointAlt{NAN};

    std::string droneMode;

    mutable std::mt19937 rng;

    mutable std::mutex stateMutex;

    mutable std::atomic<bool> moveRequested{false};
    mutable double reqX{0.0}, reqY{0.0}, reqZ{0.0};

    // Platform dimensions, taken from platform/model.sdf
    float PlatformLengthM = 0.0f;
    float PlatformWidthM = 0.0f;
    float PlatformHeightM = 0.0f;
    static constexpr uint32_t kSmartSrpClosestRpEventId = 7944843;
    static constexpr uint32_t kSmartSrpNoClosestRpEventId = 5280179;
};

}  // namespace move_platform_pad

#endif  // MOVE_PLATFORM_PAD_SYSTEM_HPP
