#ifndef INC_TASKORIENTATIONSERVICE_HPP_
#define INC_TASKORIENTATIONSERVICE_HPP_

#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"
#include "au.hpp"

#include "_4111Spyglass.h"
#include "nunavut_assert.h"
#include "uavcan/si/sample/angle/Quaternion_1_0.h"
#include <cmath>
#include <cstring>
#include <chrono>
#include <cstdint>
#include <array>
#include <numbers>

std::array<float, 3> getYawPitchRoll(const std::array<float, 4> &q)
{
    float sinp = 2.f * (q[0] * q[2] - q[3] * q[1]);
    sinp = std::clamp(sinp, -1.f, 1.f);

    float yaw = atan2f(2.f * (q[0] * q[3] + q[1] * q[2]),
                       1.f - 2.f * (q[2] * q[2] + q[3] * q[3]));
    float pitch = asinf(sinp);
    float roll = atan2f(2.f * (q[0] * q[1] + q[2] * q[3]), 1.f - 2.f * (q[1] * q[1] + q[2] * q[2]));
    return {yaw, pitch, roll};
}


template <typename Tracker, typename... Adapters>
class TaskOrientationService : public TaskWithPublication<Adapters...>
{
public:
    TaskOrientationService() = delete;
    TaskOrientationService(Tracker &tracker, uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskWithPublication<Adapters...>(interval, tick, transfer_id, adapters), tracker_(tracker) {}

    virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) override;
    virtual void handleTaskImpl() override;

private:
    Tracker &tracker_;
};

template <typename Tracker, typename... Adapters>
void TaskOrientationService<Tracker, Adapters...>::handleTaskImpl()
{
    std::array<float, 4> q;
    au::QuantityU64<au::Milli<au::Seconds>> timestamp;
    if (!tracker_.predict(q, timestamp))
    {
        return;
    };

    uavcan_si_sample_angle_Quaternion_1_0 data;
    data.timestamp.microsecond = timestamp.in(au::micro(au::seconds));
    memcpy(data.wxyz, q.data(), sizeof(data.wxyz));

    std::array<float, 3> orientation = getYawPitchRoll(q);
    constexpr float m_pif = static_cast<float>(std::numbers::pi);
    constexpr float RAD_TO_DEG = 180.0f / m_pif;
    log(LOG_LEVEL_DEBUG, "TaskOrientationService %f %f %f\r\n", orientation[0]*RAD_TO_DEG, orientation[1]*RAD_TO_DEG, orientation[2]*RAD_TO_DEG);

    constexpr size_t PAYLOAD_SIZE = uavcan_si_sample_angle_Quaternion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskOrientationService<Tracker, Adapters...>::publish(PAYLOAD_SIZE, payload, &data,
                                   reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_si_sample_angle_Quaternion_1_0_serialize_),
                                   _uavcan_si_sample_angle_Quaternion_1_0_PORT_ID_);
}

template <typename Tracker, typename... Adapters>
void TaskOrientationService<Tracker, Adapters...>::registerTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->publish(_uavcan_si_sample_angle_Quaternion_1_0_PORT_ID_, task);
}

template <typename Tracker, typename... Adapters>
void TaskOrientationService<Tracker, Adapters...>::unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task)
{
    manager->unpublish(_uavcan_si_sample_angle_Quaternion_1_0_PORT_ID_, task);
}

#endif // INC_TASKORIENTATIONSERVICE_HPP_
