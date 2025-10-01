#ifndef __ORIENTATION_SERVICE_HPP__
#define __ORIENTATION_SERVICE_HPP__

#include <functional>
#include "IMU.hpp"
#include "au.hpp"
#include "TimeUtils.hpp"
#include "OrientationTracker.hpp"
#include "Quaternion.hpp"

#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

Eigen::Vector3f magVector(const MagneticFieldInBodyFrame &magnetic)
{
    return Eigen::Vector3f(
        magnetic[0].in(au::bodys * au::tesla),
        magnetic[1].in(au::bodys * au::tesla),
        magnetic[2].in(au::bodys * au::tesla));
}

Eigen::Vector3f accVector(const AccelerationInBodyFrame &acceleration)
{
    return Eigen::Vector3f(
        acceleration[0].in(au::bodys * au::metersPerSecondSquared),
        acceleration[1].in(au::bodys * au::metersPerSecondSquared),
        acceleration[2].in(au::bodys * au::metersPerSecondSquared));
}

Eigen::Vector3f gyrVector(const AngularVelocityInBodyFrame &angular_velocity)
{
    return Eigen::Vector3f(
        angular_velocity[0].in(au::radiansPerSecondInBodyFrame),
        angular_velocity[1].in(au::radiansPerSecondInBodyFrame),
        angular_velocity[2].in(au::radiansPerSecondInBodyFrame));
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
class GyrMagOrientation
{
public:
    GyrMagOrientation() = delete;
    GyrMagOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu, MAG &mag) : hrtc_(hrtc), tracker_(tracker), imu_(imu), mag_(mag) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
    MAG &mag_;
};

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
bool GyrMagOrientation<Tracker, IMU, MAG>::predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);
    update(timestamp);

    auto q_ = tracker_.getOrientation();
    q[0] = q_.w();
    q[1] = q_.x();
    q[2] = q_.y();
    q[3] = q_.z();

    return true;
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG>
void GyrMagOrientation<Tracker, IMU, MAG>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    auto optional_angular = imu_.readGyroscope();
    auto optional_magnetic = imu_.readMagnetometer();
    if (optional_angular.has_value() && optional_magnetic.has_value())
    {
        tracker_.updateSensorFusion(gyrVector(optional_angular.value()), magVector(optional_magnetic.value()), timestamp);
    }
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
class AccGyrMagOrientation
{
public:
    AccGyrMagOrientation() = delete;
    AccGyrMagOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu, MAG &mag) : hrtc_(hrtc), tracker_(tracker), imu_(imu), mag_(mag) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
    MAG &mag_;
};

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
bool AccGyrMagOrientation<Tracker, IMU, MAG>::predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);
    update(timestamp);

    auto q_ = tracker_.getOrientation();
    q[0] = q_.w();
    q[1] = q_.x();
    q[2] = q_.y();
    q[3] = q_.z();

    return true;
}

template <typename Tracker, typename IMU, typename MAG>
    requires HasBodyGyroscope<IMU> && HasBodyMagnetometer<MAG> && HasBodyAccelerometer<IMU>
void AccGyrMagOrientation<Tracker, IMU, MAG>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    auto optional_angular = imu_.readGyroscope();
    auto optional_accel = imu_.readAccelerometer();
    auto optional_magnetic = imu_.readMagnetometer();

    if (optional_angular.has_value() && optional_accel.has_value() && optional_magnetic.has_value())
    {
        tracker_.updateSensorFusion(gyrVector(optional_angular.value()), accVector(optional_accel.value()), magVector(optional_magnetic.value()), timestamp);
    }
}

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
class AccGyrOrientation
{
public:
    AccGyrOrientation() = delete;
    AccGyrOrientation(RTC_HandleTypeDef *hrtc, Tracker &tracker, IMU &imu)
        : hrtc_(hrtc), tracker_(tracker), imu_(imu) {}

    bool predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp);
    void update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp);

private:
    RTC_HandleTypeDef *hrtc_;
    Tracker &tracker_;
    IMU &imu_;
};

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
bool AccGyrOrientation<Tracker, IMU>::predict(std::array<float, 4> &q, au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    TimeUtils::RTCDateTimeSubseconds rtc;
    HAL_RTC_GetTime(hrtc_, &rtc.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_, &rtc.date, RTC_FORMAT_BIN);
    timestamp = TimeUtils::from_rtc(rtc, hrtc_->Init.SynchPrediv);
    update(timestamp);

    auto q_ = tracker_.getOrientation();
    q[0] = q_.w();
    q[1] = q_.x();
    q[2] = q_.y();
    q[3] = q_.z();

    return true;
}

template <typename Tracker, typename IMU>
    requires HasBodyGyroscope<IMU> && HasBodyAccelerometer<IMU>
void AccGyrOrientation<Tracker, IMU>::update(au::QuantityU64<au::Milli<au::Seconds>> &timestamp)
{
    auto optional_angular = imu_.readGyroscope();
    auto optional_accel = imu_.readAccelerometer();

    if (optional_angular.has_value() && optional_accel.has_value())
    {
        tracker_.updateSensorFusion(gyrVector(optional_angular.value()), accVector(optional_accel.value()), timestamp);
    }
}

#endif // __ORIENTATION_SERVICE_HPP__
