#ifndef __IMUEXTENSION__HPP_
#define __IMUEXTENSION__HPP_

#include <concepts>
#include <optional>
#include <cmath>
#include <array>
#include "au.hpp"

#include "IMU.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "coordinate_transformations.hpp"

template <typename IMU, typename OrientationProvider, typename PositionProvider>
class IMUWithReorientation
{
public:
    IMUWithReorientation(IMU &imu, OrientationProvider &orientation, PositionProvider &position)
        : imu_(imu), orientation_(orientation), position_(position) {}

    std::optional<std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3>> readAccelerometer()
    {
        auto optional_accel_body = imu_.readAccelerometer();
        if (!optional_accel_body.has_value())
        {
            return std::nullopt;
        }
        const auto &accel_body = optional_accel_body.value();

        au::QuantityU64<au::Milli<au::Seconds>> timestamp;
        std::array<float, 4> q_body_to_ned;
        orientation_.predict(q_body_to_ned, timestamp);

        std::array<au::QuantityF<au::MetersInEcefFrame>, 3> pos_ecef;
        std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> v;
        position_.predict(pos_ecef, v, timestamp);

        // Convert to Eigen vector
        Eigen::Vector3f a_body(
            accel_body[0].in(au::metersPerSecondSquaredInBodyFrame),
            accel_body[1].in(au::metersPerSecondSquaredInBodyFrame),
            accel_body[2].in(au::metersPerSecondSquaredInBodyFrame));

        // Rotate from body to NED
        Eigen::Quaternionf q_b2n(q_body_to_ned[3], q_body_to_ned[0], q_body_to_ned[1], q_body_to_ned[2]);
        Eigen::Vector3f a_ned = q_b2n * a_body;

        // Compute NED-to-ECEF rotation matrix from position
        Eigen::Matrix3f R_ned_to_ecef = computeNEDtoECEFRotation(pos_ecef);

        // Rotate from NED to ECEF
        Eigen::Vector3f a_ecef = R_ned_to_ecef * a_ned;

        // std::cout << "Rotated accel in IMUWithReorientation: " << a_ecef.transpose() << std::endl;

        return std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3>{
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(a_ecef.x()),
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(a_ecef.y()),
            au::make_quantity<au::MetersPerSecondSquaredInEcefFrame>(a_ecef.z())};
    }

private:
    IMU &imu_;
    OrientationProvider &orientation_;
    PositionProvider &position_;

    Eigen::Matrix3f computeNEDtoECEFRotation(std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &ecef)
    {
        // Convert ECEF to geodetic (lat, lon)
        coordinate_transformations::ECEF ecef_ = { ecef[0], ecef[1], ecef[2]};
        coordinate_transformations::Geodetic geo = coordinate_transformations::ecefToGeodetic(ecef_);


        float lat = geo.latitude.in(au::radiansInGeodeticFrame);
        float lon = geo.longitude.in(au::radiansInGeodeticFrame);

        float sin_lat = std::sin(lat);
        float cos_lat = std::cos(lat);
        float sin_lon = std::sin(lon);
        float cos_lon = std::cos(lon);

        // NED to ECEF rotation matrix
        Eigen::Matrix3f R;
        R << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
            -sin_lon, cos_lon, 0,
            -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;

        return R;
    }
};

template <typename IMU>
class IMUWithMagneticCorrection
{
public:
    IMUWithMagneticCorrection(IMU& imu,
                               const Eigen::Vector3f& hardIronOffset,
                               const Eigen::Matrix3f& softIronMatrix)
        : imu_(imu), hardIronOffset_(hardIronOffset), softIronMatrix_(softIronMatrix) {}

    std::optional<MagneticFieldInBodyFrame> readMagnetometer()
    {
        auto raw = imu_.readMagnetometer();
        if (!raw.has_value()) return std::nullopt;

        const auto& m_raw = raw.value();

        // Convert to Eigen vector
        Eigen::Vector3f m_vec(
            m_raw[0].in(au::teslaInBodyFrame),
            m_raw[1].in(au::teslaInBodyFrame),
            m_raw[2].in(au::teslaInBodyFrame)
        );

        // Apply hard and soft iron corrections
        Eigen::Vector3f m_corrected = softIronMatrix_ * (m_vec - hardIronOffset_);

        return MagneticFieldInBodyFrame{
            au::make_quantity<au::TeslaInBodyFrame>(m_corrected.x()),
            au::make_quantity<au::TeslaInBodyFrame>(m_corrected.y()),
            au::make_quantity<au::TeslaInBodyFrame>(m_corrected.z())
        };
    }

private:
    IMU& imu_;
    Eigen::Vector3f hardIronOffset_;
    Eigen::Matrix3f softIronMatrix_;
};

#endif // __IMUEXTENSION__HPP_