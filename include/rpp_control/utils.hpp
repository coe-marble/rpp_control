#pragma once
#include <array>
#include <cmath>
#include <memory>
#include <rpp_schema/rpp_common/Quaternion.hpp>

namespace rpp_control {

    template<typename ... Args>
    [[maybe_unused]] std::string format( const std::string& format, Args ... args )
    {
        int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
        auto size = static_cast<size_t>( size_s );
        std::unique_ptr<char[]> buf( new char[ size ] );
        std::snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }

    [[maybe_unused]] static double wrap_angle(double angle)
    {
        constexpr double PI = 3.14159265358979323846;

        while (angle > PI)
        {
            angle -= 2.0 * PI;
        }

        while (angle < -PI)
        {
            angle += 2.0 * PI;
        }

        return angle;
    }

    inline double quat_roll(double x, double y, double z, double w)
    {
        return std::atan2(2 * (y * z + x * w),
                1 - 2 * (x * x + y * y));
    }

    inline double quat_pitch(double x, double y, double z, double w)
    {
        return -std::asin(2 * (x * z - y * w));
    }

    inline double quat_yaw(double x, double y, double z, double w)
    {
        return std::atan2(2 * (y * x + w * z),
                1 - 2 * (y * y + z * z));
    }

    inline std::tuple<double, double, double>
    quat2euler(double x, double y, double z, double w)
    {
        return {quat_roll(x, y, z, w),
            quat_pitch(x, y, z, w), quat_yaw(x, y, z, w)};
    }

    inline std::tuple<double, double, double>
    quat2euler(const rpp_schema::rpp_common::Quaternion_Native& quat)
    {
        return quat2euler(quat.x, quat.y, quat.z, quat.w);
    }

    inline std::tuple<double, double, double, double>
    euler2quat(double roll, double pitch, double yaw)
    {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);

        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;
        double w = cr * cp * cy + sr * sp * sy;

        return {x, y, z, w};
    }
}