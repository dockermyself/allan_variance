#pragma once

#include <Eigen/Core>
#include <string>

class ImuMeasurement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t t = 0; ///< ROS time message received (nanoseconds).

    Eigen::Vector3d I_a_WI; ///< Raw acceleration from the IMU (m/s/s)
    Eigen::Vector3d I_w_WI; ///< Raw angular velocity from the IMU (deg/s)

    ~ImuMeasurement() {}
    ImuMeasurement() : I_a_WI(0, 0, 0), I_w_WI(0, 0, 0) {}
    friend std::ostream &operator<<(std::ostream &stream, const ImuMeasurement &meas)
    {
        stream << "IMU Measurement at time = " << meas.t << " : \n"
               << "I_a_WI: " << meas.I_a_WI.transpose() << "\n"
               << "I_w_WI: " << meas.I_w_WI.transpose() << "\n";
        return stream;
    }
};
