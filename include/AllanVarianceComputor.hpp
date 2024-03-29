#pragma once

#include <iomanip>
#include <fstream>
#include <mutex>

// allan_variance
#include "ImuMeasurement.hpp"
#include "yaml_parsers.hpp"

namespace allan_variance
{

    template <class T>
    using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

    struct ImuSensorMsg
    {
        double timestamp;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
        ImuSensorMsg(double t, double wx, double wy, double wz, double ax, double ay, double az)
            : timestamp(t), angular_velocity(wx, wy, wz), linear_acceleration(ax, ay, az) {}
    };
    struct ImuFormat
    {
        double time;
        double accX;
        double accY;
        double accZ;
        double gyroX;
        double gyroY;
        double gyroZ;
        double qx;
        double qy;
        double qz;
        double qw;

        void writeOnFile(std::ofstream &file)
        {
            file << std::setprecision(19) << time << std::setprecision(7) << " " << accX << " " << accY << " " << accZ << " "
                 << gyroX << " " << gyroY << " " << gyroZ << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
        }
    };

    struct AllanVarianceFormat
    {
        double period;
        double accX;
        double accY;
        double accZ;
        double gyroX;
        double gyroY;
        double gyroZ;

        void writeOnFile(std::ofstream &file)
        {
            file << std::setprecision(19) << period << std::setprecision(7) << " " << accX << " " << accY << " " << accZ << " "
                 << gyroX << " " << gyroY << " " << gyroZ << " " << std::endl;
        }
    };

    class AllanVarianceComputor
    {
    public:
        AllanVarianceComputor(std::string config_file, std::string output_path);

        virtual ~AllanVarianceComputor() { closeOutputs(); }

        void run(std::string bag_path);
        void closeOutputs();
        void allanVariance();
        void writeAllanDeviation(std::vector<double> variance, double period);

    private:
        // Data
        AllanVarianceFormat aVRecorder_{};
        std::ofstream av_output_;
        std::string imu_output_file_;

        // Config
        // int sequence_time_{};
        int measure_rate_{};
        double imu_rate_ = 100.0;

        int skipped_imu_{};
        int imu_skip_;
        uint64_t tCurrNanoSeconds_{};
        uint64_t lastImuTime_{};
        uint64_t firstTime_{};
        EigenVector<ImuMeasurement> imuBuffer_;
        bool firstMsg_;
        float overlap_; // Percent to overlap bins
    };
} // namespace allan_variance