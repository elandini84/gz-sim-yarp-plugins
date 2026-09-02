#include <BaseStateShared.hh>
#include <DeviceRegistry.hh>
#include <gzyarp/Common.hh>

#include <cstddef>
#include <mutex>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <gzyarp/YarpDevReturnValueCompat.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{
class BaseStateDriver;
}
} // namespace dev
} // namespace yarp

const std::string YarpBaseStateScopedName = "modelScopedName";
const unsigned YarpBaseStateChannelsNumber = 18; // The BaseState sensor has 18 fixed channels

class yarp::dev::gzyarp::BaseStateDriver : public yarp::dev::DeviceDriver,
                                           public yarp::dev::IAnalogSensor,
                                           public yarp::dev::IPreciselyTimed,
                                           public yarp::dev::IPositionSensors,
                                           public yarp::dev::IOrientationSensors,
                                           public yarp::dev::ILinearVelocitySensors,
                                           public yarp::dev::IThreeAxisGyroscopes,
                                           public yarp::dev::IThreeAxisLinearAccelerometers,
                                           public yarp::dev::IThreeAxisAngularAccelerometers,
                                           public ::gzyarp::IBaseStateData
{
public:
    BaseStateDriver()
    {
    }
    virtual ~BaseStateDriver()
    {
    }

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override
    {
        std::string m_modelScopedName(
            config.find(YarpBaseStateScopedName.c_str()).asString().c_str());

        std::string separator = "/";
        auto pos = config.find("baseLink").asString().find_last_of(separator);
        if (pos == std::string::npos)
        {
            m_baseLinkName = config.find("baseLink").asString();
        } else
        {
            m_baseLinkName = config.find("baseLink").asString().substr(pos + separator.size() - 1);
        }

        return true;
    }

    bool close() override
    {
        return true;
    }

    // IAnalogSensor
    int read(yarp::sig::Vector& out) override
    {
        if (out.size() != YarpBaseStateChannelsNumber)
        {
            out.resize(YarpBaseStateChannelsNumber);
        }

        {
            std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);

            if (!m_baseLinkData->dataAvailable)
            {
                yWarning() << "BaseState data not available";
                return AS_ERROR;
            }

            // Pack split buffers into legacy flat vector order for IAnalogSensor (SI units)
            out[0] = m_baseLinkData->position[0];
            out[1] = m_baseLinkData->position[1];
            out[2] = m_baseLinkData->position[2];

            out[3] = m_baseLinkData->orientation[0];
            out[4] = m_baseLinkData->orientation[1];
            out[5] = m_baseLinkData->orientation[2];

            out[6] = m_baseLinkData->linVel[0];
            out[7] = m_baseLinkData->linVel[1];
            out[8] = m_baseLinkData->linVel[2];

            out[9] = m_baseLinkData->angVel[0];
            out[10] = m_baseLinkData->angVel[1];
            out[11] = m_baseLinkData->angVel[2];

            out[12] = m_baseLinkData->linAcc[0];
            out[13] = m_baseLinkData->linAcc[1];
            out[14] = m_baseLinkData->linAcc[2];

            out[15] = m_baseLinkData->angAcc[0];
            out[16] = m_baseLinkData->angAcc[1];
            out[17] = m_baseLinkData->angAcc[2];
        }

        return AS_OK;
    }

    yarp::os::Stamp getLastInputStamp() override
    {
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);

        if (!m_baseLinkData->dataAvailable)
        {
            yDebug() << "BaseState data not available.";
        }

        return m_baseLinkData->simTimestamp;
    }

    int calibrateChannel(int ch, double v) override
    {
        return AS_OK;
    }
    int calibrateChannel(int ch) override
    {
        return AS_OK;
    }
    int calibrateSensor(const yarp::sig::Vector& value) override
    {
        return AS_OK;
    }
    int calibrateSensor() override
    {
        return AS_OK;
    }

    int getChannels() override
    {
        return YarpBaseStateChannelsNumber;
    }
    int getState(int ch) override
    {
        return AS_OK;
    }

    // MultipleAnalogSensors family implementations

    // IPositionSensors (worldBasePose.Pos())
    yarp::dev::ReturnValue getNrOfPositionSensors(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::MAS_status getPositionSensorStatus(size_t idx) const override
    {
        if (idx != 0)
        {
            return yarp::dev::MAS_status::MAS_ERROR;
        }
        return yarp::dev::MAS_status::MAS_OK;
    }
    yarp::dev::ReturnValue getPositionSensorName(size_t idx, std::string& name) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        name = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getPositionSensorFrameName(size_t idx, std::string& frame) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        frame = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getPositionSensorMeasure(size_t idx, yarp::sig::Vector& out, double& ts) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out.resize(3);
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);
        if (!m_baseLinkData->dataAvailable)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out[0] = m_baseLinkData->position[0];
        out[1] = m_baseLinkData->position[1];
        out[2] = m_baseLinkData->position[2];
        ts = m_baseLinkData->simTimestamp.getTime();
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // IOrientationSensors (worldBasePose.Rot()) as roll-pitch-yaw
    yarp::dev::ReturnValue getNrOfOrientationSensors(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::MAS_status getOrientationSensorStatus(size_t idx) const override
    {
        if (idx != 0)
        {
            return yarp::dev::MAS_status::MAS_ERROR;
        }
        return yarp::dev::MAS_status::MAS_OK;
    }
    yarp::dev::ReturnValue getOrientationSensorName(size_t idx, std::string& name) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        name = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getOrientationSensorFrameName(size_t idx, std::string& frame) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        frame = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getOrientationSensorMeasureAsRollPitchYaw(size_t idx,
                                                                      yarp::sig::Vector& rpy,
                                                                      double& ts) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        rpy.resize(3);
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);
        if (!m_baseLinkData->dataAvailable)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        // Convert from radians (Gazebo) to degrees (YARP)
        rpy[0] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->orientation[0]);
        rpy[1] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->orientation[1]);
        rpy[2] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->orientation[2]);
        ts = m_baseLinkData->simTimestamp.getTime();
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // ILinearVelocitySensors (worldBaseLinVel)
    yarp::dev::ReturnValue getNrOfLinearVelocitySensors(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::MAS_status getLinearVelocitySensorStatus(size_t idx) const override
    {
        if (idx != 0)
        {
            return yarp::dev::MAS_status::MAS_ERROR;
        }
        return yarp::dev::MAS_status::MAS_OK;
    }
    yarp::dev::ReturnValue getLinearVelocitySensorName(size_t idx, std::string& name) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        name = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getLinearVelocitySensorFrameName(size_t idx, std::string& frame) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        frame = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getLinearVelocitySensorMeasure(size_t idx, yarp::sig::Vector& out, double& ts) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out.resize(3);
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);
        if (!m_baseLinkData->dataAvailable)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out[0] = m_baseLinkData->linVel[0];
        out[1] = m_baseLinkData->linVel[1];
        out[2] = m_baseLinkData->linVel[2];
        ts = m_baseLinkData->simTimestamp.getTime();
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // IThreeAxisGyroscopes (worldBaseAngVel)
    yarp::dev::ReturnValue getNrOfThreeAxisGyroscopes(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t idx) const override
    {
        if (idx != 0)
        {
            return yarp::dev::MAS_status::MAS_ERROR;
        }
        return yarp::dev::MAS_status::MAS_OK;
    }
    yarp::dev::ReturnValue getThreeAxisGyroscopeName(size_t idx, std::string& name) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        name = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getThreeAxisGyroscopeFrameName(size_t idx, std::string& frame) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        frame = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getThreeAxisGyroscopeMeasure(size_t idx, yarp::sig::Vector& out, double& ts) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out.resize(3);
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);
        if (!m_baseLinkData->dataAvailable)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        // Convert from rad/s to deg/s
        out[0] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->angVel[0]);
        out[1] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->angVel[1]);
        out[2] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->angVel[2]);
        ts = m_baseLinkData->simTimestamp.getTime();
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // IThreeAxisLinearAccelerometers (worldBaseLinAcc)
    yarp::dev::ReturnValue getNrOfThreeAxisLinearAccelerometers(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t idx) const override
    {
        if (idx != 0)
        {
            return yarp::dev::MAS_status::MAS_ERROR;
        }
        return yarp::dev::MAS_status::MAS_OK;
    }
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerName(size_t idx, std::string& name) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        name = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerFrameName(size_t idx, std::string& frame) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        frame = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getThreeAxisLinearAccelerometerMeasure(size_t idx,
                                                                yarp::sig::Vector& out,
                                                                double& ts) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out.resize(3);
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);
        if (!m_baseLinkData->dataAvailable)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out[0] = m_baseLinkData->linAcc[0];
        out[1] = m_baseLinkData->linAcc[1];
        out[2] = m_baseLinkData->linAcc[2];
        ts = m_baseLinkData->simTimestamp.getTime();
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // IThreeAxisAngularAccelerometers (worldBaseAngAcc)
    yarp::dev::ReturnValue getNrOfThreeAxisAngularAccelerometers(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::MAS_status getThreeAxisAngularAccelerometerStatus(size_t idx) const override
    {
        if (idx != 0)
        {
            return yarp::dev::MAS_status::MAS_ERROR;
        }
        return yarp::dev::MAS_status::MAS_OK;
    }
    yarp::dev::ReturnValue getThreeAxisAngularAccelerometerName(size_t idx, std::string& name) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        name = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getThreeAxisAngularAccelerometerFrameName(size_t idx, std::string& frame) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        frame = m_baseLinkName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    yarp::dev::ReturnValue getThreeAxisAngularAccelerometerMeasure(size_t idx,
                                                                 yarp::sig::Vector& out,
                                                                 double& ts) const override
    {
        if (idx != 0)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        out.resize(3);
        std::lock_guard<std::mutex> lock(m_baseLinkData->mutex);
        if (!m_baseLinkData->dataAvailable)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }
        // Convert from rad/s^2 to deg/s^2
        out[0] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->angAcc[0]);
        out[1] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->angAcc[1]);
        out[2] = ::gzyarp::convertRadiansToDegrees(m_baseLinkData->angAcc[2]);
        ts = m_baseLinkData->simTimestamp.getTime();
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // IBaseStateData
    void setBaseStateData(::gzyarp::BaseStateData* baseLinkData) override
    {
        m_baseLinkData = baseLinkData;
    }

private:
    ::gzyarp::BaseStateData* m_baseLinkData = nullptr;
    std::string m_baseLinkName;
};
