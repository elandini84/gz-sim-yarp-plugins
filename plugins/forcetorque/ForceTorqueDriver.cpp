#include <DeviceRegistry.hh>
#include <ForceTorqueShared.hh>

#include <cstddef>
#include <mutex>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <gzyarp/YarpDevReturnValueCompat.h>
#include <yarp/sig/Vector.h>

namespace yarp
{
namespace dev
{
namespace gzyarp
{
class ForceTorqueDriver;
}
} // namespace dev
} // namespace yarp

const unsigned YarpForceTorqueChannelsNumber = 6; // The ForceTorque sensor has 6 fixed channels
const std::string YarpForceTorqueScopedName = "sensorScopedName";

const unsigned YarpTemperatureChannelsNumber = 1; // The Temperature sensor has 1 fixed channel
const double fakeTemperatureValue = 25.0;

class yarp::dev::gzyarp::ForceTorqueDriver : public yarp::dev::DeviceDriver,
                                             public yarp::dev::ISixAxisForceTorqueSensors,
                                             public yarp::dev::ITemperatureSensors,
                                             public ::gzyarp::IForceTorqueData
{
public:
    ForceTorqueDriver()
    {
    }
    virtual ~ForceTorqueDriver()
    {
    }

    // DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override
    {

        std::string sensorScopedName(
            config.find(YarpForceTorqueScopedName.c_str()).asString().c_str());

        std::string separator = "/";
        auto pos = config.find("sensor_name").asString().find_last_of(separator);
        if (pos == std::string::npos)
        {
            m_sensorName = config.find("sensor_name").asString();
        } else
        {
            m_sensorName = config.find("sensor_name").asString().substr(pos + separator.size() - 1);
        }

        m_frameName = m_sensorName;

        return true;
    }

    bool close() override
    {
        return true;
    }

    // SIX AXIS FORCE TORQUE SENSORS
    yarp::dev::ReturnValue getNrOfSixAxisForceTorqueSensors(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const override
    {
        if (sens_index >= 1)
        {
            return MAS_UNKNOWN;
        }

        return MAS_OK;
    }

    yarp::dev::ReturnValue getSixAxisForceTorqueSensorName(size_t sens_index, std::string& name) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        name = m_sensorName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    yarp::dev::ReturnValue getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string& frameName) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        frameName = m_frameName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }
    virtual yarp::dev::ReturnValue getSixAxisForceTorqueSensorMeasure(size_t sens_index,
                                                                    yarp::sig::Vector& out,
                                                                    double& timestamp) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        if (out.size() != YarpForceTorqueChannelsNumber)
        {
            out.resize(YarpForceTorqueChannelsNumber);
        }
        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        yarp::sig::Vector m_forceTorqueData;

        m_forceTorqueData.resize(YarpForceTorqueChannelsNumber, 0.0);
        m_forceTorqueData[0] = m_sensorData->m_data[0];
        m_forceTorqueData[1] = m_sensorData->m_data[1];
        m_forceTorqueData[2] = m_sensorData->m_data[2];
        m_forceTorqueData[3] = m_sensorData->m_data[3];
        m_forceTorqueData[4] = m_sensorData->m_data[4];
        m_forceTorqueData[5] = m_sensorData->m_data[5];
        out = m_forceTorqueData;

        timestamp = m_sensorData->simTime;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // TEMPERATURE SENSORS

    yarp::dev::ReturnValue getNrOfTemperatureSensors(size_t &n) const override
    {
        n = 1;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    yarp::dev::MAS_status getTemperatureSensorStatus(size_t sens_index) const override
    {
        if (sens_index >= 1)
        {
            return MAS_UNKNOWN;
        }

        return MAS_OK;
    }

    yarp::dev::ReturnValue getTemperatureSensorName(size_t sens_index, std::string& name) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        name = m_sensorName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    yarp::dev::ReturnValue getTemperatureSensorFrameName(size_t sens_index, std::string& frameName) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        frameName = m_frameName;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    yarp::dev::ReturnValue getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        out = fakeTemperatureValue;
        timestamp = m_sensorData->simTime;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    yarp::dev::ReturnValue getTemperatureSensorMeasure(size_t sens_index,
                                     yarp::sig::Vector& out,
                                     double& timestamp) const override
    {
        if (sens_index >= 1)
        {
            return YARP_DEV_RETURN_VALUE_ERROR_GENERIC_CH40;
        }

        if (out.size() != YarpTemperatureChannelsNumber)
        {
            out.resize(YarpTemperatureChannelsNumber);
        }

        std::lock_guard<std::mutex> lock(m_sensorData->m_mutex);
        out[0] = fakeTemperatureValue;
        timestamp = m_sensorData->simTime;
        return YARP_DEV_RETURN_VALUE_OK_CH40;
    }

    // IForceTorqueData

    void setForceTorqueData(::gzyarp::ForceTorqueData* ftData) override
    {
        m_sensorData = ftData;
    }

private:
    ::gzyarp::ForceTorqueData* m_sensorData = nullptr;
    std::string m_sensorName;
    std::string m_frameName;
};
