#pragma once

#include "types.hpp"
#include <functional>
#include <memory>

namespace stampfly {
namespace estimator {

/**
 * @brief Abstract base class for sensor data providers
 *
 * This interface allows the estimator to be decoupled from specific
 * hardware implementations. Different platforms can implement this
 * interface to provide sensor data to the estimator.
 */
class ISensorProvider {
public:
    virtual ~ISensorProvider() = default;

    /**
     * @brief Initialize the sensor
     * @return true if initialization successful
     */
    virtual bool initialize() = 0;

    /**
     * @brief Check if sensor is ready to provide data
     */
    virtual bool isReady() const = 0;

    /**
     * @brief Get sensor name/identifier
     */
    virtual std::string getName() const = 0;
};

/**
 * @brief IMU sensor provider interface
 */
class IImuProvider : public ISensorProvider {
public:
    /**
     * @brief Register callback for IMU data
     * @param callback Function to be called when new IMU data arrives
     */
    virtual void registerCallback(std::function<void(const ImuData&)> callback) = 0;

    /**
     * @brief Get current IMU data (polling mode)
     * @param data Output IMU data
     * @return true if data is valid
     */
    virtual bool getData(ImuData& data) = 0;

    /**
     * @brief Get IMU sample rate
     */
    virtual double getSampleRate() const = 0;
};

/**
 * @brief Magnetometer sensor provider interface
 */
class IMagnetometerProvider : public ISensorProvider {
public:
    virtual void registerCallback(std::function<void(const MagnetometerData&)> callback) = 0;
    virtual bool getData(MagnetometerData& data) = 0;
    virtual double getSampleRate() const = 0;
};

/**
 * @brief Barometer sensor provider interface
 */
class IBarometerProvider : public ISensorProvider {
public:
    virtual void registerCallback(std::function<void(const BarometerData&)> callback) = 0;
    virtual bool getData(BarometerData& data) = 0;
    virtual double getSampleRate() const = 0;

    /**
     * @brief Set reference altitude for relative measurements
     * @param altitude Reference altitude in meters
     */
    virtual void setReferenceAltitude(double altitude) = 0;
};

/**
 * @brief ToF sensor provider interface
 */
class ITofProvider : public ISensorProvider {
public:
    virtual void registerCallback(std::function<void(const TofData&)> callback) = 0;
    virtual bool getData(TofData& data) = 0;
    virtual double getSampleRate() const = 0;

    /**
     * @brief Get maximum measurement range
     */
    virtual double getMaxRange() const = 0;
};

/**
 * @brief Optical flow sensor provider interface
 */
class IOpticalFlowProvider : public ISensorProvider {
public:
    virtual void registerCallback(std::function<void(const OpticalFlowData&)> callback) = 0;
    virtual bool getData(OpticalFlowData& data) = 0;
    virtual double getSampleRate() const = 0;
};

/**
 * @brief Aggregated sensor suite manager
 *
 * This class manages all sensor providers and coordinates data flow
 * to the estimator. It supports both callback-based (asynchronous)
 * and polling-based (synchronous) operation modes.
 */
class SensorSuite {
public:
    SensorSuite() = default;
    ~SensorSuite() = default;

    // Register sensor providers
    void setImuProvider(std::shared_ptr<IImuProvider> provider);
    void setMagnetometerProvider(std::shared_ptr<IMagnetometerProvider> provider);
    void setBarometerProvider(std::shared_ptr<IBarometerProvider> provider);
    void setTofProvider(std::shared_ptr<ITofProvider> provider);
    void setOpticalFlowProvider(std::shared_ptr<IOpticalFlowProvider> provider);

    // Initialize all registered sensors
    bool initializeAll();

    // Check if all required sensors are ready
    bool allRequiredSensorsReady() const;

    // Get sensor providers
    std::shared_ptr<IImuProvider> getImuProvider() const { return imu_provider_; }
    std::shared_ptr<IMagnetometerProvider> getMagnetometerProvider() const { return mag_provider_; }
    std::shared_ptr<IBarometerProvider> getBarometerProvider() const { return baro_provider_; }
    std::shared_ptr<ITofProvider> getTofProvider() const { return tof_provider_; }
    std::shared_ptr<IOpticalFlowProvider> getOpticalFlowProvider() const { return flow_provider_; }

private:
    std::shared_ptr<IImuProvider> imu_provider_;
    std::shared_ptr<IMagnetometerProvider> mag_provider_;
    std::shared_ptr<IBarometerProvider> baro_provider_;
    std::shared_ptr<ITofProvider> tof_provider_;
    std::shared_ptr<IOpticalFlowProvider> flow_provider_;
};

} // namespace estimator
} // namespace stampfly
