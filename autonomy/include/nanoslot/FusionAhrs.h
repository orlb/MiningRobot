/**
IMU Fusion automatic heading reference system.

Modified from original MIT version to make one .c file and merge most headers
by Orion Lawlor, lawlor@alaska.edu, 2023-03-11 (Public Domain)

Original from: https://github.com/xioTechnologies/Fusion
 * @file FusionAhrs.h
 * @author Seb Madgwick
 * @brief AHRS algorithm to combine gyroscope, accelerometer, and magnetometer
 * measurements into a single measurement of orientation relative to gravity.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

//------------------------------------------------------------------------------
// Includes
/**
 * @brief Gravity axes convention.
 */
typedef enum {
    FusionConventionNwu, /* XYZ = North-West-Up */
    FusionConventionEnu, /* XYZ = East-North-Up */
    FusionConventionNed, /* XYZ = North-East-Down */
} FusionConvention;

#include "FusionMath.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm settings.
 */
typedef struct {
    FusionConvention convention;
    float gain;
    float accelerationRejection;
    unsigned int rejectionTimeout;
} FusionAhrsSettings;

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * must not be accessed by the application.
 */
typedef struct {
    FusionAhrsSettings settings;
    FusionQuaternion quaternion;
    FusionVector accelerometer;
    bool initialising;
    float rampedGain;
    float rampedGainStep;
    FusionVector halfAccelerometerFeedback;
    FusionVector halfMagnetometerFeedback;
    bool accelerometerIgnored;
    unsigned int accelerationRejectionTimer;
    bool accelerationRejectionTimeout;
} FusionAhrs;

/**
 * @brief AHRS algorithm internal states.
 */
typedef struct {
    float accelerationError;
    bool accelerometerIgnored;
    float accelerationRejectionTimer;
    float magneticError;
} FusionAhrsInternalStates;

/**
 * @brief AHRS algorithm flags.
 */
typedef struct {
    bool initialising;
    bool accelerationRejectionWarning;
    bool accelerationRejectionTimeout;
} FusionAhrsFlags;

//------------------------------------------------------------------------------
// Function declarations

void FusionAhrsInitialise(FusionAhrs *const ahrs);

void FusionAhrsReset(FusionAhrs *const ahrs);

void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings);

void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const float deltaTime);

void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);

void FusionAhrsUpdateExternalHeading(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float heading, const float deltaTime);

// Return estimated orientation
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs);

// Return estimated linear acceleration not due to gravity
FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs);

// Return acceleration in global coordinates (e.g., to integrate positions)
FusionVector FusionAhrsGetGlobalAcceleration(const FusionAhrs *const ahrs);

FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs);

FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs);

void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading);

float FusionAhrsMatchX(FusionAhrs *const ahrs, FusionVector targetX, float filter=0.03);


// FusionCalibration

//------------------------------------------------------------------------------
// Inline functions

/**
 * @brief Gyroscope and accelerometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param misalignment Misalignment matrix.
 * @param sensitivity Sensitivity.
 * @param offset Offset.
 * @return Calibrated measurement.
 */
static inline FusionVector FusionCalibrationInertial(const FusionVector uncalibrated, const FusionMatrix misalignment, const FusionVector sensitivity, const FusionVector offset) {
    return FusionMatrixMultiplyVector(misalignment, FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
}

//------------------------------------------------------------------------------
// Function declarations

float FusionCompassCalculateHeading(const FusionConvention convention, const FusionVector accelerometer, const FusionVector magnetometer);

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Gyroscope offset algorithm structure.  Structure members are used
 * internally and must not be accessed by the application.
 */
typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    FusionVector gyroscopeOffset;
} FusionOffset;

//------------------------------------------------------------------------------
// Function declarations

void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate);

FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);




#endif

//------------------------------------------------------------------------------
// End of file
