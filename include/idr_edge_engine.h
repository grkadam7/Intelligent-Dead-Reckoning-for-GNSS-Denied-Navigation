/**
 * @file idr_edge_engine.h
 * @brief C/C++ Interface for Intelligent Dead Reckoning System (IDR-INS) Edge Core Engine.
 * SIH Problem Statement ID: 26168 | ISRO
 *
 * Defines memory layouts, data structures, and C-linkage function prototypes
 * for integrating external tactical and Fiber Optic Gyro (FOG) IMUs up to 200 Hz.
 */

#ifndef IDR_EDGE_ENGINE_H
#define IDR_EDGE_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 8)

/**
 * @brief Navigation state mode enum matching IDR-INS core.
 */
typedef enum {
    IDR_NAV_MODE_NORMAL_FUSION = 0,
    IDR_NAV_MODE_OUTAGE_DEAD_RECKONING = 1,
    IDR_NAV_MODE_RECONVERGING = 2
} IDRNavigationMode_t;

/**
 * @brief High-rate 3-axis accelerometer and gyroscope sample.
 */
typedef struct {
    double timestamp_s;     /**< Epoch timestamp in seconds */
    double accel_x;         /**< Accelerometer X (forward/body) [m/s^2] */
    double accel_y;         /**< Accelerometer Y (lateral/body) [m/s^2] */
    double accel_z;         /**< Accelerometer Z (vertical/up)  [m/s^2] */
    double gyro_x;          /**< Gyroscope X (roll rate)        [rad/s] */
    double gyro_y;          /**< Gyroscope Y (pitch rate)       [rad/s] */
    double gyro_z;          /**< Gyroscope Z (yaw/heading rate) [rad/s] */
} IDR_IMUSample;

/**
 * @brief Asynchronous GNSS position/velocity measurement sample.
 */
typedef struct {
    double timestamp_s;     /**< Fix timestamp in seconds */
    double pos_east_m;      /**< East coordinate in ENU frame [m] */
    double pos_north_m;     /**< North coordinate in ENU frame [m] */
    double pos_up_m;        /**< Up coordinate in ENU frame [m] */
    double vel_east_mps;    /**< East velocity component [m/s] */
    double vel_north_mps;   /**< North velocity component [m/s] */
    double vel_up_mps;      /**< Up velocity component [m/s] */
    double accuracy_m;      /**< Estimated horizontal accuracy [m] */
    bool   valid;           /**< True if valid GNSS fix */
} IDR_GNSSSample;

/**
 * @brief Real-time fused navigation state output at each epoch.
 */
typedef struct {
    double timestamp_s;         /**< Output timestamp in seconds */
    double pos_east_m;          /**< Fused East position [m] */
    double pos_north_m;         /**< Fused North position [m] */
    double pos_up_m;            /**< Fused Up position [m] */
    double vel_east_mps;        /**< Fused East velocity [m/s] */
    double vel_north_mps;       /**< Fused North velocity [m/s] */
    double vel_up_mps;          /**< Fused Up velocity [m/s] */
    double heading_deg;         /**< Fused vehicle heading [degrees, 0=North, clockwise] */
    int32_t mode;               /**< IDRNavigationMode_t (0=Normal, 1=Outage, 2=Reconverging) */
    bool   is_outage;           /**< True if active GNSS blackout */
    double outage_duration_s;   /**< Current blackout duration in seconds */
    double cross_track_error_m; /**< Map-matched road cross-track distance [m] */
    double epoch_latency_ms;    /**< Execution latency for this epoch in ms */
} IDR_NavOutput;

/**
 * @brief Engine operational telemetry and performance statistics.
 */
typedef struct {
    uint64_t processed_samples;     /**< Total IMU samples processed */
    uint64_t dropped_samples;       /**< Number of dropped samples (buffer overflow) */
    double   avg_latency_ms;        /**< Exponential moving average of processing latency [ms] */
    double   max_latency_ms;        /**< Peak epoch latency encountered [ms] */
    double   frequency_hz;          /**< Measured operational frequency [Hz] */
} IDR_EngineStats;

#pragma pack(pop)

/* --- C API Prototypes --- */

/**
 * @brief Creates a new IDR-INS Edge Engine instance.
 * @param sampling_rate_hz Target IMU sampling rate (e.g., 50.0, 100.0, 200.0).
 * @return Opaque pointer to engine handle, or NULL on error.
 */
void* idr_engine_create(double sampling_rate_hz);

/**
 * @brief Destroys an IDR-INS Edge Engine instance and frees resources.
 * @param handle Engine instance handle.
 */
void idr_engine_destroy(void* handle);

/**
 * @brief Pushes an IMU sample into the engine's high-rate buffer.
 * @param handle Engine instance handle.
 * @param sample Pointer to IMU sample struct.
 * @return 0 on success, negative error code on failure.
 */
int idr_engine_push_imu(void* handle, const IDR_IMUSample* sample);

/**
 * @brief Pushes an asynchronous GNSS measurement into the engine.
 * @param handle Engine instance handle.
 * @param sample Pointer to GNSS sample struct.
 * @return 0 on success, negative error code on failure.
 */
int idr_engine_push_gnss(void* handle, const IDR_GNSSSample* sample);

/**
 * @brief Advances the navigation state by one epoch.
 * @param handle Engine instance handle.
 * @param output Pointer to output struct where fused state will be written.
 * @return 0 on success, negative error code on failure (e.g., no IMU data available).
 */
int idr_engine_step(void* handle, IDR_NavOutput* output);

/**
 * @brief Retrieves performance telemetry and health statistics.
 * @param handle Engine instance handle.
 * @param stats Pointer to stats struct to receive metrics.
 * @return 0 on success, negative error code on failure.
 */
int idr_engine_get_stats(void* handle, IDR_EngineStats* stats);

/**
 * @brief Resets the engine state.
 * @param handle Engine instance handle.
 */
void idr_engine_reset(void* handle);

#ifdef __cplusplus
}
#endif

#endif /* IDR_EDGE_ENGINE_H */
