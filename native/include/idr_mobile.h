/**
 * @file idr_mobile.h
 * @brief High-performance C-ABI interface for IDR-INS Mobile Navigation Engine.
 * 
 * Provides thread-safe, memory-safe, foreign-function interface (FFI) bindings
 * for Flutter (dart:ffi), Android NDK (JNI), and iOS (Swift/ObjC).
 * 
 * SIH Problem Statement ID: 26168 | ISRO
 */

#ifndef IDR_MOBILE_H
#define IDR_MOBILE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IDR_MOBILE_ABI_VERSION 1

/* Status return codes */
#define IDR_SUCCESS              0
#define IDR_ERR_INVALID_HANDLE  -1
#define IDR_ERR_NULL_POINTER    -2
#define IDR_ERR_QUEUE_FULL      -3
#define IDR_ERR_NOT_READY       -4

/** Navigation modes matching mobile UI specification */
typedef enum {
    IDR_MODE_INITIALIZING    = 0,
    IDR_MODE_GNSS_AIDED      = 1,
    IDR_MODE_DEAD_RECKONING  = 2,
    IDR_MODE_RECOVERING      = 3,
    IDR_MODE_STALE           = 4,
    IDR_MODE_SUSPENDED       = 5,
    IDR_MODE_FAILED          = 6
} IDRNavigationMode_t;

/** Phone-to-vehicle dynamic auto-calibration stage */
typedef enum {
    IDR_CALIB_WAITING_STILLNESS = 0,
    IDR_CALIB_LEVELING          = 1,
    IDR_CALIB_WAITING_STRAIGHT  = 2,
    IDR_CALIB_ALIGNING_YAW      = 3,
    IDR_CALIB_CALIBRATED        = 4,
    IDR_CALIB_MOUNT_CHANGED     = 5,
    IDR_CALIB_FAILED            = 6
} IDRCalibrationStage_t;

/** Raw IMU epoch measurement from device sensors */
#pragma pack(push, 8)
typedef struct {
    double timestamp_s;     /**< Monotonic sensor capture timestamp in seconds */
    double accel_x_mps2;    /**< Device phone frame acceleration X (m/s^2) */
    double accel_y_mps2;    /**< Device phone frame acceleration Y (m/s^2) */
    double accel_z_mps2;    /**< Device phone frame acceleration Z (m/s^2) */
    double gyro_x_rads;     /**< Device phone frame angular rate X (rad/s) */
    double gyro_y_rads;     /**< Device phone frame angular rate Y (rad/s) */
    double gyro_z_rads;     /**< Device phone frame angular rate Z (rad/s) */
} IDRMobileIMUSample;

/** GNSS measurement sample */
typedef struct {
    double timestamp_s;     /**< Monotonic timestamp of GNSS fix */
    double latitude_deg;    /**< WGS84 latitude in degrees */
    double longitude_deg;   /**< WGS84 longitude in degrees */
    double altitude_m;      /**< Ellipsoidal altitude in meters */
    double accuracy_m;      /**< 1-sigma horizontal accuracy estimate in meters */
    double speed_mps;       /**< GNSS ground speed in m/s */
    double heading_deg;     /**< GNSS ground course azimuth in degrees */
    int32_t is_valid;       /**< 1 if fix is valid and healthy, 0 if blackout */
} IDRMobileGNSSSample;

/** Published navigation snapshot matching mobile Dart NavSnapshot contract */
typedef struct {
    int64_t timestamp_ms;               /**< Monotonic publication time in ms */
    int64_t sequence;                   /**< Strictly monotonic snapshot counter */
    int32_t mode;                       /**< IDRNavigationMode_t */
    int32_t calibration_stage;          /**< IDRCalibrationStage_t */
    double latitude_deg;                /**< Filtered/snapped WGS84 latitude */
    double longitude_deg;               /**< Filtered/snapped WGS84 longitude */
    double altitude_m;                  /**< WGS84 altitude in meters */
    double raw_latitude_deg;            /**< Raw dead-reckoned latitude */
    double raw_longitude_deg;           /**< Raw dead-reckoned longitude */
    double speed_mps;                   /**< Forward vehicle velocity in m/s */
    double heading_deg;                 /**< Vehicle azimuth (0=North, 90=East) */
    double horizontal_uncertainty_m;    /**< 95% horizontal uncertainty radius */
    double roll_deg;                    /**< Vehicle roll leveling angle */
    double pitch_deg;                   /**< Vehicle pitch leveling angle */
    double yaw_deg;                     /**< Vehicle forward alignment yaw */
    int32_t is_matched;                 /**< 1 if snapped to offline road link */
    int32_t state_age_ms;               /**< Processing latency in milliseconds */
} IDRMobileNavOutput;
#pragma pack(pop)

typedef void* idr_mobile_handle_t;

/* C-ABI Function Exports */

/**
 * Returns the library ABI version number (currently 1).
 */
int32_t idr_mobile_get_abi_version(void);

/**
 * Initializes a new instance of the mobile navigation core.
 * @param sampling_rate_hz Target rate (typically 10.0 Hz).
 * @return Opaque engine handle, or NULL on allocation failure.
 */
idr_mobile_handle_t idr_mobile_create(double sampling_rate_hz);

/**
 * Destroys an engine instance and frees all allocated ring buffers and filters.
 * @param handle Valid engine handle.
 * @return IDR_SUCCESS or error code.
 */
int32_t idr_mobile_destroy(idr_mobile_handle_t handle);

/**
 * Ingests a raw IMU sample into the lock-free input ring buffer.
 * @param handle Valid engine handle.
 * @param sample Pointer to IDRMobileIMUSample.
 * @return IDR_SUCCESS, IDR_ERR_QUEUE_FULL, or error code.
 */
int32_t idr_mobile_push_imu(idr_mobile_handle_t handle, const IDRMobileIMUSample* sample);

/**
 * Ingests a GNSS fix sample.
 * @param handle Valid engine handle.
 * @param sample Pointer to IDRMobileGNSSSample.
 * @return IDR_SUCCESS or error code.
 */
int32_t idr_mobile_push_gnss(idr_mobile_handle_t handle, const IDRMobileGNSSSample* sample);

/**
 * Executes a single navigation epoch update and populates the latest snapshot.
 * @param handle Valid engine handle.
 * @param out_snapshot Output snapshot struct to populate.
 * @return IDR_SUCCESS, IDR_ERR_NOT_READY, or error code.
 */
int32_t idr_mobile_step(idr_mobile_handle_t handle, IDRMobileNavOutput* out_snapshot);

/**
 * Resets navigation state, clears buffers, resets calibration and EKF.
 * @param handle Valid engine handle.
 * @return IDR_SUCCESS or error code.
 */
int32_t idr_mobile_reset(idr_mobile_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* IDR_MOBILE_H */

