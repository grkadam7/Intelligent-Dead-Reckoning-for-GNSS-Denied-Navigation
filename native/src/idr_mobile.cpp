/**
 * @file idr_mobile.cpp
 * @brief Implementation of the C-ABI interface for IDR-INS Mobile Navigation Core.
 * 
 * Provides concrete implementation of exported symbols declared in idr_mobile.h.
 */

#include "../include/idr_mobile.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <mutex>
#include <chrono>

namespace {

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_F = 1.0 / 298.257223563;
constexpr double WGS84_B = WGS84_A * (1.0 - WGS84_F);
constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F;
constexpr double PI = 3.14159265358979323846;

struct EngineInstance {
    double sampling_rate_hz;
    int64_t sequence{0};
    int32_t current_mode{IDR_MODE_INITIALIZING};
    int32_t calibration_stage{IDR_CALIB_CALIBRATED};

    // Reference Origin Datum (Coventry UK initial)
    double lat0_deg{52.40166};
    double lon0_deg{-1.50529};
    double alt0_m{147.5};

    // Navigation State in ENU
    double east_m{0.0};
    double north_m{0.0};
    double up_m{0.0};
    double speed_mps{0.0};
    double heading_deg{90.0};
    double horizontal_uncertainty_m{1.8};

    // Orientation Leveling
    double roll_deg{1.2};
    double pitch_deg{-0.8};
    double yaw_deg{90.0};

    // Timing & Outage Tracking
    double last_timestamp_s{0.0};
    double blackout_start_s{0.0};
    bool is_blackout{false};

    std::mutex mtx;

    EngineInstance(double rate_hz) : sampling_rate_hz(rate_hz) {}

    void enu_to_geodetic(double e, double n, double u, double& out_lat, double& out_lon, double& out_alt) const {
        double lat0_rad = lat0_deg * PI / 180.0;
        double lon0_rad = lon0_deg * PI / 180.0;
        double sin_lat = std::sin(lat0_rad);
        double cos_lat = std::cos(lat0_rad);
        double sin_lon = std::sin(lon0_rad);
        double cos_lon = std::cos(lon0_rad);

        double n0 = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
        double x0 = (n0 + alt0_m) * cos_lat * cos_lon;
        double y0 = (n0 + alt0_m) * cos_lat * sin_lon;
        double z0 = (n0 * (1.0 - WGS84_E2) + alt0_m) * sin_lat;

        double dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u;
        double dy =  cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u;
        double dz =  cos_lat * n + sin_lat * u;

        double x = x0 + dx;
        double y = y0 + dy;
        double z = z0 + dz;

        double p = std::sqrt(x * x + y * y);
        double theta = std::atan2(z * WGS84_A, p * WGS84_B);
        double e_prime_sq = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);

        double lat = std::atan2(
            z + e_prime_sq * WGS84_B * std::pow(std::sin(theta), 3),
            p - WGS84_E2 * WGS84_A * std::pow(std::cos(theta), 3)
        );
        double lon = std::atan2(y, x);
        double n_cur = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
        double alt = p / std::cos(lat) - n_cur;

        out_lat = lat * 180.0 / PI;
        out_lon = lon * 180.0 / PI;
        out_alt = alt;
    }
};

} // anonymous namespace

int32_t idr_mobile_get_abi_version(void) {
    return IDR_MOBILE_ABI_VERSION;
}

idr_mobile_handle_t idr_mobile_create(double sampling_rate_hz) {
    if (sampling_rate_hz <= 0.0) {
        sampling_rate_hz = 10.0;
    }
    return static_cast<idr_mobile_handle_t>(new EngineInstance(sampling_rate_hz));
}

int32_t idr_mobile_destroy(idr_mobile_handle_t handle) {
    if (!handle) return IDR_ERR_INVALID_HANDLE;
    auto* inst = static_cast<EngineInstance*>(handle);
    delete inst;
    return IDR_SUCCESS;
}

int32_t idr_mobile_push_imu(idr_mobile_handle_t handle, const IDRMobileIMUSample* sample) {
    if (!handle) return IDR_ERR_INVALID_HANDLE;
    if (!sample) return IDR_ERR_NULL_POINTER;

    auto* inst = static_cast<EngineInstance*>(handle);
    std::lock_guard<std::mutex> lock(inst->mtx);

    double dt = (inst->last_timestamp_s > 0.0) ? (sample->timestamp_s - inst->last_timestamp_s) : (1.0 / inst->sampling_rate_hz);
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / inst->sampling_rate_hz;
    inst->last_timestamp_s = sample->timestamp_s;

    // Kinematic forward velocity propagation
    double fwd_accel = sample->accel_x_mps2;
    inst->speed_mps = std::max(0.0, inst->speed_mps + fwd_accel * dt);

    // Yaw integration
    double yaw_rate = sample->gyro_z_rads;
    inst->heading_deg = std::fmod(inst->heading_deg + yaw_rate * dt * 180.0 / PI + 360.0, 360.0);

    // ENU position integration
    double rad = inst->heading_deg * PI / 180.0;
    inst->east_m += inst->speed_mps * std::sin(rad) * dt;
    inst->north_m += inst->speed_mps * std::cos(rad) * dt;

    if (inst->is_blackout) {
        inst->horizontal_uncertainty_m += 0.05 * dt;
    }

    return IDR_SUCCESS;
}

int32_t idr_mobile_push_gnss(idr_mobile_handle_t handle, const IDRMobileGNSSSample* sample) {
    if (!handle) return IDR_ERR_INVALID_HANDLE;
    if (!sample) return IDR_ERR_NULL_POINTER;

    auto* inst = static_cast<EngineInstance*>(handle);
    std::lock_guard<std::mutex> lock(inst->mtx);

    if (sample->is_valid) {
        if (inst->is_blackout) {
            inst->is_blackout = false;
            inst->current_mode = IDR_MODE_RECOVERING;
        } else {
            inst->current_mode = IDR_MODE_GNSS_AIDED;
        }
        inst->speed_mps = sample->speed_mps;
        inst->heading_deg = sample->heading_deg;
        inst->horizontal_uncertainty_m = sample->accuracy_m;
    } else {
        if (!inst->is_blackout) {
            inst->is_blackout = true;
            inst->blackout_start_s = sample->timestamp_s;
            inst->current_mode = IDR_MODE_DEAD_RECKONING;
        }
    }

    return IDR_SUCCESS;
}

int32_t idr_mobile_step(idr_mobile_handle_t handle, IDRMobileNavOutput* out_snapshot) {
    if (!handle) return IDR_ERR_INVALID_HANDLE;
    if (!out_snapshot) return IDR_ERR_NULL_POINTER;

    auto* inst = static_cast<EngineInstance*>(handle);
    std::lock_guard<std::mutex> lock(inst->mtx);

    inst->sequence++;

    double lat, lon, alt;
    inst->enu_to_geodetic(inst->east_m, inst->north_m, inst->up_m, lat, lon, alt);

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    out_snapshot->timestamp_ms = now_ms;
    out_snapshot->sequence = inst->sequence;
    out_snapshot->mode = inst->current_mode;
    out_snapshot->calibration_stage = inst->calibration_stage;
    out_snapshot->latitude_deg = lat;
    out_snapshot->longitude_deg = lon;
    out_snapshot->altitude_m = alt;
    out_snapshot->raw_latitude_deg = lat;
    out_snapshot->raw_longitude_deg = lon;
    out_snapshot->speed_mps = inst->speed_mps;
    out_snapshot->heading_deg = inst->heading_deg;
    out_snapshot->horizontal_uncertainty_m = inst->horizontal_uncertainty_m;
    out_snapshot->roll_deg = inst->roll_deg;
    out_snapshot->pitch_deg = inst->pitch_deg;
    out_snapshot->yaw_deg = inst->yaw_deg;
    out_snapshot->is_matched = 1;
    out_snapshot->state_age_ms = 15;

    return IDR_SUCCESS;
}

int32_t idr_mobile_reset(idr_mobile_handle_t handle) {
    if (!handle) return IDR_ERR_INVALID_HANDLE;

    auto* inst = static_cast<EngineInstance*>(handle);
    std::lock_guard<std::mutex> lock(inst->mtx);

    inst->sequence = 0;
    inst->east_m = 0.0;
    inst->north_m = 0.0;
    inst->up_m = 0.0;
    inst->speed_mps = 0.0;
    inst->heading_deg = 90.0;
    inst->horizontal_uncertainty_m = 1.8;
    inst->is_blackout = false;
    inst->current_mode = IDR_MODE_INITIALIZING;

    return IDR_SUCCESS;
}

