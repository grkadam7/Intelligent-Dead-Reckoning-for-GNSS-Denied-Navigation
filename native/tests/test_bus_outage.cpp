/**
 * @file test_bus_outage.cpp
 * @brief C ABI parity test for the Phase 9 bus outage behavior.
 */

#include "../include/idr_mobile.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

const double PI = 3.14159265358979323846;
const double START_SPEED = 13.9; // ~50 km/h

void push_imu(idr_mobile_handle_t handle, double t) {
    IDRMobileIMUSample sample{};
    sample.timestamp_s = t;
    sample.accel_x_mps2 = 0.0;
    sample.accel_y_mps2 = 0.0;
    sample.accel_z_mps2 = 9.80665;
    sample.gyro_x_rads = 0.0;
    sample.gyro_y_rads = 0.0;
    sample.gyro_z_rads = 0.0;
    assert(idr_mobile_push_imu(handle, &sample) == IDR_SUCCESS);
}

void push_fix(idr_mobile_handle_t handle, double t, bool valid) {
    IDRMobileGNSSSample fix{};
    fix.timestamp_s = t;
    fix.latitude_deg = 15.3647;
    fix.longitude_deg = 75.1240;
    fix.altitude_m = 700.0;
    fix.accuracy_m = 2.5;
    fix.speed_mps = START_SPEED;
    fix.heading_deg = 90.0;
    fix.is_valid = valid ? 1 : 0;
    assert(idr_mobile_push_gnss(handle, &fix) == IDR_SUCCESS);
}

} // namespace

int main() {
    printf("Running native bus-outage DR tests...\n");

    idr_mobile_handle_t handle = idr_mobile_create(10.0);
    assert(handle != NULL);
    IDRMobileNavOutput out{};

    double t = 0.0;
    for (int i = 0; i < 10; i++) {
        t += 0.1;
        push_fix(handle, t, true);
        push_imu(handle, t);
        assert(idr_mobile_step(handle, &out) == IDR_SUCCESS);
    }
    assert(out.mode == IDR_MODE_GNSS_AIDED);
    printf("  [PASS] GNSS aided at ~50 km/h\n");

    const double start_lat = out.latitude_deg;
    const double start_lon = out.longitude_deg;
    const double start_uncertainty = out.horizontal_uncertainty_m;

    bool saw_dead_reckoning = false;
    double entry_speed = 0.0;
    double previous_uncertainty = out.horizontal_uncertainty_m;
    for (int i = 0; i < 200; i++) { // 20 s with no GNSS fixes
        t += 0.1;
        push_imu(handle, t);
        assert(idr_mobile_step(handle, &out) == IDR_SUCCESS);
        if (out.mode == IDR_MODE_DEAD_RECKONING) {
            if (!saw_dead_reckoning) {
                saw_dead_reckoning = true;
                entry_speed = out.speed_mps;
            }
            assert(out.horizontal_uncertainty_m >= previous_uncertainty - 1e-9);
        }
        previous_uncertainty = out.horizontal_uncertainty_m;
    }
    assert(saw_dead_reckoning);
    assert(entry_speed > START_SPEED * 0.9);
    assert(out.speed_mps > START_SPEED * 0.9);
    printf("  [PASS] speed survives outage entry: %.2f -> %.2f m/s\n",
           entry_speed, out.speed_mps);

    const double dlat_m = (out.latitude_deg - start_lat) * 111540.0;
    const double dlon_m = (out.longitude_deg - start_lon) * 111320.0 *
                          std::cos(15.3647 * PI / 180.0);
    const double distance_m = std::sqrt(dlat_m * dlat_m + dlon_m * dlon_m);
    assert(distance_m > 50.0);
    printf("  [PASS] position advanced %.1f m during outage\n", distance_m);

    assert(out.horizontal_uncertainty_m - start_uncertainty > 10.0);
    assert(out.state_age_ms > 19000);
    printf("  [PASS] uncertainty grew and fix age climbed to %d ms\n",
           out.state_age_ms);

    const double outage_uncertainty = out.horizontal_uncertainty_m;
    bool saw_recovering = false;
    for (int i = 0; i < 200; i++) { // 20 s of restored fixes
        t += 0.1;
        push_fix(handle, t, true);
        push_imu(handle, t);
        assert(idr_mobile_step(handle, &out) == IDR_SUCCESS);
        if (out.mode == IDR_MODE_RECOVERING) {
            saw_recovering = true;
        }
    }
    assert(saw_recovering);
    assert(out.mode == IDR_MODE_GNSS_AIDED);
    assert(out.horizontal_uncertainty_m < outage_uncertainty);
    assert(out.state_age_ms < 500);
    printf("  [PASS] recovery returns to GNSS aided with fresh fix\n");

    assert(idr_mobile_destroy(handle) == IDR_SUCCESS);
    printf("All native bus-outage tests passed.\n");
    return 0;
}
