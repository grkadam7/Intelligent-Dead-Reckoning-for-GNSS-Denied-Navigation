/**
 * @file test_abi.cpp
 * @brief Unit tests for IDR-INS mobile C ABI symbols and memory alignment.
 */

#include "../include/idr_mobile.h"
#include <cassert>
#include <cstdio>
#include <cmath>

int main() {
    printf("Running IDR Mobile C ABI Tests...\n");

    // 1. ABI Version
    assert(idr_mobile_get_abi_version() == IDR_MOBILE_ABI_VERSION);
    printf("  [PASS] ABI version = %d\n", IDR_MOBILE_ABI_VERSION);

    // 2. Struct size and alignment checks
    assert(sizeof(IDRMobileIMUSample) == 56); // 7 * 8 bytes
    assert(sizeof(IDRMobileGNSSSample) == 64); // 7 * 8 bytes + int32 (aligned to 64)
    printf("  [PASS] Memory alignment: IMU sample = %zu bytes, GNSS sample = %zu bytes\n",
           sizeof(IDRMobileIMUSample), sizeof(IDRMobileGNSSSample));

    // 3. Engine lifecycle
    idr_mobile_handle_t handle = idr_mobile_create(10.0);
    assert(handle != NULL);
    printf("  [PASS] Engine handle created successfully\n");

    // 4. Ingest IMU sample
    IDRMobileIMUSample imu{};
    imu.timestamp_s = 0.1;
    imu.accel_x_mps2 = 1.4;
    imu.accel_z_mps2 = 9.80665;
    int32_t rc = idr_mobile_push_imu(handle, &imu);
    assert(rc == IDR_SUCCESS);
    printf("  [PASS] idr_mobile_push_imu succeeded\n");

    // 5. Ingest GNSS sample
    IDRMobileGNSSSample gnss{};
    gnss.timestamp_s = 0.1;
    gnss.latitude_deg = 52.40166;
    gnss.longitude_deg = -1.50529;
    gnss.altitude_m = 147.5;
    gnss.accuracy_m = 1.5;
    gnss.speed_mps = 1.4;
    gnss.heading_deg = 90.0;
    gnss.is_valid = 1;
    rc = idr_mobile_push_gnss(handle, &gnss);
    assert(rc == IDR_SUCCESS);
    printf("  [PASS] idr_mobile_push_gnss succeeded\n");

    // 6. Step engine and inspect snapshot
    IDRMobileNavOutput out{};
    rc = idr_mobile_step(handle, &out);
    assert(rc == IDR_SUCCESS);
    assert(out.sequence == 1);
    assert(out.mode == IDR_MODE_GNSS_AIDED);
    assert(std::abs(out.latitude_deg - 52.40166) < 0.01);
    assert(std::abs(out.longitude_deg - (-1.50529)) < 0.01);
    printf("  [PASS] idr_mobile_step generated snapshot #%lld (Lat: %.5f, Lon: %.5f)\n",
           (long long)out.sequence, out.latitude_deg, out.longitude_deg);

    // 7. Reset and verify sequence reset
    rc = idr_mobile_reset(handle);
    assert(rc == IDR_SUCCESS);
    rc = idr_mobile_step(handle, &out);
    assert(rc == IDR_SUCCESS);
    assert(out.sequence == 1); // Reset sequence back to 0, next step = 1
    printf("  [PASS] idr_mobile_reset restored engine state\n");

    // 8. Destroy
    rc = idr_mobile_destroy(handle);
    assert(rc == IDR_SUCCESS);
    printf("  [PASS] idr_mobile_destroy cleaned up resources\n");

    printf("\nAll IDR Mobile C ABI Tests PASSED!\n");
    return 0;
}

