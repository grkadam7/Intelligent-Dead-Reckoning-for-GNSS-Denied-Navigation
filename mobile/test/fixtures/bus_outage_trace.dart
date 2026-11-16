// Replays the shape of the 2026-11-16 bus trace: steady 50 km/h driving with
// near-gravity IMU samples, a GNSS blackout, then GNSS restoration.

import 'package:idr_mobile/navigation/live_sensor_source.dart';
import 'manual_clock.dart';

class BusOutageTrace {
  BusOutageTrace(this.source, this.clock);

  final LiveSensorSource source;
  final ManualClock clock;

  static const double speedMps = 13.9; // ~50 km/h
  static const double latitude = 15.3647;
  static const double longitude = 75.1240;

  void initialFix() {
    // Mark the engine running but keep the real timer inert; tests call tick().
    source.start(interval: const Duration(hours: 1));
    _pushFix();
  }

  void driveSeconds(int seconds) {
    for (var i = 0; i < seconds * 10; i++) {
      _epoch(withFix: true);
    }
  }

  void outageSeconds(int seconds) {
    for (var i = 0; i < seconds * 10; i++) {
      // Fixes keep arriving but the engine must ignore them during the outage.
      _epoch(withFix: true);
    }
  }

  void recoverSeconds(int seconds) {
    for (var i = 0; i < seconds * 10; i++) {
      _epoch(withFix: true);
    }
  }

  void _epoch({required bool withFix}) {
    clock.advance(const Duration(milliseconds: 100));
    source.pushImuSample(
      ax: 0.0,
      ay: 0.0,
      az: 9.80665,
      gx: 0.0,
      gy: 0.0,
      gz: 0.0,
    );
    if (withFix) _pushFix();
    source.tick();
  }

  void _pushFix() {
    source.pushGnssFix(
      latitude: latitude,
      longitude: longitude,
      altitude: 700.0,
      accuracy: 2.5,
      speedMps: speedMps,
      headingDeg: 90.0,
    );
  }
}
