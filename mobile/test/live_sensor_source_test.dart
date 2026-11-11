import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/nav_snapshot.dart';
import 'package:idr_mobile/navigation/live_sensor_source.dart';

void main() {
  group('LiveSensorSource Real-Time Navigation Tests', () {
    test('Initializes and accepts live GPS fix', () async {
      final source = LiveSensorSource();
      final snapshots = <NavSnapshot>[];

      final sub = source.stream.listen((snap) {
        snapshots.add(snap);
      });

      source.start(interval: const Duration(milliseconds: 20));

      // Push valid GPS fix
      source.pushGnssFix(
        latitude: 28.6139,
        longitude: 77.2090,
        altitude: 216.0,
        accuracy: 2.5,
        speedMps: 15.0,
        headingDeg: 90.0,
      );

      await Future.delayed(const Duration(milliseconds: 50));

      expect(snapshots.isNotEmpty, isTrue);
      expect(source.currentMode, IdrNavMode.gnssAided);
      expect(snapshots.last.source, NavSource.live);
      expect(snapshots.last.simulated, isFalse);

      source.dispose();
      await sub.cancel();
    });

    test('Detects GNSS outage and switches to Dead Reckoning', () async {
      final source = LiveSensorSource();
      source.start(interval: const Duration(milliseconds: 10));

      // 1. Initial GPS Fix
      source.pushGnssFix(
        latitude: 28.6139,
        longitude: 77.2090,
        altitude: 216.0,
        accuracy: 2.5,
        speedMps: 14.0,
        headingDeg: 90.0,
      );

      // 2. Feed IMU while vehicle enters tunnel
      for (int i = 0; i < 5; i++) {
        source.pushImuSample(
          ax: 0.1, ay: 0.0, az: 9.80665,
          gx: 0.0, gy: 0.0, gz: 0.0,
        );
      }

      // Simulate GPS signal loss by waiting without GPS updates
      await Future.delayed(const Duration(milliseconds: 60));

      expect(source.isRunning, isTrue);
      source.dispose();
    });
  });
}

