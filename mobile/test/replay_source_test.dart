import 'dart:convert';
import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/nav_snapshot.dart';
import 'package:idr_mobile/navigation/replay_source.dart';

void main() {
  group('NavSnapshot serialization and parsing', () {
    test('Correctly deserializes JSON snapshot', () {
      final jsonMap = {
        'schema_version': '1.0.0',
        'sequence': 42,
        'source': 'replay',
        'timestamp_ms': 4200,
        'state_age_ms': 30,
        'latitude': 52.40166,
        'longitude': -1.50529,
        'altitude_m': 147.5,
        'raw_latitude': 52.40170,
        'raw_longitude': -1.50525,
        'speed_mps': 14.0,
        'speed_kmh': 50.4,
        'speed_valid': true,
        'heading_deg': 90.0,
        'heading_valid': true,
        'navigation_mode': 'dead_reckoning',
        'mode_reason': 'GNSS blackout detected',
        'horizontal_uncertainty_m': 3.2,
        'uncertainty_valid': true,
        'is_matched': true,
        'matched_link_id': 'corridor_0',
        'calibration_stage': 'calibrated',
        'roll_deg': 1.0,
        'pitch_deg': -0.5,
        'yaw_deg': 90.0,
        'simulated': true,
      };

      final snapshot = NavSnapshot.fromJson(jsonMap);

      expect(snapshot.sequence, 42);
      expect(snapshot.source, NavSource.replay);
      expect(snapshot.mode, IdrNavMode.deadReckoning);
      expect(snapshot.speedKmh, 50.4);
      expect(snapshot.headingDeg, 90.0);
      expect(snapshot.horizontalUncertaintyM, 3.2);
      expect(snapshot.calibrationStage, CalibrationStage.calibrated);
      expect(snapshot.simulated, isTrue);
    });
  });

  group('ReplaySource streaming logic', () {
    test('Publishes sequence of snapshots monotonically', () async {
      final rawSnapshots = [
        {
          'sequence': 1,
          'timestamp_ms': 100,
          'latitude': 52.401,
          'longitude': -1.505,
          'navigation_mode': 'gnss_aided',
          'speed_valid': true,
          'heading_valid': true,
          'uncertainty_valid': true,
          'simulated': true
        },
        {
          'sequence': 2,
          'timestamp_ms': 200,
          'latitude': 52.402,
          'longitude': -1.505,
          'navigation_mode': 'dead_reckoning',
          'speed_valid': true,
          'heading_valid': true,
          'uncertainty_valid': true,
          'simulated': true
        },
      ];

      final source = ReplaySource(interval: const Duration(milliseconds: 10));
      source.loadFromString(json.encode(rawSnapshots));

      final received = <NavSnapshot>[];
      final subscription = source.stream.listen((snap) {
        received.add(snap);
      });

      source.start();

      await Future.delayed(const Duration(milliseconds: 50));
      source.dispose();
      await subscription.cancel();

      expect(received.length, greaterThanOrEqualTo(2));
      expect(received[0].sequence, 1);
      expect(received[0].mode, IdrNavMode.gnssAided);
      expect(received[1].sequence, 2);
      expect(received[1].mode, IdrNavMode.deadReckoning);
    });
  });
}

