import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/nav_snapshot.dart';
import 'package:idr_mobile/navigation/native_source.dart';

void main() {
  group('NativeNavigationSource unit tests', () {
    test('Gracefully handles unlinked dynamic library on host', () {
      final source = NativeNavigationSource(libraryPath: 'non_existent_idr_mobile.dll');
      expect(source.isAvailable, isFalse);

      // Verify safe fallback and non-crashing behavior
      source.start(const Duration(milliseconds: 100));
      source.step();
      source.reset();
      source.dispose();
    });

    test('Maps live snapshot fields to Dart NavSnapshot', () {
      const snap = NavSnapshot(
        schemaVersion: '1.0.0',
        sequence: 1,
        source: NavSource.live,
        timestampMs: 1000,
        stateAgeMs: 12,
        latitude: 52.40166,
        longitude: -1.50529,
        altitudeM: 147.5,
        rawLatitude: 52.40166,
        rawLongitude: -1.50529,
        speedMps: 15.0,
        speedKmh: 54.0,
        speedValid: true,
        headingDeg: 90.0,
        headingValid: true,
        mode: NavigationMode.gnssAided,
        modeReason: 'Live sensor fusion active',
        horizontalUncertaintyM: 1.5,
        uncertaintyValid: true,
        isMatched: true,
        calibrationStage: CalibrationStage.calibrated,
        rollDeg: 0.5,
        pitchDeg: -0.2,
        yawDeg: 90.0,
        simulated: false,
      );

      expect(snap.source, NavSource.live);
      expect(snap.simulated, isFalse);
      expect(snap.speedKmh, 54.0);
    });
  });
}
