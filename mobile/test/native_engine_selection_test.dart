// An unavailable native library must never block the Dart live engine.

import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/device_sensor_feed.dart';
import 'package:idr_mobile/navigation/live_sensor_source.dart';
import 'package:idr_mobile/navigation/native_source.dart';

void main() {
  test('unavailable native source falls back without emitting or throwing', () async {
    final native = NativeNavigationSource(libraryPath: 'definitely_not_a_library.so');
    expect(native.isAvailable, isFalse);
    expect(native.engineStatus, contains('unavailable'));

    final events = <dynamic>[];
    final sub = native.stream.listen(events.add);
    native.start(const Duration(milliseconds: 10));
    await Future<void>.delayed(const Duration(milliseconds: 50));
    expect(events, isEmpty, reason: 'unavailable native engine must not publish');
    native.stop();

    final dart = LiveSensorSource();
    final feed = DeviceSensorFeed(dart, nativeSource: native);
    expect(feed.nativeSource, same(native));

    // These must be safe no-ops when the library failed to load.
    native.pushImu(0.1, 0.0, 0.0, 9.80665, 0.0, 0.0, 0.0);
    native.pushGnss(0.1, 15.36, 75.12, 700.0, 13.9, 90.0, 2.5);

    native.dispose();
    await sub.cancel();
  });
}
