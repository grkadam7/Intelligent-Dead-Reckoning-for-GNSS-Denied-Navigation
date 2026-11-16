// DR-01: a bus entering a GNSS blackout at 50 km/h keeps its speed and keeps moving.

import 'dart:math' as math;
import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/live_sensor_source.dart';
import 'package:idr_mobile/navigation/nav_snapshot.dart';
import 'fixtures/bus_outage_trace.dart';
import 'fixtures/manual_clock.dart';

void main() {
  test('speed and position survive a 20 s outage at constant speed', () async {
    final clock = ManualClock(DateTime(2026, 1, 1));
    final source = LiveSensorSource(clock: clock.call);
    final snapshots = <NavSnapshot>[];
    final sub = source.stream.listen(snapshots.add);
    final trace = BusOutageTrace(source, clock);

    trace.initialFix();
    trace.driveSeconds(1);
    source.simulateGnssOutage(const Duration(seconds: 20));
    trace.outageSeconds(20);
    // Broadcast streams deliver on a microtask; let pending events land.
    await Future<void>.delayed(Duration.zero);

    final dr = snapshots.where((s) => s.mode == IdrNavMode.deadReckoning).toList();
    expect(dr, isNotEmpty, reason: 'outage never switched to dead reckoning');

    expect(
      dr.first.speedMps,
      greaterThan(BusOutageTrace.speedMps * 0.9),
      reason: 'speed collapsed at outage entry to ${dr.first.speedMps.toStringAsFixed(2)} m/s',
    );

    final dLat = dr.last.latitude - dr.first.latitude;
    final dLon = dr.last.longitude - dr.first.longitude;
    final metres = math.sqrt(
      math.pow(dLat * 111540.0, 2) +
          math.pow(dLon * 111320.0 * math.cos(15.3647 * math.pi / 180.0), 2),
    );
    expect(metres, greaterThan(50), reason: 'position barely advanced: $metres m');

    source.dispose();
    await sub.cancel();
  });
}
