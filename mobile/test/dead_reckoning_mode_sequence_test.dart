// DR-03: outage and restoration drive the observable mode sequence.

import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/live_sensor_source.dart';
import 'package:idr_mobile/navigation/nav_snapshot.dart';
import 'fixtures/bus_outage_trace.dart';
import 'fixtures/manual_clock.dart';

void main() {
  test('outage enters dead reckoning and recovery returns to GNSS aided', () async {
    final clock = ManualClock(DateTime(2026, 1, 1));
    final source = LiveSensorSource(clock: clock.call);
    final snapshots = <NavSnapshot>[];
    final sub = source.stream.listen(snapshots.add);
    final trace = BusOutageTrace(source, clock);

    trace.initialFix();
    trace.driveSeconds(1);
    await Future<void>.delayed(Duration.zero);
    expect(snapshots.last.mode, IdrNavMode.gnssAided);

    source.simulateGnssOutage(const Duration(seconds: 20));
    trace.outageSeconds(20);
    await Future<void>.delayed(Duration.zero);

    final dr = snapshots.where((s) => s.mode == IdrNavMode.deadReckoning).toList();
    expect(dr, isNotEmpty, reason: 'no dead-reckoning epochs');
    expect(dr.last.stateAgeMs, greaterThan(19000),
        reason: 'fix age should climb through the outage, got ${dr.last.stateAgeMs} ms');

    trace.recoverSeconds(20);
    await Future<void>.delayed(Duration.zero);

    final modes = snapshots.map((s) => s.mode).toList();
    final drIndex = modes.indexOf(IdrNavMode.deadReckoning);
    final recIndex = modes.indexOf(IdrNavMode.recovering, drIndex + 1);
    final restoredIndex = modes.indexOf(IdrNavMode.gnssAided, recIndex + 1);
    expect(drIndex, greaterThanOrEqualTo(0));
    expect(recIndex, greaterThan(drIndex), reason: 'no recovering state after the outage');
    expect(restoredIndex, greaterThan(recIndex), reason: 'never returned to GNSS aided');
    expect(snapshots.last.mode, IdrNavMode.gnssAided);
    expect(snapshots.last.stateAgeMs, lessThan(500),
        reason: 'fix age should reset once GNSS fixes resume');

    source.dispose();
    await sub.cancel();
  });
}
