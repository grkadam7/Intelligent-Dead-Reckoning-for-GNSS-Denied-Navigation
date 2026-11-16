// DR-02: uncertainty grows monotonically through a blackout and falls on recovery.

import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/live_sensor_source.dart';
import 'package:idr_mobile/navigation/nav_snapshot.dart';
import 'fixtures/bus_outage_trace.dart';
import 'fixtures/manual_clock.dart';

void main() {
  test('uncertainty grows through a 20 s outage and shrinks after GNSS returns', () async {
    final clock = ManualClock(DateTime(2026, 1, 1));
    final source = LiveSensorSource(clock: clock.call);
    final snapshots = <NavSnapshot>[];
    final sub = source.stream.listen(snapshots.add);
    final trace = BusOutageTrace(source, clock);

    trace.initialFix();
    trace.driveSeconds(1);
    source.simulateGnssOutage(const Duration(seconds: 20));
    trace.outageSeconds(20);
    await Future<void>.delayed(Duration.zero);

    final dr = snapshots.where((s) => s.mode == IdrNavMode.deadReckoning).toList();
    expect(dr.length, greaterThan(10), reason: 'no dead-reckoning epochs emitted');
    for (var i = 1; i < dr.length; i++) {
      expect(
        dr[i].horizontalUncertaintyM,
        greaterThanOrEqualTo(dr[i - 1].horizontalUncertaintyM),
        reason: 'uncertainty decreased at epoch $i during the outage',
      );
    }
    final startU = dr.first.horizontalUncertaintyM;
    final endU = dr.last.horizontalUncertaintyM;
    expect(endU - startU, greaterThan(10.0),
        reason: 'uncertainty barely grew: ${startU.toStringAsFixed(1)} -> ${endU.toStringAsFixed(1)} m');

    trace.recoverSeconds(20);
    await Future<void>.delayed(Duration.zero);

    expect(snapshots.last.mode, IdrNavMode.gnssAided);
    expect(snapshots.last.horizontalUncertaintyM, lessThan(endU));

    source.dispose();
    await sub.cancel();
  });
}
