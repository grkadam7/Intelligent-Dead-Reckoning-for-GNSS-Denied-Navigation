// Bridges on-device IMU and GNSS into the Dart navigation engine.

import 'dart:async';
import 'package:geolocator/geolocator.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'live_sensor_source.dart';

class DeviceSensorFeed {
  DeviceSensorFeed(this._source);

  final LiveSensorSource _source;
  StreamSubscription<AccelerometerEvent>? _accelSub;
  StreamSubscription<GyroscopeEvent>? _gyroSub;
  StreamSubscription<Position>? _positionSub;

  // Latest gyro sample; accelerometer events drive the fusion step.
  double _gx = 0.0, _gy = 0.0, _gz = 0.0;

  Future<void> start() async {
    if (_accelSub != null) return;

    const period = Duration(milliseconds: 20); // ~50 Hz
    _accelSub = accelerometerEventStream(samplingPeriod: period).listen(
      (e) => _source.pushImuSample(ax: e.x, ay: e.y, az: e.z, gx: _gx, gy: _gy, gz: _gz),
    );
    _gyroSub = gyroscopeEventStream(samplingPeriod: period).listen((e) {
      _gx = e.x;
      _gy = e.y;
      _gz = e.z;
    });

    if (!await _ensureLocationAccess()) return;

    _positionSub = Geolocator.getPositionStream(
      locationSettings: AndroidSettings(
        accuracy: LocationAccuracy.best,
        distanceFilter: 0,
        intervalDuration: const Duration(seconds: 1),
      ),
    ).listen((p) {
      _source.pushGnssFix(
        latitude: p.latitude,
        longitude: p.longitude,
        altitude: p.altitude,
        accuracy: p.accuracy,
        speedMps: p.hasSpeed ? p.speed : 0.0,
        headingDeg: p.hasHeading ? p.heading : -1.0,
      );
    });
  }

  Future<bool> _ensureLocationAccess() async {
    if (!await Geolocator.isLocationServiceEnabled()) {
      _source.setStatus('Location services are disabled. Enable GPS for live navigation.');
      return false;
    }

    var permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
    }
    if (permission == LocationPermission.denied || permission == LocationPermission.deniedForever) {
      _source.setStatus('Location permission denied. Grant it to use live navigation.');
      return false;
    }
    return true;
  }

  void stop() {
    _accelSub?.cancel();
    _accelSub = null;
    _gyroSub?.cancel();
    _gyroSub = null;
    _positionSub?.cancel();
    _positionSub = null;
  }
}
