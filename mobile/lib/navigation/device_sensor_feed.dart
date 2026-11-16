// Bridges on-device IMU and GNSS into the Dart and native navigation engines.

import 'dart:async';
import 'package:geolocator/geolocator.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'live_sensor_source.dart';
import 'native_source.dart';

class DeviceSensorFeed {
  DeviceSensorFeed(this._source, {this.nativeSource});

  final LiveSensorSource _source;
  final NativeNavigationSource? nativeSource;
  StreamSubscription<AccelerometerEvent>? _accelSub;
  StreamSubscription<GyroscopeEvent>? _gyroSub;
  StreamSubscription<Position>? _positionSub;
  Timer? _gapTimer;
  final Stopwatch _uptime = Stopwatch()..start();

  // Latest gyro sample; accelerometer events drive the fusion step.
  double _gx = 0.0, _gy = 0.0, _gz = 0.0;
  DateTime? _lastFixWallTime;
  double _lastLat = 0.0, _lastLon = 0.0, _lastAlt = 0.0;
  double _lastSpeed = 0.0, _lastHeading = -1.0, _lastAccuracy = 999.0;

  double get _timestampS => _uptime.elapsedMicroseconds / 1000000.0;

  Future<void> start() async {
    if (_accelSub != null) return;

    const period = Duration(milliseconds: 20); // ~50 Hz
    _accelSub = accelerometerEventStream(samplingPeriod: period).listen((e) {
      _source.pushImuSample(ax: e.x, ay: e.y, az: e.z, gx: _gx, gy: _gy, gz: _gz);
      nativeSource?.pushImu(_timestampS, e.x, e.y, e.z, _gx, _gy, _gz);
    });
    _gyroSub = gyroscopeEventStream(samplingPeriod: period).listen((e) {
      _gx = e.x;
      _gy = e.y;
      _gz = e.z;
    });

    if (nativeSource != null) {
      _gapTimer = Timer.periodic(const Duration(milliseconds: 500), (_) {
        final last = _lastFixWallTime;
        if (last == null) return;
        if (DateTime.now().difference(last).inMilliseconds > 1500) {
          nativeSource?.pushGnss(
            _timestampS, _lastLat, _lastLon, _lastAlt,
            _lastSpeed, _lastHeading, _lastAccuracy,
            isValid: false,
          );
        }
      });
    }

    if (!await _ensureLocationAccess()) return;

    _positionSub = Geolocator.getPositionStream(
      locationSettings: AndroidSettings(
        accuracy: LocationAccuracy.best,
        distanceFilter: 0,
        intervalDuration: const Duration(seconds: 1),
      ),
    ).listen((p) {
      final speed = p.hasSpeed ? p.speed : 0.0;
      final heading = p.hasHeading ? p.heading : -1.0;
      _lastFixWallTime = DateTime.now();
      _lastLat = p.latitude;
      _lastLon = p.longitude;
      _lastAlt = p.altitude;
      _lastSpeed = speed;
      _lastHeading = heading;
      _lastAccuracy = p.accuracy;

      _source.pushGnssFix(
        latitude: p.latitude,
        longitude: p.longitude,
        altitude: p.altitude,
        accuracy: p.accuracy,
        speedMps: speed,
        headingDeg: heading,
      );
      nativeSource?.pushGnss(
        _timestampS, p.latitude, p.longitude, p.altitude,
        speed, heading, p.accuracy,
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
    _gapTimer?.cancel();
    _gapTimer = null;
  }
}
