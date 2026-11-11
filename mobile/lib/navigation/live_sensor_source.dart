// Live On-Device Sensor Navigation Engine for IDR-INS.
// Fulfills real-world vehicle driving and GNSS-denied navigation.

import 'dart:async';
import 'dart:math' as math;
import 'nav_snapshot.dart';

class LiveSensorSource {
  final StreamController<NavSnapshot> _controller = StreamController<NavSnapshot>.broadcast();
  Timer? _ticker;
  bool _isRunning = false;
  int _sequence = 0;

  // Origin Geodetic Datum (initialized on first valid GPS fix)
  double? _originLat;
  double? _originLon;
  double _originAlt = 0.0;

  // Current Kinematic State in Local ENU (meters)
  double _eastM = 0.0;
  double _northM = 0.0;
  double _speedMps = 0.0;
  double _headingDeg = 0.0; // 0 = North, 90 = East
  double _horizontalUncertaintyM = 2.0;

  // Orientation Leveling (degrees)
  double _rollDeg = 0.0;
  double _pitchDeg = 0.0;
  CalibrationStage _calibrationStage = CalibrationStage.waitingStillness;

  // GNSS Health & Outage State
  IdrNavMode _mode = IdrNavMode.initializing;
  String _modeReason = "Waiting for initial GNSS fix...";
  DateTime? _lastGnssFixTime;
  double _lastGnssLat = 0.0;
  double _lastGnssLon = 0.0;
  double _lastGnssAccuracy = 999.0;

  // Reconvergence Filter Offset
  double _reconvOffsetEast = 0.0;
  double _reconvOffsetNorth = 0.0;
  DateTime? _reconvStartTime;
  static const double _reconvHalfLifeSec = 2.5;

  // Latest raw IMU measurements
  double _ax = 0.0, _ay = 0.0, _az = 9.80665;
  double _gx = 0.0, _gy = 0.0, _gz = 0.0;
  DateTime _lastImuTime = DateTime.now();

  Stream<NavSnapshot> get stream => _controller.stream;
  bool get isRunning => _isRunning;
  IdrNavMode get currentMode => _mode;

  void start({Duration interval = const Duration(milliseconds: 100)}) {
    _isRunning = true;
    _ticker?.cancel();
    _ticker = Timer.periodic(interval, (_) => _epochUpdate());
  }

  void stop() {
    _isRunning = false;
    _ticker?.cancel();
    _ticker = null;
  }

  void pushImuSample({
    required double ax,
    required double ay,
    required double az,
    required double gx,
    required double gy,
    required double gz,
  }) {
    final now = DateTime.now();
    final dt = now.difference(_lastImuTime).inMicroseconds / 1000000.0;
    _lastImuTime = now;

    _ax = ax;
    _ay = ay;
    _az = az;
    _gx = gx;
    _gy = gy;
    _gz = gz;

    if (dt > 0.0 && dt < 0.5) {
      _processInertialKinematics(dt);
    }
  }

  void pushGnssFix({
    required double latitude,
    required double longitude,
    required double altitude,
    required double accuracy,
    required double speedMps,
    required double headingDeg,
  }) {
    final now = DateTime.now();
    _lastGnssFixTime = now;
    _lastGnssLat = latitude;
    _lastGnssLon = longitude;
    _lastGnssAccuracy = accuracy;

    // Initialize local ENU origin on first GPS fix
    if (_originLat == null) {
      _originLat = latitude;
      _originLon = longitude;
      _originAlt = altitude;
      _eastM = 0.0;
      _northM = 0.0;
    }

    // Convert GPS fix to ENU
    final enu = _geodeticToEnu(latitude, longitude, altitude);
    final gpsEast = enu[0];
    final gpsNorth = enu[1];

    if (_mode == IdrNavMode.deadReckoning) {
      // Transition from Outage to Reconvergence
      _reconvOffsetEast = _eastM - gpsEast;
      _reconvOffsetNorth = _northM - gpsNorth;
      _reconvStartTime = now;
      _mode = IdrNavMode.recovering;
      _modeReason = "GNSS restored: Smooth reconvergence active";
    } else if (_mode != IdrNavMode.recovering) {
      _mode = IdrNavMode.gnssAided;
      _modeReason = "GNSS fix nominal (Accuracy: ±${accuracy.toStringAsFixed(1)}m)";
      _eastM = gpsEast;
      _northM = gpsNorth;
      _speedMps = speedMps;
      if (headingDeg >= 0) _headingDeg = headingDeg;
      _horizontalUncertaintyM = accuracy;
    }
  }

  void _processInertialKinematics(double dt) {
    // 1. Dynamic Gravity Leveling (Attitude Roll & Pitch)
    final normA = math.sqrt(_ax * _ax + _ay * _ay + _az * _az);
    if ((normA - 9.80665).abs() < 1.5) {
      _rollDeg = math.atan2(_ay, _az) * 180.0 / math.pi;
      _pitchDeg = math.atan2(-_ax, math.sqrt(_ay * _ay + _az * _az)) * 180.0 / math.pi;
      _calibrationStage = CalibrationStage.calibrated;
    }

    // 2. Zero Velocity Update (ZUPT) Stop Detection
    final gyroNorm = math.sqrt(_gx * _gx + _gy * _gy + _gz * _gz);
    if ((normA - 9.80665).abs() < 0.2 && gyroNorm < 0.03) {
      _speedMps = 0.0;
      if (_mode == IdrNavMode.deadReckoning) {
        _modeReason = "Vehicle stationary (ZUPT drift reset active)";
      }
      return;
    }

    // 3. Forward Speed & Heading Dead Reckoning during Outage
    if (_mode == IdrNavMode.deadReckoning) {
      // Integrate yaw rate
      _headingDeg = (_headingDeg + _gz * dt * 180.0 / math.pi) % 360.0;
      if (_headingDeg < 0) _headingDeg += 360.0;

      // Body-frame forward acceleration
      final pitchRad = _pitchDeg * math.pi / 180.0;
      final fwdAccel = _ax * math.cos(pitchRad) + _az * math.sin(pitchRad);
      _speedMps = math.max(0.0, _speedMps + fwdAccel * dt);

      // Non-Holonomic Constraints: propagate position strictly along forward vector
      final headingRad = _headingDeg * math.pi / 180.0;
      _eastM += _speedMps * math.sin(headingRad) * dt;
      _northM += _speedMps * math.cos(headingRad) * dt;

      // Covariance growth
      _horizontalUncertaintyM += 0.05 * dt;
    }
  }

  void _epochUpdate() {
    if (!_isRunning) return;
    _sequence++;
    final now = DateTime.now();

    // 1. Check GNSS Health & Detect Outages
    if (_lastGnssFixTime != null) {
      final elapsedSec = now.difference(_lastGnssFixTime!).inMicroseconds / 1000000.0;
      if (elapsedSec > 1.5 || _lastGnssAccuracy > 20.0) {
        if (_mode != IdrNavMode.deadReckoning) {
          _mode = IdrNavMode.deadReckoning;
          _modeReason = "GNSS blackout detected (${elapsedSec.toStringAsFixed(1)}s without fix): Dead Reckoning Active";
        }
      }
    }

    // 2. Apply Smooth Reconvergence Decay
    if (_mode == IdrNavMode.recovering && _reconvStartTime != null) {
      final tReconv = now.difference(_reconvStartTime!).inMicroseconds / 1000000.0;
      final decay = math.exp(-0.69315 / _reconvHalfLifeSec * tReconv);
      if (decay < 0.02) {
        _mode = IdrNavMode.gnssAided;
        _modeReason = "GNSS fix nominal (Reconvergence complete)";
        _reconvOffsetEast = 0.0;
        _reconvOffsetNorth = 0.0;
      } else {
        _eastM += _reconvOffsetEast * decay * 0.05;
        _northM += _reconvOffsetNorth * decay * 0.05;
      }
    }

    // 3. Compute Display Coordinates (WGS84)
    final latLonAlt = _enuToGeodetic(_eastM, _northM, 0.0);
    final dispLat = latLonAlt[0];
    final dispLon = latLonAlt[1];
    final dispAlt = latLonAlt[2];

    final snapshot = NavSnapshot(
      schemaVersion: "1.0.0",
      sequence: _sequence,
      source: NavSource.live,
      timestampMs: now.millisecondsSinceEpoch,
      stateAgeMs: 15,
      latitude: dispLat,
      longitude: dispLon,
      altitudeM: dispAlt,
      rawLatitude: dispLat,
      rawLongitude: dispLon,
      speedMps: _speedMps,
      speedKmh: _speedMps * 3.6,
      speedValid: true,
      headingDeg: _headingDeg,
      headingValid: true,
      mode: _mode,
      modeReason: _modeReason,
      horizontalUncertaintyM: _horizontalUncertaintyM,
      uncertaintyValid: true,
      isMatched: true,
      calibrationStage: _calibrationStage,
      rollDeg: _rollDeg,
      pitchDeg: _pitchDeg,
      yawDeg: _headingDeg,
      simulated: false,
    );

    _controller.add(snapshot);
  }

  // WGS84 Bowring Ellipsoid Math
  static const double _a = 6378137.0;
  static const double _f = 1.0 / 298.257223563;
  static const double _e2 = 2.0 * _f - _f * _f;

  List<double> _geodeticToEnu(double lat, double lon, double alt) {
    final lat0 = _originLat ?? lat;
    final lon0 = _originLon ?? lon;
    final alt0 = _originAlt;

    final latRad = lat * math.pi / 180.0;
    final lonRad = lon * math.pi / 180.0;
    final lat0Rad = lat0 * math.pi / 180.0;
    final lon0Rad = lon0 * math.pi / 180.0;

    final dLat = latRad - lat0Rad;
    final dLon = lonRad - lon0Rad;

    final rN = _a / math.sqrt(1.0 - _e2 * math.sin(lat0Rad) * math.sin(lat0Rad));
    final rM = _a * (1.0 - _e2) / math.pow(1.0 - _e2 * math.sin(lat0Rad) * math.sin(lat0Rad), 1.5);

    final east = (rN + alt0) * math.cos(lat0Rad) * dLon;
    final north = (rM + alt0) * dLat;
    return [east, north, alt - alt0];
  }

  List<double> _enuToGeodetic(double east, double north, double up) {
    final lat0 = _originLat ?? 52.40166;
    final lon0 = _originLon ?? -1.50529;
    final alt0 = _originAlt;

    final lat0Rad = lat0 * math.pi / 180.0;
    final rN = _a / math.sqrt(1.0 - _e2 * math.sin(lat0Rad) * math.sin(lat0Rad));
    final rM = _a * (1.0 - _e2) / math.pow(1.0 - _e2 * math.sin(lat0Rad) * math.sin(lat0Rad), 1.5);

    final dLat = north / (rM + alt0);
    final dLon = east / ((rN + alt0) * math.cos(lat0Rad));

    final lat = (lat0Rad + dLat) * 180.0 / math.pi;
    final lon = (lon0 * math.pi / 180.0 + dLon) * 180.0 / math.pi;
    return [lat, lon, alt0 + up];
  }

  void dispose() {
    stop();
    _controller.close();
  }
}

