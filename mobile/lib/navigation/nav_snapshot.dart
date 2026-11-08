// Core typed navigation snapshot contract for IDR-INS.
// Fulfills MOBILE-01, MOBILE-02, and MOBILE-03 state delivery.

enum NavigationMode {
  initializing,
  gnssAided,
  deadReckoning,
  recovering,
  stale,
  suspended,
  failed,
}

enum CalibrationStage {
  waitingStillness,
  leveling,
  waitingStraightMovement,
  aligningYaw,
  calibrated,
  mountChanged,
  failed,
}

enum NavSource {
  replay,
  live,
}

class NavSnapshot {
  final String schemaVersion;
  final int sequence;
  final NavSource source;
  final int timestampMs;
  final int stateAgeMs;

  // WGS84 Display Coordinates
  final double latitude;
  final double longitude;
  final double altitudeM;

  // Raw Dead-Reckoning Coordinates (before map snapping)
  final double rawLatitude;
  final double rawLongitude;

  // Kinematics
  final double speedMps;
  final double speedKmh;
  final bool speedValid;
  final double headingDeg; // 0 = North, 90 = East
  final bool headingValid;

  // State estimation & Health
  final NavigationMode mode;
  final String modeReason;
  final double horizontalUncertaintyM; // 95% 2-sigma radius
  final bool uncertaintyValid;

  // Map Matching State
  final bool isMatched;
  final String? matchedLinkId;

  // Vehicle Phone Calibration
  final CalibrationStage calibrationStage;
  final double rollDeg;
  final double pitchDeg;
  final double yawDeg;

  // Safety & Provenance
  final bool simulated;

  const NavSnapshot({
    required this.schemaVersion,
    required this.sequence,
    required this.source,
    required this.timestampMs,
    required this.stateAgeMs,
    required this.latitude,
    required this.longitude,
    required this.altitudeM,
    required this.rawLatitude,
    required this.rawLongitude,
    required this.speedMps,
    required this.speedKmh,
    required this.speedValid,
    required this.headingDeg,
    required this.headingValid,
    required this.mode,
    required this.modeReason,
    required this.horizontalUncertaintyM,
    required this.uncertaintyValid,
    required this.isMatched,
    this.matchedLinkId,
    required this.calibrationStage,
    required this.rollDeg,
    required this.pitchDeg,
    required this.yawDeg,
    required this.simulated,
  });

  factory NavSnapshot.fromJson(Map<String, dynamic> json) {
    NavigationMode parseMode(String? m) {
      switch (m) {
        case 'gnss_aided':
          return NavigationMode.gnssAided;
        case 'dead_reckoning':
          return NavigationMode.deadReckoning;
        case 'recovering':
          return NavigationMode.recovering;
        case 'stale':
          return NavigationMode.stale;
        case 'suspended':
          return NavigationMode.suspended;
        case 'failed':
          return NavigationMode.failed;
        default:
          return NavigationMode.initializing;
      }
    }

    CalibrationStage parseCalib(String? c) {
      switch (c) {
        case 'waiting_stillness':
          return CalibrationStage.waitingStillness;
        case 'leveling':
          return CalibrationStage.leveling;
        case 'waiting_straight_movement':
          return CalibrationStage.waitingStraightMovement;
        case 'aligning_yaw':
          return CalibrationStage.aligningYaw;
        case 'calibrated':
          return CalibrationStage.calibrated;
        case 'mount_changed':
          return CalibrationStage.mountChanged;
        default:
          return CalibrationStage.failed;
      }
    }

    return NavSnapshot(
      schemaVersion: json['schema_version'] as String? ?? '1.0.0',
      sequence: json['sequence'] as int? ?? 0,
      source: json['source'] == 'live' ? NavSource.live : NavSource.replay,
      timestampMs: json['timestamp_ms'] as int? ?? 0,
      stateAgeMs: json['state_age_ms'] as int? ?? 0,
      latitude: (json['latitude'] as num?)?.toDouble() ?? 0.0,
      longitude: (json['longitude'] as num?)?.toDouble() ?? 0.0,
      altitudeM: (json['altitude_m'] as num?)?.toDouble() ?? 0.0,
      rawLatitude: (json['raw_latitude'] as num?)?.toDouble() ?? 0.0,
      rawLongitude: (json['raw_longitude'] as num?)?.toDouble() ?? 0.0,
      speedMps: (json['speed_mps'] as num?)?.toDouble() ?? 0.0,
      speedKmh: (json['speed_kmh'] as num?)?.toDouble() ?? 0.0,
      speedValid: json['speed_valid'] as bool? ?? false,
      headingDeg: (json['heading_deg'] as num?)?.toDouble() ?? 0.0,
      headingValid: json['heading_valid'] as bool? ?? false,
      mode: parseMode(json['navigation_mode'] as String?),
      modeReason: json['mode_reason'] as String? ?? '',
      horizontalUncertaintyM: (json['horizontal_uncertainty_m'] as num?)?.toDouble() ?? 0.0,
      uncertaintyValid: json['uncertainty_valid'] as bool? ?? false,
      isMatched: json['is_matched'] as bool? ?? false,
      matchedLinkId: json['matched_link_id'] as String?,
      calibrationStage: parseCalib(json['calibration_stage'] as String?),
      rollDeg: (json['roll_deg'] as num?)?.toDouble() ?? 0.0,
      pitchDeg: (json['pitch_deg'] as num?)?.toDouble() ?? 0.0,
      yawDeg: (json['yaw_deg'] as num?)?.toDouble() ?? 0.0,
      simulated: json['simulated'] as bool? ?? true,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'schema_version': schemaVersion,
      'sequence': sequence,
      'source': source == NavSource.live ? 'live' : 'replay',
      'timestamp_ms': timestampMs,
      'state_age_ms': stateAgeMs,
      'latitude': latitude,
      'longitude': longitude,
      'altitude_m': altitudeM,
      'raw_latitude': rawLatitude,
      'raw_longitude': rawLongitude,
      'speed_mps': speedMps,
      'speed_kmh': speedKmh,
      'speed_valid': speedValid,
      'heading_deg': headingDeg,
      'heading_valid': headingValid,
      'navigation_mode': mode.name,
      'mode_reason': modeReason,
      'horizontal_uncertainty_m': horizontalUncertaintyM,
      'uncertainty_valid': uncertaintyValid,
      'is_matched': isMatched,
      'matched_link_id': matchedLinkId,
      'calibration_stage': calibrationStage.name,
      'roll_deg': rollDeg,
      'pitch_deg': pitchDeg,
      'yaw_deg': yawDeg,
      'simulated': simulated,
    };
  }
}

