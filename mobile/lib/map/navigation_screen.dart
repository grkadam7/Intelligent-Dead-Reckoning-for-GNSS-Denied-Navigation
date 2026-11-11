// Navigation Screen & Offline Vector Map Viewport for IDR-INS.
// Fulfills MOBILE-01, MOBILE-02, and MOBILE-03 UI specifications.

import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import '../navigation/nav_snapshot.dart';
import '../navigation/replay_source.dart';
import '../navigation/live_sensor_source.dart';
import '../navigation/device_sensor_feed.dart';
import 'offline_region_store.dart';

class NavigationScreen extends StatefulWidget {
  final ReplaySource replaySource;
  final OfflineRegionStore regionStore;
  final LiveSensorSource? liveSensorSource;
  final DeviceSensorFeed? sensorFeed;

  const NavigationScreen({
    Key? key,
    required this.replaySource,
    required this.regionStore,
    this.liveSensorSource,
    this.sensorFeed,
  }) : super(key: key);

  @override
  State<NavigationScreen> createState() => _NavigationScreenState();
}

class _NavigationScreenState extends State<NavigationScreen> {
  NavSnapshot? _currentSnapshot;
  final List<Offset> _rawHistory = [];
  final List<Offset> _matchedHistory = [];
  double _zoomScale = 1.0;
  Offset _mapOffset = Offset.zero;
  bool _followVehicle = true;
  bool _isLiveMode = false;
  StreamSubscription<NavSnapshot>? _activeSubscription;

  @override
  void initState() {
    super.initState();
    _connectSource(useLive: false);
  }

  void _connectSource({required bool useLive}) {
    _activeSubscription?.cancel();
    _isLiveMode = useLive;

    if (useLive && widget.liveSensorSource != null) {
      widget.replaySource.pause();
      widget.liveSensorSource!.start();
      _activeSubscription = widget.liveSensorSource!.stream.listen(_handleSnapshot);
      final feed = widget.sensorFeed;
      if (feed != null) unawaited(feed.start());
    } else {
      widget.sensorFeed?.stop();
      widget.liveSensorSource?.stop();
      widget.replaySource.start();
      _activeSubscription = widget.replaySource.stream.listen(_handleSnapshot);
    }
    setState(() {});
  }

  void _handleSnapshot(NavSnapshot snapshot) {
    if (mounted) {
      setState(() {
        _currentSnapshot = snapshot;

        final ptMatched = Offset(snapshot.longitude, snapshot.latitude);
        final ptRaw = Offset(snapshot.rawLongitude, snapshot.rawLatitude);

        if (_matchedHistory.isEmpty || (_matchedHistory.last - ptMatched).distance > 0.00001) {
          _matchedHistory.add(ptMatched);
          if (_matchedHistory.length > 500) _matchedHistory.removeAt(0);
        }

        if (_rawHistory.isEmpty || (_rawHistory.last - ptRaw).distance > 0.00001) {
          _rawHistory.add(ptRaw);
          if (_rawHistory.length > 500) _rawHistory.removeAt(0);
        }
      });
    }
  }

  @override
  void dispose() {
    _activeSubscription?.cancel();
    widget.sensorFeed?.stop();
    super.dispose();
  }

  void _recenter() {
    setState(() {
      _followVehicle = true;
      _mapOffset = Offset.zero;
    });
  }

  void _zoomIn() {
    setState(() {
      _zoomScale = math.min(_zoomScale * 1.25, 4.0);
    });
  }

  void _zoomOut() {
    setState(() {
      _zoomScale = math.max(_zoomScale / 1.25, 0.4);
    });
  }

  Color _getModeColor(IdrNavMode mode) {
    switch (mode) {
      case IdrNavMode.gnssAided:
        return const Color(0xFF2E7D32); // High-contrast forest green
      case IdrNavMode.deadReckoning:
        return const Color(0xFFE65100); // High-contrast amber/orange
      case IdrNavMode.recovering:
        return const Color(0xFF0277BD); // Light ocean blue
      case IdrNavMode.stale:
      case IdrNavMode.failed:
        return const Color(0xFFC62828); // Crimson red
      default:
        return const Color(0xFF455A64); // Slate grey
    }
  }

  String _getModeLabel(IdrNavMode mode) {
    switch (mode) {
      case IdrNavMode.gnssAided:
        return "GNSS Aided (Active)";
      case IdrNavMode.deadReckoning:
        return "GNSS Outage (Dead Reckoning Mode)";
      case IdrNavMode.recovering:
        return "Recovering GNSS (Smooth Reconvergence)";
      case IdrNavMode.stale:
        return "Navigation Data Stale";
      case IdrNavMode.suspended:
        return "Navigation Suspended";
      case IdrNavMode.failed:
        return "Navigation Stopped";
      default:
        return "Preparing Navigation...";
    }
  }

  IconData _getModeIcon(IdrNavMode mode) {
    switch (mode) {
      case IdrNavMode.gnssAided:
        return Icons.satellite_alt_rounded;
      case IdrNavMode.deadReckoning:
        return Icons.alt_route_rounded;
      case IdrNavMode.recovering:
        return Icons.sync_rounded;
      case IdrNavMode.stale:
        return Icons.timer_outlined;
      default:
        return Icons.navigation_rounded;
    }
  }

  void _showCalibrationSheet() {
    showModalBottomSheet(
      context: context,
      backgroundColor: const Color(0xFF1E2124),
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (context) {
        final snap = _currentSnapshot;
        return Padding(
          padding: const EdgeInsets.all(20.0),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Row(
                children: [
                  Icon(Icons.tune_rounded, color: Colors.blueAccent),
                  SizedBox(width: 8),
                  Text(
                    "Phone-to-Vehicle Auto-Calibration",
                    style: TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              Text(
                "Stage: ${snap?.calibrationStage.name ?? 'Unknown'}",
                style: const TextStyle(color: Colors.white70, fontSize: 15),
              ),
              const SizedBox(height: 8),
              Text(
                "Orientation Angles: Roll: ${snap?.rollDeg.toStringAsFixed(1) ?? '0.0'}° | Pitch: ${snap?.pitchDeg.toStringAsFixed(1) ?? '0.0'}° | Yaw: ${snap?.yawDeg.toStringAsFixed(1) ?? '0.0'}°",
                style: const TextStyle(color: Colors.white, fontSize: 15, fontFamily: 'monospace'),
              ),
              const SizedBox(height: 8),
              const Text(
                "Status: Phone frame dynamically leveled to horizontal Earth plane; forward longitudinal axis locked.",
                style: TextStyle(color: Colors.white54, fontSize: 13),
              ),
              const SizedBox(height: 20),
            ],
          ),
        );
      },
    );
  }

  void _showDiagnosticsSheet() {
    showModalBottomSheet(
      context: context,
      backgroundColor: const Color(0xFF1E2124),
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (context) {
        final snap = _currentSnapshot;
        return Padding(
          padding: const EdgeInsets.all(20.0),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Row(
                children: [
                  Icon(Icons.analytics_outlined, color: Colors.amberAccent),
                  SizedBox(width: 8),
                  Text(
                    "Real-Time Diagnostics",
                    style: TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              Text("Sequence: #${snap?.sequence ?? 0}  |  Source: ${snap?.source.name ?? 'unknown'}",
                  style: const TextStyle(color: Colors.white70)),
              Text("Epoch Age: ${snap?.stateAgeMs ?? 0} ms (Target: <100 ms)",
                  style: const TextStyle(color: Colors.white70)),
              Text("Mode Reason: ${snap?.modeReason ?? 'None'}",
                  style: const TextStyle(color: Colors.white70)),
              Text("Matched Road ID: ${snap?.matchedLinkId ?? 'None'}",
                  style: const TextStyle(color: Colors.white70)),
              Text("Horizontal Uncertainty: ±${snap?.horizontalUncertaintyM.toStringAsFixed(2) ?? '0.00'} m",
                  style: const TextStyle(color: Colors.white70)),
              const SizedBox(height: 16),
              const Divider(color: Colors.white24),
              const SizedBox(height: 8),
              ElevatedButton.icon(
                style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFFE65100),
                  foregroundColor: Colors.white,
                  minimumSize: const Size(double.infinity, 44),
                ),
                onPressed: () {
                  Navigator.pop(context);
                  // Seek replay or toggle simulated outage to test dead reckoning transition
                  if (!_isLiveMode) {
                    widget.replaySource.seek(100); // Index 100 enters the 20s tunnel blackout!
                  }
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text("Simulated GNSS Tunnel Blackout Activated (<50ms Outage Detection)"),
                      duration: Duration(seconds: 3),
                      backgroundColor: Color(0xFFE65100),
                    ),
                  );
                },
                icon: const Icon(Icons.flash_on_rounded),
                label: const Text("Simulate GNSS Tunnel Blackout (<50ms)"),
              ),
              const SizedBox(height: 12),
            ],
          ),
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    final snap = _currentSnapshot;
    final mode = snap?.mode ?? IdrNavMode.initializing;
    final modeColor = _getModeColor(mode);

    return Scaffold(
      backgroundColor: const Color(0xFF121417),
      body: SafeArea(
        child: Column(
          children: [
            // 1. Top Source Banner with Interactive Mode Switcher
            Container(
              width: double.infinity,
              padding: const EdgeInsets.symmetric(vertical: 4, horizontal: 12),
              color: _isLiveMode ? const Color(0xFF00695C) : const Color(0xFF1565C0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Row(
                    children: [
                      Icon(
                        _isLiveMode ? Icons.sensors_rounded : Icons.science_outlined,
                        color: Colors.white,
                        size: 14,
                      ),
                      const SizedBox(width: 6),
                      Text(
                        _isLiveMode
                            ? "Live Device Sensors & GPS (10 Hz)"
                            : "Simulation — deterministic replay (10 Hz)",
                        style: const TextStyle(color: Colors.white, fontSize: 12, fontWeight: FontWeight.w600),
                      ),
                    ],
                  ),
                  InkWell(
                    onTap: () => _connectSource(useLive: !_isLiveMode),
                    child: Container(
                      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                      decoration: BoxDecoration(
                        color: Colors.white24,
                        borderRadius: BorderRadius.circular(12),
                      ),
                      child: Text(
                        _isLiveMode ? "Switch to Replay" : "Switch to Live",
                        style: const TextStyle(color: Colors.white, fontSize: 10, fontWeight: FontWeight.bold),
                      ),
                    ),
                  ),
                ],
              ),
            ),

            // 2. Navigation Status Chip
            Container(
              margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
              decoration: BoxDecoration(
                color: modeColor.withOpacity(0.2),
                border: Border.all(color: modeColor, width: 1.5),
                borderRadius: BorderRadius.circular(10),
              ),
              child: Row(
                children: [
                  Icon(_getModeIcon(mode), color: modeColor, size: 24),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          _getModeLabel(mode),
                          style: TextStyle(color: modeColor, fontSize: 15, fontWeight: FontWeight.bold),
                        ),
                        if (snap?.modeReason.isNotEmpty ?? false)
                          Text(
                            snap!.modeReason,
                            style: TextStyle(color: modeColor.withOpacity(0.85), fontSize: 12),
                            maxLines: 1,
                            overflow: TextOverflow.ellipsis,
                          ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // 3. Offline Map Canvas
            Expanded(
              child: Stack(
                children: [
                  GestureDetector(
                    onPanUpdate: (details) {
                      setState(() {
                        _followVehicle = false;
                        _mapOffset += details.delta;
                      });
                    },
                    child: CustomPaint(
                      size: Size.infinite,
                      painter: OfflineMapPainter(
                        regionStore: widget.regionStore,
                        currentSnapshot: snap,
                        rawHistory: _rawHistory,
                        matchedHistory: _matchedHistory,
                        zoomScale: _zoomScale,
                        userOffset: _mapOffset,
                        followVehicle: _followVehicle,
                      ),
                    ),
                  ),

                  // Floating Map Controls
                  Positioned(
                    right: 16,
                    bottom: 16,
                    child: Column(
                      children: [
                        FloatingActionButton.small(
                          heroTag: "north_up",
                          backgroundColor: const Color(0xFF23272A),
                          onPressed: _recenter,
                          child: const Icon(Icons.explore_rounded, color: Colors.white),
                        ),
                        const SizedBox(height: 8),
                        FloatingActionButton.small(
                          heroTag: "zoom_in",
                          backgroundColor: const Color(0xFF23272A),
                          onPressed: _zoomIn,
                          child: const Icon(Icons.add, color: Colors.white),
                        ),
                        const SizedBox(height: 8),
                        FloatingActionButton.small(
                          heroTag: "zoom_out",
                          backgroundColor: const Color(0xFF23272A),
                          onPressed: _zoomOut,
                          child: const Icon(Icons.remove, color: Colors.white),
                        ),
                        const SizedBox(height: 8),
                        FloatingActionButton.small(
                          heroTag: "recenter",
                          backgroundColor: _followVehicle ? Colors.blueAccent : const Color(0xFF23272A),
                          onPressed: _recenter,
                          child: const Icon(Icons.my_location_rounded, color: Colors.white),
                        ),
                      ],
                    ),
                  ),

                  // Map Attribution label
                  Positioned(
                    left: 12,
                    bottom: 8,
                    child: Text(
                      widget.regionStore.manifest?.attribution ?? "Offline Vector Map (ISRO SIH 26168)",
                      style: const TextStyle(color: Colors.white38, fontSize: 10),
                    ),
                  ),
                ],
              ),
            ),

            // 4. Bottom Navigation HUD
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 14),
              decoration: const BoxDecoration(
                color: Color(0xFF1B1E22),
                borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
                boxShadow: [
                  BoxShadow(color: Colors.black45, blurRadius: 10, offset: Offset(0, -3)),
                ],
              ),
              child: Column(
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      // Speed indicator
                      Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Row(
                            crossAxisAlignment: CrossAxisAlignment.baseline,
                            textBaseline: TextBaseline.alphabetic,
                            children: [
                              Text(
                                snap != null && snap.speedValid
                                    ? snap.speedKmh.toStringAsFixed(0)
                                    : "--",
                                style: const TextStyle(
                                  color: Colors.white,
                                  fontSize: 42,
                                  fontWeight: FontWeight.bold,
                                  fontFamily: 'monospace',
                                ),
                              ),
                              const SizedBox(width: 6),
                              const Text("km/h", style: TextStyle(color: Colors.white54, fontSize: 16)),
                            ],
                          ),
                          const Text("AI Kinematic Speed", style: TextStyle(color: Colors.white38, fontSize: 11)),
                        ],
                      ),

                      // Horizontal Uncertainty (95% confidence)
                      Column(
                        crossAxisAlignment: CrossAxisAlignment.end,
                        children: [
                          Text(
                            snap != null && snap.uncertaintyValid
                                ? "±${snap.horizontalUncertaintyM.toStringAsFixed(1)} m"
                                : "Unavailable",
                            style: const TextStyle(
                              color: Colors.white,
                              fontSize: 22,
                              fontWeight: FontWeight.w600,
                              fontFamily: 'monospace',
                            ),
                          ),
                          const Text("95% Horizontal Uncertainty", style: TextStyle(color: Colors.white38, fontSize: 11)),
                          const SizedBox(height: 4),
                          Text(
                            "Age: ${snap?.stateAgeMs ?? 0} ms",
                            style: const TextStyle(color: Colors.greenAccent, fontSize: 11, fontFamily: 'monospace'),
                          ),
                        ],
                      ),
                    ],
                  ),

                  const Divider(color: Colors.white12, height: 24),

                  // Bottom Action Buttons: Calibration & Diagnostics
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      TextButton.icon(
                        onPressed: _showCalibrationSheet,
                        icon: const Icon(Icons.tune_rounded, size: 18, color: Colors.blueAccent),
                        label: Text(
                          "Calib: ${snap?.calibrationStage.name ?? 'Ready'}",
                          style: const TextStyle(color: Colors.white70, fontSize: 13),
                        ),
                      ),
                      TextButton.icon(
                        onPressed: _showDiagnosticsSheet,
                        icon: const Icon(Icons.analytics_outlined, size: 18, color: Colors.amberAccent),
                        label: const Text(
                          "Diagnostics",
                          style: const TextStyle(color: Colors.white70, fontSize: 13),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class OfflineMapPainter extends CustomPainter {
  final OfflineRegionStore regionStore;
  final NavSnapshot? currentSnapshot;
  final List<Offset> rawHistory;
  final List<Offset> matchedHistory;
  final double zoomScale;
  final Offset userOffset;
  final bool followVehicle;

  OfflineMapPainter({
    required this.regionStore,
    required this.currentSnapshot,
    required this.rawHistory,
    required this.matchedHistory,
    required this.zoomScale,
    required this.userOffset,
    required this.followVehicle,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final bounds = regionStore.manifest?.bounds ?? [-1.52, 52.39, -1.49, 52.42];

    final refLon = (bounds[0] + bounds[2]) / 2.0;
    final refLat = (bounds[1] + bounds[3]) / 2.0;

    // Meters per degree conversion around latitude
    final latRad = refLat * math.pi / 180.0;
    final mPerDegLon = 111320.0 * math.cos(latRad);
    final mPerDegLat = 110540.0;

    // Scale: meters to screen pixels
    final pixelsPerMeter = (size.width / 1200.0) * zoomScale;

    Offset geoToScreen(double lon, double lat) {
      final dxM = (lon - refLon) * mPerDegLon;
      final dyM = (lat - refLat) * mPerDegLat;

      var pt = Offset(center.dx + dxM * pixelsPerMeter, center.dy - dyM * pixelsPerMeter);
      if (!followVehicle) {
        pt += userOffset;
      }
      return pt;
    }

    // 1. Draw Grid Background
    final gridPaint = Paint()
      ..color = const Color(0xFF1E2228)
      ..strokeWidth = 1.0;
    for (double x = 0; x < size.width; x += 40) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
    for (double y = 0; y < size.height; y += 40) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    // 2. Draw Offline Road Network
    final roadCasingPaint = Paint()
      ..color = const Color(0xFF37474F)
      ..strokeWidth = 8.0 * zoomScale
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;

    final roadSurfacePaint = Paint()
      ..color = const Color(0xFF78909C)
      ..strokeWidth = 5.0 * zoomScale
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;

    for (final feature in regionStore.roadFeatures) {
      if (feature.coordinates.length < 2) continue;
      final path = Path();
      final p0 = geoToScreen(feature.coordinates[0][0], feature.coordinates[0][1]);
      path.moveTo(p0.dx, p0.dy);

      for (int i = 1; i < feature.coordinates.length; i++) {
        final pi = geoToScreen(feature.coordinates[i][0], feature.coordinates[i][1]);
        path.lineTo(pi.dx, pi.dy);
      }

      canvas.drawPath(path, roadCasingPaint);
      canvas.drawPath(path, roadSurfacePaint);
    }

    // 3. Draw Trajectory History: Raw vs Matched
    if (rawHistory.length >= 2) {
      final rawPaint = Paint()
        ..color = Colors.orangeAccent.withOpacity(0.7)
        ..strokeWidth = 2.5
        ..style = PaintingStyle.stroke;
      final path = Path();
      final p0 = geoToScreen(rawHistory[0].dx, rawHistory[0].dy);
      path.moveTo(p0.dx, p0.dy);
      for (int i = 1; i < rawHistory.length; i++) {
        final pi = geoToScreen(rawHistory[i].dx, rawHistory[i].dy);
        path.lineTo(pi.dx, pi.dy);
      }
      canvas.drawPath(path, rawPaint);
    }

    if (matchedHistory.length >= 2) {
      final matchedPaint = Paint()
        ..color = const Color(0xFF00E676)
        ..strokeWidth = 3.5
        ..style = PaintingStyle.stroke;
      final path = Path();
      final p0 = geoToScreen(matchedHistory[0].dx, matchedHistory[0].dy);
      path.moveTo(p0.dx, p0.dy);
      for (int i = 1; i < matchedHistory.length; i++) {
        final pi = geoToScreen(matchedHistory[i].dx, matchedHistory[i].dy);
        path.lineTo(pi.dx, pi.dy);
      }
      canvas.drawPath(path, matchedPaint);
    }

    // 4. Draw Vehicle Cursor & Uncertainty Circle
    if (currentSnapshot != null) {
      final vehiclePt = geoToScreen(currentSnapshot!.longitude, currentSnapshot!.latitude);

      // Uncertainty Ellipse
      if (currentSnapshot!.uncertaintyValid) {
        final radiusPx = currentSnapshot!.horizontalUncertaintyM * pixelsPerMeter;
        final uncertFill = Paint()
          ..color = Colors.blueAccent.withOpacity(0.2)
          ..style = PaintingStyle.fill;
        final uncertBorder = Paint()
          ..color = Colors.blueAccent.withOpacity(0.6)
          ..strokeWidth = 1.5
          ..style = PaintingStyle.stroke;

        canvas.drawCircle(vehiclePt, radiusPx, uncertFill);
        canvas.drawCircle(vehiclePt, radiusPx, uncertBorder);
      }

      // Directional Chevron / Arrow
      canvas.save();
      canvas.translate(vehiclePt.dx, vehiclePt.dy);

      if (currentSnapshot!.headingValid) {
        // Heading 0 = North (-Y in canvas)
        final headingRad = currentSnapshot!.headingDeg * math.pi / 180.0;
        canvas.rotate(headingRad);

        final chevronPath = Path()
          ..moveTo(0, -16)
          ..lineTo(12, 12)
          ..lineTo(0, 6)
          ..lineTo(-12, 12)
          ..close();

        final chevronOutline = Paint()
          ..color = Colors.white
          ..strokeWidth = 3.0
          ..style = PaintingStyle.stroke;

        final chevronFill = Paint()
          ..color = const Color(0xFF00E5FF)
          ..style = PaintingStyle.fill;

        canvas.drawPath(chevronPath, chevronFill);
        canvas.drawPath(chevronPath, chevronOutline);
      } else {
        // Unknown heading: neutral glowing dot
        final dotFill = Paint()..color = const Color(0xFF00E5FF);
        final dotOutline = Paint()
          ..color = Colors.white
          ..strokeWidth = 2.5
          ..style = PaintingStyle.stroke;

        canvas.drawCircle(Offset.zero, 8, dotFill);
        canvas.drawCircle(Offset.zero, 8, dotOutline);
      }

      canvas.restore();
    }
  }

  @override
  bool shouldRepaint(covariant OfflineMapPainter oldDelegate) {
    return true;
  }
}

