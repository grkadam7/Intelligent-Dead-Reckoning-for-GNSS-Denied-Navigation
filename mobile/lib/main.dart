// Main Application Entry Point for IDR-INS Mobile Navigation Client.
// Supports both offline deterministic replay and live on-device sensor navigation.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'navigation/replay_source.dart';
import 'navigation/live_sensor_source.dart';
import 'map/offline_region_store.dart';
import 'map/navigation_screen.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
      systemNavigationBarColor: Color(0xFF1B1E22),
      systemNavigationBarIconBrightness: Brightness.light,
    ),
  );

  final regionStore = OfflineRegionStore();
  final replaySource = ReplaySource();
  final liveSensorSource = LiveSensorSource();

  runApp(IDRMobileApp(
    regionStore: regionStore,
    replaySource: replaySource,
    liveSensorSource: liveSensorSource,
  ));
}

class IDRMobileApp extends StatefulWidget {
  final OfflineRegionStore regionStore;
  final ReplaySource replaySource;
  final LiveSensorSource liveSensorSource;

  const IDRMobileApp({
    Key? key,
    required this.regionStore,
    required this.replaySource,
    required this.liveSensorSource,
  }) : super(key: key);

  @override
  State<IDRMobileApp> createState() => _IDRMobileAppState();
}

class _IDRMobileAppState extends State<IDRMobileApp> {
  bool _isLoading = true;
  String? _errorMessage;

  @override
  void initState() {
    super.initState();
    _loadData();
  }

  Future<void> _loadData() async {
    try {
      await widget.regionStore.loadAssets();
      await widget.replaySource.loadFromAsset('assets/replay/navigation.json');
      setState(() {
        _isLoading = false;
      });
    } catch (e) {
      setState(() {
        _errorMessage = e.toString();
        _isLoading = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'IDR-INS Navigation',
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF121417),
        primaryColor: Colors.blueAccent,
      ),
      home: _isLoading
          ? const Scaffold(
              body: Center(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    CircularProgressIndicator(color: Colors.blueAccent),
                    SizedBox(height: 16),
                    Text("Loading packaged offline region...", style: TextStyle(color: Colors.white70)),
                  ],
                ),
              ),
            )
          : _errorMessage != null
              ? Scaffold(
                  body: Center(
                    child: Padding(
                      padding: const EdgeInsets.all(24.0),
                      child: Text(
                        "Error initializing offline assets:\n$_errorMessage",
                        textAlign: TextAlign.center,
                        style: const TextStyle(color: Colors.redAccent),
                      ),
                    ),
                  ),
                )
              : NavigationScreen(
                  regionStore: widget.regionStore,
                  replaySource: widget.replaySource,
                  liveSensorSource: widget.liveSensorSource,
                ),
    );
  }
}
