// Deterministic 10 Hz replay source for IDR-INS.
// Streams NavSnapshot instances with monotonically advancing timestamps.

import 'dart:async';
import 'dart:convert';
import 'package:flutter/services.dart' show rootBundle;
import 'nav_snapshot.dart';

class ReplaySource {
  final List<NavSnapshot> _snapshots = [];
  final StreamController<NavSnapshot> _controller = StreamController<NavSnapshot>.broadcast();
  Timer? _timer;
  int _currentIndex = 0;
  bool _isPlaying = false;
  final Duration interval;

  ReplaySource({this.interval = const Duration(milliseconds: 100)});

  bool get isPlaying => _isPlaying;
  int get currentIndex => _currentIndex;
  int get totalSnapshots => _snapshots.length;
  Stream<NavSnapshot> get stream => _controller.stream;

  /// Loads replay snapshots from a raw JSON string.
  void loadFromString(String jsonContent) {
    _snapshots.clear();
    final dynamic parsed = json.decode(jsonContent);
    if (parsed is List) {
      for (final item in parsed) {
        if (item is Map<String, dynamic>) {
          _snapshots.add(NavSnapshot.fromJson(item));
        }
      }
    }
    _currentIndex = 0;
  }

  /// Loads replay snapshots from Flutter assets bundle.
  Future<void> loadFromAsset(String assetPath) async {
    final jsonContent = await rootBundle.loadString(assetPath);
    loadFromString(jsonContent);
  }

  /// Starts or resumes playback at the configured interval (10 Hz).
  void start() {
    if (_snapshots.isEmpty) return;

    _isPlaying = true;
    _timer?.cancel();

    _timer = Timer.periodic(interval, (timer) {
      if (!_isPlaying || _snapshots.isEmpty) return;

      if (_currentIndex < _snapshots.length) {
        final current = _snapshots[_currentIndex];
        _controller.add(current);
        _currentIndex++;
      } else {
        // Loop simulation
        _currentIndex = 0;
      }
    });
  }

  void pause() {
    _isPlaying = false;
  }

  void resume() {
    if (_snapshots.isNotEmpty) {
      _isPlaying = true;
    }
  }

  void seek(int index) {
    if (index >= 0 && index < _snapshots.length) {
      _currentIndex = index;
      if (_snapshots.isNotEmpty) {
        _controller.add(_snapshots[_currentIndex]);
      }
    }
  }

  void stop() {
    _isPlaying = false;
    _timer?.cancel();
    _timer = null;
    _currentIndex = 0;
  }

  void dispose() {
    stop();
    _controller.close();
  }
}

