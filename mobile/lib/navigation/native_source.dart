// Dart FFI bindings and adapter for libidr_mobile C ABI.
// Fulfills MOBILE-01, MOBILE-02, and MOBILE-03 native execution.

import 'dart:async';
import 'dart:ffi' as ffi;
import 'dart:io';
import 'package:ffi/ffi.dart';
import 'nav_snapshot.dart';

// FFI Struct representations matching native/include/idr_mobile.h
final class NativeIMUSample extends ffi.Struct {
  @ffi.Double()
  external double timestampS;
  @ffi.Double()
  external double accelXMps2;
  @ffi.Double()
  external double accelYMps2;
  @ffi.Double()
  external double accelZMps2;
  @ffi.Double()
  external double gyroXRadS;
  @ffi.Double()
  external double gyroYRadS;
  @ffi.Double()
  external double gyroZRadS;
}

final class NativeGNSSSample extends ffi.Struct {
  @ffi.Double()
  external double timestampS;
  @ffi.Double()
  external double latitudeDeg;
  @ffi.Double()
  external double longitudeDeg;
  @ffi.Double()
  external double altitudeM;
  @ffi.Double()
  external double accuracyM;
  @ffi.Double()
  external double speedMps;
  @ffi.Double()
  external double headingDeg;
  @ffi.Int32()
  external int isValid;
}

final class NativeNavOutput extends ffi.Struct {
  @ffi.Int64()
  external int timestampMs;
  @ffi.Int64()
  external int sequence;
  @ffi.Int32()
  external int mode;
  @ffi.Int32()
  external int calibrationStage;
  @ffi.Double()
  external double latitudeDeg;
  @ffi.Double()
  external double longitudeDeg;
  @ffi.Double()
  external double altitudeM;
  @ffi.Double()
  external double rawLatitudeDeg;
  @ffi.Double()
  external double rawLongitudeDeg;
  @ffi.Double()
  external double speedMps;
  @ffi.Double()
  external double headingDeg;
  @ffi.Double()
  external double horizontalUncertaintyM;
  @ffi.Double()
  external double rollDeg;
  @ffi.Double()
  external double pitchDeg;
  @ffi.Double()
  external double yawDeg;
  @ffi.Int32()
  external int isMatched;
  @ffi.Int32()
  external int stateAgeMs;
}

// C function signatures
typedef NativeGetAbiVersion = ffi.Int32 Function();
typedef DartGetAbiVersion = int Function();

typedef NativeCreate = ffi.Pointer<ffi.Void> Function(ffi.Double);
typedef DartCreate = ffi.Pointer<ffi.Void> Function(double);

typedef NativeDestroy = ffi.Int32 Function(ffi.Pointer<ffi.Void>);
typedef DartDestroy = int Function(ffi.Pointer<ffi.Void>);

typedef NativePushIMU = ffi.Int32 Function(ffi.Pointer<ffi.Void>, ffi.Pointer<NativeIMUSample>);
typedef DartPushIMU = int Function(ffi.Pointer<ffi.Void>, ffi.Pointer<NativeIMUSample>);

typedef NativePushGNSS = ffi.Int32 Function(ffi.Pointer<ffi.Void>, ffi.Pointer<NativeGNSSSample>);
typedef DartPushGNSS = int Function(ffi.Pointer<ffi.Void>, ffi.Pointer<NativeGNSSSample>);

typedef NativeStep = ffi.Int32 Function(ffi.Pointer<ffi.Void>, ffi.Pointer<NativeNavOutput>);
typedef DartStep = int Function(ffi.Pointer<ffi.Void>, ffi.Pointer<NativeNavOutput>);

typedef NativeReset = ffi.Int32 Function(ffi.Pointer<ffi.Void>);
typedef DartReset = int Function(ffi.Pointer<ffi.Void>);

class NativeNavigationSource {
  ffi.DynamicLibrary? _dylib;
  ffi.Pointer<ffi.Void>? _handle;
  Timer? _ticker;
  final StreamController<NavSnapshot> _controller = StreamController<NavSnapshot>.broadcast();

  DartCreate? _createFn;
  DartDestroy? _destroyFn;
  DartPushIMU? _pushImuFn;
  DartPushGNSS? _pushGnssFn;
  DartStep? _stepFn;
  DartReset? _resetFn;

  bool get isAvailable => _handle != null && _handle != ffi.nullptr;
  Stream<NavSnapshot> get stream => _controller.stream;

  NativeNavigationSource({String? libraryPath}) {
    _initialize(libraryPath);
  }

  void _initialize(String? libraryPath) {
    try {
      if (libraryPath != null && libraryPath.isNotEmpty) {
        _dylib = ffi.DynamicLibrary.open(libraryPath);
      } else if (Platform.isAndroid) {
        _dylib = ffi.DynamicLibrary.open('libidr_mobile.so');
      } else if (Platform.isIOS || Platform.isMacOS) {
        _dylib = ffi.DynamicLibrary.process();
      } else if (Platform.isWindows) {
        _dylib = ffi.DynamicLibrary.open('idr_mobile.dll');
      } else {
        _dylib = ffi.DynamicLibrary.open('libidr_mobile.so');
      }

      _createFn = _dylib!.lookupFunction<NativeCreate, DartCreate>('idr_mobile_create');
      _destroyFn = _dylib!.lookupFunction<NativeDestroy, DartDestroy>('idr_mobile_destroy');
      _pushImuFn = _dylib!.lookupFunction<NativePushIMU, DartPushIMU>('idr_mobile_push_imu');
      _pushGnssFn = _dylib!.lookupFunction<NativePushGNSS, DartPushGNSS>('idr_mobile_push_gnss');
      _stepFn = _dylib!.lookupFunction<NativeStep, DartStep>('idr_mobile_step');
      _resetFn = _dylib!.lookupFunction<NativeReset, DartReset>('idr_mobile_reset');

      _handle = _createFn!(10.0);
    } catch (_) {
      // Library not loaded in host unit test / pure Dart environment
      _dylib = null;
      _handle = null;
    }
  }

  void start(Duration interval) {
    if (!isAvailable) return;
    _ticker?.cancel();
    _ticker = Timer.periodic(interval, (_) => step());
  }

  void step() {
    if (!isAvailable || _stepFn == null) return;

    final outPtr = calloc<NativeNavOutput>();
    try {
      final rc = _stepFn!(_handle!, outPtr);
      if (rc == 0) {
        final out = outPtr.ref;
        final snapshot = NavSnapshot(
          schemaVersion: '1.0.0',
          sequence: out.sequence,
          source: NavSource.live,
          timestampMs: out.timestampMs,
          stateAgeMs: out.stateAgeMs,
          latitude: out.latitudeDeg,
          longitude: out.longitudeDeg,
          altitudeM: out.altitudeM,
          rawLatitude: out.rawLatitudeDeg,
          rawLongitude: out.rawLongitudeDeg,
          speedMps: out.speedMps,
          speedKmh: out.speedMps * 3.6,
          speedValid: true,
          headingDeg: out.headingDeg,
          headingValid: true,
          mode: NavigationMode.values[out.mode.clamp(0, NavigationMode.values.length - 1)],
          modeReason: out.mode == 2 ? 'GNSS outage active' : 'Live sensor fusion active',
          horizontalUncertaintyM: out.horizontalUncertaintyM,
          uncertaintyValid: true,
          isMatched: out.isMatched == 1,
          calibrationStage: CalibrationStage.values[out.calibrationStage.clamp(0, CalibrationStage.values.length - 1)],
          rollDeg: out.rollDeg,
          pitchDeg: out.pitchDeg,
          yawDeg: out.yawDeg,
          simulated: false,
        );
        _controller.add(snapshot);
      }
    } finally {
      calloc.free(outPtr);
    }
  }

  void stop() {
    _ticker?.cancel();
    _ticker = null;
  }

  void reset() {
    if (isAvailable && _resetFn != null) {
      _resetFn!(_handle!);
    }
  }

  void dispose() {
    stop();
    if (isAvailable && _destroyFn != null) {
      _destroyFn!(_handle!);
      _handle = null;
    }
    _controller.close();
  }
}

