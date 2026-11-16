// Locks the Dart FFI struct layouts to the C ABI in native/include/idr_mobile.h.

import 'dart:ffi';
import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/navigation/native_source.dart';

void main() {
  test('Dart FFI structs match C ABI sizes', () {
    expect(sizeOf<NativeIMUSample>(), 56, reason: 'IDRMobileIMUSample must stay 7 doubles');
    expect(sizeOf<NativeGNSSSample>(), 64, reason: 'IDRMobileGNSSSample must stay 7 doubles + int32');
    expect(sizeOf<NativeNavOutput>(), 120, reason: 'IDRMobileNavOutput layout changed');
  });
}
