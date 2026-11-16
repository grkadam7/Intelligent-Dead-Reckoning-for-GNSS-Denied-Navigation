// Deterministic clock for navigation-engine tests.

class ManualClock {
  ManualClock(this._now);

  DateTime _now;

  DateTime call() => _now;

  void advance(Duration duration) {
    _now = _now.add(duration);
  }
}
