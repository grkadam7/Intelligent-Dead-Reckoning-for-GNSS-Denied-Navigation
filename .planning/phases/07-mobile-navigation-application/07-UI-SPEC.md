---
phase: 07-mobile-navigation-application
status: proposed
approval: not-user-approved
created: 2026-10-31
requirements: [MOBILE-01, MOBILE-02, MOBILE-03]
---

# Phase 7 UI specification — PROPOSED, not user-approved

## Intent and decisions

A map-first, readable in-vehicle diagnostic navigation client: offline map, continuously updated vehicle cursor, truthful navigation state, uncertainty and phone-to-vehicle calibration. This is not turn-by-turn routing or a lane-level positioning claim. Flutter + MapLibre Native is a **provisional implementation candidate**, pending 07-01 feasibility and explicit decision checkpoints. No framework, device, initial region, provider, visual design or native deployment approach has user approval. Review this proposal before implementation.

Foreground operation only unless separately approved/tested. Background/screen-off interruptions must show suspension and require valid resume; no background-service promise. A remote Python service is incompatible with the offline requirement. Phase 6 verification remains unresolved and is not hidden by a polished mobile screen.

## Proposed information architecture

One navigation screen with bottom sheets for **Calibration**, **Offline region**, and **Diagnostics**; no additional app-level navigation hierarchy needed. Core information remains available even when the map cannot render.

```text
┌─────────────────────────────────────┐
│ IDR Navigation         Offline region│
│ Simulation — deterministic replay   │ ← persistent source label
│ [GNSS aided / DR / Recovering]       │ ← icon + explicit text
├─────────────────────────────────────┤
│                                     │
│           OFFLINE MAP               │
│          vehicle cursor             │
│        uncertainty ellipse          │
│                                     │
│                         [North up]  │
│                         [Recenter]  │
│ Attribution / package coverage      │
├─────────────────────────────────────┤
│ Speed       Estimated uncertainty   │
│ State age   Calibration stage       │
│ [Calibration]        [Diagnostics]  │
└─────────────────────────────────────┘
```

Use safe areas; a compact bottom HUD on portrait phones, a side panel in landscape/large screens. At 200% text size wrap/reflow rather than truncate mode, error or uncertainty labels. Map controls and attribution must remain reachable. Details sheets scroll independently; no essential information requires reading moving map labels.

## Visual system (proposal)

- Platform system sans-serif, tabular numerals for speed/age; proposed sizes: title 24sp, speed 32sp, main status/body 16sp, secondary/attribution 14sp, with OS text scaling supported.
- 4dp spacing base: 8dp within groups, 16dp screen/panel padding, 24dp between sections. Controls minimum 48×48dp; sheets have a visible title/close action.
- Neutral high-contrast surfaces; semantic green=GNSS aided, amber=dead reckoning/recovery caution, red=blocking fault, blue=simulation/info. Always pair with text/icons; do not encode mode only in color. Validate actual palette in light/dark themes to >=4.5:1 normal text and >=3:1 controls/meaningful graphics; no palette is approved yet.
- Vehicle cursor is a directional arrow with high-contrast outline; unknown heading uses a neutral dot, not an arbitrary North arrow. Raw trace and matched trace have distinct line styles and a legend in diagnostics. No route line implying turn instructions.
- Covariance ellipse is optional when valid and within sensible map scale; textual uncertainty remains primary. Off-screen/very large uncertainty shows a textual warning, never clips into false precision.
- Smooth camera/cursor interpolation between valid states; reduced motion disables animated recenter/transitions. Do not generate extra navigation fixes for smoothness.

## Required screen states and copy

| State | Visible presentation | Actions and constraints |
|---|---|---|
| Replay | Persistent **Simulation — deterministic replay** banner, simulated mode/calibration details | Play/pause/reset in diagnostics; never styled as a live capture. All replay-derived uncertainty/calibration labelled simulated. |
| Live initializing | **Live device · Preparing navigation**; explicit waiting reason | Request permission or select region; no fabricated origin, heading, speed or calibrated badge. |
| Waiting for fix/origin | **Waiting for initial GNSS position** | Do not place a live cursor at (0,0). Map may show selected region center, explicitly not vehicle location. |
| Waiting for model/window | **Preparing motion model** and readiness stage | Window fill is not inference latency; no constant-speed fallback advertised as AI. |
| GNSS aided | **GNSS Aided (Active)** with satellite icon | Only fresh accepted GNSS/fused state; optional metrics absent → unavailable. |
| Blackout | **GNSS Outage (Dead Reckoning Mode)**, reason and outage duration | Show estimated uncertainty or unavailable; do not claim physical loss time from a timeout alone. |
| Recovery | **Recovering GNSS** with progress/stage derived from core | Smoothly display the reviewed reconvergence output; no timed UI completion or coordinate teleport. |
| Stale/gap | **Navigation data stale · last update … ago** | Mark/freeze last-known cursor; stop indefinite extrapolation, show age and cause. Do not keep a green active chip. |
| Suspended | **Navigation paused while app inactive** | Resume revalidates clocks/calibration/windows/fix freshness; no background tracking claim. |
| Permission denied/revoked | **Location permission needed** / **Motion access needed** | Explain foreground use; retry/system-settings action where appropriate; replay remains labelled and optional, never automatic live replacement. |
| Missing sensor/model | **Required sensor unavailable** / **Motion model unavailable** | Diagnostic reason and retry/manage action; no fabricated live motion/confidence. |
| No region | **Offline region not installed** | Show coverage/size/license summary and **Import region**; no tile-network fallback. Textual navigation status remains usable. |
| Corrupt/incompatible region | **Offline region could not be opened** | Retain last valid package where possible; **Choose another region** / retry import; checksum/schema reason in diagnostics. |
| Insufficient storage | **Not enough space for offline region** | Show required versus available space before install; atomic install, retain old package. |
| Outside coverage | **Outside offline map coverage** | Show boundary and last local map if available; **Manage regions**; navigation/match availability separate, no false road snapping. |
| Matcher uncertain/unavailable | **Road match unavailable** or **Unmatched position** | Keep raw fused cursor with explicit match state, not forced road placement. |
| Fatal native error | **Navigation stopped** plus actionable cause | Retry/reset confirmation; no continued valid-state cursor or success-shaped stub. |

No app control intentionally toggles real GNSS loss in normal live mode. Test injection belongs in explicitly labelled diagnostics/test builds. User interaction during drives is passenger/test-rig only.

## Navigation data/HUD contract

Proposed `mobile/lib/navigation/nav_snapshot.dart` is the single typed source shared by replay and live adapters. Include schema/session/sequence/source, capture/model/state/publish times in a normalized monotonic domain, origin ID, raw ENU and WGS84 display coordinates, speed/heading validity, mode/reason, model readiness, calibration stage/angles, covariance/uncertainty validity, match availability and fault/drop counters. Missing/invalid fields render **Unavailable** or an em dash with an accessible label, never zero-as-known.

- Speed displays in selected user units (proposed km/h, internal m/s), not a GPS-derived substitute for learned blackout speed. Unknown speed is not `0 km/h`.
- Publication >=10Hz is measured by advancing source/state/sequence, not UI frame rate. Steady-state current sensor-to-publication age target <=100ms; display stale warning once that contract is exceeded, with separate model observation age in diagnostics.
- Position uncertainty text: **Estimated horizontal 95% uncertainty: … m** when covariance is valid. Details state Gaussian/filter assumptions and show whether it applies to raw fused state rather than snapped/smoothed display position. Invalid/missing covariance: **Uncertainty unavailable**. No arbitrary confidence percentages, no cross-track error relabelled as accuracy, no lane-level assurance.
- Diagnostics separates sensor acquisition, inference time (<20ms target on declared devices), effective model window age, current state age, publication rate, render delay and queue drops. Physical loss detection versus callback/timeout reaction must not be conflated. User sees the distinction without needing a developer console.
- ENU integration stays in the core; WGS84 map conversion occurs only at boundaries with shared graph/origin metadata. Platform units/clocks are never guessed by the widget.

## Actual calibration sheet

Proposed `mobile/lib/calibration/calibration_panel.dart` presents real core-driven stages:

1. **Waiting for stillness** — “Secure the phone in its mount and keep the vehicle still.”
2. **Leveling phone** — qualified sample/window count if available; roll/pitch only once valid.
3. **Waiting for straight movement** — “When safe, drive normally in a straight line.” No instruction to perform unsafe maneuvers or interact while driving.
4. **Aligning forward axis** — report insufficient excitation/turn rejection/sign ambiguity, not a time-based progress bar.
5. **Calibrated** — valid phone-to-vehicle roll/pitch/yaw in degrees, orientation graphic with text equivalent; alignment quality labelled separately from position uncertainty.
6. **Mount changed / Recalibration needed** — invalidate affected orientation/model window, show cause and readiness.
7. **Unavailable / Failed** — missing sensor, invalid sample rate/clock, insufficient motion or failure reason; retry only where meaningful.

Progress is stage/qualified evidence, not an invented percentage. Replay may exercise all these views only with persistent Simulation labelling. A **Recalibrate** action confirms clearing alignment/window state and waits for new physical evidence; reset alone cannot mark calibrated.

## Offline-region sheet

Display approved package name, bbox/coverage preview, zoom range, version, checksum status, size, graph availability and origin, license/attribution, and import status. Distinguish rendering package from matching graph. An installed map does not imply a connected matching graph or route planner. Cold-start resources include local style/glyphs/fonts/sprites, not warm remote cache.

Use licensed self-produced or explicitly permitted offline/redistributable resources. **Never bulk-download/prefetch public tile.openstreetmap.org tiles.** Provider/fees/keys remain a user decision; no hardcoded secret. Any explicit pre-travel provisioning network operation is separate from runtime, disclosed to the user and not necessary once package installed. Imported package validation handles unsafe paths, hashes, schema, disk-space and atomic replacement.

## Accessibility and interaction acceptance

- TalkBack/VoiceOver logical order: source → mode → fault/readiness → speed/uncertainty → calibration → controls; map has an equivalent concise textual summary. Recenter/North-up/region buttons have explicit labels and focus feedback.
- Do not announce every 10Hz snapshot. Announce mode/error/calibration stage changes; rate-limit numerical summary announcements on request. No focus theft when status changes.
- Text+icon mode signalling, large text, minimum touch targets, high contrast, reduced motion, safe-area/landscape layouts; no color-only status or precision-dependent gestures.
- Recenter preserves manual pan until activated; camera does not fight the user. North-up toggle reports state accessibly. Attribution is legible and reachable, not hidden behind a sheet handle.

## Verification and approval boundary

Proposed tests: `mobile/test/navigation_screen_test.dart`, `mobile/test/calibration_panel_test.dart`, `mobile/test/offline_region_store_test.dart`, `mobile/integration_test/offline_replay_test.dart`, `mobile/integration_test/live_navigation_test.dart`. See 07-VALIDATION.md for exact future commands and physical-device protocol. Test every state above, 200% text, both themes/orientations, reduced motion and screen readers; capture offline cold start with network blocked. Simulation/UI screenshots cannot close live sensor, calibration, <20ms model or <10% held-out drift gates. All tests are unrun and this design remains proposed, not approved.
