# Phase 7: Mobile Navigation Application — Planning Context

Date: 2026-10-31
Status: Planning authorized; implementation not started

## User decisions

- Continue with Phase 7 planning rather than resolving Phase 6 verification first.
- Carry Phase 6 verification as outstanding. Do not mark it verified, weaken its target, or overwrite its staged changes.
- No framework, device, map region, visual design, or native deployment approach has been explicitly approved by the user.

## Scope inherited from the project

- Cross-platform Android/iOS client; offline on-device navigation, not a remote Python service.
- MOBILE-01: offline map and genuine position updates at >=10 Hz.
- MOBILE-02: GNSS-aided, dead-reckoning, and recovery states with honest confidence/drift bounds.
- MOBILE-03: actual calibration progress and phone-to-vehicle orientation.
- Roadmap keeps two plans: 07-01 offline rendering/replay; 07-02 live sensor, ONNX, and navigation integration.
- No OBD-II dependency, cloud runtime, turn-by-turn routing feature expansion, or production/background-service claims.

## Planning assumptions (not user-approved decisions)

- Start with an explicitly labelled, deterministic replay demonstration before real sensor integration.
- Evaluate Flutter + MapLibre Native + shared C++/ONNX Runtime as a provisional architecture; include a decision checkpoint before committing to the stack or a costly port.
- Python remains a reference implementation. Existing ctypes wrappers/header are not proof of a loadable mobile native library.
- Sensor acquisition/model input rate is distinct from 10 Hz navigation publication and rendering cadence.
- Asset and toolchain discovery are execution gates: no model, normalization, map package, Android SDK, or Mac/Xcode availability is assumed.

## Outstanding Phase 6 verification (not waived)

- 06-01-PLAN.md requires <1.5% outage drift; STATE.md records 6.92%. The latter satisfies the overall <10% project threshold but not that plan's stricter requirement.
- Completion summary/verification evidence is absent despite the roadmap complete marker.
- Phase 6 work is staged and must remain untouched.
- Parent attempted `python -m pytest tests/test_phase6.py -q`; terminal could not launch due missing `libasound.so.2`. No fresh test result exists.
- Phase 7 planning and UI/replay scaffolding may proceed; Phase 7 integration and project completion must not silently inherit Phase 6 accuracy/native-export claims.

## Evidence and boundaries

Use RESEARCH.md and cited repository paths for technical planning. Model/road assets were not found during research. The edge loop does not currently execute ONNX or map matching and its output lacks real uncertainty/calibration telemetry. The plan must include this missing work rather than pretend a mobile binding already exists.

## Required checkpoints

1. Before implementation: confirm provisional stack, target devices, demo region, and access to Android/iOS build tools.
2. Before live inference: recover/produce versioned model, normalization and sample-rate/timestamp contract; validate held-out accuracy without GNSS/ground-truth speed leakage.
3. Before completion: separate replay/simulator evidence from physical-device sensor/timing results; retain unresolved Phase 6 gate.

## Workflow limitation

GSD CLI initialization/hook checks cannot currently run because terminal launch fails. Planning uses file inspection and independent research/planning/review agents. This is not an automated GSD gate pass. No commits or branches are authorized.
