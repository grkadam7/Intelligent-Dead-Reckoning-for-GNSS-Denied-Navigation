"""
C ABI Interface Wrapper for IDR-INS Edge Core Engine.
SIH Problem Statement ID: 26168 | ISRO

Provides C-callable entry points and pointer management for foreign function
interfaces (ctypes, Cython, C/C++ FFI) to embed IDR-INS into edge hardware.
"""

import ctypes
from typing import Dict, Optional

from src.edge.engine import HighRateEdgeEngine
from src.edge.types import (
    IDRIMUSample,
    IDRGNSSSample,
    IDRNavOutput,
    IDREngineStats,
)

# Registry of active engine instances keyed by integer handle
_ENGINE_REGISTRY: Dict[int, HighRateEdgeEngine] = {}
_NEXT_HANDLE: int = 1


def idr_engine_create(sampling_rate_hz: float = 200.0) -> int:
    """
    Creates a new HighRateEdgeEngine instance and returns an opaque handle.
    
    Args:
        sampling_rate_hz: IMU sampling frequency (e.g. 50.0, 100.0, 200.0).
        
    Returns:
        Opaque integer handle (>0) on success, or 0 on error.
    """
    global _NEXT_HANDLE
    try:
        engine = HighRateEdgeEngine(sampling_rate=sampling_rate_hz)
        handle = _NEXT_HANDLE
        _NEXT_HANDLE += 1
        _ENGINE_REGISTRY[handle] = engine
        return handle
    except Exception:
        return 0


def idr_engine_destroy(handle: int) -> None:
    """Destroys an engine instance and frees associated resources."""
    _ENGINE_REGISTRY.pop(handle, None)


def idr_engine_push_imu(handle: int, sample: IDRIMUSample) -> int:
    """
    Pushes an IMU sample into the engine's ring buffer.
    
    Returns:
        0 on success, -1 if handle invalid, -2 if sample dropped/error.
    """
    engine = _ENGINE_REGISTRY.get(handle)
    if engine is None:
        return -1
    success = engine.push_imu(sample)
    return 0 if success else -2


def idr_engine_push_gnss(handle: int, sample: IDRGNSSSample) -> int:
    """
    Buffers an asynchronous GNSS fix measurement.
    
    Returns:
        0 on success, -1 if handle invalid.
    """
    engine = _ENGINE_REGISTRY.get(handle)
    if engine is None:
        return -1
    engine.push_gnss(sample)
    return 0


def idr_engine_step(handle: int, output_ptr: Optional[ctypes.POINTER(IDRNavOutput)] = None) -> int:
    """
    Executes one navigation epoch and writes the result into output_ptr if provided.
    
    Returns:
        0 on success, -1 if handle invalid, -2 if no IMU data available.
    """
    engine = _ENGINE_REGISTRY.get(handle)
    if engine is None:
        return -1

    out = engine.step()
    if out is None:
        return -2

    if output_ptr is not None:
        if isinstance(output_ptr, ctypes.Structure):
            target = ctypes.byref(output_ptr)
        elif hasattr(output_ptr, "contents"):
            target = ctypes.byref(output_ptr.contents)
        else:
            target = output_ptr
        ctypes.memmove(target, ctypes.byref(out), ctypes.sizeof(IDRNavOutput))
    return 0


def idr_engine_get_stats(handle: int, stats_ptr: Optional[ctypes.POINTER(IDREngineStats)] = None) -> int:
    """
    Retrieves engine performance telemetry.
    
    Returns:
        0 on success, -1 if handle invalid.
    """
    engine = _ENGINE_REGISTRY.get(handle)
    if engine is None:
        return -1

    stats = engine.get_stats()
    if stats_ptr is not None:
        if isinstance(stats_ptr, ctypes.Structure):
            target = ctypes.byref(stats_ptr)
        elif hasattr(stats_ptr, "contents"):
            target = ctypes.byref(stats_ptr.contents)
        else:
            target = stats_ptr
        ctypes.memmove(target, ctypes.byref(stats), ctypes.sizeof(IDREngineStats))
    return 0


def idr_engine_reset(handle: int) -> int:
    """Resets navigation engine state and buffers."""
    engine = _ENGINE_REGISTRY.get(handle)
    if engine is None:
        return -1
    engine.reset()
    return 0


def get_engine_by_handle(handle: int) -> Optional[HighRateEdgeEngine]:
    """Helper to inspect active engine in Python tests."""
    return _ENGINE_REGISTRY.get(handle)
