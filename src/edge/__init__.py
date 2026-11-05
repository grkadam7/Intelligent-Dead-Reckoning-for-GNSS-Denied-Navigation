"""
IDR-INS Edge Core Engine Package.
SIH Problem Statement ID: 26168 | ISRO

Provides high-rate (up to 200 Hz) modular dead reckoning, C ABI types,
ring buffering, and headless edge deployment tools.
"""

from src.edge.types import (
    IDRNavigationMode,
    IDRIMUSample,
    IDRGNSSSample,
    IDRNavOutput,
    IDREngineStats,
)
from src.edge.buffer import HighRateRingBuffer
from src.edge.engine import HighRateEdgeEngine
from src.edge.c_api import (
    idr_engine_create,
    idr_engine_destroy,
    idr_engine_push_imu,
    idr_engine_push_gnss,
    idr_engine_step,
    idr_engine_get_stats,
    idr_engine_reset,
)

__all__ = [
    "IDRNavigationMode",
    "IDRIMUSample",
    "IDRGNSSSample",
    "IDRNavOutput",
    "IDREngineStats",
    "HighRateRingBuffer",
    "HighRateEdgeEngine",
    "idr_engine_create",
    "idr_engine_destroy",
    "idr_engine_push_imu",
    "idr_engine_push_gnss",
    "idr_engine_step",
    "idr_engine_get_stats",
    "idr_engine_reset",
]
