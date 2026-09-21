"""
Offline Vector Road Network Representation, Spatial Indexing, and Topological Routing.
SIH Problem Statement ID: 26168 | ISRO

Implements MAP-02:
Integrates offline OpenStreetMap (OSM) / GeoJSON vector road link geometries
with O(1) 2D spatial hash grid indexing and Dijkstra graph shortest-path routing.
"""

from collections import defaultdict
from dataclasses import dataclass, field
import heapq
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union
import numpy as np

from src.data.iovnbd_loader import geodetic_to_enu, enu_to_geodetic


@dataclass
class RoadSegment:
    """Represents a directional road link / polyline segment."""
    segment_id: str
    name: str
    start_node: str
    end_node: str
    coords_enu: np.ndarray         # (M, 2) or (M, 3) [East, North] in meters
    length_m: float
    heading_deg: float             # Azimuth in degrees (0 = North, 90 = East)
    road_type: str = "primary"
    one_way: bool = False
    speed_limit_mps: float = 13.88 # 50 km/h default
    coords_wgs84: Optional[np.ndarray] = None  # (M, 2) [Lat, Lon]

    @property
    def start_point_enu(self) -> np.ndarray:
        return self.coords_enu[0, :2]

    @property
    def end_point_enu(self) -> np.ndarray:
        return self.coords_enu[-1, :2]

    def project_point(self, point_enu: np.ndarray) -> Tuple[np.ndarray, float, float, float]:
        """
        Projects a 2D ENU point onto this road link polyline.
        
        Args:
            point_enu: (2,) or (3,) [East, North] query point.
            
        Returns:
            (projected_point_enu (2,), perpendicular_distance_m, fraction_along_link, segment_heading_deg)
        """
        pt = np.asarray(point_enu)[:2]
        coords = self.coords_enu[:, :2]
        
        best_proj = coords[0]
        min_dist_sq = float("inf")
        best_fraction = 0.0
        best_heading = self.heading_deg
        
        # Calculate cumulative distances along polyline
        sub_lengths = [0.0]
        for k in range(len(coords) - 1):
            sub_len = float(np.linalg.norm(coords[k + 1] - coords[k]))
            sub_lengths.append(sub_lengths[-1] + sub_len)
        total_len = max(1e-6, sub_lengths[-1])

        # Test each sub-segment of polyline
        for k in range(len(coords) - 1):
            a = coords[k]
            b = coords[k + 1]
            ab = b - a
            ab_len_sq = float(np.dot(ab, ab))

            if ab_len_sq < 1e-10:
                t = 0.0
                proj = a.copy()
            else:
                t = float(np.clip(np.dot(pt - a, ab) / ab_len_sq, 0.0, 1.0))
                proj = a + t * ab

            dist_sq = float(np.sum((pt - proj) ** 2))
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_proj = proj
                dist_along = sub_lengths[k] + t * math.sqrt(ab_len_sq)
                best_fraction = float(np.clip(dist_along / total_len, 0.0, 1.0))
                
                # Heading of this specific sub-segment
                de = b[0] - a[0]
                dn = b[1] - a[1]
                sub_heading = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
                best_heading = sub_heading

        perp_dist = math.sqrt(min_dist_sq)
        return best_proj, perp_dist, best_fraction, best_heading


class SpatialGridIndex:
    """
    2D spatial hash grid index for rapid O(1) candidate road segment lookup.
    Avoids O(N) full network scans on every dead reckoning epoch.
    """

    def __init__(self, cell_size_m: float = 50.0):
        self.cell_size = cell_size_m
        self.grid: Dict[Tuple[int, int], List[RoadSegment]] = defaultdict(list)
        self.indexed_segments: Set[str] = set()

    def _get_cell(self, east: float, north: float) -> Tuple[int, int]:
        return int(math.floor(east / self.cell_size)), int(math.floor(north / self.cell_size))

    def insert(self, segment: RoadSegment):
        """Indexes a road segment into overlapping grid cells."""
        if segment.segment_id in self.indexed_segments:
            return
        self.indexed_segments.add(segment.segment_id)

        coords = segment.coords_enu[:, :2]
        min_e, min_n = np.min(coords, axis=0)
        max_e, max_n = np.max(coords, axis=0)

        min_cx, min_cy = self._get_cell(min_e, min_n)
        max_cx, max_cy = self._get_cell(max_e, max_n)

        for cx in range(min_cx, max_cx + 1):
            for cy in range(min_cy, max_cy + 1):
                self.grid[(cx, cy)].append(segment)

    def query(self, point_enu: np.ndarray, radius_m: float = 30.0) -> List[RoadSegment]:
        """
        Retrieves all candidate road segments within search radius of a query point.
        """
        pt = np.asarray(point_enu)[:2]
        pe, pn = float(pt[0]), float(pt[1])

        min_cx, min_cy = self._get_cell(pe - radius_m, pn - radius_m)
        max_cx, max_cy = self._get_cell(pe + radius_m, pn + radius_m)

        candidates: Dict[str, RoadSegment] = {}
        for cx in range(min_cx, max_cx + 1):
            for cy in range(min_cy, max_cy + 1):
                cell_segments = self.grid.get((cx, cy), [])
                for seg in cell_segments:
                    candidates[seg.segment_id] = seg

        return list(candidates.values())


class RoadNetwork:
    """
    Offline vector road network graph container with spatial indexing and routing.
    """

    def __init__(self, origin_geodetic: Optional[Tuple[float, float, float]] = None):
        self.origin_geodetic = origin_geodetic # (lat0, lon0, alt0)
        self.segments: Dict[str, RoadSegment] = {}
        self.nodes: Dict[str, np.ndarray] = {}  # node_id -> [East, North]
        # Adjacency list: node_u -> list of (node_v, segment_id, length_m)
        self.adj_list: Dict[str, List[Tuple[str, str, float]]] = defaultdict(list)
        self.spatial_index = SpatialGridIndex(cell_size_m=50.0)
        self._node_dist_cache: Dict[Tuple[str, str], float] = {}

    def add_node(self, node_id: str, coords_enu: np.ndarray):
        self.nodes[node_id] = np.asarray(coords_enu)[:2]

    def add_segment(self, segment: RoadSegment):
        self.segments[segment.segment_id] = segment
        self.nodes[segment.start_node] = segment.start_point_enu
        self.nodes[segment.end_node] = segment.end_point_enu

        # Forward edge
        self.adj_list[segment.start_node].append(
            (segment.end_node, segment.segment_id, segment.length_m)
        )
        
        # If bidirectional, add reverse edge with same geometry
        if not segment.one_way:
            self.adj_list[segment.end_node].append(
                (segment.start_node, segment.segment_id, segment.length_m)
            )

        self.spatial_index.insert(segment)
        self._node_dist_cache.clear()

    def _get_node_distance(self, u: str, v: str, max_cost: float = 800.0) -> float:
        """Memoized Dijkstra shortest path between two network nodes."""
        if u == v:
            return 0.0
        cache_key = (u, v)
        if cache_key in self._node_dist_cache:
            return self._node_dist_cache[cache_key]

        pq: List[Tuple[float, str]] = [(0.0, u)]
        distances: Dict[str, float] = {u: 0.0}

        while pq:
            d, curr = heapq.heappop(pq)
            if d > distances.get(curr, float("inf")):
                continue
            if curr == v:
                self._node_dist_cache[cache_key] = d
                return d
            if d >= max_cost:
                break

            for nxt, _, edge_len in self.adj_list.get(curr, []):
                new_d = d + edge_len
                if new_d < distances.get(nxt, float("inf")) and new_d < max_cost:
                    distances[nxt] = new_d
                    heapq.heappush(pq, (new_d, nxt))

        res = distances.get(v, float("inf"))
        self._node_dist_cache[cache_key] = res
        return res

    def find_nearest_segments(
        self,
        point_enu: np.ndarray,
        radius_m: float = 50.0,
        max_candidates: int = 5
    ) -> List[Tuple[RoadSegment, np.ndarray, float, float, float]]:
        """
        Finds candidate road links near a point within radius_m.
        
        Returns list of:
            (segment, projected_point, perpendicular_distance, fraction, segment_heading)
        sorted by perpendicular distance ascending.
        """
        candidates = self.spatial_index.query(point_enu, radius_m=radius_m)
        
        # Fallback to all segments if spatial index query returned nothing
        if not candidates and len(self.segments) < 200:
            candidates = list(self.segments.values())

        results = []
        for seg in candidates:
            proj_pt, dist_m, frac, heading = seg.project_point(point_enu)
            if dist_m <= radius_m:
                results.append((seg, proj_pt, dist_m, frac, heading))

        results.sort(key=lambda item: item[2])
        return results[:max_candidates]

    def shortest_path_distance(
        self,
        from_segment_id: str,
        to_segment_id: str,
        from_fraction: float = 0.5,
        to_fraction: float = 0.5,
    ) -> float:
        """
        Computes the shortest network path distance between two road links
        using memoized Dijkstra distances.
        """
        if from_segment_id == to_segment_id:
            seg = self.segments[from_segment_id]
            # Same segment: direct distance along segment
            return abs(to_fraction - from_fraction) * seg.length_m

        if from_segment_id not in self.segments or to_segment_id not in self.segments:
            return float("inf")

        seg_from = self.segments[from_segment_id]
        seg_to = self.segments[to_segment_id]

        # Depart from end node (forward) or start node (two-way)
        dep_end_dist = (1.0 - from_fraction) * seg_from.length_m
        target_start_dist = to_fraction * seg_to.length_m
        target_end_dist = (1.0 - to_fraction) * seg_to.length_m

        best_dist = float("inf")

        # Option 1: seg_from.end_node -> seg_to.start_node
        d1 = self._get_node_distance(seg_from.end_node, seg_to.start_node)
        if d1 < float("inf"):
            best_dist = min(best_dist, dep_end_dist + d1 + target_start_dist)

        # Option 2: seg_from.end_node -> seg_to.end_node (if seg_to two-way)
        if not seg_to.one_way:
            d2 = self._get_node_distance(seg_from.end_node, seg_to.end_node)
            if d2 < float("inf"):
                best_dist = min(best_dist, dep_end_dist + d2 + target_end_dist)

        # If seg_from is bidirectional:
        if not seg_from.one_way:
            dep_start_dist = from_fraction * seg_from.length_m
            d3 = self._get_node_distance(seg_from.start_node, seg_to.start_node)
            if d3 < float("inf"):
                best_dist = min(best_dist, dep_start_dist + d3 + target_start_dist)
            if not seg_to.one_way:
                d4 = self._get_node_distance(seg_from.start_node, seg_to.end_node)
                if d4 < float("inf"):
                    best_dist = min(best_dist, dep_start_dist + d4 + target_end_dist)

        return best_dist

    @classmethod
    def from_geojson(
        cls,
        geojson_source: Union[str, Path, dict],
        origin_geodetic: Optional[Tuple[float, float, float]] = None
    ) -> "RoadNetwork":
        """
        Parses a standard GeoJSON FeatureCollection of LineStrings into a RoadNetwork.
        """
        if isinstance(geojson_source, (str, Path)):
            with open(geojson_source, "r", encoding="utf-8") as f:
                data = json.load(f)
        else:
            data = geojson_source

        features = data.get("features", [])
        if not features:
            raise ValueError("GeoJSON contains no features.")

        # Determine reference origin if not provided
        if origin_geodetic is None:
            # Pick first valid coordinate
            first_coords = features[0]["geometry"]["coordinates"]
            if isinstance(first_coords[0], list) and isinstance(first_coords[0][0], (int, float)):
                lon0, lat0 = first_coords[0][0], first_coords[0][1]
            else:
                lon0, lat0 = first_coords[0][0][0], first_coords[0][0][1]
            origin_geodetic = (float(lat0), float(lon0), 0.0)

        lat0, lon0, alt0 = origin_geodetic
        network = cls(origin_geodetic=origin_geodetic)

        for idx, feat in enumerate(features):
            geom = feat.get("geometry", {})
            props = feat.get("properties", {})
            g_type = geom.get("type", "")
            raw_coords = geom.get("coordinates", [])

            if g_type == "LineString":
                linestrings = [raw_coords]
            elif g_type == "MultiLineString":
                linestrings = raw_coords
            else:
                continue

            name = props.get("name", f"Road_{idx}")
            road_type = props.get("highway", props.get("road_type", "primary"))
            one_way = props.get("oneway", False) in [True, "yes", "1"]

            for line_idx, line in enumerate(linestrings):
                if len(line) < 2:
                    continue

                lons = np.array([pt[0] for pt in line], dtype=np.float64)
                lats = np.array([pt[1] for pt in line], dtype=np.float64)
                alts = np.zeros_like(lats)

                east, north, up = geodetic_to_enu(lats, lons, alts, lat0, lon0, alt0)
                coords_enu = np.column_stack([east, north, up])
                coords_wgs84 = np.column_stack([lats, lons])

                # Segment ID
                seg_id = str(props.get("id", f"seg_{idx}_{line_idx}"))
                start_node = f"node_{seg_id}_start"
                end_node = f"node_{seg_id}_end"

                # Calculate cumulative length
                diffs = np.diff(coords_enu[:, :2], axis=0)
                length_m = float(np.sum(np.linalg.norm(diffs, axis=1)))
                if length_m < 1e-3:
                    continue

                # Chord heading
                de = coords_enu[-1, 0] - coords_enu[0, 0]
                dn = coords_enu[-1, 1] - coords_enu[0, 1]
                heading_deg = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0

                segment = RoadSegment(
                    segment_id=seg_id,
                    name=name,
                    start_node=start_node,
                    end_node=end_node,
                    coords_enu=coords_enu,
                    length_m=length_m,
                    heading_deg=heading_deg,
                    road_type=road_type,
                    one_way=one_way,
                    coords_wgs84=coords_wgs84,
                )
                network.add_segment(segment)

        return network

    @classmethod
    def create_synthetic_urban_grid(
        cls,
        grid_rows: int = 4,
        grid_cols: int = 4,
        block_size_m: float = 200.0,
        origin_geodetic: Tuple[float, float, float] = (52.40166, -1.50529, 147.5),
    ) -> "RoadNetwork":
        """
        Generates a connected synthetic urban road grid with avenues (North-South)
        and streets (East-West) for verification and testing.
        """
        network = cls(origin_geodetic=origin_geodetic)

        # Node coordinates: (r, c) -> node_id
        for r in range(grid_rows):
            for c in range(grid_cols):
                node_id = f"n_{r}_{c}"
                east = c * block_size_m
                north = r * block_size_m
                network.add_node(node_id, np.array([east, north, 0.0]))

        # East-West Streets
        for r in range(grid_rows):
            for c in range(grid_cols - 1):
                u = f"n_{r}_{c}"
                v = f"n_{r}_{c+1}"
                coords = np.array([
                    [c * block_size_m, r * block_size_m, 0.0],
                    [(c + 1) * block_size_m, r * block_size_m, 0.0]
                ])
                seg_id = f"street_{r}_{c}_to_{c+1}"
                # Heading East = 90 deg
                segment = RoadSegment(
                    segment_id=seg_id,
                    name=f"Street {r}",
                    start_node=u,
                    end_node=v,
                    coords_enu=coords,
                    length_m=block_size_m,
                    heading_deg=90.0,
                    road_type="primary",
                    one_way=False,
                )
                network.add_segment(segment)

        # North-South Avenues
        for c in range(grid_cols):
            for r in range(grid_rows - 1):
                u = f"n_{r}_{c}"
                v = f"n_{r+1}_{c}"
                coords = np.array([
                    [c * block_size_m, r * block_size_m, 0.0],
                    [c * block_size_m, (r + 1) * block_size_m, 0.0]
                ])
                seg_id = f"avenue_{c}_{r}_to_{r+1}"
                # Heading North = 0 deg
                segment = RoadSegment(
                    segment_id=seg_id,
                    name=f"Avenue {c}",
                    start_node=u,
                    end_node=v,
                    coords_enu=coords,
                    length_m=block_size_m,
                    heading_deg=0.0,
                    road_type="primary",
                    one_way=False,
                )
                network.add_segment(segment)

        return network

    @classmethod
    def create_from_route_waypoints(
        cls,
        waypoints_enu: np.ndarray,
        subsegment_length_m: float = 100.0,
        road_name: str = "Transit Corridor",
        origin_geodetic: Optional[Tuple[float, float, float]] = None,
        one_way: bool = False,
    ) -> "RoadNetwork":
        """
        Creates a road network chain along an ordered sequence of ENU waypoints.
        Partitions long stretches into subsegments for localized candidate snapping.
        """
        network = cls(origin_geodetic=origin_geodetic)
        pts = np.asarray(waypoints_enu)
        n = len(pts)
        if n < 2:
            return network

        # Cumulative distances along waypoints
        diffs = np.diff(pts[:, :2], axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        total_len = float(np.sum(seg_lens))

        # Sample at approximately subsegment_length_m intervals
        n_chunks = max(1, int(math.ceil(total_len / subsegment_length_m)))
        chunk_indices = np.linspace(0, n - 1, n_chunks + 1, dtype=int)

        for i in range(len(chunk_indices) - 1):
            idx_start = chunk_indices[i]
            idx_end = chunk_indices[i + 1]
            if idx_end <= idx_start:
                continue

            chunk_coords = pts[idx_start:idx_end + 1].copy()
            if chunk_coords.shape[1] == 2:
                chunk_coords = np.column_stack([chunk_coords, np.zeros(len(chunk_coords))])

            de = chunk_coords[-1, 0] - chunk_coords[0, 0]
            dn = chunk_coords[-1, 1] - chunk_coords[0, 1]
            heading = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0

            sub_diffs = np.diff(chunk_coords[:, :2], axis=0)
            length = float(np.sum(np.linalg.norm(sub_diffs, axis=1)))
            if length < 1e-3:
                continue

            seg_id = f"corridor_link_{i}"
            segment = RoadSegment(
                segment_id=seg_id,
                name=f"{road_name} Section {i+1}",
                start_node=f"node_{i}",
                end_node=f"node_{i+1}",
                coords_enu=chunk_coords,
                length_m=length,
                heading_deg=heading,
                road_type="primary",
                one_way=one_way,
            )
            network.add_segment(segment)

        return network

