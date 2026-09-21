"""
Hidden Markov Model (HMM) Map Matching Engine.
SIH Problem Statement ID: 26168 | ISRO

Implements MAP-03:
Applies the Newson & Krumm (2009) HMM algorithm to bind drifting dead-reckoned
coordinates to the most likely topological road links on vector networks.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple, Union
import numpy as np

from src.map_matching.road_network import RoadNetwork, RoadSegment


@dataclass
class CandidateMatch:
    """Represents a candidate projection on a road link."""
    segment: RoadSegment
    projected_enu: np.ndarray        # [East, North] in meters
    perpendicular_distance_m: float  # Euclidean distance from observation
    fraction_along_link: float       # Position along link [0.0, 1.0]
    segment_heading_deg: float       # Direction of the road at projection point
    heading_diff_deg: float          # Angular deviation from vehicle heading
    emission_log_prob: float         # Log probability of observation given state


@dataclass
class MapMatchedPoint:
    """Output matched point at a specific epoch."""
    timestamp_s: float
    original_enu: np.ndarray         # [East, North, Up]
    matched_enu: np.ndarray          # [East, North, Up] snapped to road link
    matched_segment_id: str
    matched_road_name: str
    road_type: str
    cross_track_error_m: float       # Distance between original and matched point
    heading_deg: float


class HMMMapMatcher:
    """
    Hidden Markov Model map-matching algorithm based on Newson & Krumm (2009).
    
    States: Candidate road link points.
    Observations: Drifting dead-reckoning / GNSS trajectory points.
    Emission Probability: Gaussian error model with angular heading consistency.
    Transition Probability: Exponential distribution of network path vs Euclidean path distance.
    Decoding: Dynamic programming (Viterbi) for global maximum-likelihood path.
    """

    def __init__(
        self,
        road_network: RoadNetwork,
        search_radius_m: float = 40.0,
        gps_measurement_std_m: float = 8.0,
        heading_std_deg: float = 25.0,
        beta_transition: float = 15.0,
        enable_heading_penalty: bool = True,
    ):
        """
        Args:
            road_network: Indexed RoadNetwork graph.
            search_radius_m: Maximum candidate road query radius around each DR point.
            gps_measurement_std_m: Expected sensor measurement error std (sigma_z).
            heading_std_deg: Expected heading deviation std (sigma_theta).
            beta_transition: Scale parameter for transition probability exponential distribution.
            enable_heading_penalty: Incorporate heading consistency in emission probabilities.
        """
        self.network = road_network
        self.search_radius = search_radius_m
        self.sigma_z = gps_measurement_std_m
        self.sigma_theta = heading_std_deg
        self.beta = beta_transition
        self.enable_heading_penalty = enable_heading_penalty

    def _compute_heading_diff(self, vehicle_heading: float, segment_heading: float, one_way: bool) -> float:
        """Computes smallest angular deviation between vehicle heading and road direction."""
        diff1 = abs((vehicle_heading - segment_heading + 180.0) % 360.0 - 180.0)
        if not one_way:
            # Check opposite direction for two-way roads
            opp_heading = (segment_heading + 180.0) % 360.0
            diff2 = abs((vehicle_heading - opp_heading + 180.0) % 360.0 - 180.0)
            return min(diff1, diff2)
        return diff1

    def _emission_log_prob(self, perp_dist: float, heading_diff: float) -> float:
        """
        Computes log emission probability:
        log p(z | r) = - (d^2 / (2 * sigma_z^2)) - (delta_theta^2 / (2 * sigma_theta^2))
        """
        # Distance likelihood (Gaussian)
        log_p_dist = -0.5 * (perp_dist / self.sigma_z) ** 2 - math.log(math.sqrt(2.0 * math.pi) * self.sigma_z)

        # Heading likelihood (Gaussian)
        if self.enable_heading_penalty:
            log_p_head = -0.5 * (heading_diff / self.sigma_theta) ** 2
        else:
            log_p_head = 0.0

        return log_p_dist + log_p_head

    def _transition_log_prob(self, d_euc: float, d_net: float) -> float:
        """
        Computes log transition probability:
        log p(r_t | r_t-1) = - (|d_net - d_euc| / beta) - log(beta)
        """
        diff = abs(d_net - d_euc)
        return -(diff / self.beta) - math.log(self.beta)

    def find_candidates(self, point_enu: np.ndarray, vehicle_heading_deg: float) -> List[CandidateMatch]:
        """Finds and evaluates candidate road matches for an observation point."""
        raw_candidates = self.network.find_nearest_segments(
            point_enu=point_enu,
            radius_m=self.search_radius,
            max_candidates=8
        )

        candidates: List[CandidateMatch] = []
        for seg, proj_pt, perp_dist, frac, seg_heading in raw_candidates:
            h_diff = self._compute_heading_diff(vehicle_heading_deg, seg_heading, seg.one_way)
            log_prob = self._emission_log_prob(perp_dist, h_diff)

            candidates.append(CandidateMatch(
                segment=seg,
                projected_enu=proj_pt,
                perpendicular_distance_m=perp_dist,
                fraction_along_link=frac,
                segment_heading_deg=seg_heading,
                heading_diff_deg=h_diff,
                emission_log_prob=log_prob,
            ))

        return candidates

    def match_trajectory(
        self,
        positions_enu: np.ndarray,
        headings_deg: np.ndarray,
        timestamps_s: Optional[np.ndarray] = None,
    ) -> List[MapMatchedPoint]:
        """
        Executes full Viterbi HMM map matching across a trajectory.
        
        Args:
            positions_enu: (N, 3) or (N, 2) dead-reckoned trajectory positions in ENU meters.
            headings_deg: (N,) vehicle heading azimuth in degrees.
            timestamps_s: Optional (N,) timestamps.
            
        Returns:
            List of MapMatchedPoint objects with snapped coordinates and link assignments.
        """
        n_samples = len(positions_enu)
        if n_samples == 0:
            return []

        if timestamps_s is None:
            timestamps_s = np.arange(n_samples, dtype=np.float64) * 0.1

        # 1. Candidate extraction for all epochs
        candidates_by_epoch: List[List[CandidateMatch]] = []
        for t in range(n_samples):
            pos_t = positions_enu[t, :2]
            head_t = float(headings_deg[t])
            cands = self.find_candidates(pos_t, head_t)
            
            # If no candidate found within radius, fallback to closest segment in network
            if not cands and len(self.network.segments) > 0:
                fallback_cands = self.network.find_nearest_segments(pos_t, radius_m=200.0, max_candidates=1)
                for seg, proj_pt, perp_dist, frac, seg_heading in fallback_cands:
                    h_diff = self._compute_heading_diff(head_t, seg_heading, seg.one_way)
                    log_prob = self._emission_log_prob(perp_dist, h_diff)
                    cands.append(CandidateMatch(
                        segment=seg,
                        projected_enu=proj_pt,
                        perpendicular_distance_m=perp_dist,
                        fraction_along_link=frac,
                        segment_heading_deg=seg_heading,
                        heading_diff_deg=h_diff,
                        emission_log_prob=log_prob,
                    ))
            candidates_by_epoch.append(cands)

        # 2. Viterbi Forward Trellis
        # viterbi_probs[t][i]: max log probability of path ending at candidate i of epoch t
        # backpointers[t][i]: index of candidate at epoch t-1 on the best path
        viterbi_probs: List[np.ndarray] = []
        backpointers: List[List[int]] = []

        # Initialization at t = 0
        c0 = candidates_by_epoch[0]
        if c0:
            init_probs = np.array([c.emission_log_prob for c in c0], dtype=np.float64)
        else:
            init_probs = np.array([0.0])
        viterbi_probs.append(init_probs)
        backpointers.append([])

        # Forward propagation t = 1 ... N-1
        for t in range(1, n_samples):
            prev_cands = candidates_by_epoch[t - 1]
            curr_cands = candidates_by_epoch[t]
            prev_probs = viterbi_probs[t - 1]

            if not curr_cands:
                # No candidates at this epoch, carry over dummy
                viterbi_probs.append(np.array([prev_probs.max()]))
                backpointers.append([int(np.argmax(prev_probs))])
                continue

            if not prev_cands:
                # Fresh start
                curr_probs = np.array([c.emission_log_prob for c in curr_cands], dtype=np.float64)
                viterbi_probs.append(curr_probs)
                backpointers.append([0] * len(curr_cands))
                continue

            d_euc = float(np.linalg.norm(positions_enu[t, :2] - positions_enu[t - 1, :2]))

            curr_probs = np.zeros(len(curr_cands), dtype=np.float64)
            curr_backpointers: List[int] = []

            for j, c_curr in enumerate(curr_cands):
                best_prob = -float("inf")
                best_prev_idx = 0

                for i, c_prev in enumerate(prev_cands):
                    # Network shortest path distance between road links
                    d_net = self.network.shortest_path_distance(
                        c_prev.segment.segment_id,
                        c_curr.segment.segment_id,
                        c_prev.fraction_along_link,
                        c_curr.fraction_along_link,
                    )
                    log_trans = self._transition_log_prob(d_euc, d_net)
                    total_prob = prev_probs[i] + log_trans + c_curr.emission_log_prob

                    if total_prob > best_prob:
                        best_prob = total_prob
                        best_prev_idx = i

                curr_probs[j] = best_prob
                curr_backpointers.append(best_prev_idx)

            viterbi_probs.append(curr_probs)
            backpointers.append(curr_backpointers)

        # 3. Viterbi Backtracking
        best_path_indices: List[int] = [0] * n_samples
        last_probs = viterbi_probs[-1]
        best_path_indices[-1] = int(np.argmax(last_probs)) if len(last_probs) > 0 else 0

        for t in range(n_samples - 1, 0, -1):
            curr_idx = best_path_indices[t]
            if backpointers[t] and curr_idx < len(backpointers[t]):
                best_path_indices[t - 1] = backpointers[t][curr_idx]
            else:
                best_path_indices[t - 1] = 0

        # 4. Construct Output Matched Points
        matched_points: List[MapMatchedPoint] = []
        for t in range(n_samples):
            cands = candidates_by_epoch[t]
            orig_pos = positions_enu[t]
            up_coord = float(orig_pos[2]) if len(orig_pos) > 2 else 0.0

            if cands:
                cand_idx = min(best_path_indices[t], len(cands) - 1)
                chosen = cands[cand_idx]
                matched_enu = np.array([chosen.projected_enu[0], chosen.projected_enu[1], up_coord])
                seg_id = chosen.segment.segment_id
                seg_name = chosen.segment.name
                r_type = chosen.segment.road_type
                cte = chosen.perpendicular_distance_m
                head = chosen.segment_heading_deg
            else:
                # Direct fallback to raw position
                matched_enu = orig_pos.copy()
                seg_id = "unmatched"
                seg_name = "Unmatched Off-Road"
                r_type = "none"
                cte = 0.0
                head = float(headings_deg[t])

            matched_points.append(MapMatchedPoint(
                timestamp_s=float(timestamps_s[t]),
                original_enu=orig_pos,
                matched_enu=matched_enu,
                matched_segment_id=seg_id,
                matched_road_name=seg_name,
                road_type=r_type,
                cross_track_error_m=cte,
                heading_deg=head,
            ))

        return matched_points

    def snap_online_point(
        self,
        point_enu: np.ndarray,
        vehicle_heading_deg: float,
        last_matched_segment_id: Optional[str] = None
    ) -> Tuple[np.ndarray, str, float]:
        """
        Instantaneous streaming snap for real-time mobile HUD display (10 Hz).
        
        Args:
            point_enu: (2,) or (3,) current dead-reckoning position in ENU.
            vehicle_heading_deg: Heading azimuth in degrees.
            last_matched_segment_id: ID of previously matched segment to preserve link continuity.
            
        Returns:
            (snapped_enu (3,), matched_segment_id, cross_track_error_m)
        """
        cands = self.find_candidates(point_enu, vehicle_heading_deg)
        up_val = float(point_enu[2]) if len(point_enu) > 2 else 0.0

        if not cands:
            return np.array([point_enu[0], point_enu[1], up_val]), "none", 0.0

        # Score candidates with continuity bonus for current active segment
        best_cand = cands[0]
        best_score = -float("inf")

        for c in cands:
            score = c.emission_log_prob
            if last_matched_segment_id is not None and c.segment.segment_id == last_matched_segment_id:
                # Continuity bias: avoid flickering between parallel roads
                score += 2.0
            if score > best_score:
                best_score = score
                best_cand = c

        snapped = np.array([best_cand.projected_enu[0], best_cand.projected_enu[1], up_val])
        return snapped, best_cand.segment.segment_id, best_cand.perpendicular_distance_m
