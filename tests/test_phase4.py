"""
Comprehensive Unit and Integration Tests for Phase 4:
Non-Holonomic Constraints (NHC) & Offline Map-Matching (HMM + Vector Road Networks).
SIH Problem Statement ID: 26168 | ISRO
"""

import math
import unittest
import numpy as np

from src.kinematics.nhc import NonHolonomicConstraint, NHCDeadReckoning
from src.map_matching.road_network import RoadSegment, SpatialGridIndex, RoadNetwork
from src.map_matching.hmm_matcher import HMMMapMatcher, CandidateMatch
from src.map_matching.evaluator import evaluate_map_matching
from src.data.iovnbd_loader import generate_synthetic_run


class TestPhase4KinematicsAndNHC(unittest.TestCase):
    """Unit tests for Non-Holonomic Constraints (MAP-01)."""

    def setUp(self):
        self.nhc_hard = NonHolonomicConstraint(lateral_constraint_weight=1.0, vertical_constraint_weight=1.0)
        self.nhc_soft = NonHolonomicConstraint(lateral_constraint_weight=0.8, vertical_constraint_weight=0.9)

    def test_forward_velocity_to_enu_cardinal_directions(self):
        """Test forward velocity conversion to ENU for cardinal headings."""
        v_fwd = 10.0 # m/s

        # Heading North (0 deg / 0 rad) -> [0, 10, 0]
        v_north = NonHolonomicConstraint.forward_velocity_to_enu(v_fwd, 0.0)
        np.testing.assert_allclose(v_north, [0.0, 10.0, 0.0], atol=1e-6)

        # Heading East (90 deg / pi/2 rad) -> [10, 0, 0]
        v_east = NonHolonomicConstraint.forward_velocity_to_enu(v_fwd, np.pi / 2.0)
        np.testing.assert_allclose(v_east, [10.0, 0.0, 0.0], atol=1e-6)

        # Heading South (180 deg / pi rad) -> [0, -10, 0]
        v_south = NonHolonomicConstraint.forward_velocity_to_enu(v_fwd, np.pi)
        np.testing.assert_allclose(v_south, [0.0, -10.0, 0.0], atol=1e-6)

        # Heading West (270 deg / 3pi/2 rad) -> [-10, 0, 0]
        v_west = NonHolonomicConstraint.forward_velocity_to_enu(v_fwd, 3.0 * np.pi / 2.0)
        np.testing.assert_allclose(v_west, [-10.0, 0.0, 0.0], atol=1e-6)

    def test_enu_to_body_velocity_decomposition(self):
        """Test projecting ENU velocity into forward, lateral, and vertical components."""
        # Heading North: velocity [3, 10, 1] -> East is 3 (lateral right), North is 10 (forward), Up is 1
        v_enu = np.array([3.0, 10.0, 1.0])
        heading = 0.0
        v_fwd, v_lat, v_vert = NonHolonomicConstraint.enu_to_body_velocity(v_enu, heading)

        self.assertAlmostEqual(v_fwd, 10.0, places=5)
        self.assertAlmostEqual(v_lat, -3.0, places=5) # -ve because East is right of North
        self.assertAlmostEqual(v_vert, 1.0, places=5)

    def test_hard_nhc_zeros_lateral_and_vertical_velocities(self):
        """Test that hard NHC completely zeros out lateral and vertical velocity slip."""
        v_noisy_enu = np.array([5.0, 15.0, 2.5]) # driving roughly north with unwanted sideways/vertical drift
        heading = 0.0 # North

        v_constrained, raw_lat, raw_vert = self.nhc_hard.apply_nhc(v_noisy_enu, heading)

        # After hard constraint: only forward velocity remains along North
        self.assertAlmostEqual(v_constrained[0], 0.0, places=5) # East velocity eliminated
        self.assertAlmostEqual(v_constrained[1], 15.0, places=5) # North velocity preserved
        self.assertAlmostEqual(v_constrained[2], 0.0, places=5) # Up velocity eliminated
        self.assertAlmostEqual(raw_lat, -5.0, places=5)
        self.assertAlmostEqual(raw_vert, 2.5, places=5)

    def test_soft_nhc_dampens_lateral_slip(self):
        """Test that soft NHC dampens lateral velocity by expected factor."""
        v_enu = np.array([5.0, 10.0, 0.0]) # heading North
        v_constrained, raw_lat, _ = self.nhc_soft.apply_nhc(v_enu, heading_rad=0.0)

        # lateral was 5.0, with weight 0.8: remaining is (1 - 0.8) * 5.0 = 1.0
        self.assertAlmostEqual(abs(v_constrained[0]), 1.0, places=4)
        self.assertAlmostEqual(v_constrained[1], 10.0, places=4)

    def test_nhc_dead_reckoning_engine(self):
        """Test NHCDeadReckoning trajectory propagation on synthetic vehicle data."""
        syn_data = generate_synthetic_run(duration_s=30.0, dt=0.1, seed=42)
        engine = NHCDeadReckoning(sampling_rate=10.0, enable_filtering=False, enable_zupt=True)

        pos, vel, headings_deg, stops = engine.propagate(
            timestamps=syn_data.timestamps_s,
            accel_raw=syn_data.accel,
            gyro_raw=syn_data.gyro,
            initial_enu_pos=syn_data.enu_positions[0],
            initial_heading_deg=syn_data.gps_heading_deg[0],
            ai_forward_speeds=syn_data.gps_speed_mps, # Feed Phase 3 forward speed
        )

        self.assertEqual(len(pos), len(syn_data.timestamps_s))
        self.assertEqual(len(vel), len(syn_data.timestamps_s))
        self.assertEqual(len(headings_deg), len(syn_data.timestamps_s))
        # Verify vertical velocity is constrained to zero
        np.testing.assert_allclose(vel[:, 2], 0.0, atol=1e-5)


class TestPhase4RoadNetworkAndSpatialIndex(unittest.TestCase):
    """Unit tests for Vector Road Network Representation and Spatial Indexing (MAP-02)."""

    def setUp(self):
        # Create a 3x3 synthetic urban grid (200m blocks)
        self.network = RoadNetwork.create_synthetic_urban_grid(
            grid_rows=3,
            grid_cols=3,
            block_size_m=200.0,
        )

    def test_grid_network_generation(self):
        """Verify generation of nodes, segments, and adjacency lists."""
        self.assertEqual(len(self.network.nodes), 9) # 3x3 grid
        # 3 rows * 2 horizontal streets = 6 segments
        # 3 cols * 2 vertical avenues = 6 segments
        self.assertEqual(len(self.network.segments), 12)
        self.assertTrue(len(self.network.adj_list) > 0)

    def test_point_to_segment_projection_orthogonal(self):
        """Test projecting a point onto a road segment polyline."""
        # Street from (0, 0) to (200, 0) heading East (90 deg)
        seg = self.network.segments["street_0_0_to_1"]
        # Query point at (50, 10) -> 10m North of road centerline
        query_pt = np.array([50.0, 10.0])

        proj_pt, perp_dist, frac, heading = seg.project_point(query_pt)

        np.testing.assert_allclose(proj_pt, [50.0, 0.0], atol=1e-5)
        self.assertAlmostEqual(perp_dist, 10.0, places=5)
        self.assertAlmostEqual(frac, 50.0 / 200.0, places=5)
        self.assertAlmostEqual(heading, 90.0, places=5)

    def test_point_to_segment_projection_clamping(self):
        """Test that projection clamps to segment endpoints if query point is beyond."""
        seg = self.network.segments["street_0_0_to_1"] # (0, 0) to (200, 0)
        # Point before start: (-20, 0) -> should clamp to (0, 0)
        proj_start, dist_start, frac_start, _ = seg.project_point(np.array([-20.0, 0.0]))
        np.testing.assert_allclose(proj_start, [0.0, 0.0], atol=1e-5)
        self.assertAlmostEqual(dist_start, 20.0, places=5)
        self.assertAlmostEqual(frac_start, 0.0, places=5)

        # Point after end: (250, 0) -> should clamp to (200, 0)
        proj_end, dist_end, frac_end, _ = seg.project_point(np.array([250.0, 0.0]))
        np.testing.assert_allclose(proj_end, [200.0, 0.0], atol=1e-5)
        self.assertAlmostEqual(dist_end, 50.0, places=5)
        self.assertAlmostEqual(frac_end, 1.0, places=5)

    def test_spatial_grid_index_query(self):
        """Test that spatial index retrieves candidate segments within radius."""
        index = SpatialGridIndex(cell_size_m=50.0)
        for seg in self.network.segments.values():
            index.insert(seg)

        # Query point near origin (10, 5)
        cands = index.query(np.array([10.0, 5.0]), radius_m=30.0)
        cand_ids = [s.segment_id for s in cands]

        # Should find street_0_0_to_1 and avenue_0_0_to_1
        self.assertIn("street_0_0_to_1", cand_ids)
        self.assertIn("avenue_0_0_to_1", cand_ids)
        # Should not include distant segments like street_2_1_to_2 (at North 400m)
        self.assertNotIn("street_2_1_to_2", cand_ids)

    def test_dijkstra_shortest_path_distance(self):
        """Test shortest network path calculation between segments."""
        # Distance on the same segment: from fraction 0.1 to 0.7 on 200m segment = 120m
        d_same = self.network.shortest_path_distance("street_0_0_to_1", "street_0_0_to_1", 0.1, 0.7)
        self.assertAlmostEqual(d_same, 120.0, places=3)

        # Connected corner turn:
        # street_0_0_to_1 (from fraction 0.5 = at E:100, N:0)
        # turn at (200, 0) onto avenue_1_0_to_1 (to fraction 0.5 = at E:200, N:100)
        # Remaining on street: 100m. Distance along avenue: 100m. Total = 200m.
        d_turn = self.network.shortest_path_distance(
            "street_0_0_to_1",
            "avenue_1_0_to_1",
            from_fraction=0.5,
            to_fraction=0.5
        )
        self.assertAlmostEqual(d_turn, 200.0, places=3)

    def test_geojson_parser(self):
        """Test parsing GeoJSON FeatureCollection into RoadNetwork."""
        geojson_data = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"id": "link_101", "name": "Main Blvd", "highway": "primary", "oneway": False},
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [
                            [-1.50529, 52.40166],
                            [-1.50382, 52.40166],
                            [-1.50235, 52.40166]
                        ]
                    }
                }
            ]
        }
        net = RoadNetwork.from_geojson(geojson_data)
        self.assertEqual(len(net.segments), 1)
        seg = net.segments["link_101"]
        self.assertEqual(seg.name, "Main Blvd")
        self.assertAlmostEqual(seg.heading_deg, 90.0, places=0) # Heading East


class TestPhase4HMMMapMatcher(unittest.TestCase):
    """Unit tests for Hidden Markov Model (HMM) Map Matcher (MAP-03)."""

    def setUp(self):
        self.network = RoadNetwork.create_synthetic_urban_grid(
            grid_rows=3,
            grid_cols=3,
            block_size_m=200.0,
        )
        self.matcher = HMMMapMatcher(
            road_network=self.network,
            search_radius_m=50.0,
            gps_measurement_std_m=5.0,
            heading_std_deg=20.0,
            beta_transition=10.0,
        )

    def test_emission_log_probability_decreases_with_distance(self):
        """Test that emission probability drops as perpendicular distance increases."""
        p_close = self.matcher._emission_log_prob(perp_dist=2.0, heading_diff=0.0)
        p_far = self.matcher._emission_log_prob(perp_dist=15.0, heading_diff=0.0)
        self.assertGreater(p_close, p_far)

    def test_emission_log_probability_decreases_with_heading_diff(self):
        """Test that emission probability penalizes perpendicular/opposite headings."""
        p_aligned = self.matcher._emission_log_prob(perp_dist=5.0, heading_diff=5.0)
        p_misaligned = self.matcher._emission_log_prob(perp_dist=5.0, heading_diff=80.0)
        self.assertGreater(p_aligned, p_misaligned)

    def test_transition_probability_penalizes_path_discrepancy(self):
        """Test that transition probability is highest when network distance matches Euclidean distance."""
        p_match = self.matcher._transition_log_prob(d_euc=20.0, d_net=20.5)
        p_detour = self.matcher._transition_log_prob(d_euc=20.0, d_net=150.0)
        self.assertGreater(p_match, p_detour)

    def test_viterbi_snaps_noisy_trajectory_to_road(self):
        """Test that Viterbi decoding snaps a noisy trajectory back to the road centerline."""
        # True trajectory along Street 0: East from (10, 0) to (190, 0)
        true_e = np.linspace(10.0, 190.0, 20)
        true_n = np.zeros(20)

        # Add random lateral noise up to +/- 12 meters
        rng = np.random.RandomState(42)
        noisy_n = true_n + rng.uniform(-12.0, 12.0, size=20)
        noisy_positions = np.column_stack([true_e, noisy_n, np.zeros(20)])
        headings = np.full(20, 90.0) # Heading East

        matched = self.matcher.match_trajectory(noisy_positions, headings)

        self.assertEqual(len(matched), 20)
        for pt in matched:
            self.assertEqual(pt.matched_segment_id, "street_0_0_to_1")
            # North coordinate should snap exactly to 0.0 (the road line)
            self.assertAlmostEqual(pt.matched_enu[1], 0.0, places=4)

        metrics = evaluate_map_matching(
            matched_points=matched,
            ground_truth_enu=np.column_stack([true_e, true_n, np.zeros(20)]),
            unmatched_enu=noisy_positions
        )
        self.assertLess(metrics.mean_cross_track_error_m, 15.0)
        self.assertGreater(metrics.coverage_percentage, 95.0)

    def test_parallel_road_disambiguation_via_heading(self):
        """Test that heading penalty prevents matching onto an intersecting/orthogonal street."""
        # Vehicle moving North along Avenue 0 (E:0, N: 10..150)
        pos = np.column_stack([np.full(15, 5.0), np.linspace(10.0, 150.0, 15), np.zeros(15)])
        headings = np.full(15, 0.0) # Heading North

        matched = self.matcher.match_trajectory(pos, headings)
        # Even near intersection with Street 0 (at N=0) and Street 1 (at N=200),
        # vehicle should match Avenue 0 because heading is North (0 deg)
        for pt in matched:
            self.assertIn("avenue_0", pt.matched_segment_id)

    def test_online_streaming_snap(self):
        """Test instantaneous streaming snap for 10 Hz navigation HUD."""
        point = np.array([55.0, 8.0, 0.0]) # 8m lateral offset from street_0_0_to_1
        heading = 90.0 # East

        snapped, seg_id, cte = self.matcher.snap_online_point(point, heading)
        self.assertEqual(seg_id, "street_0_0_to_1")
        self.assertAlmostEqual(snapped[1], 0.0, places=4)
        self.assertAlmostEqual(cte, 8.0, places=4)


if __name__ == "__main__":
    unittest.main()
