import 'dart:convert';
import 'package:flutter_test/flutter_test.dart';
import 'package:idr_mobile/map/offline_region_store.dart';

void main() {
  group('OfflineRegionStore tests', () {
    test('Parses manifest and GeoJSON correctly', () {
      final manifestJson = json.encode({
        'package_id': 'coventry_test_v1',
        'title': 'Test Region',
        'version': '1.0.0',
        'bounds': [-1.52, 52.39, -1.49, 52.42],
        'center': [-1.505, 52.40],
        'license': {
          'type': 'CC-BY-4.0',
          'attribution': 'ISRO SIH 26168',
        },
      });

      final geojsonContent = json.encode({
        'type': 'FeatureCollection',
        'features': [
          {
            'type': 'Feature',
            'properties': {'id': 'link_1', 'name': 'Highway 1', 'highway': 'primary'},
            'geometry': {
              'type': 'LineString',
              'coordinates': [
                [-1.505, 52.401],
                [-1.504, 52.402],
              ]
            }
          }
        ]
      });

      final store = OfflineRegionStore();
      store.loadFromStrings(manifestJson: manifestJson, geojsonContent: geojsonContent);

      expect(store.isLoaded, isTrue);
      expect(store.manifest?.packageId, 'coventry_test_v1');
      expect(store.roadFeatures.length, 1);
      expect(store.roadFeatures.first.name, 'Highway 1');

      // Point in coverage check
      expect(store.isPointInCoverage(-1.505, 52.40), isTrue);
      expect(store.isPointInCoverage(0.0, 0.0), isFalse);
    });
  });
}

