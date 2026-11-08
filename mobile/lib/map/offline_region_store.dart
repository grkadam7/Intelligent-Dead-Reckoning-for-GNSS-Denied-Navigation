// Offline region manager and vector road network store for IDR-INS.
// Fulfills MOBILE-01 offline vector map requirement.

import 'dart:convert';
import 'package:flutter/services.dart' show rootBundle;

class RoadFeature {
  final String id;
  final String name;
  final String highway;
  final List<List<double>> coordinates; // [[lon, lat], ...]

  const RoadFeature({
    required this.id,
    required this.name,
    required this.highway,
    required this.coordinates,
  });
}

class OfflineRegionManifest {
  final String packageId;
  final String title;
  final String version;
  final List<double> bounds; // [min_lon, min_lat, max_lon, max_lat]
  final List<double> center; // [lon, lat]
  final String licenseType;
  final String attribution;

  const OfflineRegionManifest({
    required this.packageId,
    required this.title,
    required this.version,
    required this.bounds,
    required this.center,
    required this.licenseType,
    required this.attribution,
  });

  factory OfflineRegionManifest.fromJson(Map<String, dynamic> json) {
    final license = json['license'] as Map<String, dynamic>? ?? {};
    final rawBounds = (json['bounds'] as List?)?.map((e) => (e as num).toDouble()).toList() ?? [];
    final rawCenter = (json['center'] as List?)?.map((e) => (e as num).toDouble()).toList() ?? [];

    return OfflineRegionManifest(
      packageId: json['package_id'] as String? ?? 'unknown',
      title: json['title'] as String? ?? 'Offline Map Region',
      version: json['version'] as String? ?? '1.0.0',
      bounds: rawBounds.length == 4 ? rawBounds : [-1.52, 52.39, -1.49, 52.42],
      center: rawCenter.length == 2 ? rawCenter : [-1.50529, 52.40166],
      licenseType: license['type'] as String? ?? 'CC-BY-4.0',
      attribution: license['attribution'] as String? ?? 'ISRO SIH 26168',
    );
  }
}

class OfflineRegionStore {
  OfflineRegionManifest? _manifest;
  final List<RoadFeature> _roadFeatures = [];
  bool _isLoaded = false;

  bool get isLoaded => _isLoaded;
  OfflineRegionManifest? get manifest => _manifest;
  List<RoadFeature> get roadFeatures => List.unmodifiable(_roadFeatures);

  void loadFromStrings({
    required String manifestJson,
    required String geojsonContent,
  }) {
    _roadFeatures.clear();
    final manifestMap = json.decode(manifestJson) as Map<String, dynamic>;
    _manifest = OfflineRegionManifest.fromJson(manifestMap);

    final geo = json.decode(geojsonContent) as Map<String, dynamic>;
    final features = geo['features'] as List? ?? [];

    for (final feat in features) {
      if (feat is Map<String, dynamic>) {
        final props = feat['properties'] as Map<String, dynamic>? ?? {};
        final geom = feat['geometry'] as Map<String, dynamic>? ?? {};
        final coordsRaw = geom['coordinates'] as List? ?? [];

        final List<List<double>> polyline = [];
        for (final pt in coordsRaw) {
          if (pt is List && pt.length >= 2) {
            polyline.add([
              (pt[0] as num).toDouble(),
              (pt[1] as num).toDouble(),
            ]);
          }
        }

        if (polyline.length >= 2) {
          _roadFeatures.add(RoadFeature(
            id: props['id'] as String? ?? feat['id'] as String? ?? 'segment',
            name: props['name'] as String? ?? 'Unnamed Road',
            highway: props['highway'] as String? ?? 'primary',
            coordinates: polyline,
          ));
        }
      }
    }

    _isLoaded = true;
  }

  Future<void> loadAssets({
    String manifestAsset = 'assets/regions/manifest.json',
    String geojsonAsset = 'assets/regions/urban_route.geojson',
  }) async {
    final manifestStr = await rootBundle.loadString(manifestAsset);
    final geojsonStr = await rootBundle.loadString(geojsonAsset);
    loadFromStrings(manifestJson: manifestStr, geojsonContent: geojsonStr);
  }

  bool isPointInCoverage(double lon, double lat) {
    if (_manifest == null) return false;
    final b = _manifest!.bounds;
    return lon >= b[0] && lat >= b[1] && lon <= b[2] && lat <= b[3];
  }
}

