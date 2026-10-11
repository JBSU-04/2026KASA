"""Per-mission post-ground-removal profiles. No ground model parameters here."""
import math

FILTER_KEYS = (
    'roi_x_min', 'roi_x_max', 'roi_y_min', 'roi_y_max',
    'cluster_eps', 'cluster_max_points', 'min_points_for_confirm', 'cluster_voxel_m',
    'cluster_max_span_y', 'cluster_max_span_x',
    'merge_cluster_dist', 'near_merge_dist',
    'path_corridor_gather_half_width', 'collision_corridor_half_width',
    'min_obstacle_elevation', 'min_obstacle_elevation_dynamic',
    'min_obstacle_elevation_outside', 'curb_linear_min_length',
    'curb_linear_max_width', 'curb_max_aspect', 'low_obstacle_max_length',
)


def validate_profiles(profiles):
    for name, values in profiles.items():
        for key, value in values.items():
            if key not in FILTER_KEYS:
                raise ValueError('Unknown mission filter %s/%s' % (name, key))
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Non-finite mission filter %s/%s' % (name, key))
            if key not in ('roi_x_min', 'roi_y_min', 'roi_y_max', 'cluster_voxel_m') and value <= 0:
                raise ValueError('Non-positive mission filter %s/%s' % (name, key))
            if key == 'cluster_voxel_m' and value < 0:
                raise ValueError('Negative voxel size for %s' % name)
        if values.get('roi_y_min', -1) >= values.get('roi_y_max', 1):
            raise ValueError('Invalid lateral ROI for %s' % name)
    return profiles


def resolve_profile(section_id):
    return {'M2': 'static', 'M3': 'dynamic', 'M8': 'tunnel'}.get(section_id)
