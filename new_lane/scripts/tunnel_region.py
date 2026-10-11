"""One route-distance definition for M8 perception and obstacle control.
GPS-shadow section bounds remain owned by the router and are never moved.
"""
import csv
import math


def approach_bounds(path, first, control_m=40., prepare_m=60.):
    with open(path, newline='') as stream:
        points = {int(row[0]): (float(row[1]), float(row[2])) for row in csv.reader(stream) if row}
    if first not in points or not 0 <= control_m <= prepare_m:
        raise ValueError('invalid M8 approach route/distance')
    distance = 0.; control = prepare = first
    for idx in range(first-1, -1, -1):
        if idx not in points:
            break
        step = math.hypot(points[idx+1][0]-points[idx][0], points[idx+1][1]-points[idx][1])
        if not math.isfinite(step) or step <= 0:
            raise ValueError('invalid M8 route segment')
        distance += step
        if distance <= control_m + 1e-6: control = idx
        if distance <= prepare_m + 1e-6: prepare = idx
        else: break
    return control, prepare


def read_region(get_param):
    sections = get_param('/section_router/sections', None)
    preview = int(get_param('/tunnel_recovery/perception_prepare_idx', 100))
    if preview < 0: raise ValueError('negative tunnel perception preparation range')
    if sections is None: return (3, 7201, 8850, preview)
    for section in sections:
        if section.get('id') == 'M8':
            first, last = map(int, section['idx'])
            if first > last: raise ValueError('invalid M8 bounds')
            if get_param('/tunnel_recovery/approach_control_enabled', False):
                control, prepare = approach_bounds(get_param('/tunnel_recovery/approach_route_csv', ''), first,
                    float(get_param('/tunnel_recovery/control_prepare_distance_m',40.)),
                    float(get_param('/tunnel_recovery/perception_prepare_distance_m',60.)))
                # Never arm during highway: clamp to the configured M7B boundary.
                start = next((int(s['idx'][0]) for s in sections if s.get('id')=='M7B' and int(s['csv'])==int(section['csv'])), first)
                first, prepare = max(start,control), max(start,prepare)
                preview = first-prepare
            return (int(section['csv']), first, last, preview)
    raise ValueError('section_router has no M8 section')


def contains(region, csv, idx, prepare=False):
    lane, first, last, preview = region
    return csv == lane and first - (preview if prepare else 0) <= idx <= last
