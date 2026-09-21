"""Pure BT lawnmower geometry; usable without ROS or importing BT runtime code.

Keep the lane count, centre and zigzag rules identical to
the scenario's ``bt_nodes._lawnmower``. The config helper reads the
existing BT configuration, including each drone's search-altitude override.
"""


def lawnmower(zone, spacing, lane_axis='y', altitude=0.0):
    """Return (x, y, z) waypoints at an explicit altitude (default: ground plane)."""
    x0, x1 = zone['x']
    y0, y1 = zone['y']
    z = float(altitude)
    if lane_axis == 'y':
        lane_lo, lane_hi, run_lo, run_hi = y0, y1, x0, x1
    else:
        lane_lo, lane_hi, run_lo, run_hi = x0, x1, y0, y1
    n_lanes = max(1, int(round(abs(lane_hi - lane_lo) / float(spacing))))
    points = []
    for i in range(n_lanes):
        lane_c = lane_lo + float(spacing) * (i + 0.5)
        runs = (run_lo, run_hi) if i % 2 == 0 else (run_hi, run_lo)
        for run in runs:
            points.append((float(run), float(lane_c), z) if lane_axis == 'y'
                          else (float(lane_c), float(run), z))
    return points


def lanes_from_config(bt_config):
    """Return searcher-name -> waypoints from a complete parsed BT YAML dict."""
    config = bt_config['coshow']
    search = config['search']
    lanes = {}
    for name in config['searchers']:
        altitudes = config['drones'].get(name, {}).get('altitudes', {})
        altitude = altitudes.get('search', config['altitudes']['search'])
        lanes[name] = lawnmower(search['zones'][name], search['lane_spacing'],
                               search.get('lane_axis', 'y'), altitude)
    return lanes
