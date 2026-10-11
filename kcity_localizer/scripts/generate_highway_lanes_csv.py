import json
import csv
import os
from pyproj import Proj

def main():
    # Define the sequence of links for each of the 5 lanes
    lane_chains = {
        1: ['A2256W000429', 'A2256W000433', 'A2256W000421', 'A2256W000425'],
        2: ['A2256W000418', 'A2256W000431', 'A2256W000435', 'A2256W000423'],
        3: ['A2256W000410', 'A2256W000430', 'A2256W000434', 'A2256W000422'],
        4: ['A2256W000409', 'A2256W000420', 'A2256W000408', 'A2256W000445'],
        5: ['A2256W000411', 'A2256W000420', 'A2256W000179']
    }

    # Load links database
    print("Loading link set...")
    with open('/home/kuuve/catkin_ws/src/kcity_localizer/data/link_set.json', 'r') as f:
        link_set = json.load(f)

    links_map = {lk['idx']: lk for lk in link_set}

    # Setup Projection (UTM Zone 52N, WGS84)
    proj = Proj(proj='utm', zone=52, ellps='WGS84')
    x_offset = 302595.0
    y_offset = 4124145.0

    output_dir = '/home/kuuve/catkin_ws/src/kcity_localizer/data'

    # Generate CSV files
    for lane_num, chain in lane_chains.items():
        csv_rows = []
        prev_pt = None
        for link_id in chain:
            lk = links_map.get(link_id)
            if not lk:
                print(f"Error: link {link_id} not found!")
                continue
            
            pts = lk['points']
            ego_lane = str(lane_num)
            
            for pt in pts:
                x, y = pt[0], pt[1]
                # Filter out consecutive duplicate points at link boundaries
                if prev_pt and abs(prev_pt[0] - x) < 0.001 and abs(prev_pt[1] - y) < 0.001:
                    continue
                
                utm_e = x + x_offset
                utm_n = y + y_offset
                lon, lat = proj(utm_e, utm_n, inverse=True)
                
                csv_rows.append([
                    f"{lat:.8f}",
                    f"{lon:.8f}",
                    f"{utm_e:.6f}",
                    f"{utm_n:.6f}",
                    f"{x:.6f}",
                    f"{y:.6f}",
                    link_id,
                    ego_lane
                ])
                prev_pt = (x, y)
                
        # Write to CSV
        filename = f"kcity_highway_lane_{lane_num}.csv"
        filepath = os.path.join(output_dir, filename)
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Latitude', 'Longitude', 'UTM_Easting', 'UTM_Northing', 'Local_X', 'Local_Y', 'Link_ID', 'Ego_Lane'])
            writer.writerows(csv_rows)
        print(f"Generated {filepath} with {len(csv_rows)} points.")

if __name__ == '__main__':
    main()
