import json
import csv
import math

def main():
    print("Loading link set...")
    with open('/home/kuuve/catkin_ws/src/kcity_localizer/data/link_set.json', 'r') as f:
        link_set = json.load(f)

    csv_path = '/home/kuuve/catkin_ws/src/kcity_localizer/data/kcity_driving_path_gps_all.csv'
    print(f"Reading CSV from {csv_path}...")
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for r in reader:
            rows.append(r)

    print("Indexing link points...")
    link_points_db = []
    for lk in link_set:
        idx = lk['idx']
        lane = lk.get('ego_lane', '')
        if lane is None:
            lane = ''
        for p in lk['points']:
            link_points_db.append((p[0], p[1], idx, lane))

    print(f"Total link points: {len(link_points_db)}")
    grid = {}
    grid_size = 10.0 # 10 meters grid

    for px, py, idx, lane in link_points_db:
        gx = int(px // grid_size)
        gy = int(py // grid_size)
        grid.setdefault((gx, gy), []).append((px, py, idx, lane))

    def find_nearest(x, y):
        gx = int(x // grid_size)
        gy = int(y // grid_size)
        min_d = float('inf')
        best_idx = ''
        best_lane = ''
        
        # Check 3x3 neighborhood of grids
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                pts = grid.get((gx + dx, gy + dy), [])
                for px, py, idx, lane in pts:
                    d = math.hypot(px - x, py - y)
                    if d < min_d:
                        min_d = d
                        best_idx = idx
                        best_lane = lane
        return best_idx, best_lane, min_d

    print("Matching trajectory points...")
    new_rows = []
    for i, row in enumerate(rows):
        x = float(row[4]) # Local_X
        y = float(row[5]) # Local_Y
        idx, lane, dist = find_nearest(x, y)
        row.append(idx)
        row.append(str(lane))
        if i % 1000 == 0:
            print(f"Processed {i}/{len(rows)}: ({x}, {y}) -> {idx} (lane {lane}, dist {dist:.2f}m)")

    new_header = header + ['Link_ID', 'Ego_Lane']

    print(f"Writing CSV back to {csv_path}...")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(new_header)
        writer.writerows(rows)
    print("Successfully completed!")

if __name__ == '__main__':
    main()
