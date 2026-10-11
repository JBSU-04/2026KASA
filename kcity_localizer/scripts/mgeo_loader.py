#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K-City 2025 mgeo(node_set / link_set / global_info) 로더.

morai_mgeo lib 에 의존하지 않고 추출된 raw JSON(list 포맷, maj_ver3/min_ver0)을
직접 읽어, 링크 폴리라인 위 모든 점에 대한 KD-tree 를 만들어
"임의 좌표 -> 가장 가까운 링크/노드" 매칭을 제공한다.

좌표계: global_info.local_origin_in_global 기준의 로컬 좌표(=UTM - origin).
        MORAI /gps 의 (east/north)Offset 을 빼서 얻는 좌표와 동일 프레임.
"""
import os
import json
import math

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover
    cKDTree = None


DEFAULT_DATA_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data"
)


class Link(object):
    __slots__ = ("idx", "from_node", "to_node", "points", "max_speed",
                 "length", "left_dst", "right_dst")

    def __init__(self, d):
        self.idx = d.get("idx")
        self.from_node = d.get("from_node_idx")
        self.to_node = d.get("to_node_idx")
        # points: [[x,y,z], ...] 로컬좌표 (xy 만 사용)
        self.points = [(float(p[0]), float(p[1])) for p in d.get("points", [])]
        self.max_speed = d.get("max_speed", 0)
        self.length = d.get("link_length", None)
        self.left_dst = d.get("left_lane_change_dst_link_idx")
        self.right_dst = d.get("right_lane_change_dst_link_idx")


class MGeoMap(object):
    def __init__(self, data_dir=None):
        self.data_dir = data_dir or DEFAULT_DATA_DIR
        self.origin = [0.0, 0.0, 0.0]
        self.coord_system = "UTM52N"
        self.nodes = {}          # idx -> (x, y)
        self.node_stop = {}      # idx -> bool (MGeo on_stop_line)
        self.links = {}          # idx -> Link
        self._kd = None
        self._pt_xy = None       # (N,2)
        self._pt_link = []       # link idx per point
        self.succ = {}           # link idx -> [다음 link idx] (to_node 공유)
        self.tl_by_link = {}     # link idx -> [traffic light dict] (car/bus만)
        self.stop_by_link = {}   # link idx -> (stop_s, stop_x, stop_y, from_stopline)
        self._load()
        self._build_index()
        self._build_graph()
        self._load_traffic_stops()

    # ---------------------------------------------------------------- load
    def _path(self, name):
        return os.path.join(self.data_dir, name)

    def _load(self):
        with open(self._path("global_info.json"), "r") as f:
            g = json.load(f)
        self.origin = g.get("local_origin_in_global", [0.0, 0.0, 0.0])
        self.coord_system = g.get("global_coordinate_system", "UTM52N")

        with open(self._path("node_set.json"), "r") as f:
            nodes = json.load(f)
        for n in nodes:
            p = n.get("point", [0, 0, 0])
            self.nodes[n["idx"]] = (float(p[0]), float(p[1]))
            self.node_stop[n["idx"]] = bool(n.get("on_stop_line", False))

        with open(self._path("link_set.json"), "r") as f:
            links = json.load(f)
        self.n_synthetic = 0
        for d in links:
            if self._is_synthetic(d):
                self.n_synthetic += 1
                continue
            lk = Link(d)
            if len(lk.points) >= 2:
                self.links[lk.idx] = lk

    @staticmethod
    def _is_synthetic(d):
        """차선변경 합성 스텁 링크인가?

        K-City 2025 link_set.json 895개 중 261개는 실제 도로 기하가 아니라
        "출발노드-도착노드" 두 점만 잇는 차선변경 경로 스텁이다(idx 가
        'A2256W000543-A2256W000546' 처럼 노드쌍 결합형, link_type=None,
        max_speed=0). 이들의 직선 현(chord)은 최대 829m 로 지도를 가로질러,
        KD-tree 최근접 '정점' 매칭에 섞이면 진짜 링크보다 가깝게 잡혀 엉뚱한
        링크/s/lateral 을 경고 없이 돌려준다.

        실측(2026-08-05): 희소 구간 4058개 질의점 중 66.8% 가 오매칭.
        이 필터 적용 후 진짜 링크 634개만 남기면 오매칭 0.00%(2385점).
        진짜 링크의 정점 간격은 예외 없이 0.5m 라 densify 도 불필요하다.
        """
        return d.get("link_type") is None

    # --------------------------------------------------------------- index
    def _build_index(self):
        xs, ys, owner = [], [], []
        for idx, lk in self.links.items():
            for (x, y) in lk.points:
                xs.append(x)
                ys.append(y)
                owner.append(idx)
        self._pt_link = owner
        if np is not None:
            self._pt_xy = np.column_stack([xs, ys]).astype(float)
            if cKDTree is not None and len(xs) > 0:
                self._kd = cKDTree(self._pt_xy)
        else:
            self._pt_xy = list(zip(xs, ys))

    # ----------------------------------------------------- traffic / stops
    @staticmethod
    def _polyline_len(pts):
        return sum(math.hypot(pts[k + 1][0] - pts[k][0],
                              pts[k + 1][1] - pts[k][1])
                   for k in range(len(pts) - 1))

    def _build_graph(self):
        """to_node -> from_node 연결로 링크 후속(successor) 그래프 구성."""
        by_from = {}
        for idx, lk in self.links.items():
            if lk.from_node:
                by_from.setdefault(lk.from_node, []).append(idx)
        for idx, lk in self.links.items():
            self.succ[idx] = by_from.get(lk.to_node, []) if lk.to_node else []

    def _load_traffic_stops(self):
        """traffic_light_set(차량용) + lane_boundary_set(530 정지선) 로딩.

        신호등이 제어하는 각 링크에 대해 정지점(stop_s)을 계산:
          링크 끝점 반경 15m 내 정지선(530)이 있으면 그 정지선의 최근접점을
          링크에 투영한 s, 없으면 링크 끝점.
        """
        def _load(name):
            path = self._path(name)
            if not os.path.exists(path):
                return []
            try:
                with open(path, "r") as f:
                    return json.load(f)
            except Exception:
                return []

        tls = [t for t in _load("traffic_light_set.json")
               if t.get("type") in ("car", "bus")]
        stoplines = [b for b in _load("lane_boundary_set.json")
                     if 530 in (b.get("lane_type") or [])]

        for t in tls:
            for lid in t.get("link_id_list") or []:
                if lid in self.links:
                    self.tl_by_link.setdefault(lid, []).append(t)

        for lid in self.tl_by_link:
            lk = self.links[lid]
            # In K-City, traffic-light link_id_list commonly points to the
            # link *after* the stop line.  Its from_node is therefore the
            # actual stop-line node.  The old endpoint assumption placed the
            # stop target tens of metres inside the intersection.
            if lk.from_node and self.node_stop.get(lk.from_node, False):
                sx, sy = lk.points[0]
                self.stop_by_link[lid] = (0.0, sx, sy, True)
                continue
            if lk.to_node and self.node_stop.get(lk.to_node, False):
                sx, sy = lk.points[-1]
                self.stop_by_link[lid] = (self._polyline_len(lk.points),
                                          sx, sy, True)
                continue
            ex, ey = lk.points[-1]
            # 링크 끝 15m 내에서 가장 가까운 정지선 점 탐색
            best = None
            for sl in stoplines:
                for p in sl.get("points") or []:
                    d = math.hypot(p[0] - ex, p[1] - ey)
                    if d < 15.0 and (best is None or d < best[0]):
                        best = (d, float(p[0]), float(p[1]))
            if best is not None:
                proj = self._project_on_link(lk, best[1], best[2])
                self.stop_by_link[lid] = (proj["s"], proj["proj"][0],
                                          proj["proj"][1], True)
            else:
                self.stop_by_link[lid] = (self._polyline_len(lk.points),
                                          ex, ey, False)

    def stop_ahead(self, link_idx, s, lookahead=80.0, max_depth=6):
        """현재 링크/진행거리 기준 전방 lookahead[m] 내 신호등 정지점 탐색.

        반환: {dist, stop_x, stop_y, tl_idx, tl_point, link_idx} 또는 None.
        BFS로 후속 링크를 누적거리와 함께 탐색, 가장 가까운 정지점 반환.
        """
        if link_idx not in self.links:
            return None
        best = None
        visited = {link_idx}
        # (링크, 링크 시작까지의 누적거리, 깊이)
        queue = [(link_idx, -s, 0)]
        while queue:
            lid, base, depth = queue.pop(0)
            lk = self.links[lid]
            if lid in self.stop_by_link:
                stop_s, sx, sy, _ = self.stop_by_link[lid]
                dist = base + stop_s
                if -2.0 < dist <= lookahead and (best is None or dist < best["dist"]):
                    tl = self.tl_by_link[lid][0]
                    best = {"dist": max(dist, 0.0), "stop_x": sx, "stop_y": sy,
                            "tl_idx": tl.get("idx", ""),
                            "tl_point": tl.get("point", [0, 0, 0]),
                            "link_idx": lid}
            length = lk.length or self._polyline_len(lk.points)
            nxt_base = base + length
            if nxt_base < lookahead and depth < max_depth:
                for nid in self.succ.get(lid, []):
                    if nid not in visited:
                        visited.add(nid)
                        queue.append((nid, nxt_base, depth + 1))
        return best

    def stop_ahead_route(self, route_links, route_pos, s, lookahead=80.0):
        """Find the next stop only along the published global route."""
        if not route_links or route_pos < 0 or route_pos >= len(route_links):
            return None
        base = -float(s)
        for pos in range(route_pos, len(route_links)):
            lid = route_links[pos]
            lk = self.links.get(lid)
            if lk is None:
                continue
            if lid in self.stop_by_link:
                stop_s, sx, sy, _ = self.stop_by_link[lid]
                dist = base + stop_s
                if -2.0 < dist <= lookahead:
                    tl = self.tl_by_link[lid][0]
                    return {"dist": max(dist, 0.0),
                            "stop_x": sx, "stop_y": sy,
                            "tl_idx": tl.get("idx", ""),
                            "tl_point": tl.get("point", [0, 0, 0]),
                            "link_idx": lid}
            base += lk.length or self._polyline_len(lk.points)
            if base > lookahead:
                break
        return None

    def match_on_links(self, x, y, link_ids):
        """Project a point onto the nearest link in a restricted link list."""
        best = None
        for lid in link_ids:
            lk = self.links.get(lid)
            if lk is None:
                continue
            m = self._project_on_link(lk, x, y)
            if m is not None and (best is None or m["dist"] < best["dist"]):
                best = m
        return best

    # --------------------------------------------------------------- query
    def nearest_point_index(self, x, y):
        """가장 가까운 링크-폴리라인 정점의 (전역 점 인덱스, 거리)."""
        if self._kd is not None:
            dist, i = self._kd.query([x, y])
            return int(i), float(dist)
        if np is not None:
            d2 = (self._pt_xy[:, 0] - x) ** 2 + (self._pt_xy[:, 1] - y) ** 2
            i = int(d2.argmin())
            return i, math.sqrt(float(d2[i]))
        # 순수 파이썬 폴백
        best_i, best_d2 = -1, float("inf")
        for i, (px, py) in enumerate(self._pt_xy):
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return best_i, math.sqrt(best_d2)

    def match(self, x, y):
        """
        (x,y) 로컬좌표를 가장 가까운 링크에 정합.

        반환 dict:
          link_idx, dist(정점까지 거리), proj(링크 위 투영점 x,y),
          lateral(부호 없는 가로 오프셋), heading(투영 세그먼트 진행방향 yaw),
          s(링크 시작부터 투영점까지 누적거리), max_speed
        매칭 실패 시 None.
        """
        if self._kd is not None:
            k = min(15, len(self._pt_xy))
            dists, idxs = self._kd.query([x, y], k=k)
            if k == 1:
                idxs = [idxs]
            idxs = [int(idx) for idx in idxs]
        elif np is not None:
            d2 = (self._pt_xy[:, 0] - x) ** 2 + (self._pt_xy[:, 1] - y) ** 2
            idxs = d2.argsort()[:15].tolist()
        else:
            d2s = [((px - x) ** 2 + (py - y) ** 2) for px, py in self._pt_xy]
            idxs = sorted(range(len(d2s)), key=lambda idx: d2s[idx])[:15]

        unique_links = []
        for idx in idxs:
            if idx < 0 or idx >= len(self._pt_link):
                continue
            lid = self._pt_link[idx]
            if lid not in unique_links:
                unique_links.append(lid)

        best_proj = None
        for lid in unique_links:
            lk = self.links[lid]
            proj = self._project_on_link(lk, x, y)
            if proj is not None:
                if best_proj is None or proj["dist"] < best_proj["dist"]:
                    best_proj = proj
        return best_proj

    def _project_on_link(self, lk, x, y):
        pts = lk.points
        best = None
        s_accum = 0.0
        best_s = 0.0
        for k in range(len(pts) - 1):
            ax, ay = pts[k]
            bx, by = pts[k + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-9:
                continue
            t = ((x - ax) * dx + (y - ay) * dy) / seg2
            t = max(0.0, min(1.0, t))
            px, py = ax + t * dx, ay + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or d2 < best[0]:
                heading = math.atan2(dy, dx)
                # 부호 있는 가로 오프셋(좌:+, 우:-)
                cross = dx * (y - ay) - dy * (x - ax)
                lateral = cross / math.sqrt(seg2)
                best = (d2, px, py, heading, lateral, s_accum + t * math.sqrt(seg2))
            s_accum += math.sqrt(seg2)
        if best is None:
            return None
        d2, px, py, heading, lateral, s = best
        return {
            "link_idx": lk.idx,
            "from_node": lk.from_node,
            "to_node": lk.to_node,
            "dist": math.sqrt(d2),
            "proj": (px, py),
            "lateral": lateral,
            "heading": heading,
            "s": s,
            "length": lk.length,
            "max_speed": lk.max_speed,
        }

    # --------------------------------------------------------------- stats
    def summary(self):
        npts = len(self._pt_link)
        return "nodes=%d links=%d (synthetic dropped=%d) link_pts=%d origin=%s kd=%s" % (
            len(self.nodes), len(self.links), getattr(self, "n_synthetic", 0),
            npts, self.origin,
            "scipy" if self._kd is not None else ("numpy" if np is not None else "py"),
        )


if __name__ == "__main__":
    m = MGeoMap()
    print(m.summary())
    # 원점 부근 매칭 테스트
    for (tx, ty) in [(0.0, 0.0), (-263.0, 569.0)]:
        r = m.match(tx, ty)
        if r:
            print("query (%.1f,%.1f) -> link=%s dist=%.2f lat=%.2f yaw=%.1fdeg s=%.1f" % (
                tx, ty, r["link_idx"], r["dist"], r["lateral"],
                math.degrees(r["heading"]), r["s"]))
