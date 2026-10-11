#!/usr/bin/env python3
"""LabelMe 신호등 데이터를 class-aware copy-paste로 red 개수에 맞춘다."""
import argparse, json, os, random, shutil
import cv2

CLASSES = ("red", "green", "left_arrow", "yellow")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.expanduser("~/Downloads/kcity_traffic_light"))
    ap.add_argument("--out", default=os.path.expanduser("~/Downloads/kcity_traffic_light_balanced"))
    ap.add_argument("--seed", type=int, default=20260721)
    ap.add_argument("--target", type=int, default=None,
                    help="각 클래스 목표 개수 (기본: 현재 최대 클래스)")
    args = ap.parse_args()
    random.seed(args.seed)
    os.makedirs(args.out, exist_ok=True)
    records, counts = [], {c: 0 for c in CLASSES}
    for jf in sorted(os.listdir(args.src)):
        if not jf.endswith(".json"):
            continue
        jp = os.path.join(args.src, jf)
        d = json.load(open(jp, encoding="utf-8"))
        ip = os.path.join(args.src, d.get("imagePath", jf[:-5] + ".jpg"))
        if not os.path.isfile(ip):
            continue
        shapes = [s for s in d.get("shapes", []) if s.get("label") in CLASSES and
                  s.get("shape_type") == "rectangle" and len(s.get("points", [])) >= 2]
        for s in shapes: counts[s["label"]] += 1
        records.append((ip, d, shapes))
        shutil.copy2(ip, os.path.join(args.out, os.path.basename(ip)))
        d["imagePath"] = os.path.basename(ip); d["imageData"] = None
        json.dump(d, open(os.path.join(args.out, jf), "w", encoding="utf-8"),
                  ensure_ascii=False, indent=2)
    target = args.target if args.target is not None else max(counts.values())
    if target < max(counts.values()):
        raise ValueError("target must be >= existing maximum count")
    print("original:", counts, "target:", target)
    sources = {c: [(ip, d, s) for ip, d, shapes in records for s in shapes if s["label"] == c]
               for c in CLASSES}
    serial = 0
    for cls in CLASSES:
        for n in range(target - counts[cls]):
            ip, d, source = random.choice(sources[cls])
            img = cv2.imread(ip, cv2.IMREAD_COLOR)
            if img is None: continue
            h, w = img.shape[:2]
            (x1, y1), (x2, y2) = source["points"][:2]
            x1, x2 = sorted((int(x1), int(x2))); y1, y2 = sorted((int(y1), int(y2)))
            pad = max(2, int(max(x2-x1, y2-y1) * 0.18))
            sx1, sy1 = max(0, x1-pad), max(0, y1-pad)
            sx2, sy2 = min(w, x2+pad), min(h, y2+pad)
            crop = img[sy1:sy2, sx1:sx2]
            if crop.size == 0: continue
            scale = random.uniform(0.85, 1.15)
            nw, nh = max(4, int(crop.shape[1]*scale)), max(4, int(crop.shape[0]*scale))
            if nw >= w or nh >= h: continue
            crop = cv2.resize(crop, (nw, nh), interpolation=cv2.INTER_LINEAR)
            # 신호등은 전방 상단 영역에 배치하고 차량/노면 하단은 피한다.
            dx = random.randint(0, w-nw)
            dy = random.randint(0, max(0, int(h*0.70)-nh))
            base = img.copy(); base[dy:dy+nh, dx:dx+nw] = crop
            serial += 1
            stem = "copypaste_%04d_%s" % (serial, cls)
            out_img = stem + ".jpg"; out_json = stem + ".json"
            cv2.imwrite(os.path.join(args.out, out_img), base,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            nd = {"version": d.get("version", "5.0.0"), "flags": {},
                  "shapes": [{"label": cls, "text": "",
                              "points": [[dx + (x1-sx1)*scale, dy + (y1-sy1)*scale],
                                         [dx + (x2-sx1)*scale, dy + (y2-sy1)*scale]],
                              "group_id": None, "shape_type": "rectangle", "flags": {}}],
                  "imagePath": out_img, "imageData": None,
                  "imageHeight": h, "imageWidth": w}
            json.dump(nd, open(os.path.join(args.out, out_json), "w", encoding="utf-8"),
                      ensure_ascii=False, indent=2)
    print("generated:", serial, "output:", args.out)

if __name__ == "__main__": main()
