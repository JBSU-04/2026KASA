#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI 런처의 K-City 2025 Unity AssetBundle(.scene) 안에 들어있는
MGeo TextAsset(node_set / link_set / global_info ...)을 추출한다.

기본 대상:
  MoraiLauncher_Lin_카사 / .../Bundle/v.S4.251001.MolitComp03/r_kr_pr_k-city_2025.scene
출력:
  <이 패키지>/data/   (node_set.json, link_set.json, global_info.json ...)

usage:
    python3 extract_mgeo.py [--list] [--scene <path>] [--out <dir>]
"""
import os
import sys
import argparse
import UnityPy

DEFAULT_SCENE = (
    "/home/kuuve/Downloads/MoraiLauncher_Lin_카사/MoraiLauncher_Lin_Data/"
    "Bundle/v.S4.251001.MolitComp03/r_kr_pr_k-city_2025.scene"
)
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="TextAsset 목록만 출력")
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()

    if not os.path.isfile(args.scene):
        print("scene not found:", args.scene)
        sys.exit(1)

    os.makedirs(args.out, exist_ok=True)
    print("loading scene (this can take a while for large bundles)...")
    env = UnityPy.load(args.scene)

    n = 0
    for obj in env.objects:
        if obj.type.name != "TextAsset":
            continue
        data = obj.read()
        name = getattr(data, "m_Name", None) or getattr(data, "name", "")
        script = getattr(data, "m_Script", None)
        if script is None:
            script = getattr(data, "script", b"")
        if isinstance(script, str):
            raw = script.encode("utf-8", "surrogateescape")
        else:
            raw = bytes(script)
        size = len(raw)
        print(f"[TextAsset] name={name!r:40} size={size}")
        n += 1
        if args.list:
            continue
        safe = name if name else f"textasset_{n}"
        if not os.path.splitext(safe)[1]:
            safe += ".json"
        path = os.path.join(args.out, safe)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as f:
            f.write(raw)
    print(f"\nTextAsset 총 {n}개. 출력 폴더: {args.out}")


if __name__ == "__main__":
    main()
