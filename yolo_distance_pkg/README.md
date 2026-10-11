# yolo_distance_pkg — YOLO-seg + 단안깊이 거리 추정 (단일 파일 배포판)

MORAI 전방 카메라(1280×960/720, HFOV 90°) 이미지에서 오브젝트 2D 검출 + 거리(m)를
출력합니다. 오브젝트 거리 MAE 1.9 m, 상대오차 9% (MORAI BBox3D GT val 기준).

## 구성

| 파일 | 내용 |
|---|---|
| `distance_estimator.py` | 전부 포함된 단일 스크립트 (모델 정의 + 추론 + CLI) |
| `depth_unet.pt` | 깊이 모델 (ResNet18-UNet, 960×1280 학습, 유효거리 ~100 m) |
| `yolov8n-seg.pt` | YOLO 세그 가중치 (COCO) |
| `scale.json` | 학습 데이터 캘리브 기록 (fx=640, HFOV 90°, depth=raw×100/255) |

## 설치 (대상 PC)

```bash
pip install torch torchvision opencv-python numpy ultralytics
```

## 사용

```bash
# CLI — 이미지/디렉토리 추론 + 오버레이 저장
python3 distance_estimator.py --input <이미지|디렉토리>

# 라이브러리
from distance_estimator import DistanceEstimator
est = DistanceEstimator()          # 같은 폴더의 가중치 자동 로드
dets, depth_map, padded = est(bgr) # dets = [{cls, conf, box, dist}, ...]
```

## 주의

- **카메라 조건**: HFOV 90°(fx=640@1280px) 카메라로 학습됨. 1280×720/960 입력은
  자동 패딩 처리(검증됨). FOV가 다른 카메라는 거리가 틀어짐 → 재학습 필요.
- **TRT 엔진 미동봉**: .engine 은 빌드 머신(GPU+TRT 버전) 종속. 대상 PC에서
  `yolo export model=yolov8n-seg.pt format=engine imgsz=1280` 후
  `DistanceEstimator(yolo_weights="yolov8n-seg.engine")` 로 지정.
- **imgsz=1280 권장**: 640이면 원거리 소형 오브젝트 검출이 크게 감소.
- 거리 산출 = seg 마스크 내부 깊이 중앙값 (마스크 없으면 bbox 하단중앙 폴백).
  자차 후드 오검출 필터 내장 (`hood_filter=False` 로 해제 가능).
- GPU 권장 (1280×960 기준 약 85 ms/frame @ RTX 4080급). CPU도 동작하나 느림.
