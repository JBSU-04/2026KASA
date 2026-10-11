#!/usr/bin/env python3
"""
COCO 모델 + 커스텀 파인튜닝 모델의 결과를 합치되,
서로 겹치는(IoU 높은) 박스는 confidence가 더 높은 것만 남기는 스크립트.

예: 자동차 위치에서 COCO가 car(conf 0.75)로, 커스텀 모델이 pe_barrigate(conf 0.45)로
    동시에 잡았다면 -> 겹치므로 conf 높은 COCO의 car만 채택.
    반대로 겹치는 게 없는 감지 결과는 그대로 유지.

custom_exclude_names로 지정한 클래스는 커스텀 모델에서 아예 결과에서 제외되어,
해당 클래스는 COCO 모델이 전담하게 됨 (예: motorcycle).

draw()는 바운딩박스 대신 세그멘테이션 마스크를 반투명하게 채워서 그립니다.
"""

import numpy as np
import cv2
from ultralytics import YOLO


def compute_iou(box1, box2):
    """box: [x1, y1, x2, y2]"""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter_area = inter_w * inter_h

    area1 = max(0.0, box1[2] - box1[0]) * max(0.0, box1[3] - box1[1])
    area2 = max(0.0, box2[2] - box2[0]) * max(0.0, box2[3] - box2[1])

    union = area1 + area2 - inter_area
    if union <= 0:
        return 0.0
    return inter_area / union


# 클래스 이름별 고정 색상을 주고 싶으면 여기에 추가 (없으면 자동으로 랜덤 고정 색 배정)
_COLOR_CACHE = {}


def get_color_for_name(name):
    if name not in _COLOR_CACHE:
        # 이름 문자열 기반으로 고정된(매 실행 동일한) 색상 생성
        h = abs(hash(name)) % 180
        color_hsv = np.uint8([[[h, 200, 255]]])
        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0][0]
        _COLOR_CACHE[name] = tuple(int(c) for c in color_bgr)
    return _COLOR_CACHE[name]


class DualModelDetector:
    def __init__(self, coco_weight='weights/yolo11s-seg.pt',
                 custom_weight='weights/best.pt',
                 coco_classes=None, iou_threshold=0.4,
                 coco_conf=0.5, custom_conf=0.5,
                 custom_exclude_names=None):
        """
        coco_classes: COCO 쪽에서 사용할 클래스 인덱스 리스트.
                      None이면 전체 COCO 80개 클래스 다 사용.
                      예: [0, 1, 2, 3, 5, 7, 16] -> person, bicycle, car, motorcycle, bus, truck, dog
        iou_threshold: 이 값 이상 겹치면 "같은 물체"로 보고 conf 낮은 쪽 제거
        custom_exclude_names: 커스텀 모델 결과에서 제외할 클래스 이름 리스트.
                               예: ['motorcycle'] -> 커스텀 모델이 motorcycle을 잡아도 결과에서 제거,
                               COCO 모델이 motorcycle을 전담하게 됨.
        """
        self.coco_model = YOLO(coco_weight)
        self.custom_model = YOLO(custom_weight)
        self.coco_classes = coco_classes
        self.iou_threshold = iou_threshold
        self.coco_conf = coco_conf
        self.custom_conf = custom_conf
        self.custom_exclude_names = set(custom_exclude_names or [])

    def _extract_detections(self, result, model, source, exclude_names=None):
        exclude_names = exclude_names or set()
        detections = []

        boxes = result.boxes
        masks = result.masks  # None일 수 있음 (세그멘테이션 모델이 아니거나 감지 없을 때)

        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i])
            name = model.names[cls_id]
            if name in exclude_names:
                continue

            xyxy = boxes.xyxy[i].tolist()
            conf = float(boxes.conf[i])

            mask_poly = None
            if masks is not None and i < len(masks.xy):
                mask_poly = masks.xy[i]  # (N, 2) 픽셀 좌표 폴리곤

            detections.append({
                'box': xyxy,
                'conf': conf,
                'name': name,
                'source': source,
                'mask': mask_poly,
            })

        return detections

    def detect(self, img):
        """
        반환: [{'box': [x1,y1,x2,y2], 'conf': float, 'name': str,
                'source': 'coco'|'custom', 'mask': (N,2) polygon or None}, ...]
        """
        coco_res = self.coco_model(img, classes=self.coco_classes,
                                    conf=self.coco_conf, verbose=False)[0]
        coco_dets = self._extract_detections(coco_res, self.coco_model, 'coco')

        custom_res = self.custom_model(img, conf=self.custom_conf, verbose=False)[0]
        custom_dets = self._extract_detections(custom_res, self.custom_model, 'custom',
                                                exclude_names=self.custom_exclude_names)

        detections = coco_dets + custom_dets
        return self._cross_model_nms(detections)

    def _cross_model_nms(self, detections):
        """conf 내림차순 정렬 후, 이미 채택된 박스와 많이 겹치는(IoU>=threshold) 낮은 conf 박스 제거."""
        detections = sorted(detections, key=lambda d: d['conf'], reverse=True)
        kept = []
        for det in detections:
            suppressed = False
            for kept_det in kept:
                iou = compute_iou(det['box'], kept_det['box'])
                if iou >= self.iou_threshold:
                    suppressed = True
                    break
            if not suppressed:
                kept.append(det)
        return kept

    def draw(self, img, detections, alpha=0.45, draw_box_outline=True):
        """
        세그멘테이션 마스크를 반투명하게 채워서 그립니다.
        mask가 없는 검출(세그멘테이션 결과가 없는 경우)은 박스로 대체 표시합니다.
        """
        out = img.copy()
        overlay = img.copy()

        for det in detections:
            color = get_color_for_name(det['name'])

            if det['mask'] is not None and len(det['mask']) >= 3:
                pts = det['mask'].astype(np.int32).reshape(-1, 1, 2)
                cv2.fillPoly(overlay, [pts], color)
            else:
                # 마스크 정보가 없으면 박스 영역이라도 채워서 표시
                x1, y1, x2, y2 = [int(v) for v in det['box']]
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)

        # 원본과 색칠된 오버레이를 알파블렌딩 (반투명 효과)
        out = cv2.addWeighted(overlay, alpha, out, 1 - alpha, 0)

        # 마스크 윤곽선 + 라벨 텍스트는 선명하게 별도로 그림
        for det in detections:
            color = get_color_for_name(det['name'])
            x1, y1, x2, y2 = [int(v) for v in det['box']]

            if draw_box_outline:
                if det['mask'] is not None and len(det['mask']) >= 3:
                    pts = det['mask'].astype(np.int32).reshape(-1, 1, 2)
                    cv2.polylines(out, [pts], True, color, 2)
                else:
                    cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

            label = "%s %.2f [%s]" % (det['name'], det['conf'], det['source'])
            cv2.putText(out, label, (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return out


if __name__ == '__main__':
    # 간단 테스트용 (이미지 파일 하나로 확인하고 싶을 때)
    import sys
    detector = DualModelDetector(
        coco_weight='weights/yolo11s-seg.pt',
        custom_weight='weights/best.pt',
        coco_classes=[0, 1, 2, 3, 5, 7, 16],  # person, bicycle, car, motorcycle, bus, truck, dog
        iou_threshold=0.4,
        custom_exclude_names=['motorcycle'],
       
    )
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        dets = detector.detect(img)
        out = detector.draw(img, dets)
        cv2.imshow("result", out)
        cv2.waitKey(0)
