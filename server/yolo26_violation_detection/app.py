#!/usr/bin/env python3
"""HTTP inference service for the standalone YOLO26 violation detector."""

import ast
import os
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
from fastapi import FastAPI, File, HTTPException, Query, UploadFile


SERVICE_DIR = Path(__file__).resolve().parent
MODEL_PATH = Path(os.getenv("YOLO26_MODEL_PATH", SERVICE_DIR / "model" / "yolo26n_substation.onnx"))
MAX_IMAGE_BYTES = int(os.getenv("YOLO26_MAX_IMAGE_BYTES", str(10 * 1024 * 1024)))
MAX_IMAGE_PIXELS = int(os.getenv("YOLO26_MAX_IMAGE_PIXELS", "25000000"))
PROVIDER = os.getenv("YOLO26_PROVIDER", "cpu").lower()

EXPECTED_NAMES = {
    0: "hard hat",
    1: "work clothes",
    2: "sleep",
    3: "smoke",
    4: "personnel gathering",
    5: "play mobile",
    6: "danger area",
    7: "tool legacy",
}


class YOLO26Detector:
    """Run fixed-shape, end-to-end YOLO26 ONNX inference."""

    def __init__(self, model_path: Path, provider: str):
        if not model_path.is_file():
            raise FileNotFoundError("YOLO26 ONNX model not found: {}".format(model_path))

        available = ort.get_available_providers()
        if provider == "cuda":
            if "CUDAExecutionProvider" not in available:
                raise RuntimeError("CUDAExecutionProvider requested but unavailable: {}".format(available))
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        elif provider == "cpu":
            providers = ["CPUExecutionProvider"]
        else:
            raise ValueError("YOLO26_PROVIDER must be 'cpu' or 'cuda'")

        self.model_path = model_path
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        model_input = self.session.get_inputs()[0]
        model_output = self.session.get_outputs()[0]
        if model_input.shape != [1, 3, 640, 640]:
            raise RuntimeError("Unexpected model input shape: {}".format(model_input.shape))
        if model_output.shape != [1, 300, 6]:
            raise RuntimeError("Unexpected model output shape: {}".format(model_output.shape))

        self.input_name = model_input.name
        self.output_name = model_output.name
        self.image_size = 640
        metadata = self.session.get_modelmeta().custom_metadata_map
        self.names = self._parse_names(metadata.get("names", ""))
        if self.names != EXPECTED_NAMES:
            raise RuntimeError("Unexpected model classes: {}".format(self.names))

    @staticmethod
    def _parse_names(value: str):
        try:
            names = ast.literal_eval(value)
            return {int(class_id): str(name) for class_id, name in names.items()}
        except (SyntaxError, ValueError, AttributeError) as error:
            raise RuntimeError("Invalid class metadata in ONNX model") from error

    def _preprocess(self, image):
        height, width = image.shape[:2]
        scale = min(self.image_size / width, self.image_size / height)
        resized_width = round(width * scale)
        resized_height = round(height * scale)
        resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

        pad_width = self.image_size - resized_width
        pad_height = self.image_size - resized_height
        left = round(pad_width / 2 - 0.1)
        right = round(pad_width / 2 + 0.1)
        top = round(pad_height / 2 - 0.1)
        bottom = round(pad_height / 2 + 0.1)
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        tensor = padded[:, :, ::-1].transpose(2, 0, 1)
        tensor = np.ascontiguousarray(tensor, dtype=np.float32) / 255.0
        return tensor[None], scale, left, top

    def predict(self, image, confidence: float):
        tensor, scale, pad_left, pad_top = self._preprocess(image)
        started = time.perf_counter()
        predictions = self.session.run([self.output_name], {self.input_name: tensor})[0][0]
        inference_ms = (time.perf_counter() - started) * 1000.0

        height, width = image.shape[:2]
        detections = []
        for x1, y1, x2, y2, score, class_id in predictions:
            score = float(score)
            if score < confidence:
                continue

            class_id = int(class_id)
            x1 = max(0.0, min(float(width), (float(x1) - pad_left) / scale))
            y1 = max(0.0, min(float(height), (float(y1) - pad_top) / scale))
            x2 = max(0.0, min(float(width), (float(x2) - pad_left) / scale))
            y2 = max(0.0, min(float(height), (float(y2) - pad_top) / scale))
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": self.names.get(class_id, str(class_id)),
                    "confidence": round(score, 6),
                    "bbox_xyxy": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
                }
            )

        return detections, inference_ms


detector = YOLO26Detector(MODEL_PATH, PROVIDER)
app = FastAPI(
    title="YOLO26 Violation Detection API",
    version="1.0.0",
    description="Standalone ONNX inference for uploaded real-world images.",
)


@app.get("/health")
def health():
    """Report model readiness and active ONNX Runtime provider."""
    return {
        "status": "ok",
        "model": detector.model_path.name,
        "provider": detector.session.get_providers()[0],
        "input_size": detector.image_size,
        "classes": detector.names,
    }


@app.post("/predict")
def predict(
    file: UploadFile = File(...),
    confidence: float = Query(0.25, ge=0.0, le=1.0),
):
    """Detect safety-related classes in one uploaded image."""
    image_bytes = file.file.read(MAX_IMAGE_BYTES + 1)
    if not image_bytes:
        raise HTTPException(status_code=400, detail="Uploaded image is empty")
    if len(image_bytes) > MAX_IMAGE_BYTES:
        raise HTTPException(status_code=413, detail="Uploaded image is too large")

    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise HTTPException(status_code=400, detail="File is not a decodable image")

    height, width = image.shape[:2]
    if height * width > MAX_IMAGE_PIXELS:
        raise HTTPException(status_code=413, detail="Image dimensions are too large")

    detections, inference_ms = detector.predict(image, confidence)
    return {
        "filename": file.filename,
        "image": {"width": width, "height": height},
        "inference_ms": round(inference_ms, 3),
        "count": len(detections),
        "detections": detections,
    }
