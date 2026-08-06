# YOLO26 真实图片违规识别服务

这是独立于 AstraDrone ROS、Gazebo、PX4 和无人机仿真的服务器端图片识别服务。服务接收用户上传的真实世界图片，使用 YOLO26n ONNX 模型完成推理，并返回检测类别、置信度和目标框坐标。

本目录不包含训练数据、训练脚本、训练日志、PyTorch 权重或 Ultralytics 源码。

## 文件结构

```text
yolo26_violation_detection/
├── app.py
├── Dockerfile
├── requirements.txt
├── README.md
└── model/
    └── yolo26n_substation.onnx
```

## 模型

- 格式：ONNX FP32
- Opset：17
- 输入：`images`，`(1, 3, 640, 640)`
- 输出：`output0`，`(1, 300, 6)`
- 推理方式：YOLO26 端到端、无需额外 NMS
- SHA256：`b9d0b5d84fb6e30a7d710fbc5febe1a2d3c8546f351dbb22b86ef141aaeb06db`

类别：

| ID | 模型类别 | 中文含义 |
|---:|---|---|
| 0 | `hard hat` | 安全帽 |
| 1 | `work clothes` | 工作服 |
| 2 | `sleep` | 睡觉 |
| 3 | `smoke` | 吸烟 |
| 4 | `personnel gathering` | 人员聚集 |
| 5 | `play mobile` | 玩手机 |
| 6 | `danger area` | 危险区域 |
| 7 | `tool legacy` | 工具遗留 |

API 返回模型检测结果，不额外推断企业业务规则。例如是否将某个类别认定为违规、是否需要连续多帧确认，应由调用方的业务策略决定。

## 本地环境

推荐 Python 3.11：

```bash
cd server/yolo26_violation_detection
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

启动服务：

```bash
uvicorn app:app --host 0.0.0.0 --port 8000 --workers 1
```

接口文档：

```text
http://127.0.0.1:8000/docs
```

## API

健康检查：

```bash
curl http://127.0.0.1:8000/health
```

上传图片：

```bash
curl -X POST "http://127.0.0.1:8000/predict?confidence=0.25" \
  -F "file=@/path/to/image.jpg"
```

返回示例：

```json
{
  "filename": "image.jpg",
  "image": {"width": 1920, "height": 1080},
  "inference_ms": 38.215,
  "count": 1,
  "detections": [
    {
      "class_id": 3,
      "class_name": "smoke",
      "confidence": 0.912345,
      "bbox_xyxy": [420.5, 188.2, 612.8, 522.1]
    }
  ]
}
```

## Docker 部署

```bash
cd server/yolo26_violation_detection
docker build -t astradrone-yolo26-server:1.0 .
docker run --rm -p 8000:8000 astradrone-yolo26-server:1.0
```

生产环境应在该服务前配置 HTTPS、身份认证、访问日志和请求限流。服务自身默认限制上传文件为 10 MiB、解码图片为 2500 万像素。

## 配置项

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `YOLO26_MODEL_PATH` | `model/yolo26n_substation.onnx` | 模型路径 |
| `YOLO26_PROVIDER` | `cpu` | `cpu` 或 `cuda` |
| `YOLO26_MAX_IMAGE_BYTES` | `10485760` | 最大上传字节数 |
| `YOLO26_MAX_IMAGE_PIXELS` | `25000000` | 最大解码像素数 |

如需 GPU，必须安装与服务器 CUDA/cuDNN 匹配的 `onnxruntime-gpu`，并设置：

```bash
export YOLO26_PROVIDER=cuda
```

不要在同一个环境中同时安装 `onnxruntime` 和 `onnxruntime-gpu`。

## 与 AstraDrone 其他 YOLO 模型的关系

- 本服务只处理服务器收到的真实图片。
- 不订阅 ROS Topic。
- 不启动或控制 Gazebo。
- 不修改 `AstraDrone_ros1_ws/src/Detection/yolo_detect`。
- Gazebo 中使用的 YOLOv8n 及其训练工作保持独立。

## 许可证

AstraDroneOpen 源码使用 MIT License。该 ONNX 模型由 Ultralytics YOLO26 导出，模型元数据声明 AGPL-3.0；部署和再分发时还应遵守 [Ultralytics License](https://ultralytics.com/license)。
