# Model

YOLOv8n, 13 classes, trained at 640×640 and exported at 416×416.

| File | Use |
|---|---|
| `best_full_integer_quant.tflite` | used by the app (`MODEL_PATH` in `config.py`) |
| `best_float16.tflite` | float16 version |
| `best_float32.tflite` | float32 version |
| `best.pt` | PyTorch model (Ultralytics) |

Export new files with `training/export_tflite.ipynb`.
