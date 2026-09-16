# Quantization score shift, YOLO26n, matched detections

Inputs (conf=0.001, controlled postprocess, matched raw-tensor head):
- CPU FP32: detections_yolo26n_cpu_conf0.001_controlled_raw_tensor.json (596,200 detections)
- NPU INT8: detections_yolo26n_npu_conf0.001_controlled.json (453,077 detections)

```
matched pairs (IoU >= 0.5)      : 330,879
CPU detections with no NPU match      : 265,321
mean   score shift (NPU - CPU)        : -0.01580
median score shift (NPU - CPU)        : -0.00387
fraction of matches scored LOWER on NPU: 0.7870
matched objects above 0.2 on CPU     : 28,281
  ... pushed BELOW 0.2 by INT8        : 6,654 (0.2353 of them)
  ... pulled ABOVE 0.2 by INT8        : 889
net loss across the operating threshold: +5,765
CPU detections above 0.2 with no NPU match at all: 309
Independent cross-check, counted without any matching:
  CPU detections above 0.2                 : 28,590
  NPU detections above 0.2                 : 22,602
  difference                              : +5,988
```
