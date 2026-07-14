# Robot Detector Workflow

1. Capture images from both low camera streams:

```powershell
venv\Scripts\python.exe capture_robot_dataset.py
```

Use `S` to save one stereo pair, or `A` to autosave while you move the robot and people around the scene. Capture varied lighting, robot positions, partial occlusions, and backgrounds.

2. Label the images in YOLO format.

Use one class named `robot`. Save labels beside the dataset as:

```text
robot_dataset/images/train/*.jpg
robot_dataset/labels/train/*.txt
```

Each label file should contain YOLO boxes:

```text
0 x_center y_center width height
```

All coordinates are normalized from `0` to `1`.

3. Move about 20 percent of labeled images and labels to validation:

```text
robot_dataset/images/val
robot_dataset/labels/val
```

4. Train:

```powershell
venv\Scripts\python.exe train_robot_detector.py
```

5. Run the live pipeline:

```powershell
venv\Scripts\python.exe test_dual_stream_pipeline.py
```

The pipeline automatically uses:

```text
runs/detect/robot_detector/weights/best.pt
```

If that file is missing, it falls back to the old color detector.
