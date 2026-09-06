# Headless Video Streaming Notes

## Goal

Replace all runtime `cv2.imshow()` usage with a small display abstraction that can publish frames from the robot in headless mode.

This applies to both runtime and conditionally-runtime/debug vision paths, including:

- `Soccer/Vision/class_Vision_RPI.py`
- `Soccer/Vision/class_Vision_General.py`
- `Soccer/Localisation/class_Local.py`
- `Soccer/Vision/lookARUCO.py`
- `Soccer/Vision/lookAtLine.py`
- `Soccer/Vision/neuro_client.py` for neural-worker results

Tools under `tools/` may keep GUI/window behavior.

## Backends

Default backend: `GStreamerDisplay`.

Required fallback backend: `NullDisplay`.

Optional backend: `OpenCvDisplay`, useful when running on a non-headless robot or developer machine.

Runtime code should call a shared API, for example:

```python
display.show("track", frame)
display.show("mask", mask)
display.show("aruco", frame)
```

The calling code should not know whether the frame is going to RTSP, OpenCV HighGUI, or nowhere.

## GStreamer Design

Use Python GObject bindings:

```python
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib
```

Use a `GstRtspServer` with one RTSP mount per logical stream:

- `/main`
- `/track`
- `/mask`
- `/aruco`
- `/neural`

Frames are pushed through `appsrc`.

Pipeline properties should avoid backpressure into vision/motion code:

```text
appsrc is-live=true block=false format=time do-timestamp=true
queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0
```

`display.show()` should be non-blocking:

- if RTSP backend is not initialized, drop the frame;
- if no client is connected, drop the frame;
- if pipeline is not `PLAYING`, drop the frame;
- if `push-buffer` returns anything except `Gst.FlowReturn.OK`, drop the frame;
- do not retry inside vision/motion code.

Do not keep a Python-side `latest_frame` cache for the first implementation. The producer should push only the current frame when the backend is ready.

## Encoding

Current robot vision frames are mostly BGR numpy arrays after `camera.snapshot()`.

Simple initial pipeline:

```text
appsrc caps=video/x-raw,format=BGR,width=...,height=...,framerate=...
  ! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0
  ! videoconvert
  ! video/x-raw,format=NV12
  ! v4l2h264enc
  ! h264parse
  ! rtph264pay name=pay0 pt=96
```

The hardware encoder is expected to prefer YUV formats, typically `NV12` or `I420`, not BGR. Therefore BGR frames need conversion before hardware H264/JPEG encoding.

Potential later optimization:

- expose YUV/NV12 frames directly from the camera path for raw camera streams;
- keep BGR only for vision algorithms and overlays;
- add a stream format option per stream: `BGR`, `GRAY`, `I420`, `NV12`.

## Startup Behavior

Start the RTSP server at process startup.

Create/prepare stream pipelines independently from vision loops. A first client connection may still take time due to caps negotiation or encoder initialization, but runtime code must only drop frames during that period.

No robot control path should wait for:

- RTSP client connection;
- encoder startup;
- network IO;
- stream negotiation.

## Open Questions

- Which hardware encoder plugin will be available on Artix ARM64: `v4l2h264enc`, `v4l2jpegenc`, or another platform-specific encoder?
- Which raw formats each encoder accepts on the target image?
- Whether Picamera2 can provide a second stream in an encoder-friendly format while keeping the current BGR path for vision.
- Whether debug streams need H264 only, JPEG only, or mixed output depending on stream type.

## Current Video Path Audit

Current camera setup:

- `Soccer/Vision/camera.py` configures Picamera2 with `create_video_configuration(...)`.
- Runtime uses one stream: `main={"format": "RGB888", "size": (800, 650)}`.
- Picamera2/libcamera naming is counter-intuitive here: `RGB888` is the format that gives Python/OpenCV a `(B, G, R)` array.
- Runtime snapshots read `main` via `request.make_array("main")`.
- The previous `lores=YUV420` path and `YUV420 -> BGR` conversion have been removed from the runtime camera wrapper.
- Most runtime vision code receives BGR numpy frames directly from Picamera2.

Current distortion handling:

- `Vision_RPI` loads `Soccer/Vision/undistortPointMap_x_y.npy` and `Camera_calibration_P.npy`.
- Runtime does not remap whole frames.
- Runtime undistorts only selected points with `undistort_points(column, row)`, then computes floor coordinates.
- Whole-frame `cv2.remap()` is currently only in calibration/test tools.

Main processing layers:

- `Soccer/Vision/reload.py::Image` copies every input frame in `Image.__init__`.
- `Image.find_blobs()` converts ROI/frame BGR to LAB on every threshold pass.
- `Image.binary()` converts full BGR frame to LAB and returns a 3-channel mask.
- `Image.find_lines()` / `find_line_segments()` convert or process grayscale/edges for line detection.
- `Vision_General` calls `Image(...)`, `find_blobs()`, `binary()`, `cv2.resize()`, `cv2.cvtColor()`, and display calls.
- `class_Local` reuses vision frames for localisation, but also performs separate BGR->LAB conversions for post/penalty validation.
- `lookARUCO.py` and `lookAtLine.py` are runtime consumers of the shared `Vision_RPI.camera` pipeline. They should not create their own Picamera2 instances.
- Their standalone `__main__` test loops may create their own Picamera2 pipeline because no main robot runtime is active in that mode.
- OpenVINO inference is handled by an external worker outside this repository.
- `Soccer/Vision/neuro_client.py` sends BGR frames to that worker through shared memory and Unix socket commands.

Immediate optimization candidates:

- Replace `cv2.imshow()`/`waitKey()` calls with display backend calls.
- Keep `NullDisplay` and `GStreamerDisplay` in runtime; keep `OpenCvDisplay` only as optional backend.
- Keep one Picamera2 owner for the robot process: `Glob -> Camera -> Vision_RPI`.
- Special runtime modes such as sprint ARUCO tracking should run as threads/consumers of the shared camera, not as separate Picamera2 processes.
- Avoid repeated BGR->LAB conversion in `Image.find_blobs()` when multiple thresholds are tested against the same frame or ROI.
- Avoid returning 3-channel binary masks from `Image.binary()` where later code needs grayscale only.
- Remove duplicate `Image(img1)` construction in line follow code.
- Push BGR frames into GStreamer initially and let `videoconvert` feed the encoder.
- Later expose a YUV/NV12 camera stream for raw video streaming so encoder input does not need BGR roundtrip.

OpenCL status:

- No runtime `cv2.ocl`, `cv2.UMat`, CUDA, or OpenCL backend usage found.
- OpenCL should not be added blindly around random operations; it is only worth testing around large, repeated operations like color conversion, resize, thresholding, morphology, and maybe remap.
- On Raspberry Pi / ARM64, OpenCL benefit is not guaranteed and can lose to copy/synchronization overhead.

## Known Inefficiencies

### Central frame format conversion

- Runtime vision no longer performs `YUV420 -> BGR` in `Soccer/Vision/camera.py::snapshot()`.
- Future raw RTSP streaming still needs a separate plan because raw stream encoding will likely want `NV12`/`I420`, while current vision wants BGR.

### Repeated BGR -> LAB conversion

- `Soccer/Vision/reload.py::Image.find_blobs()` converts the same frame or ROI from `BGR` to `LAB` inside the threshold loop.
- When multiple thresholds are tested against the same ROI, the same `BGR -> LAB` conversion is repeated.
- `Soccer/Localisation/class_Local.py::detect_Post_In_image()` creates `Image(img_)` and also manually converts the same frame to `LAB`.
- `Soccer/Localisation/class_Local.py::detect_penalty_marks()` does the same pattern: `Image(img)` plus manual `LAB`.
- `Soccer/Localisation/class_Local.py::detect_line_in_image()` calls `binary()` twice on the same frame, causing two full-frame `BGR -> LAB` conversions.

### Binary mask format churn

- `Soccer/Vision/reload.py::Image.binary()` converts `BGR` to `LAB`, makes a one-channel mask, then expands it into a 3-channel image.
- `Soccer/Localisation/class_Local.py::detect_line_in_image()` later converts that 3-channel mask back to grayscale before thinning.
- This causes a useless `gray -> 3-channel -> gray` style roundtrip.

### Unconditional frame copies

- `Soccer/Vision/reload.py::Image.__init__()` unconditionally performs `img_.copy()`.
- `Soccer/Vision/class_Vision_General.py::detect_Line_Follow_Stream()` constructs `Image(img1)` twice on the same frame.
- `Soccer/Vision/class_Vision_General.py::detect_Line_Follow_One_Shot()` also constructs `Image(img1)` twice on the same frame.
- These are not color conversions, but they are still expensive full-frame memory copies on the hot path.

### Resize chains

- `Soccer/Vision/class_Vision_General.py::detect_Line_Follow_One_Shot()` resizes the frame up to `(800, 650)` again for display/debug.
- Some of these resizes are valid for algorithm behavior, but the chain should be reviewed to ensure that no redundant intermediate size is kept.

### Mixed manual and helper-based image processing

- `class_Local` uses both:
  - manual `cv2.cvtColor(..., COLOR_BGR2LAB)` plus `inRange`;
  - helper methods `Image.find_blobs()` / `Image.binary()`.
- This makes it easy to convert the same frame multiple times in different code paths instead of reusing one cached derived representation.

### Standalone-only conversions that are not runtime-critical

- `Soccer/Vision/lookARUCO.py` and `Soccer/Vision/lookAtLine.py` convert runtime `BGR` frames to `GRAY`. This is probably justified.
- Their standalone test loops convert camera `YUV420` directly to `GRAY`, which is fine for local tests.
- Neural inference no longer performs OpenVINO preprocessing in the main robot process.

## Optimization Plan

### Phase 1: cheap wins with low risk

1. Add a cached `LAB` representation to `reload.Image`.
2. Make `find_blobs()` convert `BGR -> LAB` only once per frame or once per ROI, not once per threshold.
3. Remove duplicate `Image(img)` construction in line-follow functions.
4. Audit `Image.__init__()` and make copying explicit only where mutation of the source frame is really needed.

Expected result:

- less repeated `cvtColor`;
- fewer full-frame copies;
- no change to algorithm logic or thresholds.

### Phase 2: fix binary mask format churn

1. Change `Image.binary()` to return a one-channel mask by default.
2. Only convert masks to 3-channel where a drawing or debug display path explicitly needs it.
3. Update thinning and line-detection code to work directly with one-channel masks.

Expected result:

- remove unnecessary 3-channel mask expansion;
- reduce memory bandwidth and extra `cvtColor` calls.

### Phase 3: unify derived image reuse in localisation

1. Refactor `class_Local` routines so post, penalty, and line checks reuse one `Image`-owned cached representation instead of mixing manual `LAB` conversion and helper methods.
2. Remove cases where the same frame is turned into `LAB` twice by different helpers.

Expected result:

- cleaner image-processing flow;
- fewer duplicated conversions in localisation hot paths.

### Phase 4: separate raw streaming path from vision path

1. Keep the current one-stream `BGR` vision path for robot algorithms.
2. Add a separate raw stream path later if direct hardware-encoded camera video is needed.
3. Feed raw camera stream to GStreamer in an encoder-friendly format such as `I420`/`NV12` without a `BGR` roundtrip.

Expected result:

- better streaming efficiency;
- no forced compromise between encoder needs and vision needs.

### Phase 5: only then consider acceleration backends

1. Measure the cost of:
   - `cvtColor(BGR <-> LAB / GRAY)`,
   - `resize`,
   - `inRange`,
   - morphology,
   - thinning.
2. Only after measurement, test whether OpenCL or another accelerated backend helps on the target ARM64 image.
3. Avoid introducing OpenCL before the duplicate conversions and copies are removed, because algorithmic cleanup is more predictable and lower risk.

Expected result:

- acceleration work is based on measured hotspots, not guesswork.
