# ROS2 Simple Camera Publisher

Minimal ROS 2 node that grabs frames from a camera, publishes them as ROS images, and (optionally) encodes them to compressed formats such as JPEG, PNG or WEBP.

The node is designed to be **multi-platform**:

- **Jetson ORIN/NANO** using `GStreamer` pipeline.
- **Raspberry Pi** using the `rpicam-vid` pipeline and mirroring to a **virtual V4L2 camera** (e.g. via `v4l2loopback` + `ffmpeg`).
- **PC / laptop** using OpenCV (`cv::VideoCapture`) for standard USB / UVC cameras.

The main goal is to have a **simple camera source** for ROS 2 that can be reused in different projects and hardware setups.

---

## Features

- **Multi-platform capture**
  - `board=jnano/jorin`: uses OpenCV and `GStreamer` for Raspberry Pi cameras.
  - `board=rasp`: uses OpenCV and `rpicam-vid` for Raspberry Pi cameras.
  - `board=PC`: uses OpenCV and a standard camera device.
- **Raw or compressed publishing**
  - Publishes either:
    - `sensor_msgs/msg/Image` on `/video_source/raw`, or
    - `sensor_msgs/msg/CompressedImage` on `/video_source/compressed`
  - Compression handled with OpenCV’s `cv::imencode`.
- **Multiple compression formats**
  - JPEG (`format = 0`)
  - PNG  (`format = 1`)
  - WEBP (`format = 2`)
- **Resolution / FPS presets**
  - `mode` parameter selects a pre-defined `width × height @ fps` combination (different maps for NANO/ORIN/PC vs Raspberry Pi).
- **Dynamic parameters**
  - Most parameters can be changed at runtime; the node restarts the camera pipeline when needed.

---

## Build & Installation

Assuming you are using a standard ROS 2 workspace:

```bash
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/IvanMeloFGLab/ROS2-Simple-camera-publlisher.git

cd ..
colcon build --packages-select cpp_camera
source install/setup.bash
```

---

## Running the node

In a sourced terminal:

```bash
ros2 run cpp_camera camera_pub --ros-args -p board:=jorin -p Compression:=false -p preview:=true
```

### Example: Jetson Orin/Nano (compressed PNG, no preview, flipped vertically)

```bash
ros2 run cpp_camera camera_pub --ros-args -p board:=jorin -p Compression:=true -p format:=1 -p preview:=false -p mode:=0 -p Vflip:=true
```

This will:

- Use the Jetson Orin camera (`GStreamer`) backend.
- Encode each frame as PNG and publish `sensor_msgs/msg/CompressedImage` on `/video_source/compressed`.
- Disable the local preview window.
- Flip the image vertically.

---

### Example: PC / Laptop (USB webcam, raw images)

```bash
ros2 run cpp_camera camera_pub --ros-args -p board:=PC -p Compression:=false -p preview:=true -p mode:=0
```

This will:

- Use the PC/OpenCV backend.
- Publish **raw** `sensor_msgs/msg/Image` messages on `/video_source/raw`.
- Open a preview window.
- Use resolution/FPS preset `mode=0` (see parameter section).

### Example: Raspberry Pi (compressed JPEG, no preview)

```bash
ros2 run cpp_camera camera_pub --ros-args -p board:=rasp -p Compression:=true -p format:=0 -p preview:=false -p mode:=0
```

This will:

- Use the Raspberry Pi camera (`rpicam-vid`) backend.
- Encode each frame as JPEG and publish `sensor_msgs/msg/CompressedImage` on `/video_source/compressed`.
- Disable the local preview window.

---

## Parameters

All parameters are ROS 2 parameters of the `camera_publisher` node.

### Core behaviour

| Name          | Type    | Default | Description |
|---------------|---------|---------|-------------|
| `board`       | string  | `"jorin"`  | Capture backend. `"PC"` for OpenCV/USB cameras, `"rasp"` for Raspberry Pi, `"jorin/jnano"` for Jetson Orin and Jetson Nano. |
| `Compression` | bool    | `false` | If `false`, publish raw `sensor_msgs/msg/Image` on `/video_source/raw`. If `true`, encode frames and publish `sensor_msgs/msg/CompressedImage` on `/video_source/compressed`. |
| `preview`     | bool    | `false` | Enable local preview. |
| `mode`        | int     | `0`     | Resolution/FPS preset. Each integer index maps to a `{width, height, fps}` triplet. There are separate maps for NANO/ORIN/PC and Raspberry Pi backends. See mode section or parameter descriptors for more information. |
| `format`      | int     | `0`     | Compression format when `Compression=true`: `0 = JPEG`, `1 = PNG`, `2 = WEBP`. |
| `VCam_num`  | int  | `100`   | Virtual camera index that the code read from. **ONLY for Raspberry Pi**. |

### Mode for Jetson Nano/Orin and PC

| Mode      | Width   | Height | FPS  |
|-----------|---------|--------|------|
| `0`       | `640`   | `480`  | `30` |
| `1`       | `640`   | `480`  | `60` |
| `2`       | `1280`  | `720`  | `30` |
| `3`       | `1280`  | `720`  | `60` |
| `4`       | `1640`  | `1232` | `30` |
| `5`       | `1920`  | `1080` | `30` |
| `6`       | `3280`  | `1848` | `28` |
| `7`       | `3264`  | `2464` | `21` |

### Mode for Raspberry Pi

| Mode      | Width   | Height | FPS  |
|-----------|---------|--------|------|
| `0`       | `640`   | `480`  | `30` |
| `1`       | `640`   | `480`  | `60` |
| `2`       | `640`   | `480`  | `90` |
| `3`       | `640`   | `480`  | `103` |
| `4`       | `1640`  | `1232` | `30` |
| `5`       | `1640`  | `1232` | `41` |
| `6`       | `1920`  | `1080` | `30` |
| `7`       | `1920`  | `1080` | `47` |

### Flips

| Name        | Type | Default | Description |
|-------------|------|---------|-------------|
| `Hflip`     | bool | `false` | Flip image horizontally. For Raspberry Pi this is passed as `--hflip` to `rpicam-vid`. On PC it uses `cv::flip`. |
| `Vflip`     | bool | `true`  | Flip image vertically. Passed as `--vflip` / `cv::flip` depending on backend. |

### Compression quality

These are only used when `Compression=true`.

| Name            | Type | Range   | Default | Description |
|-----------------|------|---------|---------|-------------|
| `JPEG_quality`  | int  | 0–100   | `90`    | Quality passed to `cv::imencode` for JPEG (`cv::IMWRITE_JPEG_QUALITY`). Higher = better quality, larger files. |
| `PNG_quality`   | int  | 0–9     | `1`     | Compression level passed to `cv::imencode` for PNG (`cv::IMWRITE_PNG_COMPRESSION`). Higher = smaller, slower. |
| `WEBP_quality`  | int  | 0–101   | `90`    | Quality passed to `cv::imencode` for WEBP (`cv::IMWRITE_WEBP_QUALITY`). |

The code also contains placeholders for AVIF / JPEG XL quality and speed parameters, but those parts are currently commented out and not active. If support for this two is needed uncomment the lines and install latest OPENCV release.

---

## How it works (high-level)

Internally the node:

1. Reads the configured parameters (`board`, `Compression`, `preview`, `mode`, `VCam_num`, flips, etc.).
2. Builds a capture pipeline:
   - For **Jetson ORIN/NANO**: uses `cv::VideoCapture` by a `GStreamer` pipeline.
   - For **Raspberry Pi**: constructs a `rpicam-vid` command piped into `ffmpeg` for camera frame reading.
   - For **PC**: uses `cv::VideoCapture` directly.
3. Starts a timer callback that:
   - Grabs a frame from the active capture.
   - Applies vertical/horizontal flips if requested.
   - Optionally shows a preview window.
   - Either:
     - Converts the frame to `sensor_msgs/msg/Image` and publishes to `/video_source/raw`, or
     - Encodes to the selected compressed format and publishes `sensor_msgs/msg/CompressedImage` to `/video_source/compressed`.
4. Listens for parameter updates and restarts the camera pipeline when critical parameters change (e.g., `board`, `mode`, `format`, flips or compression settings).

---

## References

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html

https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga461f9ac09887e47797a54567df3b8b63

https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html

https://askubuntu.com/questions/1542652/getting-rpicam-tools-rpicam-apps-working-on-ubuntu-22-04-lts-for-the-raspber

https://github.com/raspberrypi/libcamera

https://github.com/raspberrypi/rpicam-apps.git

https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera#Working_with_Raspberry_Pi_5_.28libcamera.29:
%7E:text=Camera%20Documentation.-,Working%20with%20Raspberry%20Pi%205%20(libcamera),-Bookworm%20will%20not

https://github.com/umlaeute/v4l2loopback

https://wiki.archlinux.org/title/V4l2loopback

https://github.com/IvanMeloFGLab/I-Manual-Pizzlebot-I
