# Computer Vision & SLAM Reference

## Raspberry Pi Camera Setup

### Camera Module Types

#### Pi Camera V2
- 8 megapixel sensor (Sony IMX219)
- 1080p30, 720p60, 640x480p60/90 video
- Fixed focus
- 62.2° horizontal, 48.8° vertical field of view

#### Pi Camera V3
- 12 megapixel sensor (Sony IMX708)
- Improved low-light performance
- Autofocus support
- HDR capabilities

#### HQ Camera
- 12.3 megapixel sensor (Sony IMX477)
- C/CS-mount for interchangeable lenses
- Larger sensor for better image quality

### Hardware Installation

1. **Locate CSI connector** on Raspberry Pi (between HDMI and audio jack)
2. **Gently pull up** on connector edges to open
3. **Insert ribbon cable** with blue side facing audio jack (contacts facing HDMI)
4. **Push down** connector to secure cable

### Software Configuration

```bash
# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable

# Update system
sudo apt update
sudo apt full-upgrade

# Install Picamera2 (pre-installed on Raspberry Pi OS Bullseye+)
sudo apt install -y python3-picamera2

# Legacy camera stack (for older projects)
sudo apt install -y python3-picamera
```

### Test Camera

```bash
# Capture still image (libcamera)
libcamera-still -o test.jpg

# Capture video (5 seconds)
libcamera-vid -t 5000 -o test.h264

# Show preview
libcamera-hello
```

## Picamera2 Python Library

### Basic Still Capture

```python
from picamera2 import Picamera2
import time

# Initialize camera
picam2 = Picamera2()

# Configure for still capture
config = picam2.create_still_configuration()
picam2.configure(config)

# Start camera
picam2.start()

# Wait for auto exposure/white balance to settle
time.sleep(2)

# Capture image
picam2.capture_file("image.jpg")

# Stop camera
picam2.stop()
```

### Video Recording

```python
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

picam2 = Picamera2()

# Configure for video
config = picam2.create_video_configuration()
picam2.configure(config)

# Set up encoder
encoder = H264Encoder(bitrate=10000000)
output = FileOutput("video.h264")

# Start recording
picam2.start_recording(encoder, output)

# Record for 10 seconds
time.sleep(10)

# Stop recording
picam2.stop_recording()
```

### Capture NumPy Array (for OpenCV)

```python
from picamera2 import Picamera2
import cv2
import numpy as np

picam2 = Picamera2()

# Configure for low-resolution video (faster processing)
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)

picam2.start()

try:
    while True:
        # Capture frame as NumPy array
        frame = picam2.capture_array()

        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Process with OpenCV
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Display
        cv2.imshow("Edges", edges)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
```

### Camera Configuration Options

```python
from picamera2 import Picamera2

picam2 = Picamera2()

# Get available configurations
print(picam2.sensor_modes)

# Custom configuration
config = picam2.create_video_configuration(
    main={"size": (1920, 1080), "format": "RGB888"},
    controls={
        "FrameRate": 30,
        "ExposureTime": 10000,  # microseconds
        "AnalogueGain": 1.0,
        "AeEnable": True,  # Auto exposure
        "AwbEnable": True,  # Auto white balance
        "Brightness": 0.0,  # -1.0 to 1.0
        "Contrast": 1.0,    # 0.0 to 32.0
    }
)

picam2.configure(config)
picam2.start()
```

## OpenCV Installation & Cross-Compilation

### Quick Install (Pre-built)

```bash
# Install OpenCV with Python bindings
sudo apt install -y python3-opencv

# Verify installation
python3 -c "import cv2; print(cv2.__version__)"
```

**Note**: Pre-built version may lack optimizations and latest features.

### Cross-Compilation for Raspberry Pi (Ubuntu Host)

#### Prerequisites

Host system: Ubuntu 23.04+ (x86-64)
Target: Raspberry Pi OS (armv7 or aarch64)

**Critical**: Host and target must use **identical Ubuntu/Debian versions**.

#### Step 1: MultiArch Setup

```bash
# Add target architecture
sudo dpkg --add-architecture arm64  # For 64-bit Pi OS
# or
sudo dpkg --add-architecture armhf  # For 32-bit Pi OS

# Update package lists
sudo apt update

# Install cross-compilation tools
sudo apt install -y crossbuild-essential-arm64 \
                     cmake ninja-build pkgconf git

# For 32-bit (armhf):
# sudo apt install -y crossbuild-essential-armhf
```

#### Step 2: Install Target Dependencies

```bash
# Install ARM libraries on host system
sudo apt install -y \
    libavcodec-dev:arm64 \
    libavformat-dev:arm64 \
    libavutil-dev:arm64 \
    libswscale-dev:arm64 \
    libfreetype-dev:arm64 \
    libharfbuzz-dev:arm64 \
    libgtk-3-dev:arm64 \
    libjpeg-dev:arm64 \
    libpng-dev:arm64 \
    libtiff-dev:arm64 \
    libatlas-base-dev:arm64 \
    gfortran:arm64

# For Python support
sudo apt install -y \
    python3-dev:arm64 \
    python3-numpy:arm64
```

#### Step 3: Download OpenCV

```bash
mkdir ~/opencv_build && cd ~/opencv_build

# Download OpenCV
git clone --depth 1 --branch 4.8.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.8.0 https://github.com/opencv/opencv_contrib.git
```

#### Step 4: Configure Build (64-bit ARM)

```bash
cd opencv
mkdir build && cd build

# Set pkg-config path for cross-compilation
export PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig

cmake -GNinja \
    -DCMAKE_TOOLCHAIN_FILE=../platforms/linux/aarch64-gnu.toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/opencv-arm64 \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DWITH_TBB=ON \
    -DWITH_V4L=ON \
    -DWITH_OPENGL=ON \
    -DENABLE_NEON=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DPYTHON3_INCLUDE_DIR=/usr/include/aarch64-linux-gnu/python3.11 \
    -DPYTHON3_LIBRARIES=/usr/lib/aarch64-linux-gnu/libpython3.11.so \
    -DPYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include \
    ..
```

#### Step 5: Compile

```bash
# Build (adjust -j based on CPU cores)
ninja -j4

# Install to staging directory
sudo ninja install
```

#### Step 6: Package and Deploy

```bash
# Create archive
cd /opt
sudo tar czf opencv-arm64-4.8.0.tar.gz opencv-arm64/

# Transfer to Raspberry Pi
scp opencv-arm64-4.8.0.tar.gz pi@raspberrypi:~

# On Raspberry Pi: Extract
cd /opt
sudo tar xzf ~/opencv-arm64-4.8.0.tar.gz

# Update library path
echo "/opt/opencv-arm64/lib" | sudo tee /etc/ld.so.conf.d/opencv.conf
sudo ldconfig

# Verify dependencies
ldd /opt/opencv-arm64/lib/libopencv_core.so
```

#### Step 7: Python Integration

```bash
# On Raspberry Pi: Link to Python
sudo ln -s /opt/opencv-arm64/lib/python3.11/site-packages/cv2 \
           /usr/local/lib/python3.11/dist-packages/cv2

# Test
python3 -c "import cv2; print(cv2.__version__)"
```

### Performance Optimization Flags

For additional ARM optimizations:

```cmake
-DENABLE_NEON=ON          # ARM NEON SIMD instructions
-DENABLE_VFPV3=ON         # Vector Floating Point v3
-DCPU_BASELINE=NEON       # Minimum CPU features
-DCPU_DISPATCH=NEON,VFPV3 # Runtime CPU detection
```

## OpenCV Basic Operations

### Image Processing

```python
import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

while True:
    # Capture frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Grayscale conversion
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Gaussian blur (noise reduction)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Contour detection
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

    # Display
    cv2.imshow("Frame", frame)
    cv2.imshow("Edges", edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
```

### Color Detection

```python
import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color range (red example)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Morphological operations (remove noise)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter small objects
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "Red Object", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
```

### Object Tracking (CAMShift)

```python
import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

# Initial target selection (first frame)
frame = picam2.capture_array()
frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

# Select ROI (Region of Interest)
bbox = cv2.selectROI("Select Target", frame, False)
cv2.destroyWindow("Select Target")

x, y, w, h = bbox
track_window = (x, y, w, h)

# Set up initial histogram
roi = frame[y:y+h, x:x+w]
hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)),
                   np.array((180., 255., 255.)))
roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

# Termination criteria
term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

    # Apply CAMShift
    ret, track_window = cv2.CamShift(dst, track_window, term_crit)

    # Draw tracking result
    pts = cv2.boxPoints(ret)
    pts = np.intp(pts)
    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
```

## ORB-SLAM2 on Raspberry Pi

### Overview

ORB-SLAM2 is a real-time monocular, stereo, and RGB-D SLAM system for:
- **Localization**: Track camera pose in real-time
- **Mapping**: Build sparse 3D map of environment
- **Loop Closure**: Recognize previously visited locations
- **Relocalization**: Recover from tracking loss

### Hardware Requirements

- **Raspberry Pi 3 B+ or better** (Pi 4 recommended)
- **Pi Camera V2** (or compatible camera)
- **MPU9250 IMU** (optional, for sensor fusion)
- **Adequate cooling** (heatsink + fan)
- **Power supply**: 5V 3A minimum

### Software Dependencies

#### 1. Pangolin (Visualization)

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

#### 2. OpenCV 3.4.6

```bash
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.6.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.6.zip

unzip opencv.zip
unzip opencv_contrib.zip

cd opencv-3.4.6
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.4.6/modules \
      -D ENABLE_NEON=ON \
      -D ENABLE_VFPV3=ON \
      -D BUILD_TESTS=OFF \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D BUILD_EXAMPLES=OFF ..

make -j4
sudo make install
sudo ldconfig
```

**Note**: Compilation takes 2-4 hours on Raspberry Pi 3. Consider cross-compilation for faster builds.

#### 3. Eigen 3.2.10

```bash
sudo apt install libeigen3-dev
```

#### 4. ORB-SLAM2

```bash
cd ~
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2

# Edit build script if needed (username paths)
chmod +x build.sh
./build.sh
```

### Camera Calibration

ORB-SLAM2 requires camera intrinsics and distortion parameters.

#### Calibration Process

1. **Print calibration pattern**: Download checkerboard from OpenCV docs
2. **Capture images**: 20+ images of pattern from different angles
3. **Run calibration**:

```python
import cv2
import numpy as np
import glob

# Checkerboard dimensions (internal corners)
CHECKERBOARD = (7, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 3D points
imgpoints = []  # 2D points

# Load calibration images
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                     gray.shape[::-1], None, None)

# Print results
print("Camera matrix:")
print(mtx)
print("\nDistortion coefficients:")
print(dist)

# Save to YAML (ORB-SLAM2 format)
# fx, fy, cx, cy, k1, k2, p1, p2
```

#### Configuration File (YAML)

Create `Examples/Monocular/PiCamera.yaml`:

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters
#--------------------------------------------------------------------------------------------

# Camera calibration (from calibration script)
Camera.fx: 530.0
Camera.fy: 530.0
Camera.cx: 320.0
Camera.cy: 240.0

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

# Camera resolution
Camera.width: 640
Camera.height: 480

# Camera frames per second
Camera.fps: 30.0

# Color order (0: BGR, 1: RGB)
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold (adjust if features not detected)
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```

### Running ORB-SLAM2

#### Test with Public Dataset (TUM)

```bash
cd ~/ORB_SLAM2

# Download TUM dataset
wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
tar xzf rgbd_dataset_freiburg1_xyz.tgz

# Run ORB-SLAM2
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/TUM1.yaml \
    rgbd_dataset_freiburg1_xyz
```

#### Live with Pi Camera

Create Python wrapper (`run_picamera_slam.py`):

```python
from picamera2 import Picamera2
import cv2
import subprocess
import numpy as np

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# Start ORB-SLAM2 process (reading from stdin)
slam_process = subprocess.Popen([
    "./Examples/Monocular/mono_live",
    "Vocabulary/ORBvoc.txt",
    "Examples/Monocular/PiCamera.yaml"
], stdin=subprocess.PIPE)

try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Send frame to ORB-SLAM2
        slam_process.stdin.write(frame_bgr.tobytes())
        slam_process.stdin.flush()

except KeyboardInterrupt:
    slam_process.terminate()

finally:
    picam2.stop()
```

**Note**: This requires modifying ORB-SLAM2 to accept frames from stdin. Alternatively, save frames to disk and process in batch.

### ORB-SLAM2 Output

- **Camera trajectory**: Estimated poses over time
- **Point cloud map**: Sparse 3D reconstruction
- **Loop closures**: Detected revisited locations
- **Tracking status**: SLAM state (initializing, tracking, lost)

### Integration with Robot Navigation

```python
# Pseudocode for navigation integration

while robot_is_running:
    # Get current frame
    frame = capture_camera_frame()

    # Feed to ORB-SLAM2
    pose = orbslam.process_frame(frame)

    if pose is not None:
        # Extract position and orientation
        x, y, z = pose.translation
        roll, pitch, yaw = pose.rotation

        # Send to path planner
        next_waypoint = path_planner.get_next_waypoint(x, y, yaw)

        # Calculate motor commands
        linear_vel, angular_vel = motion_controller.compute(next_waypoint, pose)

        # Send to motors
        send_motor_command(linear_vel, angular_vel)
    else:
        # Tracking lost, stop robot
        stop_motors()
```

## Troubleshooting

### Camera Issues

**Problem**: Camera not detected
- Enable camera in `raspi-config`
- Check ribbon cable connection (contacts facing HDMI)
- Test with `libcamera-hello`

**Problem**: Poor image quality
- Clean lens
- Adjust focus (if adjustable lens)
- Configure exposure and white balance
- Ensure adequate lighting

### OpenCV Performance

**Problem**: Slow frame processing
- Reduce resolution (e.g., 320x240)
- Optimize code (avoid unnecessary conversions)
- Use NEON-optimized OpenCV build
- Overclock Raspberry Pi (with cooling)

**Problem**: Import cv2 fails
- Check Python version matches OpenCV build
- Verify library paths with `ldconfig -p | grep opencv`
- Check dependencies with `ldd /path/to/cv2.so`

### ORB-SLAM2 Issues

**Problem**: Initialization fails
- Move camera slowly with textured environment in view
- Increase `ORBextractor.nFeatures` in config
- Reduce `ORBextractor.iniThFAST` threshold
- Ensure adequate lighting

**Problem**: Tracking frequently lost
- Avoid rapid camera motion
- Improve lighting conditions (avoid backlight/glare)
- Reduce speed of robot movement
- Add IMU for sensor fusion (requires modification)

**Problem**: High CPU usage
- Reduce camera resolution
- Decrease `ORBextractor.nFeatures`
- Disable visualization (comment out Viewer code)
- Overclock Raspberry Pi

**Problem**: Memory errors
- Increase swap file size
- Close other applications
- Use Raspberry Pi 4 with 4GB+ RAM
- Reduce map size (implement map pruning)

## Best Practices

### Camera Usage
- Always use try-finally to ensure camera.stop() is called
- Wait for auto-exposure to settle before critical captures
- Use appropriate resolution for task (higher ≠ always better)
- Consider frame rate vs. processing time tradeoff

### OpenCV Optimization
- Convert images to grayscale when color not needed
- Use appropriate blur kernel sizes (odd numbers: 3, 5, 7)
- Pre-allocate arrays when possible (avoid repeated allocation)
- Profile code to identify bottlenecks

### SLAM Integration
- Calibrate camera carefully (affects accuracy significantly)
- Test SLAM with datasets before live deployment
- Implement fallback navigation (odometry) when tracking lost
- Log trajectory and map data for post-analysis
- Monitor CPU temperature and add cooling if necessary
