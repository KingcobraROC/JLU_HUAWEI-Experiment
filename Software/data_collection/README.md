# YOLO Image Collector

A modern image collection tool for creating YOLO datasets with a user-friendly Qt interface.

## Features

- Live camera feed display
- Manual image capture with spacebar support
- Auto-capture at configurable intervals
- Customizable color prefixes for image naming
- Configurable maximum number of images to collect
- High-quality JPEG output
- Pause/resume functionality
- Live preview of captured images

## Installation

1. Make sure you have Python 3.8 or higher installed
2. Install the required dependencies:
   ```bash
   pip install opencv-python pyside6
   ```

## Usage

Run the application:
```bash
python main.py
```

### Interface Guide

1. **Color Prefix**: Enter a color name or identifier that will be part of the image filename
2. **Max Images**: Set the maximum number of images you want to collect (default: 120)
3. **Output Folder**: Select where captured images will be saved
4. **Capture Methods**:
   - Manual: Click the "Capture Image" button or press SPACEBAR
   - Auto: Set a capture frequency (1-10 seconds) 

### Controls

- Start Camera: Begin the camera feed
- Stop Camera: Stop the camera feed
- Pause/Resume Camera: Temporarily pause or resume capture
- Progress: Shows current capture count against the maximum

## Configuration

- The application allows customization of the maximum images to capture
- Images are saved with the format: `{prefix}_{number}_{timestamp}.jpg`
- Supports camera selection (currently configured for camera 1, can be modified in code)

## Requirements

- Python 3.8+
- OpenCV
- PySide6