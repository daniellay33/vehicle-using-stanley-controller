# CARLA Vision-Based Stanley Controller

This project implements a vision-based Stanley controller for an autonomous vehicle in the [CARLA](https://carla.org/) simulator. The script uses a front-facing dashcam, detects lane lines in real-time, and calculates the required steering angle to keep the vehicle centered in its lane.

## Features

* **Vision-Based Lane Detection:** Uses OpenCV to extract lane lines via color filtering (HSV), edge detection (Canny), and Hough Transform.
* **Stanley Controller:** A steering control algorithm that combines cross-track error (distance from the lane center) and heading error to provide smooth and stable driving.
* **Automated Environment Setup:** Automatically loads the Town04 map (highway), spawns a vehicle at a random spawn point to avoid collisions, and positions a Spectator camera in a bird's-eye view above the car.
* **Debug Visualization:** Displays popup windows showing the raw video stream and the processed image with the detected lane lines.

## Prerequisites

To run this code, ensure you have the following installed and running:

* CARLA Simulator (version 0.9.x) running in the background.
* Python 3.x
* Python libraries (can be installed via `pip`):
  * `carla`
  * `opencv-python` (cv2)
  * `numpy`

## Usage

1. Start your CARLA server (usually by running `CarlaUE4.exe` or the equivalent executable for your operating system).
2. Note that the script is configured to connect to `localhost` on port `2000` with a 60-second timeout.
3. Open a terminal, navigate to the directory where you saved the script (e.g., `vision_stanley.py`), and run:

   ```bash
   python vision_stanley.py
