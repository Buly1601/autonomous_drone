# Autonomous Drone Competition

### In collaboration with José Miguel Zúñiga Juárez, Pablo Emilio Peredo Vega and Aldo Oziel Peña Gamboa

## Dynamic
This competition's dynamic consists on an algorithm that enables the drone to autonomously pass through the squares using OpenCV, ROS, and Python. The winner team is the team that is able to cross through the most squares.

## Artificial Vision
The first part of the project is to create an artificial vision algorithm that recognizes orange squares, computes the center of those squares and filters other squares from the background. The first contribution and try was made with image filters, no deep learning was used for it.
### Color
For the first try, the color was determined at the last minute as an HSV color value, this made it hard being that it was in the open field, and due to the sun, the orange color could shift the brightness to a near white or a super light orange. This issue has to be solved with other types of filter for version 2 of the algorithm. On that day, the upper and lower HSV of values were `(0, 240, 160), (30, 255, 255)`.
### Boxes, Masks and squares
After getting only the pixels that fall in the range of the HSV values, then opening and closing is done to the image to reduce noise. First opening was done with a (9,9) eliptical kernel, then closing using another (15,15) eliptical kernel. Refer [here](https://docs.opencv.org/3.4/d9/d61/tutorial_py_morphological_ops.html) to understand more about morfology.
After getting the contours drawn, then the image locates the center of a square (if any) and draws a contour around it.

## Drone Control
For controlling the drone, 'teleop_twist_keyboard.py' can be used which has been modified from https://github.com/yakovkor/keyboard-control-for-bebop-2-drone, this version allows the user to control the drone with WASD and also it adds functions in order to customize orders so the drone can follow an specific path.

## Usage
1. Follow [this](https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev) repo to setup bebop's ROS Noetic's drivers and dependencies.
2. Clone [this](https://github.com/Buly1601/autonomous_drone) repo into your local machine (Has to be Ubuntu 20.04.6 with ROS Noetic).
3. Install requirements using `pip install -r requirements.txt`





used : https://bebop-autonomy.readthedocs.io/en/latest/
https://answers.ros.org/question/297036/bebop2-keyboard-control/

