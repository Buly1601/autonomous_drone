# Autonomous Drone Competition

### In collaboration with José Miguel Zúñiga Juárez, Pablo Emilio Peredo Vega and Aldo Oziel

## Dynamic
This competition's dynamic consists on an algorithm that enables the drone to autonomously pass through the squares using OpenCV, ROS, and Python. The winner team is the team that is able to cross through the most squares.

## Artificial Vision
The first part of the project is to create an artificial vision algorithm that recognizes orange squares, computes the center of those squares and filters other squares from the background. The first contribution and try was made with image filters, no deep learning was used for it.
### Color
For the first try, the color was determined at the last minute as an HSV color value, this made it hard being that it was in the open field, and due to the sun, the orange color could shift the brightness to a near white or a super light orange. This issue has to be solved with other types of filter for version 2 of the algorithm. On that day, the upper and lower HSV of values were `(0, 240, 160), (30, 255, 255)`.
### Boxes, Masks and squares
After getting only the pixels that fall in the range of the HSV values, then opening and closing is done to the image to reduce noise. First opening was done with a (9,9) eliptical kernel, then closing using another (15,15) eliptical kernel. Refer [here](https://docs.opencv.org/3.4/d9/d61/tutorial_py_morphological_ops.html) to understand more about morfology.
After getting the contours drawn, then the image locates the center of a square (if any) and draws a contour around it.






used : https://bebop-autonomy.readthedocs.io/en/latest/
https://answers.ros.org/question/297036/bebop2-keyboard-control/

