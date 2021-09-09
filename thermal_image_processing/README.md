# Thermal_image_processing Package Detail

## NODES: `thermal_detection`
This package takes care of detecting a highest spot in the camera view and reading its temperature from the thermal camera.
Using USB video class (UVC), it captures thermal imaging data and reads the temperature from the radiometry in the gray scale.
This node was implemented based on PureThermal UVC Capture Examples. Refer to:
 https://github.com/groupgets/purethermal1-uvc-capture

### SUBSCRIBER & PUBLISHER

Subscriber:
- none

Publisher:
- object_y (std_msgs/Float32): y-coordinate of the heat source in 2D thermal image 
- object_x (std_msgs/Float32): x-coordinate of the heat source in 2D thermal image 
- highest_T_detected (std_msgs/Float32): the highest tempertaure detec

## LAUNCH
`fire_detect.launch` starts `thermal_detection` node