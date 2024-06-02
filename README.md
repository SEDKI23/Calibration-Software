# LidarCameraCalibrator Matlab Tool

![Testing_Vehicle](Testing_Vehicle.jpg)

## Intro
The LiDAR Camera Calibrator is a Matlab tool for LiDAR-camera extrinsic calibration. 
There are 2 approaches, the manual approach is to select the checkerboard inside the point clouds (The images and point clouds for this approach should be manually extracted from the Rosbag) and the calibration software that takes the Rosbags as inputs and gives the result as a vector.
(The updated version is for use with ROS2)  
Calibration Options : 
- Ouster with all 8 cameras.
- Aeva with the 3 front cameras.
- Velarray with the 3 rear cameras

## Getting started
**Calibration Software**  
In order to calibrate the sensor of the test vehicle, the folder structure should be compliant with the software, i.e. there should be exactly 5 folders for each LiDAR camera calibration (these are on each folder marked with: _c center,  
_rr far right, _r far right, _ll far left, _l left).
When the code is executed, each folder will be iterated and the preferred LiDAR-camera combination sensor data will be exported from the Rosbags and then saved as pcd-PNG pairs in an external folder.
The paths are already included in the code. So the Topics and Messages will be called automatically to generate the camera intrinsic matrix.  
Some Parameteres should be adjusted before starting the Calibration.
- squareSize : Square size of the checkerboard in mm (for the used checkerboard (600x800mm) 60 mm)
- Padding: Padding along each side of checkerboard.
- MinCornerMetric : 0.2 sets the threshold for the checkerboard corner metric to 0.2.  
Using a higher threshold value can reduce the number of false detections in a noisy or highly textured image.(Tip: This threshold should be adjusted if there is a problem with lighting (too dark or too bright) when recording sensor data.) 
- ROI is adjusted according to how LiDAR is used (for Ouster there are several cases. For some cases the ROI is manually entered and for some other cases there are functions that calculate the ROI, Ouster is flipped to negative x-axis).
- minDistance: This value should be greater than the minimum distance between two scan lines of the checkerboard. Too small value  might result in incorrect detections. 
- DimensionTolerance: A higher Dimension Tolerance indicates a more tolerant range for the rectangular plane dimensions. Default 0.2 for better results 


After the Ouster-Camera extrinsic calibration is done the Transformation should now be from the ouster to the `left_camera_optical_frame` that means the `right_camera_optical_frame` is missing.   
To get the Transformation from `ouster` to `drivers/zed_camera/front_center_base_link`we multiply the Result with `left_optical_to_camera_base_link` that is already given from the manufracturere in the A-Matrix. 

For Aeva and Velarray suffices the transofrmation from `left_camera_optical_frame` to the LiDAR.

The Result of the LiDAR-Camera Calibration will then  be printed as following:  
`The VELARRAY_LIDAR_rear_left_CAMERA yxz Yaw,Pitch,Roll is : 
[-1.46368609078609 -1.65244511223569 0.342115182697685 1.68736062800621 -0.629642949792449 -1.25897088714801]`


**Matlab Calibration App**
This approach is manual, meaning that the user should export the data from the Rosbags.    
In the calibration, a path for a Rosbag is needed to read the intrinsic parameters from this Rosbag and another path where all the PCD-PNG pairs are stored. In the code to start this application there is already an exporter that exports the pairs from only 1 bag. (The counter on the matching loop can be adjusted according to how many pairs are needed).      
To activate the calibrate button there should be at least 4 pairs on the accepted data, so the ROI and the checkerboard should be defined carefully.  
Defining a region of interest (ROI) around the checkerboard is very important to reduce the computational resources required by the transformation estimation process.    
There is also an option to adjust the dimension tolerance value to get more pairs accepted.  
Also for this calibration the checker size and the padding vector should be entered on the code before starting the application.  
The instrincs will be computed when you start the app, but to get the best result it would be best to call the intrinsic matrix from the workspace named instrincs.    
At the end of the Calibration the transformation and error metric data can be exported as workspace variables or MAT files. You can also create a MATLAB script for the entire workflow.


**Helpful Sources :**  
https://de.mathworks.com/help/lidar/ug/get-started-lidar-camera-calibrator.html
https://de.mathworks.com/help/lidar/ref/estimatecheckerboardcorners3d.html
https://de.mathworks.com/help/lidar/ref/detectrectangularplanepoints.html
https://de.mathworks.com/help/lidar/ref/estimatelidarcameratransform.html
https://de.mathworks.com/help/lidar/ug/lidar-camera-calibration-guidelines.html#mw_6dcbc9d3-ad9e-43f5-ac6e-dec5b5f7e929
https://de.mathworks.com/help/lidar/ref/lidarcameracalibrator-app.html

