# Chapter 3: Sensor Calibration for SLAM

## 3.1 Objective 
1. Understand what sensor calibration is and why it’s critical for localization and mapping.
2. Understand What is intrinisic and extrnisic callibration.
3. Perform intrinsic and extrinsic calibration using open-source tools.

The fundamental objective of sensor calibration in SLAM systems is to ensure accurate spatial perception by establishing precise mathematical relationships between sensor measurements and the physical world. As SLAM systems increasingly rely on multi-sensor fusion architectures, proper calibration becomes essential for maintaining spatial consistency across different sensing modalities. The goal is to minimize systematic errors that can propagate through the SLAM pipeline, ultimately affecting localization accuracy and map quality.

Current technological trends indicate a growing complexity in SLAM sensor configurations, with systems commonly integrating LiDAR, cameras, IMUs, wheel encoders, and other specialized sensors. Each sensor type introduces unique calibration challenges related to intrinsic parameters (internal characteristics) and extrinsic parameters (spatial relationships between sensors). The industry is moving toward more robust, automated calibration procedures that can adapt to environmental changes and sensor degradation over time.




## 3.2 Common Reference Frames in Sensor Fusion and Calibration

In localization, mapping, and multi-sensor fusion, consistent definition and transformation between reference frames is essential. Each frame represents a coordinate system attached to a specific entity (e.g., world, robot, sensor). Below are the most common frames used in SLAM and calibration systems.

  1. **World Frame**: The World frame (or Map frame) is a global, fixed reference system used to express the absolute position and orientation of the robot and environment.The Zₚ axis typically points upward (or opposite to gravity, depending on convention).The origin is arbitrarily set at the system initialization, often at the robot’s or camera’s starting pose.

  2. **Odome Frame**: is a local reference frame used to track the robot’s motion over time relative to its starting position. It is typically updated incrementally by wheel encoders, visual odometry, or other motion estimation sensors.


  3. **Body Frame**: represents the robot’s physical body, typically attached to the IMU or the vehicle chassis.
  It acts as the central reference for all onboard sensors (LiDAR, cameras, GNSS, etc.).In a right-handed coordinate system, the x-axis points forward in the robot’s motion direction, the y-axis points to the left, and the z-axis points upward.


  4. **Camera Frame**: The Camera Frame (C) is centered at the optical center of the camera and follows computer vision conventions. The z-axis points forward along the optical axis, x-axis points right, and y-axis points downward, aligned with image coordinates. This differs from the body or IMU frame (where z points upward). In multi-camera systems, each camera (C1,C2) has its own pose relative to the body frame, defined by the extrinsic transform.This frame is fundamental for projecting 3D world points onto the image plane.
  

  5. **Image (Pixel) Frame**: The Image Frame (I) is a 2D coordinate system on the camera’s image plane, where each point corresponds to a pixel location. The origin (u=0,v=0) is typically at the top-left corner, with the u-axis pointing right and v-axis pointing down. 3D points from the camera frame are projected into this frame using the camera intrinsic matrix (K) and distortion model.


## 3.3 Camera Callibration 

 **Intrinsic Callibration** 

Intrinsic calibration estimates the camera’s internal parameters, describing how 3D points in the camera frame are projected onto the 2D image plane.we use camera intrinsic parameter matrix, describes some of the physical properties of the camera lens. It is a 3x3 matrix with the following form:

  <div style="margin-left: 65px;">
  <img src="images/calib/Intinisic.png" alt="Camera coordinate system" width="300"/>
  </div>

  - fu and fv (fx and fy): These two parameters are the focal length of the camera, usually in pixels. fx for the x-direction and fy for the y-direction. Ideally, fx and fy should be equal, as most cameras have square pixels. However, in practical applications, they may differ due to reasons such as lens distortion and manufacturing errors.

  - cu and cv (cx and cy): These two parameters are the coordinates of the image center (principal point). Ideally, the principal point should be at the exact center of the image. However, in practice, it may deviate due to various reasons (e.g., lens shift, manufacturing error, etc.).

  Convert a point Pc=(Xc, Yc, Zc) in the camera coordinate system to a point Pi=(u,v) in the image/pixel coordinate system. The transformation process can be expressed as:
    
  <div style="margin-left: 65px;">
    <img src="images/calib/projection.webp" alt="Camera coordinate system" width="300"/>
  </div>


  **Distortion Coefficients**: Real lenses usually deviate from the ideal pinhole camera model, introducing significant distortion to images. As a result, camera parameters include Distortion Coefficients in addition to intrinsics and extrinsics. The main types of distortion are radial distortion and tangential distortion.


  
**Callibration Target**- Camera calibration relies on predictably shaped and easy to localize calibration targets. Knowing the targets to identify in calibration images along with their spacing allows the optimization to reason about how the targets exist in space.Popular targets include checkerboard patterns, Aruco markers, Charuco markers, and circle grids, with potential modifications on those few.

Checkerboard patterns are simple and easy to use.OpenCV has a chessboard calibration library that attempts to map points in 3D on a real-world chessboard to 2D camera coordinates.Any object could have been used (a book, a laptop computer, a car, etc.), but a chessboard has unique characteristics that make it well-suited for the job of correcting camera distortions:

- It is flat, so you don’t need to deal with the z-axis (z=0), only the x and y-axis. All the points on the chessboard lie on the same plane.

- There are clear corners and points, making it easy to map points in the 3D real world coordinate system to points on the camera’s 2D pixel coordinate system.
- The points and corners all occur on straight lines and with the same space.

This tutorial uses a 8x6 checkerboard with 0.23cm squares. Calibration uses the interior vertex points of the checkerboard, so an “9x7” board uses the interior vertex parameter “8x6” as in the example below. 

There are a number of camera calibration tools available to do this calibration, but if you’re already working in ROS, one of the easier options is the `camera_calibration` package.

1. To start first install docker and setup environment.
      ```
        docker compose exec demo bash --login
        source /opt/ros/humble/setup.bash
        colcon build 
      ```

2. The next thing we need is data for the calibration to be run on. Normally, you would be able to use a live camera feed for the intrinsic calibration, but to make this training more universally accessible and repeatable, we will be working from bag files.
Download `rosbag2_callibration1` file and put it `callibration_ws/resources` folder.

      In the first one, run the first rosbag file on loop.  
      ```
      cd ~/callibration_ws/resources
      ros2 bag play rosbag2_callibration1 
      ```
      In the second terminal, run the camera calibration node

      ```
      ros2 run camera_calibration cameracalibrator  --size 8x6 --square 0.023 --ros-args --remap image:=/zed/zed_node/left/color/raw/image  --remap camera:=zed/zed_node/left/color/raw
      ```
    We use the above command to calibrate the ZED camera’s left lens using the raw image topic `/zed/zed_node/left/color/raw/image`. The `--size 8x6` option specifies that the checkerboard used has 8 inner corners horizontally and 6 vertically, and `--square 0.023` sets the square size to 0.023 meters. The `--remap` arguments link the calibration node to the correct image and camera topics `namespace` recorded in the bag file.
    
3. You should see a pop-up.In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:checkerboard on the camera’s left, right, top and bottom of field of view
   - X bar - left/right in field of view
   - Y bar - top/bottom in field of view
   - Size bar - toward/away and tilt from the camera
   - checkerboard filling the whole field of view
   - checkerboard tilted to the left, right, top and bottom (Skew)



<table>
  <tr>
    <td>
      <figure>
        <img src="images/calib/size.gif" alt="Alt 4" width="400"/>
        <figcaption><strong>Figure 1</strong>- Size bar-toward/away from the camera </figcaption>
      </figure>
    </td>
    <td>
      <figure>
        <img src="images/calib/x-bar.gif" alt="Alt 1" width="400"/>
        <figcaption><strong>Figure 2</strong>- X bar - left/right in field of view</figcaption>
      </figure>
    </td>
    
  </tr>
  <tr>
    <td>
      <figure>
        <img src="images/calib/y-bar.gif" alt="Alt 2" width="400"/>
        <figcaption><strong>Figure 3</strong>- Y bar - top/bottom in field of view</figcaption>
      </figure>
    </td>
    <td>
      <figure>
        <img src="images/calib/skew.gif" alt="Alt 3" width="400"/>
        <figcaption><strong>Figure 4</strong>- Skew bar - checkerboard tilted.</figcaption>
      </figure>
    </td>

  </tr>
</table>

4. When all the 4 bars are green and enough data is available for calibration the **CALIBRATE** button will light up. Click it to see the results. It takes around the minute for calibration to take place.After the calibration is completed the **SAVE** and **commit** buttons light up. And you can also see the result in terminal.

    A successful calibration typically has a mean reprojection error below 0.3 px.The closer to zero, the better.In our case, the GUI shows a 0.23 px error (displayed as “lin” in Figure 5), indicating accurate calibration.



5. To record these parameters down, click save.It will save to `/tmp/calibrationdata.tar`.gz.Let’s get the files somewhere we can easily reach them

    ```
      mkdir ~/calibration_ws/mono_camera
      mv /tmp/calibrationdata.tar.gz ~/calibration_ws/mono_camera
      cd ~/calibration_ws/src/mono_camera
      tar -xvf calibrationdata.tar.gz
    ```

This records all the original images used for the calibration, as well as the calibration parameters in two files: `ost.txt` and `ost.yaml`. Different applications expect intrinsics in a number of different ways, so you’ll likely have to place particular parameters from these files in a certain structure.

The camera calibration YAML file stores the camera’s intrinsic parameters for image correction.  
The main parts are:

- **Image size** – width and height of the calibration images.  
- **Camera matrix (K)** – defines intrinsic parameters such as focal lengths and optical center.  
- **Distortion coefficients** – describe lens distortion for image undistortion.  
- **Rectification matrix (R)** – aligns images to a common plane; identity for mono cameras.  
- **Projection matrix (P)** – maps 3D camera coordinates to 2D image pixels and may differ from K after rectification.


The calibration results should be applied to the raw image so subsequent image processing can use corrected images. In ROS2, this is done in two steps:

1. Copy the calibration YAML file to `camera_pipeline` package.This uses the callibration file to rectify the raw image as `image_rect` topic.  

    ```
    cp ~/calibration_ws/camera0_cal/calibrationdata/ost.yaml \
      ~/calibration_ws/src/camera_pipeline/config/

    ros2 launch camera_pipeline camera_info.launch.py image_raw:=/zed/zed_node/left/color/raw/image

    ```

2. Open a new terminal and launch RViz with two image displays:
`/zed/zed_node/left/color/raw/image` and `/image_rect.`

Play the rosbag and pause it with the spacebar.
As illustrated in the images below, the left (raw) image appears distorted, with the vertical bar inside the red highlighted region noticeably curved. In contrast, the right (rectified) image shows the same bar as a straight line, demonstrating that the camera calibration and rectification process effectively corrected the lens distortion.


<table style="border-collapse: collapse; width: 100%; border: none;">
  <tr>
    <td style="padding: 0; text-align: center; vertical-align: top;">
      <figure style="margin: 0;">
        <img src="images/calib/1762350158_raw.png" alt="Alt 2" width="600" style="margin: 1; padding: 2; display: block;"/>
        <figcaption style="font-size: 14px; margin-top: 4px;">
          <strong>Figure 3</strong> – raw image
        </figcaption>
      </figure>
    </td>
    <td style="padding: 0; text-align: center; vertical-align: top;">
      <figure style="margin: 0;">
        <img src="images/calib/1762350158_rectified_.png" alt="Alt 3" width="600" style="margin: 1; padding: 2; display: block;"/>
        <figcaption style="font-size: 14px; margin-top: 4px;">
          <strong>Figure 4</strong> – corrected(rectifed) image
        </figcaption>
      </figure>
    </td>
  </tr>
</table>


**Extrinisic Callibration**

  Extrinsic calibration determines the camera’s pose relative to another reference frame, such as the world, body, or another sensor (e.g., camera or LiDAR). It defines how different sensors are spatially related and enables consistent projection of points between coordinate systems. In multi-camera or sensor-fusion setups, accurate extrinsic calibration ensures proper alignment between the camera, IMU, and LiDAR, which is essential for reliable stereo vision, visual-inertial odometry, and SLAM applications.

  For this experiment, we use the ZED stereo camera to perform extrinsic calibration.
  The goal is to estimate the relative pose of the right camera with respect to the left camera, which defines the stereo baseline — the physical separation and orientation difference between the two lenses.

  The resulting extrinsic parameters (rotation R and translation T) describe how to transform points from the right camera’s coordinate frame into the left camera’s coordinate frame. These parameters are later used for stereo rectification and depth estimation.

  1. First download the two provided ROS 2 bag files:

  - **`stereo_large_board_bagfile.bag`** – recorded using a small checkerboard *(2.3 cm squares)*  
  - **`stereo_small_board_bagfile.bag`** – recorded using a large checkerboard *(11.8 cm squares)*  

    Place both bag files in the `resources/` directory of the calibration workspace:

  2. Run the bag file and camera_calibration tool from the image_pipeline package to perform stereo calibration:

      In the first one, run the first rosbag file on loop.  
      ```
      cd ~/callibration_ws/resources
      ros2 bag play rosbag2_stereo_large_board
      ```
      In the second terminal, run the camera calibration node

      ```
      ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 8x6 --square 0.118 --ros-args --remap left:=/zed/zed_node/left/color/raw/image --remap right:=/zed/zed_node/right/color/raw/image    --remap left_camera:=zed/zed_node/left/color/raw --remap right_camera:=zed/zed_node/right/color/raw
      ```
  3. You should see a pop-up.In order to get a good calibration you will need to move the checkerboard around in the camera frame.When all the 4 bars are green and enough data is available for calibration the **CALIBRATE** button will light up. Click it to see the results. It takes around the minute for calibration to take place.After the calibration is completed the **SAVE** and **COMMIT** buttons light up. And you can also see the result in terminal.

  4. To record these parameters down, click save.It will save to `/tmp/calibrationdata.tar`.gz.Let’s get the files somewhere we can easily reach them

      ```
        mkdir ~/callibration_ws/resources/stereo_camera_large_board_cali/
        mv /tmp/calibrationdata.tar.gz ~/callibration_ws/resources/stereo_camera_large_board_cali/
        cd ~/calibration_ws/src/stereo_camera_large_board_cali
        tar -xvf calibrationdata.tar.gz
      ```
      Inside the extracted folder, you will find:`left.yaml`(parameters of the left camera) and `right.yaml`(parameters of the right camera) The structure of these files is similar to those obtained from intrinsic calibration. However, to analyze the extrinsic calibration, we focus on the **projection matrices** (P matrices).

      Left Camera Projection Matricx 
                                       
       ```    
         data: [ 279.01721 ,   0.      ,  318.71177,   0.   ,       
                    0.     , 279.01721 ,  179.75161,   0.   ,     
                    0.     ,   0.      ,   1.      ,   0.    ]     
       ```

      Right Camera Projection Matrix 

      ```
        data: [ 279.01721,   0.       ,  318.71177, -33.6074 ,
                  0.     ,  279.01721 ,  179.75161,   0.     ,
                  0.     ,   0.       ,   1.      ,   0.     ]
      ```

      The fourth element in the first row of the **right projection matrix** (here `-33.6074`) represents the **translation of the right camera along the x-axis** relative to the left camera.  
      This value corresponds to the **baseline distance** between the two camera centers when scaled by the focal length:

      Mathematically:
      $$
        \text{Baseline} = -\frac{P_{14}}{f_x}  
      $$

      Using the given data:
      $$
      \text{Baseline} = -\frac{-33.6074}{279.01721} \approx 0.1204 \, \text{m}
      $$

      Thus, the two cameras are separated by approximately **12.04 cm**, which matches the expected ZED stereo baseline.


      For parallel stereo cameras, the left and right cameras are almost perfectly aligned.This means the **rotation matrix** \(R\) between them is **close to the identity matrix**:which simplifies stereo processing.If the cameras were not parallel, a QR decomposition or SVD on a normalized version of the projection matrix can be used to to separate the rotation component


      Now consider the `stereo_small_board_bagfile.bag` with a small checkerboard (`--square 0.023` m):

      ```
      data: [284.05017 ,   0.       , 294.69019 , -34.89335,
                0.     ,  284.05017 , 181.08418 ,   0.     ,
                0.     ,   0.       ,   1.      ,   0.     ]
      ```

      $$
      \text{Baseline} = -\frac{-34.89335}{284.05017} \approx 0.123 \, \text{m}
      $$

      Using a large checkerboard, the reprojection error is very low, less than 0.2 pixels, whereas using a small checkerboard results in a much higher error, greater than 1.2 pixels. The manufacturer-specified baseline for the ZED stereo camera is 12 cm, which is closely matched by the baseline obtained with the large checkerboard. In contrast, the high epipolar reprojection error observed with the small checkerboard indicates that this calibration is poor, and the resulting baseline does not align with the expected manufacturer value.

      <table style="border-collapse: collapse; width: 100%; border: none;">
        <tr>
          <td style="padding: 0; text-align: center; vertical-align: top;">
            <figure style="margin: 0;">
              <img src="images/calib/big_board_projection_error.png" alt="Alt 2" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 14px; margin-top: 4px;">
                <strong>Epipolar reprojection error with large checkerboard</strong> 
              </figcaption>
            </figure>
          </td>
          <td style="padding: 0; text-align: center; vertical-align: top;">
             <figure style="margin: 0;">
              <img src="images/calib/small_board_projection_error.png" alt="Alt 2" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 14px; margin-top: 4px;">
                <strong>Epipolar reprojection error with small checkerboard</strong> 
              </figcaption>
            </figure>
          </td>
        </tr>
      </table>
