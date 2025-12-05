
## Objective 
1. Understand what sensor calibration is and why it’s critical for localization and mapping.
2. Understand What is intrinisic and extrnisic callibration.
3. Perform intrinsic and extrinsic calibration using open-source tools.

The fundamental objective of sensor calibration in SLAM systems is to ensure accurate spatial perception by establishing precise mathematical relationships between sensor measurements and the physical world. As SLAM systems increasingly rely on multi-sensor fusion architectures, proper calibration becomes essential for maintaining spatial consistency across different sensing modalities. The goal is to minimize systematic errors that can propagate through the SLAM pipeline, ultimately affecting localization accuracy and map quality.

Current technological trends indicate a growing complexity in SLAM sensor configurations, with systems commonly integrating LiDAR, cameras, IMUs, wheel encoders, and other specialized sensors. Each sensor type introduces unique calibration challenges related to intrinsic parameters (internal characteristics) and extrinsic parameters (spatial relationships between sensors). The industry is moving toward more robust, automated calibration procedures that can adapt to environmental changes and sensor degradation over time.




## Common Reference Frames in Sensor Fusion and Calibration

In localization, mapping, and multi-sensor fusion, consistent definition and transformation between reference frames is essential. Each frame represents a coordinate system attached to a specific entity (e.g., world, robot, sensor). Below are the most common frames used in SLAM and calibration systems.

  1. **World Frame**: The World frame (or Map frame) is a global, fixed reference system used to express the absolute position and orientation of the robot and environment.The Zₚ axis typically points upward (or opposite to gravity, depending on convention).The origin is arbitrarily set at the system initialization, often at the robot’s or camera’s starting pose.

  2. **Odome Frame**: is a local reference frame used to track the robot’s motion over time relative to its starting position. It is typically updated incrementally by wheel encoders, visual odometry, or other motion estimation sensors.


  3. **Body Frame**: represents the robot’s physical body, typically attached to the IMU or the vehicle chassis.
  It acts as the central reference for all onboard sensors (LiDAR, cameras, GNSS, etc.).In a right-handed coordinate system, the x-axis points forward in the robot’s motion direction, the y-axis points to the left, and the z-axis points upward.


  4. **Camera Frame**: The Camera Frame (C) is centered at the optical center of the camera and follows computer vision conventions. The z-axis points forward along the optical axis, x-axis points right, and y-axis points downward, aligned with image coordinates. This differs from the body or IMU frame (where z points upward). In multi-camera systems, each camera (C1,C2) has its own pose relative to the body frame, defined by the extrinsic transform.This frame is fundamental for projecting 3D world points onto the image plane.
  

  5. **Image (Pixel) Frame**: The Image Frame (I) is a 2D coordinate system on the camera’s image plane, where each point corresponds to a pixel location. The origin (u=0,v=0) is typically at the top-left corner, with the u-axis pointing right and v-axis pointing down. 3D points from the camera frame are projected into this frame using the camera intrinsic matrix (K) and distortion model.


###  Camera Callibration 

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

1. To start first install docker and setup environment. This docker environment installs `camera_callibration` package within `ros2-humble`.

    ```bash
    git --recurse-submodules clone git@github.com:eliyaskidnae/slam-tutorial-practical.git
    cd slam-tutorial-practical/camera_callibration_ws/
    docker compose up --build -d 
    docker compose exec callibration bash --login
    source /opt/ros/humble/setup.bash
    colcon build
    ```

2. The next thing we need is data for the calibration to be run on. Normally, you would be able to use a live camera feed for the intrinsic calibration, but to make this training more universally accessible and repeatable, we will be working from bag files.
   Download [rosbag2_callibration1](https://saxion.data.surf.nl/s/ifDcJqiMds4yydb) file and put it `camera_callibration_ws/resources` folder.

    In the first one, run the first rosbag file on loop inside docker environment.  
    
    ```bash
    docker compose exec callibration bash --login
    cd ~/callibration_ws/resources
    ros2 bag play rosbag2_callibration1
    ```
     
    In the second terminal, run the camera calibration node

    ```bash
    docker compose exec callibration bash --login
    source install/setup.bash
    ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.023 --ros-args --remap image:=/zed/zed_node/left/color/raw/image --remap camera:=zed/zed_node/left/color/raw
    ```

    We use the above command to calibrate the ZED camera's left lens using the raw image topic `/zed/zed_node/left/color/raw/image`. The `--size 8x6` option specifies that the checkerboard used has 8 inner corners horizontally and 6 vertically, and `--square 0.023` sets the square size to 0.023 meters. The `--remap` arguments link the calibration node to the correct image and camera topics `namespace` recorded in the bag file.

3. You should see a pop-up. In order to get a good calibration you will need to move the checkerboard around in the camera frame such that: checkerboard on the camera's left, right, top and bottom of field of view
    
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
            <figcaption>a) Size bar-toward/away from the camera </figcaption>
          </figure>
        </td>
        <td>
          <figure>
            <img src="images/calib/x-bar.gif" alt="Alt 1" width="400"/>
            <figcaption>b) X bar - left/right in field of view</figcaption>
          </figure>
        </td>
        
      </tr>
      <tr>
        <td>
          <figure>
            <img src="images/calib/y-bar.gif" alt="Alt 2" width="400"/>
            <figcaption>c) Y bar - top/bottom in field of view</figcaption>
          </figure>
        </td>
        <td>
          <figure>
            <img src="images/calib/skew.gif" alt="Alt 3" width="400"/>
            <figcaption>d) Skew bar - checkerboard tilted.</figcaption>
          </figure>
        </td>

      </tr>
      
    </table>

    <p style="text-align:center;"><strong>Figure 1: Required Camera Motions During Calibration</strong></p>
    
4. When all the 4 bars are green and enough data is available for calibration the **CALIBRATE** button will light up. Click it to see the results. It takes around the minute for calibration to take place.After the calibration is completed the **SAVE** and **commit** buttons light up. And you can also see the result in terminal.

    A successful calibration typically has a mean reprojection error below 0.3 px.The closer to zero, the better.In our case, the GUI shows a 0.23 px error (displayed as “lin” in Figure 5), indicating accurate calibration.


5. To record these parameters down, click save.It will save to `/tmp/calibrationdata.tar`.gz.Let’s get the files somewhere we can easily reach them

    ```bash
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

    ```bash
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
         a) raw image
        </figcaption>
      </figure>
    </td>
    <td style="padding: 0; text-align: center; vertical-align: top;">
      <figure style="margin: 0;">
        <img src="images/calib/1762350158_rectified_.png" alt="Alt 3" width="600" style="margin: 1; padding: 2; display: block;"/>
        <figcaption style="font-size: 14px; margin-top: 4px;">
         b) corrected(rectifed) image
        </figcaption>
      </figure>
    </td>
  </tr>
</table>

<p style="text-align:center;"><strong>Figure 2: Comparison of Raw and Rectified Images</strong></p>

**Extrinisic Callibration**

  Extrinsic calibration determines the camera’s pose relative to another reference frame, such as the world, body, or another sensor (e.g., camera or LiDAR). It defines how different sensors are spatially related and enables consistent projection of points between coordinate systems. In multi-camera or sensor-fusion setups, accurate extrinsic calibration ensures proper alignment between the camera, IMU, and LiDAR, which is essential for reliable stereo vision, visual-inertial odometry, and SLAM applications.

  For this experiment, we use the ZED stereo camera to perform extrinsic calibration.
  The goal is to estimate the relative pose of the right camera with respect to the left camera, which defines the stereo baseline — the physical separation and orientation difference between the two lenses.

  The resulting extrinsic parameters (rotation R and translation T) describe how to transform points from the right camera’s coordinate frame into the left camera’s coordinate frame. These parameters are later used for stereo rectification and depth estimation.

  1. First download the two provided ROS 2 bag files:

      - [stereo_large_board_bagfile.bag](https://saxion.data.surf.nl/s/ZjYXD6xpawgeb4Z) – recorded using a small checkerboard *(2.3 cm squares)*  
      - [stereo_small_board_bagfile.bag](https://saxion.data.surf.nl/s/ZjYXD6xpawgeb4Z) – recorded using a large checkerboard *(11.8 cm squares)*  

      Place both bag files in the `camera_callibration_ws/resources/` directory of the calibration workspace:

  2. Run the bag file and camera_calibration tool from the image_pipeline package to perform stereo calibration:

      In the first one, run the first rosbag file on loop.  
      ```bash
      docker compose exec callibration bash --login
      cd resources/
      ros2 bag play rosbag2_stereo_large_board
      ```

      In the second terminal, run the camera calibration node

      ```bash
      docker compose exec callibration bash --login

      ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 8x6 \
       --square 0.118 --ros-args --remap left:=/zed/zed_node/left/color/raw/image \ 
       --remap right:=/zed/zed_node/right/color/raw/image  \
       --remap left_camera:=zed/zed_node/left/color/raw \
       --remap right_camera:=zed/zed_node/right/color/raw
      ```

  3. You should see a pop-up.In order to get a good calibration you will need to move the checkerboard around in the camera frame.When all the 4 bars are green and enough data is available for calibration the **CALIBRATE** button will light up. Click it to see the results. It takes around the minute for calibration to take place.After the calibration is completed the **SAVE** and **COMMIT** buttons light up. And you can also see the result in terminal.

  4. To record these parameters down, click save.It will save to `/tmp/calibrationdata.tar`.gz.Let’s get the files somewhere we can easily reach them

      ```bash
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

      Using a large checkerboard, the reprojection error is very low, less than 0.2 pixels, whereas using a small checkerboard results is a much higher error, greater than 1.2 pixels. The manufacturer-specified baseline for the ZED stereo camera is 12 cm, which is closely matched by the baseline obtained with the large checkerboard. In contrast, the high epipolar reprojection error observed with the small checkerboard indicates that this calibration is poor, and the resulting baseline does not align with the expected manufacturer value.

      <table style="border-collapse: collapse; width: 100%; border: none;">
        <tr>
          <td style="padding: 0; text-align: center; vertical-align: top;">
            <figure style="margin: 0;">
              <img src="images/calib/big_board_projection_error.png" alt="Alt 2" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 14px; margin-top: 4px;">
                a) shows an epipolar reprojection error of 0.12 pixels
              </figcaption>
            </figure>
          </td>
          <td style="padding: 0; text-align: center; vertical-align: top;">
            <figure style="margin: 0;">
              <img src="images/calib/small_board_projection_error.png" alt="Alt 2" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 14px; margin-top: 4px;">
                b) shows epipolar reprojection error of 1.37 pixels
              </figcaption>
            </figure>
          </td>
           <figcaption style="font-size: 14px; margin-top: 4px;">
                <strong> </strong> 
           </figcaption>
        </tr>


      </table>
      
      <p style="text-align:center;"><strong>Figure 3: Calibration results using different checkerboard sizes</strong></p>


## Camera-IMU Callibration

The goal of camera-IMU extrinsic calibration is to accurately determine the transformation that defines the spatial relationship between the camera and the IMU.In this tutorial, we use [Kalibr](https://github.com/ethz-asl/kalibr), a widely used tool for camera–IMU calibration. Kalibr uses motion-induced discrepancies between visual and inertial measurements of the same rigid body motion to solve for all unknown spatial, temporal, and intrinsic parameters simultaneously through continuous-time optimization.Camera tracks external AprilTag corners through projective geometry, while IMU senses internal accelerations and rotations. Calibration uses deliberate platform motion to create observable discrepancies between these measurements, revealing their spatial, temporal, and intrinsic relationships through optimization.

The following are prerequest to use `Kalibr` callibration tool:

A. **Prepare the calibration target:** Kalibr supports multiple target types, but an AprilGrid is strongly recommended. It allows partial visibility of the board while still resolving the pose correctly, making data collection easier.
Before starting, print an AprilGrid from the [Kalibr wiki](https://github.com/ethz-asl/kalibr/wiki/calibration-targets) and fill out the corresponding aprilgrid.yaml(check [Kalibr yaml formats](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)) file:
   - Count the number of rows and columns, then fill in the values for tagsRows and tagsCols accordingly.
   - Measure the size of one AprilTag and set it as tagSize (in meters).
   - Measure the spacing (black border gap) between two tags.
   - Compute tagSpacing = spacing / tagSize.
For our tutoriall we will use a grid with 44 mm tags and 12.5 mm spacing.

  ```yaml
    target_type: 'aprilgrid'  #gridtype
    tagCols: 6                 #number of apriltags
    tagRows: 6                 #number of apriltags
    tagSize: 0.0445            #size of apriltag, edge to edge [m]
    tagSpacing: 0.296          #ratio of space between tags to tagSize
    codeOffset: 0            #code offset for the first tag in the aprilboard
  ```

B. **Record the calibration dataset (rosbag)**: Record a rosbag containing IMU and camera data:
To achieve accurate calibration, perform the following motions while keeping the target always in view and  ensure that the calibration board can be seen from different orientations, distances, and in each part of the image plane.

   - Pitch, yaw, and roll rotations
   - Up/down, left/right, forward/backward translations
   - A short sequence of smooth random motion

   Reference motion example: [See this YouTube video as an example:](https://youtu.be/puNXsnrYWTY?t=57)


C. **IMU noise parameters**: Kalibr requires IMU noise parameters such as noise density and random walk. These can come from the manufacturer’s datasheet or tools, but it is recommended to compute them using an Allan variance calibration, since IMU noise characteristics can change depending on the physical setup, mounting, and environment. A convenient ROS-based Allan variance tool is available here [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)
       
  For our tutoriall we will use a manufacturing callibration imu parametrs.
  
  ```yaml
    #Accelerometers
    accelerometer_noise_density: 1.4e-03   #Noise density (continuous-time)
    accelerometer_random_walk:   8.0e-05   #Bias random walk
    #Gyroscopes
    gyroscope_noise_density:    8.712683324559951815e-5   #Noise density (continuous-time)
    gyroscope_random_walk:      0.00074001958110154640244   #Bias random walk
    rostopic:                    /zed/zed_node/imu/data_raw      #the IMU ROS topic
    update_rate:                 100.0     #Hz (for discretization of the values above)
  ```

Save this file as` imu-params.yaml`, which we will use as the input for the IMU calibration. After saving it, we can follow the steps below to run the calibration inside our Docker container.


1. To start first clone the docker container along with all its submodules (kalibr packages).This will build the Docker container with ROS and all necessary dependencies for `Kalibr` package.
    
    ```bash
    git --recurse-submodules clone git@github.com:eliyaskidnae/slam-tutorial-practical.git # Clone the repository with all submodules (only if you haven't cloned it yet)
    cd slam-tutorial-practical/camera_imu_cal_ws/
    docker compose up --build -d  
    ```

2. Then opens a shell inside the Docker container, builds the Kalibr workspace and  sources the setup file.

    ```bash     
    cd slam-tutorial-practical/camera_imu_cal_ws/
    docker compose exec callibration bash --login
    catkin build -DCMAKE_BUILD_TYPE=Release -j4
    source devel/setup.bash
    ```

    check all packages are installed with out error.

3. The next thing we need is data for the calibration to be run on. Normally, you would be able to use a live camera feed for the intrinsic calibration, but to make this training more universally accessible and repeatable, we will be working from bag files.
Download [kalib_ros.bag](https://saxion.data.surf.nl/s/XyAARMGimXCBooQ) file and put it `camera_imu_cal_ws/resources` folder.Put also the configuration files `april-grid.yaml` and `imu_param.yaml` inside `/camera_imu_cal_ws/resources`.

    check for the bag file if it contains left and right camera topics as well as imu-raw topic.
    
    ```bash
    cd resources/
    rosbag info kalib_ros.bag
    ```

4. The kalibr imu-camera calibration requires the intrinisic and extinisic callibration of both cameras.We can use other camera callibration and put it the [Kalibr YAML format documentation](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) or perform a new calibration using Kalibr’s camera calibration tool as foolowing command:

    run the kalibr camera calibration node

    ```bash
    rosrun kalibr kalibr_calibrate_cameras --bag resources/kalib_ros.bag --topics /zed/zed_node/left/color/rect/image /zed/zed_node/right/color/rect/image --models pinhole-radtan pinhole-radtan --target resources/april-grid.yaml --show-extraction 
    ```
    
    When the calibration is complete (it takes many minutes according to the number of image acquired) you will get the file `kalib_ros-camchain.yaml` and a full PDF report of the result of the calibration inside ~/callibration_ros1_ws/resources folder.

    The quality of the camera calibration can be verified by inspecting the reprojection error scatter plots. In these plots, each point represents the difference between the detected AprilGrid corner and its projected location based on the estimated camera model.Since the points are tightly clustered around zero and the error stays below about 1.0 pixels with a Gaussian-like distribution, this indicates a good and reliable calibration.With a good calibration  less than < 0.2-0.5 pixel reprojection errors is expected.

    <table style="border-collapse: collapse; width: 100%; border: none;">
        <tr>
          <td style="padding: 0; vertical-align: top;">
            <figure style="margin: 0;">
              <img src="images/calib/camera_reprojection_erro.png" alt="Alt 2" height="300" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 12px; margin-top: 4px; padding-left: 50px">
                <strong></strong> 
              </figcaption>
            </figure>
          </td>
        </tr>
    </table>
    <p style="text-align:center;"><strong>Figure 4: Camera reprojection error </strong></p>


5. Put the imu noise parametr as `imu-params.yaml`  in  `camera_imu_cal_ws/resources`.
    
      Then run the camera-imu callibration node.

      ```bash
        rosrun kalibr kalibr_calibrate_imu_camera \
        --bag resources/kalib_ros.bag \
        --cam resources/kalib_ros-camchain.yaml\
        --imu resources/imu-params.yaml \
        --target resources/april-grid.yaml
      ```

      After running kalibr_calibrate_imu_camera node, the camera calibration yaml will be extended by the imu-camera calibrator with imu-camera transformations.We can get also  a PDF report containing the final calibration result and calibration analyses.

      ```yaml
        cam0:
          T_cam_imu:
          - [-0.006036362046665494, -0.9999239384750238, 0.010755445032163524, 0.022391610383912294]
          - [-0.0023433046621169074, -0.010741466983213238, -0.9999395630788445, 0.0002272267079260322]
          - [0.9999790354084946, -0.006061200512015419, -0.002278287043167876, 0.0028036703012263207]
          - [0.0, 0.0, 0.0, 1.0]
          cam_overlaps: [1]
          camera_model: pinhole
          distortion_coeffs: [0.013906358974869085, -0.002773924190176049, 0.0007394764683732347, 0.002260173136216226]
          distortion_model: radtan
          intrinsics: [269.91684856827334, 269.65862976049334, 342.27630954911905, 188.31594203230588]
          resolution: [672, 376]
          rostopic: /zed/zed_node/left/color/rect/image
          timeshift_cam_imu: 0.006180185971611793
        cam1:
          T_cam_imu:
          - [0.0037000375290703325, -0.9999342469687997, 0.010854098821330815, -0.09717479095966443]
          - [-0.0023290320677171183, -0.010862760738228006, -0.9999382861150842, 0.00032509469131031287]
          - [0.9999904426202839, 0.0036745296411589967, -0.002369071549199575, 0.0027071447777197864]
          - [0.0, 0.0, 0.0, 1.0]
          T_cn_cnm1:
          - [0.9999525963422334, -0.00012135244900840625, 0.00973603317629399, -0.11959260895189187]
          - [0.00012121210899311501, 0.9999999925412439, 1.5004571660715619e-05, 9.511178288877542e-05]
          - [-0.00973603492451681, -1.3823735274610148e-05, 0.999952603593217, 0.00012161600222734961]
          - [0.0, 0.0, 0.0, 1.0]
          cam_overlaps: [0]
          camera_model: pinhole
          distortion_coeffs: [0.010210254705281033, -0.00061608438342638, 0.0004491938369714274, 0.00021575183132482067]
          distortion_model: radtan
          intrinsics: [270.25060851678194, 269.7455366770036, 339.396953247174, 188.0375678636518]
          resolution: [672, 376]
          rostopic: /zed/zed_node/right/color/rect/image
          timeshift_cam_imu: 0.006435798701052577
      ```
         
      Lets get the 3x3 rounded roation matrix from transformation matrix.

      ```yaml
      cam0_imu:[[0, -1,  0 ]
                [0,  0, -1 ]
                [1   0,  0 ]]   
                
      cam1_imu:[[0, -1,  0 ]
                [0,  0, -1 ]
                [1   0,  0 ]]
      
      ```
      The rotation matrix tells us how the IMU is oriented relative to the camera. From this result, we can see that the IMU’s X-axis is pointing in the same direction as the camera’s forward Z-axis, meaning both sensors face the same way. The IMU’s Y and Z axes are rotated so they line up with the camera’s horizontal and vertical directions. In simple terms, the IMU is mounted in a way that its forward axis matches the camera’s viewing direction, while the other axes are rotated to properly align the two coordinate frames
      
      Checking the translation part of the transformation matrix.

      ```yaml
       cam0_imu:[0.0223, 0, 0]   cam1_imu:[-0.097, 0, 0 ] cam0_cam1:[-0.1195, 0, 0]
      ```
      
      The translation part of the transformation matrix describes how far the IMU is located from each camera. For cam0, the IMU is shifted by +0.0223 m along the X-axis, meaning the IMU sits about 2.23 cm to the right of the left camera. For cam1, the translation is −0.097 m, meaning the IMU is about 9.6 cm to the left of the right camera. When we combine these two offsets, we get the total distance between the two cameras. This value is estimated as −0.1195 m, meaning the right camera is approximately 11.95 cm to the right of the left camera, which is the stereo baseline of zed-camera(12 cm).Your accelerometer and gyroscope errors are within their 3-sigma bounds (if not then your IMU noise or the dataset are incorrect).We can check also final rotation and translation with hand-measured values.
      
      The quality of the IMU–camera calibration can be assessed by examining the reprojection error scatter plots. A good calibration is indicated when the reprojection errors lie within the 3-sigma bounds and are tightly clustered around zero. Although some outliers may appear, fewer outliers and a stronger concentration near zero generally reflect a more accurate calibration. In our results, the majority of the points remain close to zero, showing that the calibration quality is acceptable and consistent.For more explanation 

      <table style="border-collapse: collapse; width: 100%; border: none;">
          <tr>
            <td style="padding: 0; vertical-align: top;">
              <figure style="margin: 0;">
                <img src="images/calib/cam_imu_reprojectionerror.png" alt="Alt 2" height="400" width="500" style="margin: 1; padding: 2; display: block;"/>
              </figure>
            </td>
          </tr>
      </table>
      <p style="text-align:center;"><strong>Figure 5: Camera reprojection error during Camera-Imu callibration </strong></p>

      Check also your accelerometer and gyroscope errors are within their 3-sigma bounds (if not then your IMU noise or the dataset are incorrect). Ensure that your estimated biases do not leave your 3-sigma bounds. If they do leave then your trajectory was too dynamic, or your noise values are not good. 
      The following figure shows angular velocity and bias error with in 3 sigma bound(red lines).
      <table style="border-collapse: collapse; width: 100%; border: none;">
        <tr>
          <td style="padding: 0; text-align: center; vertical-align: top;">
            <figure style="margin: 0;">
              <img src="images/calib/angular_velocity_erro.png" alt="Alt 2" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 14px; margin-top: 4px;">
                a) angular velocity error 
              </figcaption>
            </figure>
          </td>
          <td style="padding: 0; text-align: center; vertical-align: top;">
             <figure style="margin: 0;">
              <img src="images/calib/gyroscope_error.png" alt="Alt 2" width="500" style="margin: 1; padding: 2; display: block;"/>
              <figcaption style="font-size: 14px; margin-top: 4px;">
                b) gyroscope  bias error 
              </figcaption>
            </figure>
          </td>
        </tr>
      </table>

      <p style="text-align:center;"><strong>Figure 5: Angular velocity and bias error during Camera-Imu callibration </strong></p>


      For more explanation about imu-camera calibration using kalibr package please refer this [video tutoriall](https://www.youtube.com/watch?v=BtzmsuJemgI)
    

    
## LIDAR-IMU Callibration

Lidar-IMU extrinsic calibration aims to determine the precise spatial relationship between a LiDAR sensor and an Inertial Measurement Unit (IMU).For this experiment, we use the [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md) package, which provides tools to callibrate temporal offset and extrinsic parameter between LiDARs and IMUs, and also the gravity vector and IMU bias. The calibration should be done in an environment with some static surfaces, without moving objects in the lidar’s field of view. When recording the bag file, the box must be rotated around all axes (roll, pitch, and yaw).

Foolow the steps below to run the calibration inside our Docker container.

1. To start first clone the docker container along with all its submodules (LI-Init package).This will build the Docker container with ROS and all necessary dependencies for [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md) package.
    
    ```bash
    git --recurse-submodules clone git@github.com:eliyaskidnae/slam-tutorial-practical.git # Clone the repository with all submodules (only if you haven't cloned it yet)
    cd slam-tutorial-practical/Lidar_imu_cal_ws/
    docker compose up --build -d  
    ```
2. Then opens a shell inside the Docker container, builds the workspace and  sources the setup file.

    ```bash     
    cd slam-tutorial-practical/camera_imu_cal_ws/
    docker compose exec callibration bash --login
    catkin_make
    source devel/setup.bash
    ```

    check all packages are installed with out error.

3. The next thing we need data for the calibration to be run on.We will use dataset provided from in the callibration package repository.
  There are many dataset provided for different type of LiDAR and IMU in this [link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhufc_connect_hku_hk/EgdJ_F763sVOnkUNBRv-op8BmNL7eZrxETu2zSEAoiRX4A?e=cbNiJI) 
  Download [Hesai_apartment.bag](https://connecthkuhk-my.sharepoint.com/personal/zhufc_connect_hku_hk/_layouts/15/onedrive.aspx?viewid=bbf22c44%2D8348%2D47d9%2D84c3%2De042ccfb2e11&ga=1&id=%2Fpersonal%2Fzhufc%5Fconnect%5Fhku%5Fhk%2FDocuments%2FLidar%20IMU%20Initialization%20Datasets%2FLiDAR%20with%20Pixhawk%20IMU%2FHesai%2FHesai%5Fapartment%2Ebag&parent=%2Fpersonal%2Fzhufc%5Fconnect%5Fhku%5Fhk%2FDocuments%2FLidar%20IMU%20Initialization%20Datasets%2FLiDAR%20with%20Pixhawk%20IMU%2FHesai) file and put it `Lidar_imu_cal_ws/resources` folder.

    check for the bag file if it contains lidar, imu topics and thier topic names.
    
    ```bash
    cd resources/
    rosbag info Hesai_apartment.bag
    ```
    for this dataset the topics are `/hesai/pandar` for lidar and `/mavros/imu/data_raw` for imu. The package containes a configuration and launch file for different lidar type and their parameters. go to `Lidar_imu_cal_ws/src/LiDAR_IMU_Init/config` folder and check for the configuration file that matches your lidar type.For this dataset we will use `pandar.yaml` file which is configured for Hesai Pandar lidar. Configure the parameters in the `pandar.yaml` config file as follows:

    ```yaml

    common:
        lid_topic:  "/hesai/pandar"
        imu_topic:  "/mavros/imu/data_raw "

    preprocess:
        lidar_type: 5                # Hesai PandarXT
        scan_line: 32
        blind: 3
        feature_extract_en: false

    initialization:
        cut_frame_num: 3 # must be positive integer
        orig_odom_freq: 10
        mean_acc_norm: 9.805
        online_refine_time: 10  .0
        data_accum_length: 300
        Rot_LI_cov: [ 0.00005, 0.00005, 0.00005 ]
        Trans_LI_cov: [ 0.00001, 0.00001, 0.00000001 ]

    mapping:
        filter_size_surf: 0.1
        filter_size_map: 0.1
        gyr_cov: 20
        acc_cov: 2
        b_acc_cov: 0.0001
        b_gyr_cov: 0.0001
        det_range: 120.0

    publish:
        path_en:  true
        scan_publish_en:  true       # false: close all the point cloud output
        dense_publish_en: true       # false: low down the points number in a global-frame point clouds scan.
        scan_bodyframe_pub_en: true  # true: output the point cloud scans in IMU-body-frame

    pcd_save:
        pcd_save_en: false
        interval: -1                 # how many LiDAR frames saved in each pcd file; 
                                    # -1 : all frames will be saved in ONE pcd file, may lead to memory crash when having too much frames.

    ```
      - The `lid_topic` and `imu_topic` should match the topics in the bag file.
      - The `lidar_type` should match your lidar type(check the documentation for more details about different lidar type). `1` is for livox series(avia, mid360 and horizon LiDAR), `2` is for Velodyne LiDAR , `3` is for Ouster LiDAR , and `5` is for Hesai PandarXT.
      - The `scan_line` is the number of laser beams in your lidar. For Hesai PandarXT it is `32`.
      - The `cut_frame_num` determines the numberof to split one frame into subframe to improve odom frequency and must be positive integer.The `orig_odom_freq` is  Original LiDAR input frequency. For most LiDARs, the input frequency is 10 Hz.
      - `mean_acc_norm (m/s^2)` is the acceleration norm when IMU is stationary. Usually, 9.805 for normal IMU, 1 for livox built-in IMU.
      - `filter_size_surf (meter)` it recommended that `0.05~0.15m` for indoor scenes or `0.5m` for outdoor scenes.

5. First allow docker to access your display for visualization.This should be done in your host machine terminal.

    ```bash
    xhost +local:docker
    ```
4. Now we are ready to run the LiDAR-IMU callibration node.

    ```bash
    docker compose exec callibration bash --login
    roslaunch LiDAR_IMU_Init lidar_imu_init.launch \
    ```

    in another terminal, play the rosbag file

    ```bash
    docker compose exec callibration bash --login
    ros2 bag play resources/Hesai_apartment.bag
    ```
    During callibration there are progress bars in the terminal that reach 100% when the excitation around each axis is enough. This makes sure the box is rotated sufficiently around each axis.After initialization and refinement finished, the result would be written into `catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt `

    ```yaml
    Initialization result:
    Rotation LiDAR to IMU (degree)     =  0.453906   -1.875376 -179.619277
    Translation LiDAR to IMU (meter)   =  0.116992 -0.043558  0.131183
    Time Lag IMU to LiDAR (second)     =  50574455.962522
    Bias of Gyroscope  (rad/s)         =  0.000009 0.004068 0.001166
    Bias of Accelerometer (meters/s^2) = -0.009681 -0.010146 -0.009607
    Gravity in World Frame(meters/s^2) = -0.065864 -0.229451 -9.807095

    Homogeneous Transformation Matrix from LiDAR to IMU: 
    -0.999441  0.007135  0.032667  0.116992
    -0.006872 -0.999943  0.008146 -0.043558
    0.032723  0.007917  0.999433  0.131183
    0.000000  0.000000  0.000000  1.000000


    Refinement result:
    Rotation LiDAR to IMU (degree)     = 0.425575   -1.947652 -179.426096
    Translation LiDAR to IMU (meter)   = 0.141320 -0.045165  0.106415
    Time Lag IMU to LiDAR (second)     = 50574455.962522
    Bias of Gyroscope  (rad/s)         = -0.000145 -0.013387 -0.017624
    Bias of Accelerometer (meters/s^2) = -0.000731 -0.002866 -0.017448
    Gravity in World Frame(meters/s^2) = -0.057362 -0.222786 -9.799158

    Homogeneous Transformation Matrix from LiDAR to IMU: 
    -0.999370  0.010499  0.033905  0.141320
    -0.010241 -0.999917  0.007775 -0.045165
    0.033984  0.007423  0.999395  0.106415
    0.000000  0.000000  0.000000  1.000000

    ```
    The rotation and translation parameters specify how the LiDAR sensor is physically mounted relative to the IMU, with the angles indicating a near-180-degree rotation around one z-axis. The translation vector shows the LiDAR is positioned approximately 14.13 cm forward, 4.52 cm to the left, and 10.64 cm above the IMU. The gyroscope and accelerometer biases indicate small systematic errors in the IMU measurements, which should be accounted for in sensor fusion algorithms. Overall, these parameters provide a comprehensive understanding of the spatial and temporal relationship between the LiDAR and IMU, essential for accurate data integration in robotics applications.

    
   [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md) callibration package does not provide evaluation for the calibration result.The following are some suggestion to evaluate the calibration result:

   1. Do the callibration multiple times with different dataset and check the consistency of the result.
   2. Compare the callibration result with hand-measured values of the sensor mounting or CAD model if available.
   3. Use the calibration result in a lidar-inertial odometry system and check the trajectory accuracy compared to ground truth.

   Although [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md) supports multiple LiDAR types for calibration, if your LiDAR’s data fields differ from the formats provided by the package, you must create an intermediate (middleware) node to convert your LiDAR messages into the expected format before passing them to the calibration node.

   
## Exercise

  1. Repeat the intrinisic camera callibration using you laptop webcam or any other camera you have. 

      Hint: To feed a PC webcam into ROS, you need to create a publisher node that uses a library like OpenCV to capture frames from the camera and cv_bridge to convert them to ROS sensor_msgs/Image messages.[Here is a simple repository](https://github.com/clydemcqueen/opencv_cam) that demonstrates how to publish webcam images to a ROS topic.

  2. Download the [kalib_ros_exer.zip](https://saxion.data.surf.nl/s/n7gwTNX2s9C4sbx)  file and perform the camera-IMU callibration using the above steps(camera-imu callibration steps). Notice the visulization of feature extraction from the april target callibration target and number frames accepted for callibration compare it with previous callibration attempt [kalib_ros.bag](https://saxion.data.surf.nl/s/XyAARMGimXCBooQ).

      Hint:This bag file was recorded with  clear view of the calibration target.The number of accepted frames should be higher than from previous calibration attempt.
      Therefore, the camera reprojection error plot should be lower compared to the previous attempt.Compare your results with the provided reference in `resources/exercise1`.