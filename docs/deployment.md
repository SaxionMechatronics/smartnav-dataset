# Chapter 4: Deployment of SLAM

## 4.1 Objectives
In this chapter, we will be running SLAM algorithms on some pre-recorded data. At the end, you should get some insights on:

- How does the real-time sensor data inputted to SLAM look like
- How run an open-source SLAM
- What are common parameters to tune when running SLAM
- How to interpret SLAM results
- How to mitigate SLAM failures 


## 4.2 Deploying a camera-IMU SLAM

In previous chapters, we discussed camera sensors suitable for SLAM. We also explored different modality and configuration of cameras. When it comes to visual SLAM, one of the most repeatedly used sensor configurations is combination of a stereo-camera and IMU sensor. Of course, there many other possible additions to this setup, such as adding wheel odometry, GPS, LiDAR, or more number of cameras. However, the most repeatedly supported sensor configuration is still stereo+IMU setup. That is why there are many commercially ready to use setup follow this configuration. 

Why is this sensor setup ideal? First, stereo camera pair are able to understand the scene depth up to a certain distance. Increasing the range of depth estimation depends on how far the two cameras are from each other (baseline). Adding the IMU sensor, will make the position estimation in the SLAM more smoth and more reliable. Researches in SLAM prove better accuracy of IMU-aided visual SLAM compared to camera only.

Let us assume that we have acquired a stere+IMU sensor setup ideal for our application based on the instructions of chapter 2. Also, let us assume that we have calibrated our sensors according to the instructions in chapter 3. Now, we want to use this information to run a SLAM on our robot. If you dont have a robot already, no worries. We have prepared some data recorded on our real robots within SMARTNav dataset. The data is recorded in ROS2 bag format, meaning that by playing back the bag file, it is as if you are getting real-time stream of sensor data just like in a real robot.

### 4.2.1 VINS-Fusion algorithm

**VINS-Fusion** is widely used in research and sometomes adopted in industry because it is relatively mature, open-source, and has good performance in many real-world scenarios.
It is designed to work with several sensor setups, but the most common configuration (and the one we use in this chapter) is a stereo camera + IMU. By fusing these two sources of information, VINS-Fusion can produce smoother and more robust trajectories than a camera-only SLAM system, especially during fast motions, rotations, or in low-texture regions.

At a high level, VINS-Fusion has three main components:

- **Visual front-end**:
The visual front-end detects and tracks feature points in the images (for example, corners or small textured patches). These tracked features are used to reconstruct the relative motion of the camera between frames and to triangulate 3D landmarks in the environment. In the stereo case, depth can be obtained directly from the left–right image pair, which improves the stability of the system and helps with scale estimation.

- **Inertial (IMU) integration and optimization back-end**:
The IMU measurements are continuously integrated (often called IMU preintegration) to predict how the pose should evolve between image frames. This prediction is then combined with the visual measurements in a nonlinear optimization problem. The back-end maintains a sliding window of recent keyframes and IMU measurements and solves for the poses, velocities, IMU biases, and landmark positions that best explain all the data. This optimization-based approach allows VINS-Fusion to correct drift, handle sensor noise, and provide a consistent state estimate.

- **Loop closure and map optimization (Fusion part)**:
Over time, any odometry system will accumulate drift. VINS-Fusion includes a loop closure module that tries to recognize when the robot revisits a previously seen area. It uses image-based place recognition and geometric verification to detect these loop closures. When a loop is confirmed, a pose graph optimization step adjusts the whole trajectory to reduce accumulated error. This “global” correction can significantly improve the overall accuracy of the path and map.

From a user perspective, you can think of VINS-Fusion as a pipeline with tunable parameters rather than a black box. You can configure camera intrinsics, stereo baseline, IMU noise parameters, feature detection thresholds, and loop closure options. In this chapter, we will not go deep into the underlying mathematics, but we will see how these parameters affect the behavior of the system when we run it on real data. The goal is that by the end, you will know how to launch VINS-Fusion on a dataset (or real-time stream of data from your robot's sensors), how to recognize when it works well or fails, and which basic parameters you can adjust to improve robustness for your own application.

### 4.2.2 Running a demo
To run the VINS-Fusion algorithm, we have tried to simplify the proecess of prerequisite installation, compilation, and fixing compatibility issues, often encountered when working with open-source software. To resolve this, please make sure you have docker and Visual Studio Code installed on your system.
https://code.visualstudio.com/docs/devcontainers/containers

Next, you should clone the github repository that we prepared for this course. You can use the following command in a terminal (if you already have it cloned in the previous chapter, you can skip this step):
```bash
git clone --recursive https://github.com/SaxionMechatronics/slam-tutorial-practical.git
```

Next, navigate to the `slam-tutorial-practical/slam_deployment` folder and open the VS Code. Inside the VS Code you can usually see a pop-up at the bottom-right corner, suggesting that the folder is a docker container and that you can set up the container. Choose the **Reopen in Container** option.

<div style="display:flex; flex-wrap:wrap; gap:8px; justify-content:center; align-items:flex-start">
  <img src="images/reopen_container.png" alt="1" style="width:40%">
  <div style="font-size:0.85em; flex-basis:100%; text-align:center; margin-top:6px;">
  </div>
</div>

 Another option is to use the **Ctrl+Shift+P** kreyboard buttons, and look for and option for building and opening a container. The building and opening of the container might take a few minutes, depending on your processors, due to the compilation of all the prerequisites and the SLAM source code itself. 
If all goes well, you will be notified by this message at the VS Code's terminal: **Done. Press any key to close the terminal.** 

 <div style="display:flex; flex-wrap:wrap; gap:8px; justify-content:center; align-items:flex-start">
  <img src="images/opening_finished.png" alt="1" style="width:60%">
  <div style="font-size:0.85em; flex-basis:100%; text-align:center; margin-top:6px;">
  </div>
</div>

Here you can open a fresh terminal from the VS Code menue: **Terminal -> New Terminal**

<div style="display:flex; flex-wrap:wrap; gap:8px; justify-content:center; align-items:flex-start">
  <img src="images/new_terminal.png" alt="1" style="width:30%">
  <div style="font-size:0.85em; flex-basis:100%; text-align:center; margin-top:6px;">
  </div>
</div>

Now, make sure you have one the prerecorded data sequence that we have made available on our [web-page](https://saxionmechatronics.github.io/smartnav-dataset/). Download one of the sequences that their name starts with `optitrack`. In this example, we use `optitrack_handheld_3`. These sequences of data are recorded using a stereo camera plus and IMU sensor mounted on a drone that is either flying or being carried by hand at a small room:

<div style="display:flex; flex-wrap:wrap; gap:8px; justify-content:center; align-items:flex-start">
  <img src="images/optitrack_drone.png" alt="1" style="width:40%">
  <div style="font-size:0.85em; flex-basis:100%; text-align:center; margin-top:6px;">
  </div>
</div>

After you download the zipped file of the sequence, extract it in the `slam-tutorial-practical/slam_deployment` directory. We extracted it under a folder named `data`.
In the newly opened terminal, use the following launch command to start the SLAM running on the prerecorded sequence:
```bash
ros2 launch vins vins_rviz.launch.py config:=VINS-Fusion-ROS2/config/zed2_gray/main_conf.yaml bag_folder:=data/optitrack_handheld_3/
```

In the above command, there are two parameters that you can change. First one is the `config` argument, which is the path to a file that all the configurations needed by VINS-Fusion are stored. These configurations contain descriptions of the you sensors and the calibration info needed by the SLAM algorithm to run properly. Additionally, some other algorithm-related parameters are specified in these files. If you want to run this SLAM for your custom setup, you need to prepare a custom config file, which we will talk more about it in the next sections.

The other parameter passed in the above command (`bag_folder`) is the path to a ROS2 bag file, which already downloaded and placed in the workspace. If you wanted to use another sequence you can change this parameter. Note that the sensors used to record this data should be the same sensors that are described in the config file.

At the end, you should be able to see the `RViz` visualization tool opened similar to the following video:

<video width="800" controls>
  <source src="images/VIO.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

Let's briefly discuss what you see in this video. At the bottom of the video, the images from the left and right cameras are displayed. On the images, you can see that some feature points are drawn in form of red and green points. The algorithm here, is tracking the spots in the envrionment that easy to identify and track. The red features, are the features that the algorithm has tracked over time. These are points that are tracked between consecutive frames of left camera. Tracking them is essential for estimating the camera motion over time. The green ones, are the features that are matching between the left and right frames, and they help estimating the distance of these points from the camera.

After the features are successfully tracked, the algorithm tries estimating the 3D position of these features in the environment, and keeps refining their position. Their 3D positions are drawn with white points cubes in the 3D interactive viewport.

The camera's position and orientation is visible by the 3 axes in the viewport. The red axis is the x direction of the camera (front), the green one is the y axis (left), and the blue axis is the the z (up) direction. As the robot is carried in the environment, the history of its positions (its path) is drawn as a curve green. 

What you saw in this demo, is basically known as the Visual-Inertial Odometry. Our algorithm is fusing the camera image information (through the features it is tracking) with the IMU sensor's data and is estimation the motion of the camera in the environment. If at some point of the estimation, something goes wrong (featureless environment or very noisy IMU), and we get few inaccurate motion estimations, that inaccuracy will bias our estimation position of the camera forever. In other words, the position estimation will deviate from reality, and the odometry method does not have a solution for that. To fix this, we will later discuss the loop closure mechanism that rectifies such drifts.

### 4.2.3 Preparing a custom config file using calibration output
Each open-source SLAM usually uses a slightly different format. Since we introduced the Kalibr package for calibrating the camera-imu, we should be careful about how we should use the value outputted by the calibrator. Since we chose VINS-Fusion as a visual SLAM method in this chapter due to its common use and relative robustness, we should convert Kalibr outputs into VINS-Fusion format. This step is slightly different in different open-source SLAM algorithms. Sometime the algorithms can directly read the Kalibr output and sometime some re-formatting is needed. Generally it is a good idea for our knowledge if we transfer these values manually (as done in this chapter) so that we can also get a sense of how the calibration results go into a SLAM.

 Here we want to use a sample data recorded using a camera-imu installed on a drone that is flying in indoor environment. You can find the recorded file (ROS2 bag) in SMARTNav dataset, named `optitrack_hanfheld_3`. This configuration has a stereo camera (`cam0` and `cam1`) and an IMU. For the specific case of VINS-Fusion algorithm, for every robot setup we need 3 configuration files required.
 These 3 files are:
 - `cam0` intrinsic calibration info
 - `cam1` intrinsic calibration info
 - camera-imu and camera-camera extrinsic calibration info along with the algorithm's parameters

 For preparing these files, the easiest way to to use a previously created config file, and only change the entries according to your sensor setup calibration. In the dev container that we shared with you, there is a config folder for our camera called `zed2_gray`. You can create a similar folder there and rename it to your custom setup and start modifying the files inside it as we describe below.

 Let's start with preparing the camera intrinsic file. We will name this file `left.yaml`. This file has the following format:

```yaml
%YAML:1.0
---
model_type: PINHOLE
camera_name: camera
image_width: ...
image_height: ...
distortion_parameters:
   k1: ...
   k2: ...
   p1: ...
   p2: ...
projection_parameters:
   fx: ...
   fy: ...
   cx: ...
   cy: ...
 ```

As can be seen, there are some parameters in the file that need to filled in according to the sensor's calibration. We can find these parameters in the output of Kalibr package (usually a file named `*-camchain.yaml`). In case of a stereo camera calibration, the file has a format like below, and the params needed to prepare `left.yaml` can be found in this file from `cam0` field.:
 ```
cam0:
    distortion_coeffs: [d1, d2, p1, p2]
    intrinsics: [fx, fy, cx, cy]
    resolution: [image_width, image_height]
    ... some other paramters ...

cam1:
    distortion_coeffs: [d1, d2, p1, p2]
    intrinsics: [fx, fy, cx, cy]
    resolution: [image_width, image_height]
    ... some other paramters ...
 ```

Similarly, you can create/modify a `right.yaml` and you can fill it up based on the `cam1` parameters.

The above format is assuming that in the calibration phase, you have used the default pinhole camera model and radial-tangential dostortion model. If you need to know more about these models and about each of these parameters are, you can revisit the calibration chapter. 

There is a third file require by the VINS-Fusion which is the most important one. We create this file and will name it `main_conf.yaml`. Inside this file, there are many important paramters to be set. Let's start by filling those that related to sensors and we obtained them from Kalibr package.

In the main config file, first introduce the individual camera configuration file that you just created. The configurations for that are:
```yaml
cam0_calib: "left.yaml"
cam1_calib: "right.yaml"
```
Then, enter the image resolution parameter. A very important note is that the camera resolution when you performed the sensor calibration should be the same resolution for images that you want to use in SLAM run time. If for instance you want to feed images of size 640x480 to the SLAM, make sure during the calibration the image resolution is the same and whenever the input reslution to SLAM changed, repeat the calibration with the new resolution.
```yaml
image_width: 672
image_height: 376
```

Next, there is an extrinsics matrix required in this file with the following format:
```yaml
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data:  [r1, r2, r3, t1,
           r4, r5, r6, t2,
           r7, r8, r9, t3,
           0 , 0 , 0 , 1.0]
```

The matrix requried in `body_T_cam0` is a 4x4 matrix. This matrix contains the rotation and translation between the left camera and the IMU sensor (transforming camera to IMU). You can find these information in the outputs of Kalibr package, in a file usually named like `*-camchain-imucam.yaml`. This file hase this format:

```yaml
cam0:
  T_cam_imu:
  - [r1, r2, r3, t1]
  - [r4, r5, r6, t2]
  - [r7, r8, r9, t3]
  - [0 , 0 , 0 , 1.0]

  ...

  timeshift_cam_imu: td

```

The value from `T_cam_imu` should be transferred to the `body_T_cam0`. By convention, transformation matrices in SLAM algorithms are shown is similar formats, such that the `T_cam_imu` is read "transformation from `imu` coordinate frame to the `cam` frame". Similarly, the `body_T_cam0` is read as "transoformation from `cam0` to `body` (body frame is sometimes referred to the imu frame)".

If you follow what the above transformations mean, you will notice that they have the inverse effect of each other. Thus, the output of Kalibr should be inversed and then placed in VINS-Fusion algorithm. We have provided the following interactive python script where you can input any 4x4 transformation matrix and get its inverted version. Use the inverted values inside the `main_conf.yaml` file that you are creating.

<iframe src="https://trinket.io/embed/python/8541092e4859?runOption=run" width="100%" height="600" frameborder="0" marginwidth="0" marginheight="0" allowfullscreen></iframe>

The described procedure should be repeated for both cameras and you should create two entries in the `main_conf.yaml` file named `body_T_cam0` and `body_T_cam1`.

After that, also import the estimated time offset between the camera and IMU, measured during calibration. This time offset is very important since it shows the delay in which the camera understands motion compared to IMU. In Kalibr output file (`*-camchain-imucam.yaml`), the parameter is named `timeshift_cam_imu`. Take the value and put it in our config file:
```yaml
td: 0.008 
```

The last sensor-specific parameter to introduce to almost all SLAM algorithms, is the IMU noise characteristics. As discussed in chapter 2, all IMUs have imperfections in their data. The angular velocity and linear acceleration that they produce is either biased against the real value or has a random noise on it. The SLAM method should be able about the extent of the bias and noise. Since, it allows the SLAM to know how much can it rely on the IMU during the fusion. You can change these value in 2 ways. 
   - First, you can do IMU calibration. In this course, we did not cover this calibration in chapter 3, however, if you want to know the exact noise characteristic of the IMU, tools like *Allan Variance* calibrators might help you. This step is mostly advisable if you have a high-grade IMU sensor that you know is relatively reliable.
   - Second, for most of the cheap IMU sensors usually used in SLAM systems, using the default values used by SLAM implementations, usually yields a good trade-off. You can slightly change these value by either increasing them or decreasing them if you want to tune them for your application.
```yaml
acc_n: 0.1          
gyr_n: 0.01         
acc_w: 0.001         
gyr_w: 0.0001       
g_norm: 9.805        
```

So far, we have introduced all the parameters specific to our sensor setup. All sensor intrinsic and exterinsic calibrations and the time offsets. However, the VINS-Fusion algorithm is a capable SLAM that can also estimate and enhance the sensor extrinsics and time offsets in real-time. Unless you are very confident about your calibration, it is a good idea to allow the VINS to use your calibration info as an initial estimate and improve it on the go. For that, enable these two configs:
```yaml
# Set the values to 0 for disabling and to 1 for enabling 
estimate_extrinsic: 1   
estimate_td: 1 
```

Lastly, we should make sure our sensor data stream (or data from replaying a bag file) is correctly introduced to the algorithm, so that when we run the SLAM, it starts estimating the state using the sensor data. For that, make sure these parameters are correct:
```yaml
imu_topic: "some imu topic name"
image0_topic: "left camera topic name"
image1_topic: "right camera topic name"
```


### 4.2.4 VINS-Fusion algorithm parameters 
Other than the sensor-specific parameters, there are some other parameters that are used in VINS-Fusion and they affect the internal algorithm. Although these are specific to this algorithm, but knowing about them is useful since many other open-source algorithms share similar concepts/parameters.

The VINS-Fusion supports 3 different sensor configurations:
   - Stereo camera
   - Monocular camera + IMU
   - Stereo camera + IMU
It can be controlled via the following parameters:

```yaml
#support: 1 imu 1 cam; 1 imu 2 cam: 2 cam; 
imu: 1        
num_of_cam: 2  
```

Then, the front-end performance of the SLAM can be adjusted using the following parameters.
```yaml
#feature traker paprameters
max_cnt: 150            # max feature number in feature tracking
min_dist: 15            # min distance between two features 
freq: 10                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
F_threshold: 1.0        # ransac threshold (pixel)
show_track: 1           # publish tracking image as topic
flow_back: 1            # perform forward and backward optical flow to improve feature tracking accuracy
```

```yaml
#optimization parameters
max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
max_num_iterations: 8   # max solver itrations, to guarantee real time
keyframe_parallax: 10.0 # keyframe selection threshold (pixel)
```

### 4.2.5 Loop closure

As previously discussed, when you only run visual-inertial odometry, you will inevitably deal with the drift problem. This is why, visual-inertial odometry, alone can not be a very reliable source of navigation for your robot, and you will require some other accurate sensor that once a while, corrects the drifts of odometry. Examples of such sensors can be GPS for outdoor robots, or wireless beacons for indoor robots.

The purely vision-based solution to this is the **Loop Closure**. It is only applicable if you have a robot that may revisit the same location multiple times. Loop closure is basically the ability to identify a previously seen area and based on that, correct the odometry estimation (to zero the accumulated error and drift). To put this into perspective, let's redo out demo, this time not only running the odometry, but also the loop closure enable. To this end, you can use the following launch command inside the container:
```bash
ros2 launch vins vins_lc_rviz.launch.py config:=VINS-Fusion-ROS2/config/zed2_gray/main_conf.yaml bag_folder:=data/optitrack_handheld_3/
```

You should be able to see the `RViz` openned and the result should look something like the following video:

<video width="800" controls>
  <source src="images/VIO+Loop.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

The difference is that now, there are two axes displayed in 3D viewport. One is thinner and that is the same odometry estimated position and another is a thicker one which is the corrected position of the odometry by loop closure. There is also a new path visualized, which is the corrected path after each loop detection. As you can see, at the beginning, the two position estimations match each other almost perfectly, but as time passes, the green trajectory is drifting from the rectangular path that camera is going on, while the blue curve, corrects itself every while and remains closer to rectangular path that it is supposed to be.

Although the above observation intuitivly shows how the loop closure improves the quality of your localization, it is not a definitive and numeric measure of accuracy. We can go over such analysis in the next sections.


## 4.5 Deploying a LiDAR-IMU SLAM

### 4.5.1 Running FASTLIO

### 4.5.2 Running GLIM or Lego-LOAM

### 4.5.3 Parameters

### 4.6 Practical consideration

### 4.6.1 Processing unit

### 4.6.2 Standalone or ROS-based

### 4.6.3 QoS in ROS

### 4.6.4 Being real-time and low delay

### 4.6.5 Using in control loops

<!-- Coordinate systems -->