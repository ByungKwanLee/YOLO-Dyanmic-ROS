# YOLO-Dynamic ROS
___
Baseline of this code is strongly [this official repository](https://github.com/leggedrobotics/darknet_ros). Original README is in the followed link. However, this code has been a problem of dynamic allocation in scheduling and threading during parsing real-time images. Therfore, edited author fixes below issues: from v1 to v7. In addition, the edited author adds tracking function to YOLO by Linear Kalman Filter.

  **Original Author: [Marko Bjelonic](https://sites.google.com/site/bjelonicmarko/), marko.bjelonic@mavt.ethz.ch** <br />
  **Edited Author : [Byung-KwanLee](https://scholar.google.co.kr/citations?user=rl0JXCQAAAAJ&hl=en) (leebk@kaist.ac.kr)**
___

### V7 : solve all of the memory issue
- Divide two cpp function for yolo and lane
- Completely solve memory issue considering sharing resource using boost lock
___
### V6 : classify objects which lane they are in
- Subscribe lane detection information lane_msgs/lane_array.h, lane_msgs/lane.h
- Classify objects which lane they are in, on the top view, without vanishing point
- Remove boundingbox (made by developer in original ros yolo), then simplify publish procedure
- Chage protocol of darknet_ros_msgs, more specific
- Eigen data type change, MatrixXd -> MatrixXf, VectorXd -> VectorXf (double -> float)
___
### V5 : memory Issue
- Manage Dynamic Memory Allocation and Remove
- Critical Issue for real-time
    - Check CPU Resource
    - Should be analyzing resource of tracking_thread function made by LBK
- Extra Memory Issue (V5*)
    + Synchronization with darknet free code and tracking thread
    + Keeping prediction without detection
___

### V4 : depth Extraction from detected and tracked Objects
- Add boundingbox message to distacne from the object
___
### V3 : real-Time Object Tracking by Linear Kalman filter
- Adding Kalman Filter on the Detection thread
- Transition and Updating Error Covariance
- Dynamic Allocation and Remove Completely
- Transfer variables on each member function with no Sync Error
- Publishing Object ID with Bounding Box info for sensor fusion
___

### V2 : matching and multithread for Tracking
- Matching tech by class
- Completely generate multi thread for tracking 30HZ
___
### V1
- CompressedImage subscriber (Image topic inavailable, camera callback function)
- solution for exit code -11 (gdb : Segmentation error)
- no visualization imageview
- publish Image and CompressedImage message
