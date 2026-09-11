# Point Cloud Processing

https://github.com/realsenseai/realsense-ros/tree/ros2-master/realsense2_camera/examples/pointcloud
https://github.com/realsenseai/librealsense/tree/master/wrappers/pcl#pcl-samples-for-intel-realsense-cameras
https://pointclouds.org
https://dev.realsenseai.com/docs/post-processing-filters/


librealsense (also known as Intel RealSense SDK 2.0) is a cross-platform open-source library used to capture depth and color data from Intel RealSense depth cameras. It provides low-level device access, frame streaming, and calibration details for robotics, computer vision, and 3D scanning applications.

A depth frame from a RealSense camera is a single digital image where every individual pixel stores distance data instead of standard color values. It tells a computer how far away each object or surface is from the lens, measured in precise units like millimeters.


Enhancing Depth via RealSense SDK Filters
* Spatial Filter: Smooths out noise and flattens surfaces while keeping edges sharp.

* Temporal Filter: Reduces random temporal jitter across frames using an IIR moving average.

* Hole-Filling Filter: Replaces zero-depth gaps (missing pixels) using surrounding spatial data.

* Filter Pipeline Order: Pass raw depth through Decimation → Spatial → Temporal → Hole-Filling filters sequentially before conversion.
https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/

Utilizing OpenCV and HSV Masking
* HSV Color Segmentation: Convert the aligned RGB frame to HSV using cv2.cvtColor() to create a binary mask via cv2.inRange() targeting specific object colors.

* Noise Removal: Clean up your HSV mask using OpenCV morphological operations (cv2.morphologyEx) to remove isolated stray pixels.

* Depth Filtering via Mask: Apply the refined OpenCV mask directly to the depth array (setting unmasked background pixels to zero or NaN) so that invalid background data is excluded prior to point cloud generation.

* Alignment: Always use rs2.align(rs2.stream.color) from the RealSense Python wrapper so that your OpenCV color/HSV coordinates match the depth pixel grid 1:1.
https://dev.realsenseai.com/sdk-2-0-code-samples-wrappers-and-languages/opencv/
