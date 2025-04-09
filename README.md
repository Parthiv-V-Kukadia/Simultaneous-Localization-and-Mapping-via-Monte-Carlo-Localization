# Simultaneous Localization and Mapping via a Monte-Carlo Localization Algorithm

This MATLAB script implements a basic Simultaneous Localization and Mapping (SLAM) algorithm using a Particle Filter. It processes LiDAR scan data to build a map of corners while simultaneously estimating the robot's pose (position and orientation).

## Overview

The script performs the following steps:

1.  **Initialization:**
    * Loads LiDAR input data (`Lidar_input.mat`) and motion estimation data (`motion_estimate.mat`).
    * Configures SLAM parameters such as the effective LiDAR range, the number of particles in the filter, initial particle weights, and noise parameters for motion prediction.
    * Sets LiDAR sensor parameters like the field of view and the number of readings per scan.
    * Pre-calculates bearing angles for each LiDAR reading for faster Cartesian coordinate conversion.
    * Creates a figure to visualize the SLAM process, with separate axes for raw LiDAR data and the estimated map.
    * Optionally initializes video recording to save the SLAM process.
    * Initializes a set of particles, each representing a possible robot pose and carrying its own map of detected corners.

2.  **Data Processing Loop:**
    * Iterates through each LiDAR scan in the loaded data.
    * **LiDAR Point Conversion:** Converts the range and bearing measurements from the LiDAR scan into Cartesian coordinates (`points`) in the sensor's local frame.
    * **Feature Extraction:** Extracts line segments (`detected_lines_local`) and corner features (`detected_corners_local`) from the Cartesian point cloud in the local frame.
    * **Motion Prediction:** For each particle:
        * Applies the estimated motion (from `motion_estimate.mat`) with added Gaussian noise (both additive and proportional) to the particle's current pose (x, y, theta).
    * **Measurement Update:** For each particle:
        * Transforms the detected local corners into the global frame based on the particle's current pose (`slam_crnr_loc2glo`).
        * Performs data association between the globally transformed detected corners and the corners already present in the particle's map using Joint Compatibility Branch and Bound (JCBB) (`slam_crnr_jcbb_assoc`). This identifies which detected corners likely correspond to previously seen corners.
        * Updates the state (mean and covariance) of the associated corners in the particle's map using a Kalman Filter (`slam_crnr_kf`). The particle's weight is updated based on the likelihood of the observed measurements given the particle's state and map.
        * Adds newly detected corners (that were not associated with existing ones) to the particle's map (`slam_crnr_add`). The particle's weight is penalized for adding new, unverified features.
    * **Visualization:** Calls `slam_interface` to display the current LiDAR scan, the particles' poses, and the map of corners maintained by the highest-weight particle.
    * **Resampling:** Resamples the set of particles based on their weights (`slam_resample`). Particles with higher weights (representing more likely robot poses and maps) are more likely to be selected for the next iteration, effectively focusing the search.
    * **Block Memory Allocation:** Periodically increases the allocated memory for storing corners within each particle to accommodate a growing map.

3.  **Video Recording (Optional):** If enabled, captures each frame of the visualization and saves it to a video file (`slam.mp4`).

4.  **Motion Estimation (Separate Section):**
    * This part of the script performs a separate motion estimation based on associating corners between consecutive LiDAR scans.
    * It loads the `Lidar_input.mat` again.
    * For each pair of consecutive scans:
        * Extracts corners from both scans.
        * Attempts to associate the detected corners based on their Mahalanobis distance and a defined threshold.
        * If enough associations are found (at least 2), it estimates the rotation and translation between the two sensor frames using Singular Value Decomposition (SVD) on the matched corner pairs.
        * Applies constraints to limit the change in translation and rotation between frames.
        * Stores the estimated motion in the `motion_estimate` struct.
        * Visualizes the raw LiDAR points and the matched corners.
    * If not enough associations are found, it assumes constant velocity (no change in motion).
    * Finally, it saves the estimated motion to `motion_estimate.mat` and plots the estimated x, y, and theta over time, along with the number of associated points.

5.  **Helper Functions:** The script includes several helper functions for specific tasks:
    * `slam_crnr_add`: Adds new corners to a particle's map.
    * `slam_crnr_jcbb_assoc`: Performs data association between detected and known corners using JCBB.
    * `slam_crnr_kf`: Updates the state of a known corner using a Kalman Filter.
    * `slam_hypothesis_next`: Generates the next hypothesis for data association in JCBB.
    * `slam_interface`: Handles the visualization of the SLAM process.
    * `slam_lidar_feat_extrn`: Extracts line segments and corners from LiDAR point clouds.
    * `slam_resample`: Resamples the particle set based on their weights.
    * `slam_points`: Converts LiDAR range and bearing to Cartesian coordinates.
    * `slam_in_pi`: Normalizes an angle to the range [-pi, pi].
    * `slam_crnr_loc2glo`: Transforms local corner coordinates to the global frame.
    * `slam_lidar_split_merge`: (Likely an external function not included in the provided code) Performs line extraction from point clouds using a split-and-merge algorithm.

## Getting Started

1.  **Prerequisites:** Ensure you have MATLAB installed.
2.  **Data Files:** Make sure you have the `Lidar_input.mat` and `motion_estimate.mat` files in the same directory as the script or in a location accessible by MATLAB. The structure and content of these files are assumed by the script (e.g., `Lidar_input` likely contains a series of LiDAR range measurements over time, and `motion_estimate` likely contains estimated changes in x, y, and theta between consecutive scans).
3.  **Run the Script:** Execute the MATLAB script. It will process the LiDAR data, perform SLAM, visualize the results, and optionally save a video. The separate motion estimation part will also run and save its results.

## Notes

* The script assumes the existence of a function `slam_lidar_split_merge`, which is not provided in the code. This function is crucial for the line extraction step in the feature extraction process.
* The motion estimation part of the script is run independently of the main SLAM particle filter. The `motion_estimate.mat` file is loaded at the beginning of the SLAM process, suggesting that the motion estimates are used as input for the particle filter's motion prediction step.
* The SLAM implementation uses a Particle Filter, which is a probabilistic approach to handle uncertainty in sensor measurements and robot motion. The number of particles (`particles_count`) affects the accuracy and computational cost of the algorithm.
* The data association step (`slam_crnr_jcbb_assoc`) is a critical part of SLAM, aiming to correctly identify previously seen landmarks in the current sensor data. The Joint Compatibility Branch and Bound (JCBB) is a specific technique used here.
* The Kalman Filter (`slam_crnr_kf`) is used to update the estimated state (position and uncertainty) of the mapped corners based on new observations.
* The visualization (`slam_interface`) provides a way to monitor the SLAM process in real-time.

This script provides a foundational implementation of a particle filter-based SLAM algorithm for a robot equipped with a LiDAR sensor. Understanding the parameters and the logic within each function is key to further developing and adapting this implementation for specific applications.
