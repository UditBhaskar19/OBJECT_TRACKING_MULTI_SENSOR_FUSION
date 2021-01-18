# OBJECT_TRACKING_MULTI_SENSOR_FUSION
Sensor Fusion for Target Tracking
## 0. Introduction :-
#### We have an array of sensors installed around the ego vehicle for ADAS/AD applications. The goal of this project is to estimate Valid Object Tracks by fusing the measurements from the sensors.
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/SpatialAllignment_obj_lines.gif)
**Fig 1 : (Inputs) Measurements : Radar Reflection, Camera Detections, Lane Line Clothoid Parameters**
<br/><br/>
## 1. Estimation Output :-
### 1.1 Object Tracks :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/Tracks.gif)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/Tracks_Magnified.gif)
<br/><br/>
### 1.2 Object Tracks + Object Trajectories:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/Trajectory.gif)
<br/><br/>
### 1.3 Object Tracks + Object Trajectories + Road Model:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/TrackAndRoadEstimation.gif)
<br/><br/>
### 1.4 Object Tracks + Object Trajectories + Road Model + Mapping:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/Odometry.gif)
<br/><br/>

## 2. Fusion Architecture Overview :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/0_b_fusion_archi_overview.PNG)
<br/><br/>
## 3. High Level Fusion Architecture:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1a_high_level_archi.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1b_high_level_archi.PNG)
<br/><br/>
## 4. Module Level Fusion Architecture:-
### 4.1 Preprocessing Subsystem:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/2_a_module_archi_preprocessing.PNG)
<br/><br/>
### 4.2 Radar Fusion Subsystem :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/3_a_module_archi_radar_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/3_b_module_archi_radar_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/3_c_module_archi_radar_fusion.PNG)
<br/><br/>
### 4.3 Camera Fusion Subsystem :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/4_a_module_archi_camera_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/4_b_module_archi_camera_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/4_c_module_archi_camera_fusion.PNG)
<br/><br/>
### 4.4 Radar & Camera Fusion Subsystem :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/5_a_module_archi_track_fusion.PNG)

