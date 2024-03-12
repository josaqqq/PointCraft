# PointCraft
<img width="421" alt="image" src="https://github.com/sakajo6/PointCraft/assets/68010363/e44dc25e-1f69-44ac-84b3-2732bbc83735">

This project is a newly proposed user interface for point cloud editing. As the above picture shows, users can edit a point cloud via a sketch-based interface. Users can add new points and erase unnecessary points. 

## Sketch Tool
First, users sketch the boundary of missing areas of a point cloud. The system then automatically detects the depth of the user assumed and then smoothly interpolates the selected missing areas by adding new points to the point cloud.

## Delete Tool
This tool is similar to the Sketch Tool. First, users sketch the boundary of unnecessary areas of a point cloud and then the system erases the points inside the sketch. 
