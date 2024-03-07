********************************************************************************
Point Surfaces Library for analyzing surfaces in points
********************************************************************************

================================================
1. Libs
================================================
*** LidarDataBlock
Define data structure for raw 3D data.
Refer to LidarDataBlock.hpp.

*** PatchGeometry
Calculate geometry on surfaces in raw 3D data.
Refer to PatchGeometry.hpp.

*** PatchSegment
Segment 3D surfaces.
Refer to PatchSegment.hpp.

*** LidarDataViewer
Real-time viewer for scanning raw 3D data.
Refer to PointCloudViewer.hpp, LidarDataServer.hpp, and LidarDataSharer.h.

*** DataSharer
Used for transfering data between applications. 
DataSharer_x64.dll is for C++ and DataSharerCs.dll is for C#.


================================================
2. Apps
================================================

*** PointLab
An integrated application for raw data processing. It shows L3D data, calculate the geometry of surfaces, and save as GEO data.

*** LidarDisplay
The display terminal of the 3D scanner. Refer to the control center "Lidar3DScanner".

*** Lidar3DScanner
The control center of the 3D scanner. Refer to the display terminal "LidarDisplay".

*** LidarDisplaySim
The emulator of "Lidar3DScanner". It sends raw data to "LidarDisplay" to simulate real-time scanning.

