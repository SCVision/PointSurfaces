1. PCLdemo_model
- Visual studio demo for processing L3D data files and modelling surfaces.
- Use this demo to load L3D files, model surfaces, and save as GEO files.
- 20240719: greatly improve the efficiency of geometric modeling (PatchGeometry).
- 20241015: improve the precision of geometric modeling (PatchGeometry).


2. PCLdemo_segment
- Visual studio demo for segmenting surfaces.
- Use this demo to load GEO files, segment surfaces, and save as SEG files.
- 20240721: improve the way of searching seeds and the efficiency of segmentation. 
- 20241015: follow the upgrade of PatchGeometry. 


3. PCLdemo_regist
- Visual studio demo for surface registration.
- Use this demo to test surface matching algorithms after segmenting surfaces.
- 20240721: first beta version. Note: you may copy the data of 'lab1' from PCLdemo_segment.


4. PCLdemo_regist_demo
- Visual studio demo for surface registration.
- Use this demo to show registration result of multiple point clouds.
- 20240803: first release version. This demo combines multiple pointclouds, you may test by copying the data of 'lab1', 'lab2', 'lab3' and 'lab4' from 'matlab_codes/test_data/'.
- 20241016: involve the whole process of modeling, segmenting, and registration. 


Note 1: Please insatll and config PCL1.13 for these demo codes.
Note 2: The data files for these demo codes can be doownloaded from PointSurfaces/matlab_codes/test_data/.
