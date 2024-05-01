Force Sense: Advancing Robotic Biopsies through Redundant Joint Feedback Integration

The ForceSense program uses a redundant joint in the LibFranka Emika Panda Robot as a feedback input of the biopsy procedure.
The program must run in few steps to make sure the procedure happens with utmost precision and accuracy. 

Due to many dependencies, the running code can be found in the following directory csc496/runners/biopsy.cpp

There is a second copy of biopsy.cpp and other python scripts attached to the first depth of this submission directory, but that is only for ease of access.



SETUP:

Before we turn into the program, identify at least 4 fiducial markers in the image space (voxel units) of the requires biopsy target that will be used for registration. 

Then run 'biopsy.cpp' using the following console command line:

./biopsy.cpp 192.168.1.107

The program has multiple options when it is running, and to collect the robot-base points that are 
needed for registration please choose option 3 to collect end-effector points of the fiducial markers.

After collecting the points in image and robot-base points, write both sets of x,y,z points 
into 'registration.py' to replace "point_voxel" and "points_in_robot" variable. Running this python script 
will give the resultant registration rotational matrix and translational vector. 

Take the newly found rotational matrix and translational vector into 'biopsy.cpp' and paste them on lines 107 and 111 of the code. 



RUNNING THE CODE:

Now the program have been calibrated to the image space, we can identify the voxel space of the biopsy target in image space.

Then, we can run the code again using:

./biopsy.cpp 192.168.1.107

Now we can use option 4 to runt the impedence biopsy module, or 5 to run the more accruate stepwise module with feedback. 
