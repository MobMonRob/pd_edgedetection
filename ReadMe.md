Installation of pmd sdk:

Download full SW package (https://pmdtec.com/picofamily/software/) Password Sh!2CBpf

Further details and source found here: (https://github.com/code-iai/pico_flexx_driver)

extract
  libroyale-3.16.0.51-LINUX-arm-64Bit
  
  Extract the archive that matches your kernel architecture from the extracted SDK to 
  <catkin_ws>/src/pico_flexx_driver/royale. 
  
  You can find out what your kernel architecture is by running uname -m.

install udev rules provided by SDK
```cd royale
  sudo cp libroyale-3.16.0.51-LINUX-arm-64Bit/driver/udev/10-royale-ubuntu.rules /etc/udev/rules.d/
```

Published Topics:

/object_recognition/orientation (http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Vector3.html)
/object_recognition/position_midpoint (http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PointStamped.html)


Test:

cd ~/workspace

Terminal 1:
roslaunch edge_detection_pmd Learner.launch
 
Terminal 2:
rosrun edge_detection_pmd detection_client.py
 
Terminal 3:
rosrun edge_detection_Pmd detection_server.py
