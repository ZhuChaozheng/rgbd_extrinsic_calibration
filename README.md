# rgbd_extrinsic_calibration
this project will determine the pose of RGBD camera by detecting the plane


this node in ROS will publish the topic of value $T_W^{C}$ (the transformation matrix from world frame to camera frame), $_{W}^{C}\textrm{P}$ (the position of camera in the world frame), and also it subscribes the topic of /camera/depth/image and /camera/rgb/image



## Appendix

For the whole system, there are four key nodes in ROS that offer different functions. the followings are the details:


Node one: subscribe /camera/depth/image and /camera/rgb/image
          publish   $T_W^{C}$ (the transformation matrix from world frame to camera frame), _${W}^{C}\textrm{P}$ (the position of camera in the world frame)

Node two: detect the specific object, 
          publish  the topic of $(u,v)$ in the pixel frame


Node three: subscribe the topic of $(u,v)$, $_{W}^{C}\textrm{P}$ (the position of camera in the world frame)
            publish topic of $_{a}^{g}\textrm{T}$ (the transformation matrix from the robot arm to the grasper)


Node four:  subscribe the topic of $_{a}^{g}\textrm{T}$ (the transformation matrix from the robot arm to the grasper), execute the responding actions of the grasper.