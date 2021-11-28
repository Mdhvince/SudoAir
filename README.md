## SudoAir
Goal: Desining controller(s) in C++ for the CrazyFlie mini drone and additional computer vision based modules to allow the drone to navigate in total autonomy in complex environment.  

(I'm looking for a c++ API where I can input the sum of the thrust u1 and the moment (torque) u2, u3, u4) to a simulated quadrotor) -> If you know one, please contact :).

#### Current PID response on z-axis
![alt text](docs/PID-Tune.png)  


#### State Arrays  
- xyz_state: `x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot`
- xyz_state_des: Same for desired states
- angle_state_wf: `phi, theta, psi, phi_dot, theta_dot, psi_dot, phi_ddot, theta_ddot, psi_ddot - (World frame)`
- angle_state_wf_des: `Same for desired/cmd state angles - (World frame)`

#### install
install openCV : https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html then `sudo make install`  
download Eigen library : https://eigen.tuxfamily.org/index.php?title=Main_Page then extract and place the extracted folder in `usr/local/include`
