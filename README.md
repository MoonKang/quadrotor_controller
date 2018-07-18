
Project Writeup 
Author: Moonsu Kang

[image1]: ./img1.png
[image2]: ./img2.png
[image3]: ./img3.png


## Simulator Walkthrough ##

Now that you have all the code on your computer and the simulator running, let's walk through some of the elements of the code and the simulator itself.

### The Code ###

For the project, the majority of your code will be written in `src/QuadControl.cpp`.  This file contains all of the code for the controller that you will be developing.

All the configuration files for your controller and the vehicle are in the `config` directory.  For example, for all your control gains and other desired tuning parameters, there is a config file called `QuadControlParams.txt` set up for you.  An import note is that while the simulator is running, you can edit this file in real time and see the affects your changes have on the quad!

The syntax of the config files is as follows:

 - `[Quad]` begins a parameter namespace.  Any variable written afterwards becomes `Quad.<variablename>` in the source code.
 - If not in a namespace, you can also write `Quad.<variablename>` directly.
 - `[Quad1 : Quad]` means that the `Quad1` namespace is created with a copy of all the variables of `Quad`.  You can then overwrite those variables by specifying new values (e.g. `Quad1.Mass` to override the copied `Quad.Mass`).  This is convenient for having default values.

You will also be using the simulator to fly some difference trajectories to test out the performance of your C++ implementation of your controller. These trajectories, along with supporting code, are found in the `traj` directory of the repo.


## The Tasks ##

![alt text][image1]

1. Implemented body rate control in C++.
Based on inputs of pqr command and current pqr, the body rate error is calculated and is multiplied by moments of inertia and KpPQR.
BodyRateControl converts the errors into desired moment command, and the result gets passed to GenerateMotorCommands.

![eq1](http://latex.codecogs.com/gif.latex?%24%24%20%5Cbegin%7Balign%7D%20%5Cbar%7Bu%7D_p%20%26%3D%20k_%7Bp-p%7D%20*%20p_%7B%5Ctext%7Berror%7D%7D%20%5C%5C%20%5Cbar%7Bu%7D_q%20%26%3D%20k_%7Bp-q%7D%20*%20q_%7B%5Ctext%7Berror%7D%7D%20%5C%5C%20%5Cbar%7Bu%7D_r%20%26%3D%20k_%7Bp-r%7D%20*%20r_%7B%5Ctext%7Berror%7D%7D%20%5Cend%7Balign%7D%20%24%24) 

2. Implement roll pitch control in C++.
Roll pitch control generates outputs of p and q commands that feeds into body rate control. This process involves transformation of world frame into body frame. 
Firstly, convert local accelerations, accelCmd (accelerations in global XY coordinates), into desired rate of local change through following equations.

![eq2](http://latex.codecogs.com/gif.latex?%24%24%5Cdot%7Bb%7D%5Ex_c%20%3D%20k_p%28b%5Ex_c%20-%20b%5Ex_a%29%24%24)

![eq3](http://latex.codecogs.com/gif.latex?%24%24%5Cdot%7Bb%7D%5Ey_c%20%3D%20k_p%28b%5Ey_c%20-%20b%5Ey_a%29%24%24)

Then pqr command can be computed based on non-linear transformation based on below equation.

![eq4](http://latex.codecogs.com/gif.latex?%24%24%20%5Cbegin%7Bpmatrix%7D%20p_c%20%5C%5C%20q_c%20%5C%5C%20%5Cend%7Bpmatrix%7D%20%3D%20%5Cfrac%7B1%7D%7BR_%7B33%7D%7D%5Cbegin%7Bpmatrix%7D%20R_%7B21%7D%20%26%20-R_%7B11%7D%20%5C%5C%20R_%7B22%7D%20%26%20-R_%7B12%7D%20%5Cend%7Bpmatrix%7D%20%5Ctimes%20%5Cbegin%7Bpmatrix%7D%20%5Cdot%7Bb%7D%5Ex_c%20%5C%5C%20%5Cdot%7Bb%7D%5Ey_c%20%5Cend%7Bpmatrix%7D%20%24%24)

3. Implement altitude controller in C++.
Based on inputs of pos and vel cmd in z, calculate z_err and z_err in velocity. Altitude controller uses PID formula to derive u1bar, and then compute acceleration in altitude by dividing with $b^z$. 

![eq5](http://latex.codecogs.com/gif.latex?%24%24c%20%3D%20%5Cfrac%7B%28%5Cbar%7Bu%7D_1%20-%20g%29%7D%7Bb%5Ez%7D%24%24)


4. Implement lateral position control in C++.
Lateral Position control calculates pos and vel error from current pos/vel with target pos/val and uses the error to derive local acceleration in x,y.
![eq6](http://latex.codecogs.com/gif.latex?%24%24%5Cddot%7Bx%7D_%7B%5Ctext%7Bcommand%7D%7D%20%3D%20k%5Ex_p%28x_t-x_a%29%20&plus;%20k_d%5Ex%28%5Cdot%7Bx%7D_t%20-%20%5Cdot%7Bx%7D_a%29&plus;%20%5Cddot%7Bx%7D_t%24%24)


5. Implement yaw control in C++.
Yaw control is decoupled from other directions through below equation:
![eq7](http://latex.codecogs.com/gif.latex?%24%24r_c%20%3D%20k_p%20%28%5Cpsi_t%20-%20%5Cpsi_a%29%24%24)

yaw command is optimized by setting it to be between $[0, 2\pi]$.

6. Implement calculating the motor commands given commanded thrust and moments in C++.
In GenerateMotorCommands, the moment command and thrust gets converted into individual thrustN.
This is done by getting uBar, pBar, qBar, rBar and deciiding which wings needs how much thrust forces in order to meet those.
![eq8](http://latex.codecogs.com/gif.latex?%24%24%20%5Cbegin%7Balign%7D%20F_%7Btotal%7D%20%26%3D%20F_1%20&plus;%20F_2%20&plus;%20F_3%20&plus;%20F_4%20%5C%5C%20%5Ctau_x%20%26%3D%20%28F_1%20&plus;%20F_4%20-%20F_2%20-%20F_3%29l%20%5C%5C%20%5Ctau_y%20%26%3D%20%28F_1%20&plus;%20F_2%20-%20F_3%20-%20F_4%29l%20%5C%5C%20%5Ctau_z%20%26%3D%20%5Ctau_1%20&plus;%20%5Ctau_2%20&plus;%20%5Ctau_3%20&plus;%20%5Ctau_4%20%5Cend%7Balign%7D%20%24%24)

## Passed scenario tests ## 

![alt text][image2]
![alt text][image3]


<p align="center">
<img src="./scenario5.gif" width="500"/>
</p>

