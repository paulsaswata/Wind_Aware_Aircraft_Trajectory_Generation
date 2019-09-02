# Wind_Aware_Aircraft_Trajectory_Generation 

#### Functionalities:

* Modeling the effect of wind on trajectories
	<p align="middle">
	  <img src="https://github.com/paulsaswata/Wind_Aware_Aircraft_Trajectory_Generation/blob/master/v_H1.00/src/sample/model2d.png" width="45%" />
	  <img src="https://github.com/paulsaswata/Wind_Aware_Aircraft_Trajectory_Generation/blob/master/v_H1.00/src/sample/model3d.png" width="45%" /> 
	</p>		
		
* Generating wind-aware trajectories
	<p align="middle">
	  <img src="https://github.com/paulsaswata/Wind_Aware_Aircraft_Trajectory_Generation/blob/master/v_H1.00/src/sample/aware2d.png" width="45%" />
	  <img src="https://github.com/paulsaswata/Wind_Aware_Aircraft_Trajectory_Generation/blob/master/v_H1.00/src/sample/aware3d.png" width="45%" /> 
	</p>		

#### Before you start:

* For the instructions given below, Python3 is required with Matplotlib and Numpy installed

* The commands have been tested only for linux command line interface 

#### The instructions for using the software are given below:

* Create a configuration file with the name `config.txt` in the following format: 
		`initial_longitude,initial_latitude,initial_heading,final_longitude,final_latitude,final_heading,initial_altitude,baseline_glide_ratio,dirty_configuration_glide_ratio,best_gliding_speed,bank_angle,wind_heading,wind_speed`

* For modeling the effect of wind, create a `config.txt` file in the above format, put `model_wind` and `view.py` (for viewing the trajectories in 2D and 3D) in the same folder along with the `config.txt` file and type:

	`$./model_wind` 


	`$python view.py` (To visualize the effect of wind)

* For generating a wind-aware trajectory, create a `config.txt` file in the above format, put `wind_aware` and `view.py` (for viewing the trajectories in 2D and 3D) in the same folder along with the `config.txt` file and type:

	`$./wind_aware` 


	`$python view.py` (To visualize the wind-aware trajectory)

* The `src` folder contains the C source files.

##### Known bugs:
* Static memory allocation is prone to `segmentation fault`
* Static array sizes cannot deal with large number of points
* In case the initial Dubins (C1->S->C2) has only C1 or only C1->S, the spirals and extended runway do not start

##### Note: 
{dubins.c, dubins.h} - Copyright &copy; 2008-2014, [Andrew Walker](https://github.com/AndrewWalker "Github Link")

##### DISCLAIMER: 
Copyright &copy; 2018 Saswata Paul

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

#### <p align="middle">***** THIS IS AN OPEN SOURCE PROJECT! *****</p>
