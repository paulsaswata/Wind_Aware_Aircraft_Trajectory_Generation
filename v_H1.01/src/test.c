#include "demo.h"
#include "helper.h"
#include <stdio.h>
#include <dirent.h> 
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

//main EnTRY function
Seg basic_path(Packet data)
{
	Packet pack; //for storing complete path
 
        int i, limit,j,k;
        
//unpacking packet
        double q1[3];
	double q2[3];
	double min_radius=data.min_rad;
	double start_altitude=data.start_altitude;
	int angle=data.angle;
	double WIND_VELOCITY=data.windspeed;
	double baseline_g=data.baseline_g;

        for(i=0;i<3;i++)
	{
		q1[i]=data.p1[i];
		q2[i]=data.p2[i];
	}
	
	Result dubins=demo(q1,q2,min_radius, data.interval); //sending configs to demo to generate DP
        //dubins.arr =[long, lat, heading, ?unknown]

	Seg dubin_parts=split(dubins); //sending DP to split into segmentss

	dubin_parts=assign_altitude(dubin_parts, start_altitude, q1[0], q1[1], angle, data.baseline_g);//Send dubin_parts to assign_altitude() to get alti for each point

	//generates possible spiral segment with altitude
	Seg path_with_spiral= generate_spiral(dubin_parts,min_radius,angle,data.baseline_g);

	path_with_spiral= find_extended_runway(path_with_spiral,q2[0],q2[1],q2[2],q1[0],q1[1],q1[2],start_altitude, angle, min_radius, data.interval, data.baseline_g, data.dirty_g); //finds extended runway
	print_trajectory(path_with_spiral, angle, q2[0],q2[1],q2[2]); //saving to file
	
//	printf("No wind trajectory generated!\n");
	return path_with_spiral;
}

Seg2 model_wind(Seg path_with_spiral, Packet data)
{
        int i, limit,j,k;
        
//unpacking packet
        double q1[3];
	double q2[3];
	double min_radius=data.min_rad;
	double start_altitude=data.start_altitude;
	int angle=data.angle;
	double WIND_VELOCITY = data.windspeed;
	double WIND_HEADING =  data.wind_heading;
	double baseline_g=data.baseline_g;

        for(i=0;i<3;i++)
	{
		q1[i]=data.p1[i];
		q2[i]=data.p2[i];
	}
	
	Seg2 wind_path;	
	wind_path.spiral=false;
	wind_path.extended=false;
	wind_path.end_alt=0.0;

        Curve augmented_curve_A= wind_curveA(path_with_spiral,WIND_HEADING, WIND_VELOCITY, OMEGA_30_DEGREE_BANK, min_radius,start_altitude, q1[0], q1[1], angle, baseline_g, data.airspeed); //send first curve to be modified by wind
	wind_path.aug_C1=augmented_curve_A;

	Curve augmented_SLS= wind_SLS(path_with_spiral,WIND_HEADING, WIND_VELOCITY,augmented_curve_A,baseline_g, data.airspeed, data.dirty_g); //send middle straight line segment to be modified
	wind_path.aug_SLS=augmented_SLS;	
	Curve augmented_curve_B= wind_curveB(path_with_spiral,WIND_HEADING, WIND_VELOCITY,OMEGA_30_DEGREE_BANK, min_radius, augmented_SLS, angle,baseline_g, data.airspeed,q2[2]); //send second curve to be modified
	wind_path.aug_C2=augmented_curve_B;
	Curve augmented_spiral;
	augmented_spiral.spiral=false;
	Curve augmented_extended;
	augmented_extended.extended=false;

	if(path_with_spiral.lenspiral>0) //augmenting spiral
	{
		augmented_spiral= wind_spiral(path_with_spiral,WIND_HEADING, WIND_VELOCITY,OMEGA_30_DEGREE_BANK, min_radius, augmented_curve_B, angle,baseline_g, data.airspeed,q2[2]);
		wind_path.aug_SPIRAL=augmented_spiral;
		wind_path.spiral=true;
	}
	if(path_with_spiral.extended) //augmenting extended runway
	{
		augmented_extended= wind_extended(path_with_spiral,WIND_HEADING, WIND_VELOCITY, augmented_spiral, augmented_curve_B, q2[0],q2[1],q2[2],baseline_g, data.airspeed, data.dirty_g);
		wind_path.aug_EXTENDED=augmented_extended;
		wind_path.extended=true;

	}
	save_wind_in_file(augmented_curve_A,  augmented_SLS, augmented_curve_B, augmented_spiral, augmented_extended, data.file_name, data.alphabet);//saves augmented path in file

	//calculate total shift in path
	if (wind_path.extended)
	{
		wind_path.total_shift= horizontal(data.runway[0], data.runway[1], wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve-1][0], wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve-1][1]);	
		wind_path.end_alt=wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve-1][4];
	}
	else
	{
		if(wind_path.spiral)
		{
			wind_path.total_shift= horizontal(data.runway[0], data.runway[1], wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve-1][0], wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve-1][1]);
			wind_path.end_alt=wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve-1][4];		
		}
		else
		{
			wind_path.total_shift= horizontal(data.runway[0], data.runway[1], wind_path.aug_C2.points[wind_path.aug_C2.len_curve-1][0], wind_path.aug_C2.points[wind_path.aug_C2.len_curve-1][1]);
			wind_path.end_alt=wind_path.aug_C2.points[wind_path.aug_C2.len_curve-1][4];		
	
		}
	}

//	printf("Wind modelled! \n");
	return wind_path;        
}


int main()
{
	

	FILE *myFile;
	myFile = fopen("config.txt", "r");			
	float configArray[13];//reading configuration file into array
	if (myFile == NULL)
	{
		printf("Error Reading File\n");
		exit (0);
	}
	for (int i = 0; i < 13; i++)
	{
		fscanf(myFile, "%f,", &configArray[i] );
	}
	fclose(myFile);
	double q1[3];//Initial x,y heading //CONVERT TO 0,0, East-X
	for (int i = 0; i < 3; i++) 
	{
	        q1[i] = configArray[i];
	}
	double q2[3]; //Runway x,y, heading //CONVERT TO 0,0, East-X
	for (int i =0; i < 3; i++) 
	{
	        q2[i] = configArray[i+3];
	}

    double EMERGENCY_ALTITUDE = configArray[6]/364173.0;// in  
	double BEST_GLIDE_RATIO=configArray[7]; 
	double DIRTY_CONFIG_GLIDE_RATIO=configArray[8];
	double BEST_GLIDING_AIRSPEED= configArray[9]; //in knots
	int BANK_ANGLE= configArray[10];
	double WIND_HEADING= configArray[11];
	double WIND_SPEED= configArray[12];

	double INTERVAL =0.001; //use 0.001 for A320 and 0.0001 for c172. NEED TO CHANGE ARRAY SIZES in helper.h STRUCTS!!!!!!!!!
//------------------------------------------------------------------------
    int i, filename=0;
    char alphabet='h';
	Packet dat; //creating a packet with constants
	for(i=0;i<3;i++)
	{
		dat.p1[i]=q1[i];

		dat.runway[i]=q2[i];
	}	
    dat.interval= INTERVAL;
    dat.start_altitude=EMERGENCY_ALTITUDE; //initial altitude 	
	dat.windspeed=(WIND_SPEED*1.68781/364173.0);
	dat.wind_heading=WIND_HEADING;
	dat.airspeed= (BEST_GLIDING_AIRSPEED*1.68781/364173.0);
	dat.baseline_g=BEST_GLIDE_RATIO;
	dat.dirty_g=DIRTY_CONFIG_GLIDE_RATIO;
	dat.file_name=filename;
	dat.alphabet=alphabet;

	
	Packet dat_30; //condition specific variables will be initialized in this packet
	dat_30=dat;
	for(i=0;i<3;i++)
	{
		dat_30.p2[i]=q2[i];
	}
	dat_30.angle=30;
	dat_30.min_rad=(BEST_GLIDING_AIRSPEED*BEST_GLIDING_AIRSPEED)/(11.29* tan(dat_30.angle*PI/180))/364173.0; //v^2/(G x tan(bank_angle))

	if(true) //resetting pilot instructions
	{
		remove("pilot_instructions.txt");
	}

	Seg basic_trajectory=basic_path(dat_30); //get first_dubins

	Seg2 wind_1=model_wind(basic_trajectory,dat_30);
			
	double shift= wind_1.total_shift;
	double init_shift=shift; //used to stop loop if somehow exceeds instead of decreasing
	double wind_alt=wind_1.end_alt ;//altitude of last point of wind augmented

    // Start of heuristic iterative approach for finding P'
	if(true) //put false for not finding P' and just modeling the effect of wind
	{
		//catch runway code starts here
		int iter=1;
		float distance=0.0; //adjust this depending on shift. BASIS OF OUR HUERISTICS
		while(shift>0.000137)
		{
			if(true) //resetting pilot instructions
			{
				remove("pilot_instructions.txt");
			}
			distance=distance+shift;
			double reverse_wind_heading= WIND_HEADING + PI; 

			Pair new_point=along_heading_at_distance(q2[0], q2[1], reverse_wind_heading, (distance));

			iter=iter+1;
			Packet dat_temp; //condition specific variables will be initialized now
			dat_temp=dat;

			dat_temp.p2[1]=new_point.y;
			dat_temp.p2[0]=new_point.x;
			dat_temp.p2[2]=q2[2];
	
			dat_temp.angle=30;
			dat_temp.min_rad=(BEST_GLIDING_AIRSPEED*BEST_GLIDING_AIRSPEED)/(11.29* tan(dat_temp.angle*PI/180))/364173.0; //v^2/(G x tan(bank_angle))

			Seg basic_temp=basic_path(dat_temp); //get first_dubins

			Seg2 wind_temp=model_wind(basic_temp,dat_temp);
	
			shift= wind_temp.total_shift;
			wind_alt=wind_temp.end_alt;
		} 
	}
	return 0;
}
