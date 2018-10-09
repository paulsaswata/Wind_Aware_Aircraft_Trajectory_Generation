#include "helper.h"
#include "demo.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>

#define ARRSIZE(arr) (sizeof(arr)/sizeof(*(arr)))//for quicksort

#define FILL(arr, val) \
for(int i_##arr = 0; i_##arr < sizeof arr / sizeof *arr; ++i_##arr) \
{ \
    arr[i_##arr][3] = val;\
}


static int path_number=1; //for generating automatic incremental filenames

//for inplace sorting in thread function
int compare(const void *a, const void *b)
{
	//a and b are first columns
	double d1 = *(((const double*)a)+3);//value of 4th clolumn
	double d2 = *(((const double*)b)+3);
	if (d1 > d2) return  1;
	if (d1 < d2) return -1;
	return 0;
}

//finds distance of a point from a line
double pointdist(double a, double b, double c, int x, int y)
{
	double numerator= fabs((a*x)+(b*y)+c);
	double denominator= sqrt(a*a+b*b);
	double dist= numerator/denominator;
        return dist;   
}

//finds the position of a point wrt a line.  
double position(double A, double B, double C, double x, double y)
{ 
	/*
	We conclude that the point A(x1,y1) is ABOVE (anticlockwise) the line ax+by+c=0 if
	(i) ax1+by1+c>0 and b>0
	(ii) ax1+by1+c<0 and b<0
	Therefore, multiplication >0

	We conclude that the point A(x1,y1) is BELOW (clockwise) the line ax+by+c=0 if
	(i) ax1+by1+c<0 and b>0
	(ii) ax1+by1+c>0 and b<0
	Therefore, multiplication <0

	*/
	double s = ((A*x) + (B*y) + C) * B;
	return s;
}

//finds a new pont along a given heading 50 feet away
Pair along_heading(double x0, double y0, double theta)
{
	Pair new_point;
//      double rad= theta * PI / 180; //converting to radians if in degrees
        double rad=theta;
	new_point.x= x0 + 0.00014*cos(rad); //50 feet away
	new_point.y= y0 + 0.00014*sin(rad);
        new_point.heading=theta; 
        return new_point;
}

//finds a new pont along a given heading at a given distance
Pair along_heading_at_distance(double x0, double y0, double theta, double distance)
{
	Pair new_point;

        double rad=theta;
	new_point.x= x0 + distance*cos(rad); //distance units away
	new_point.y= y0 + distance*sin(rad);
        new_point.heading=theta; 
        return new_point;
}

//finds centre of turn given starting point x0,y0; heading of starting point theta and radius of turn.   
Pair circle_centre(double x0, double y0, double theta, double radius)
{
	Pair new_point;
    double rad=theta;
	new_point.x= x0 + radius*cos(rad); 
	new_point.y= y0 + radius*sin(rad);
    new_point.heading=theta; 
    return new_point;
}
//returns heading of line between two points in radians
double heading(double x1, double y1, double x2, double y2)
{
	double slope = (y2-y1)/(x2-x1);
	double ret= atan(slope);
	return ret;
	
}

//spits path into segments and returns them.
Seg split(Result dub)
{
	Seg segments;
	int i, j, k, l;
	for (i=0;i<dub.len;i++)
	{
		if(dub.arr[i][2]!=dub.arr[i+1][2])
		{
			segments.C1[i][0]=dub.arr[i][0];
		        segments.C1[i][1]=dub.arr[i][1];
		        segments.C1[i][2]=dub.arr[i][2];
		        segments.C1[i][3]=dub.arr[i][3];
		}
		else 
			break;		
	}
	segments.lenc1=i;
        k=0;	
	for(j=i;j<dub.len;j++)
	{
		if(dub.arr[j][2]==dub.arr[j+1][2])
		{
         		segments.SLS[k][0]=dub.arr[j][0];
	      		segments.SLS[k][1]=dub.arr[j][1];
		        segments.SLS[k][2]=dub.arr[j][2];
		        segments.SLS[k][3]=dub.arr[j][3];
			k++;	
		}
		else 
                { //for last of SLS 
                        segments.SLS[k][0]=dub.arr[j][0];
		        segments.SLS[k][1]=dub.arr[j][1];
		        segments.SLS[k][2]=dub.arr[j][2];
		        segments.SLS[k][3]=dub.arr[j][3];
			k++; 
		    	break;
		} 
	}

	segments.lensls=k;
	i=0;
	k=0;
	for(i=j+1; i<dub.len;i++)
	{
		segments.C2[k][0]=dub.arr[i][0];
		segments.C2[k][1]=dub.arr[i][1];
		segments.C2[k][2]=dub.arr[i][2];
		segments.C2[k][3]=dub.arr[i][3];
		k++;		
	}
        segments.lenc2=k;
        return segments;	
}

//Mathematical function that calculates Horizontal Distance between two points
double horizontal(double x1, double y1, double x2, double y2)
{
	double dist=sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        return dist; 
}


//Mathematical function that calculates Horizontal Distance between two points in 3D
double distance_3d(double x1, double y1, double z1, double x2, double y2, double z2)
{
	
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));
}

//Mathematical function that takes in heading WRT E=0 in radian and returns heading WRT N=0 in degrees
double azmth(double E_rad)
{
	double E_deg = E_rad * (180.0/PI);
        double h = 450.0 - E_deg;
        double N_deg=0.0;
        if(h>=360.0)
	{
		N_deg = h - 360.0;
	}
	else
	{
		N_deg = h;
	}
	return N_deg;
}

//######################################## WIND MODELLING HELPER FUNCTIONS #####################################################
//Function that finds centre of an arc given three points on that arc
Pair find_centre(double x1, double y1, double x2, double y2, double x3, double y3)
{
	Pair center; // to store the center
	//finding perpendicular bisector pA.X + pB.Y + pC = 0
		//for line 1-----2 let the line be P
	double PA, PB, PC, Pm, midX12, midY12;
	midX12= (x2+x1)/2;
	midY12= (y2+y1)/2;
	Pm= (x1-x2)/(y2-y1);

	PA=Pm;
	PB=-1;
	PC=midY12-(Pm*midX12);  

	//for line 1-----3 let the line be Q
	double QA, QB, QC, Qm, midX13, midY13;
	midX13= (x3+x1)/2;
	midY13= (y3+y1)/2;
	Qm= (x1-x3)/(y3-y1);

	QA=Qm;
	QB=-1;
	QC=midY13-(Qm*midX13);    
	/* Will be needed below
	Cross multiplication method of solving equations
	IF:
	a1x + b1y + c1 =0 ...(1)
	a2x + b2y + c2 =0 ...(2)

	THEN:
	x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1)
	y = (c1*a2 - c2*a1) / (a1*b2 - a2*b1)
	*/
	//finding centre USING CROSS MULTIPLPICATION METHOD 
	center.x= ((PB*QC)-(QB*PC))/((PA*QB)-(QA*PB));
	center.y= ((PC*QA)-(QC*PA))/((PA*QB)-(QA*PB));
	
	return center;
}

//function that calculates altitude for curves with wind
double curve_altitude(double last_height, double distance, int angle, double airspeed_heading, double wind_heading,double wind_velocity, double baseline_g, double airspeed)
{	
	// alpha is the angle between the airspeed heading and wind heading
	//theta is the angle between the actual direction of movement and airspeed
 	double alpha= fabs(airspeed_heading-wind_heading);
	double ground_speed= airspeed + ((wind_velocity) * cos(alpha)); //actual distance wrt ground 
	double Rg;
	if(angle==20)
		Rg=baseline_g*cos(20*PI/180);
	if(angle==30)
		Rg=baseline_g*cos(30*PI/180);
	if(angle==45)
		Rg=baseline_g*cos(45*PI/180);
	Rg=Rg*(ground_speed/airspeed);
    double heigt= last_height - (distance/Rg);
    return heigt;  

}

//Mathematical function that calculates new altitude for lines with wind
double line_altitude(double last_height, double distance, double airspeed_heading, double wind_heading, double wind_velocity, bool extended, double baseline_g, double airspeed,double Rg_dirty)
{
	// alpha is the angle between the airspeed heading and wind heading
	//theta is the angle between the actual direction of movement and airspeed
 	double alpha= fabs(airspeed_heading-wind_heading);
	double ground_speed= airspeed + ((wind_velocity) * cos(alpha)); //actual distance wrt ground 
	double Rg;
	if(extended)        
	{
		Rg=Rg_dirty*(ground_speed/airspeed);
	}        
	else
	{
		Rg=baseline_g*(ground_speed/airspeed);
	}
	double heigt= last_height - (distance/Rg);
    return heigt;  
}

//function to determine rletive position of a point wrt centre shift line
double pos_wrt_centre(Pair one, Pair point, double wind_heading)
{
	//one = initial centre, two = another point on centre shift
	Pair two= along_heading(one.x, one.y, wind_heading);
	//eqquation of centre shift line Ax+By+C=0
	double A, B, C, m;
	m= (two.y-one.y)/(two.x-one.x);
	A=m;
	B=-1;
	C=two.y- (m*two.x);
        //
	double pos= position(A,B,C, point.x, point.y);	
	return pos;

}
//Functions that determines if turns are clockwise or counterclockwise
int orientation(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double val= (y2 - y1)*(x3 - x2) - (y3 - y2)*(x2 - x1);
	if (val<0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

//Function to generate 100 discrete theta's for the first curve
Thetas generate_thetasA(Seg path, Pair initial_centre)
{
	Thetas collection;
	int sign; //determines increasing or decreasing

//detect angle shift in c1

	double theta_first= atan2(path.C1[0][1] - initial_centre.y, path.C1[0][0] - initial_centre.x);
	double theta_last= atan2(path.C1[path.lenc1-1][1] - initial_centre.y, path.C1[path.lenc1-1][0] - initial_centre.x);

	sign= orientation(path.C1[0][0], path.C1[0][1], path.C1[path.lenc1/2][0], path.C1[path.lenc1/2][1],path.C1[path.lenc1-1][0], path.C1[path.lenc1-1][1]); //Orientation returns -1 for clockwise turns and 1 for anticlockwise turns	


	int i;
	double step = fabs(theta_first-theta_last)/(path.lenc1-1);
	for(i=0;i<path.lenc1;i++)
	{
		collection.thetas[i]= theta_first + (i * (step * sign));
	}

	return collection;
}

//Function to generate 100 discrete theta's for the first curve
Thetas generate_thetasB(Seg path, Pair initial_centre)
{
	Thetas collection;
	int sign; //determines increasing or decreasing

	//detect angle shift in c1

	double theta_first= atan2(path.C2[0][1] - initial_centre.y, path.C2[0][0] - initial_centre.x);
	double theta_last= atan2(path.C2[path.lenc2-1][1] - initial_centre.y, path.C2[path.lenc2-1][0] - initial_centre.x);

	sign= orientation(path.C2[0][0], path.C2[0][1], path.C2[path.lenc2/2][0], path.C2[path.lenc2/2][1],path.C2[path.lenc2-1][0], path.C2[path.lenc2-1][1]); //Orientation returns -1 for clockwise turns and 1 for anticlockwise turns	


	int i;
	double step = fabs(theta_first-theta_last)/(path.lenc2-1);
	for(i=0;i<path.lenc2;i++)
	{
		collection.thetas[i]= theta_first + (i * (step * sign));
	}

	return collection;
}

//Functions that calculates the time required to travel first curve in no wind
double c1_time(Seg path, double airspeed, double radius)
{
	///divide the curve into two halves so that the sum of time can be got. This is in order to accomodate inscribed angles >= PI 
	Pair start_c1, mid_c1, end_c1;
	start_c1.x=path.C1[0][0];
	start_c1.y=path.C1[0][1];
	mid_c1.x=path.C1[(int)(path.lenc1/2)][0];
	mid_c1.y=path.C1[(int)(path.lenc1/2)][1];
	end_c1.x=path.C1[path.lenc1-1][0];
	end_c1.y=path.C1[path.lenc1-1][1];
	//finding angle traversed

	double inscribed_angle1, inscribed_angle2;
	double d1= sqrt((start_c1.x-mid_c1.x)*(start_c1.x-mid_c1.x) +(start_c1.y-mid_c1.y)*(start_c1.y-mid_c1.y));
	double cos_inscribed_angle1= 1- ((d1*d1)/(2*radius*radius));
	inscribed_angle1=acos(cos_inscribed_angle1);
	double time1= (inscribed_angle1*radius)/airspeed;

	double d2= sqrt((end_c1.x-mid_c1.x)*(end_c1.x-mid_c1.x) +(end_c1.y-mid_c1.y)*(end_c1.y-mid_c1.y));
	double cos_inscribed_angle2= 1- ((d2*d2)/(2*radius*radius));
	inscribed_angle2=acos(cos_inscribed_angle2);
	double time2= (inscribed_angle2*radius)/airspeed;

	return (time1+time2);
}

//Function that augments a 2d curve for wind and returns a modified 2d curve
//omega= rate of change of heading in standard rate turn
Curve wind_curveA(Seg path,double wind_heading,double wind_velocity,double omega, double radius, double start_altitude, double initial_x, double initial_y, int angle, double baseline_g, double airspeed)
{

	FILE *output_file;
	output_file = fopen("c1centres.csv", "w"); 

        Pair initial_centre= find_centre(path.C1[0][0], path.C1[0][1], path.C1[path.lenc1/2][0], path.C1[path.lenc1/2][1], path.C1[path.lenc1-2][0], path.C1[path.lenc1-2][1]);

	Curve augmented_C1; // to store augmented first curve
	augmented_C1.len_curve=0;    

	double total_time=c1_time(path,airspeed,radius);

	Pair last_point;
	last_point.x=path.C1[path.lenc1-1][0];
	last_point.y=path.C1[path.lenc1-1][1];
	Thetas theta_pool=generate_thetasA(path, initial_centre);	//pool od thetas to use	


	int i;
	for (i=0;i<path.lenc1;i++)
	{		

		double centre_shift=fabs(total_time/50 *wind_velocity);//distance shifted by centre in time t
		Pair current_centre;		
		if(i==0)
		{
			current_centre=initial_centre;
		}
		else
		{
			current_centre= along_heading_at_distance(initial_centre.x, initial_centre.y, wind_heading, centre_shift);
			initial_centre.x=current_centre.x;
			initial_centre.y=current_centre.y;
		}
	fprintf(output_file, "%f,%f\n", current_centre.x,current_centre.y);

	//Backcalculating to find the angle inscribed by the original point with initial centre
 		augmented_C1.points[i][0]=current_centre.x + radius*cos(theta_pool.thetas[i]); //x of new point
 		augmented_C1.points[i][1]=current_centre.y + radius*sin(theta_pool.thetas[i]); //y of new point
                augmented_C1.points[i][2]=path.C1[i][2]; //heading of aircraft at new point
		augmented_C1.len_curve=augmented_C1.len_curve+1;				

		augmented_C1.points[i][4]=path.C1[i][4];
	}           


	augmented_C1.shift= horizontal(augmented_C1.points[augmented_C1.len_curve-1][0],augmented_C1.points[augmented_C1.len_curve-1][1],path.C1[path.lenc1-1][0],path.C1[path.lenc1-1][1]);
	fclose(output_file);

	//-----------------PILOT INSTRUCTIONS
	FILE *instructions_file;
	instructions_file = fopen("pilot_instructions.txt", "a");
	fprintf(instructions_file,"30 degree bank for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",total_time,augmented_C1.points[augmented_C1.len_curve-1][0],augmented_C1.points[augmented_C1.len_curve-1][1],path.SLS[path.lensls-1][2], azmth(path.SLS[path.lensls-1][2]),augmented_C1.points[augmented_C1.len_curve-1][4]*364173.0);
	fclose(instructions_file);
	sprintf(augmented_C1.instructions,"30 degree bank for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",total_time,augmented_C1.points[augmented_C1.len_curve-1][0],augmented_C1.points[augmented_C1.len_curve-1][1],path.SLS[path.lensls-1][2], azmth(path.SLS[path.lensls-1][2]),augmented_C1.points[augmented_C1.len_curve-1][4]*364173.0); 
	//-----------------SAVED INSTRUCTIONS
	return augmented_C1;
} 

//Function that modifies the SLS straight line segment to reflect effect of wind on a line segment
Curve wind_SLS(Seg path,double wind_heading,double wind_velocity, Curve augmented_curve_A, double baseline_g, double airspeed,double Rg_dirty)
{


	Curve augmented_SLS;
	augmented_SLS.len_curve=0;
	//first point in augmented SLS
	Pair first_point= along_heading_at_distance(path.SLS[0][0], path.SLS[0][1], wind_heading, augmented_curve_A.shift);
	augmented_SLS.points[0][0]=first_point.x;
	augmented_SLS.points[0][1]=first_point.y;
	augmented_SLS.points[0][2]=path.SLS[0][2]; //heading of aircraft in original SLS

	augmented_SLS.points[0][4]=path.SLS[0][4];
	augmented_SLS.len_curve=augmented_SLS.len_curve+1;	


	// alpha is the angle between the airspeed heading and wind heading
	//theta is the angle between the actual direction of movement and airspeed
   	double airspeed_heading = path.SLS[2][2]; //heading of second element (just to be safe) of SLS

	double alpha= fabs(airspeed_heading-wind_heading);

	double w_perp= (wind_velocity) * sin(alpha); //component of windspeed perpendicular to airspeed or CROSSWIND

	double theta= atan(w_perp/(airspeed + ((wind_velocity) * cos(alpha))));

	double movement_heading=airspeed_heading-theta; //actual direction of motion wrt ground

	double original_distance= horizontal(path.SLS[0][0], path.SLS[0][1], path.SLS[path.lensls-1][0], path.SLS[path.lensls-1][1]); //original distance travelled wrt windmass, length of original SLS

    //finding end of augmented SLS
    Pair parallel_end= along_heading_at_distance(augmented_SLS.points[0][0], augmented_SLS.points[0][1], airspeed_heading, original_distance); //end of parallel line to original SLS

	double time_shift=fabs(original_distance/ (airspeed + ((wind_velocity) * cos(alpha))));  

	double shift_of_end= fabs(time_shift* w_perp); //shift of end of parallel ("dell y" in calculations) 

	Pair end_aug_SLS=along_heading_at_distance(parallel_end.x , parallel_end.y , wind_heading , shift_of_end); //end of augmented SLS

	//last point in augmented SLS
	augmented_SLS.points[1][0]=end_aug_SLS.x;
	augmented_SLS.points[1][1]=end_aug_SLS.y;
	augmented_SLS.points[1][2]=path.SLS[0][2]; //heading of aircraft in original SLS

	augmented_SLS.points[1][4]=path.SLS[path.lensls-1][4];

	augmented_SLS.len_curve=augmented_SLS.len_curve+1;	

	augmented_SLS.shift= horizontal(augmented_SLS.points[augmented_SLS.len_curve-1][0],augmented_SLS.points[augmented_SLS.len_curve-1][1],path.SLS[path.lensls-1][0],path.SLS[path.lensls-1][1]) ; //total shift until this point
	//-----------------PILOT INSTRUCTIONS
	FILE *instructions_file;
	instructions_file = fopen("pilot_instructions.txt", "a");
	fprintf(instructions_file,"Straight line glide for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",time_shift,augmented_SLS.points[augmented_SLS.len_curve-1][0],augmented_SLS.points[augmented_SLS.len_curve-1][1],path.SLS[path.lensls-1][2],azmth(path.SLS[path.lensls-1][2]),augmented_SLS.points[augmented_SLS.len_curve-1][4]*364173.0);
	fclose(instructions_file);
	sprintf(augmented_SLS.instructions,"Straight line glide for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",time_shift,augmented_SLS.points[augmented_SLS.len_curve-1][0],augmented_SLS.points[augmented_SLS.len_curve-1][1],path.SLS[path.lensls-1][2],azmth(path.SLS[path.lensls-1][2]),augmented_SLS.points[augmented_SLS.len_curve-1][4]*364173.0);

	//-----------------SAVED INSTRUCTIONS

	return augmented_SLS; //CONTAINS ONLY TWO POINTS
}

//Function that finds the centre of turn (without wind) given original dubins, (x,y,heading) of augmented SLS and radius of turn
Pair find_clockwise_centre(double x0, double y0, double heading, double radius)
{
	Pair centre;
	centre.x= x0 + radius*cos((PI/2)-heading); 
	centre.y= y0 + radius*sin((PI/2)-heading);
	centre.heading=0.0; 
        return centre;
}

//Functions that calculates the time required to travel first curve in no wind
double c2_time(Seg path, double airspeed, double radius)
{
	///divide the curve into two halves so that the sum of time can be got. This is in order to accomodate inscribed angles >= PI 
	Pair start_C2, mid_C2, end_C2;
	start_C2.x=path.C2[0][0];
	start_C2.y=path.C2[0][1];
	mid_C2.x=path.C2[(int)(path.lenc2/2)][0];
	mid_C2.y=path.C2[(int)(path.lenc2/2)][1];
	end_C2.x=path.C2[path.lenc2-1][0];
	end_C2.y=path.C2[path.lenc2-1][1];
	//finding angle traversed

	double inscribed_angle1, inscribed_angle2;
	double d1= sqrt((start_C2.x-mid_C2.x)*(start_C2.x-mid_C2.x) +(start_C2.y-mid_C2.y)*(start_C2.y-mid_C2.y));
	double cos_inscribed_angle1= 1- ((d1*d1)/(2*radius*radius));
	inscribed_angle1=acos(cos_inscribed_angle1);
	double time1= (inscribed_angle1*radius)/airspeed;

	double d2= sqrt((end_C2.x-mid_C2.x)*(end_C2.x-mid_C2.x) +(end_C2.y-mid_C2.y)*(end_C2.y-mid_C2.y));
	double cos_inscribed_angle2= 1- ((d2*d2)/(2*radius*radius));
	inscribed_angle2=acos(cos_inscribed_angle2);
	double time2= (inscribed_angle2*radius)/airspeed;

	return (time1+time2);
}

//Function that augments second 2d curve of Dubins for wind and returns a modified 2d curve
Curve wind_curveB(Seg path, double wind_heading, double wind_velocity, double omega, double radius, Curve augmented_SLS, int angle, double baseline_g, double airspeed, double rnwy_heading)
{

	//change of heading in original curve 2
	double delta_heading= path.C2[path.lenc2-1][2]-path.C2[0][2];


	FILE *output_file;
	output_file = fopen("c2centres.csv", "w"); 


	//Find centre of original curve C2
	Pair original_c2_centre= find_centre(path.C2[0][0], path.C2[0][1], path.C2[path.lenc2/2][0], path.C2[path.lenc2/2][1], path.C2[path.lenc2-1][0], path.C2[path.lenc2-1][1]);

	Pair initial_centre= along_heading_at_distance(original_c2_centre.x, original_c2_centre.y, wind_heading, augmented_SLS.shift);

	fprintf(output_file, "%f,%f\n", original_c2_centre.x,original_c2_centre.y);

	Curve augmented_C2; // to store augmented first curve
	augmented_C2.len_curve=0;

	double total_time=c2_time(path,airspeed,radius);

	Pair first_point;
	first_point.x=path.C2[0][0];
	first_point.y=path.C2[0][1];
	Thetas theta_pool=generate_thetasB(path, original_c2_centre);	//pool od thetas to use	

	int i;
	for (i=0;i<path.lenc2;i++)
	{
		double centre_shift=fabs(total_time/50 *wind_velocity);//distance shifted by centre in time t
		Pair current_centre;		
		if(i==0)
		{
			current_centre=initial_centre;
		}
		else
		{
			current_centre= along_heading_at_distance(initial_centre.x, initial_centre.y, wind_heading, centre_shift);
			initial_centre.x=current_centre.x;
			initial_centre.y=current_centre.y;
		}

	fprintf(output_file, "%f,%f\n", current_centre.x,current_centre.y);
		
 		augmented_C2.points[i][0]=current_centre.x+ radius*cos(theta_pool.thetas[i]); //x of new point
 		augmented_C2.points[i][1]=current_centre.y+ radius*sin(theta_pool.thetas[i]); //y of new point
                augmented_C2.points[i][2]=path.C2[i][2]; //heading of aircraft at new point
		augmented_C2.len_curve=augmented_C2.len_curve+1;				

		augmented_C2.points[i][4]=path.C2[i][4];
	}	
	fclose(output_file);
	augmented_C2.centre=initial_centre;
	//-----------------PILOT INSTRUCTIONS
	FILE *instructions_file;
	instructions_file = fopen("pilot_instructions.txt", "a");
	fprintf(instructions_file,"30 degree bank for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",total_time,augmented_C2.points[augmented_C2.len_curve-1][0],augmented_C2.points[augmented_C2.len_curve-1][1],rnwy_heading,azmth(rnwy_heading),augmented_C2.points[augmented_C2.len_curve-1][4]*364173.0);
	fclose(instructions_file);
	sprintf(augmented_C2.instructions,"30 degree bank for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",total_time,augmented_C2.points[augmented_C2.len_curve-1][0],augmented_C2.points[augmented_C2.len_curve-1][1],rnwy_heading,azmth(rnwy_heading),augmented_C2.points[augmented_C2.len_curve-1][4]*364173.0);

	//-----------------SAVED INSTRUCTIONS


	return augmented_C2;

}

//Function to mdel effect of wind on spiral
Curve wind_spiral(Seg path,double wind_heading, double  wind_velocity, double omega, double radius, Curve augmented_curve_B, int angle, double baseline_g, double airspeed, double rnwy_heading)
{
	
	Curve augmented_Spiral;
	augmented_Spiral.len_curve=0; //initializing the container

	double step= 2*PI/50; //100 points per spiral circle
	Pair initial_centre= augmented_curve_B.centre;//centre of turn

	double start_theta= path.spiral_start_angle; //angle of starting point wrt centre 
	
	//generate circles
	int i;

	step=-step; //for clockwise turn, make positive for anti clockwise turns	
		
	//time_between each point
	double time_step=((2*PI*radius)/airspeed)/50; //50 points per spiral

	double total_time=0;
	for(i=0;i<path.lenspiral;i++)
	{
		double time= time_step*i; //total time from beginnig
		total_time= time;
		double centre_shift= time*wind_velocity;
		Pair current_centre=along_heading_at_distance(initial_centre.x, initial_centre.y, wind_heading, centre_shift);		

		augmented_Spiral.points[i][0]=current_centre.x+(radius*cos((i*step)-start_theta)); //check - start or + start
		augmented_Spiral.points[i][1]=current_centre.y+(radius*sin((i*step)-start_theta));	
		augmented_Spiral.points[i][2]=path.Spiral[i][2]; //MIGHT NEED TO CHANGE> ROUGH CALCULATION 

		augmented_Spiral.points[i][4]=path.Spiral[i][4];

		augmented_Spiral.len_curve=augmented_Spiral.len_curve+1;
		
	}
	//-----------------PILOT INSTRUCTIONS
	FILE *instructions_file;
	instructions_file = fopen("pilot_instructions.txt", "a");
	fprintf(instructions_file,"30 degree bank spiral for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",total_time,augmented_Spiral.points[augmented_Spiral.len_curve-1][0],augmented_Spiral.points[augmented_Spiral.len_curve-1][1],rnwy_heading,azmth(rnwy_heading),augmented_Spiral.points[augmented_Spiral.len_curve-1][4]*364173.0);
	fclose(instructions_file);
	sprintf(augmented_Spiral.instructions,"30 degree bank spiral for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",total_time,augmented_Spiral.points[augmented_Spiral.len_curve-1][0],augmented_Spiral.points[augmented_Spiral.len_curve-1][1],rnwy_heading,azmth(rnwy_heading),augmented_Spiral.points[augmented_Spiral.len_curve-1][4]*364173.0);
	//-----------------SAVED INSTRUCTIONS
	if(augmented_Spiral.len_curve>0)
	{
		augmented_Spiral.spiral=true;
	}
	return augmented_Spiral;
}


//Function to model effect of wind on extended segment
Curve wind_extended(Seg path,double wind_heading,double wind_velocity,Curve augmented_spiral,Curve augmented_C2,double rnwy_x,double rnwy_y,double rnwy_heading, double baseline_g, double airspeed,double Rg_dirty)
{
	Curve augmented_extended;
	augmented_extended.len_curve=0;
	//determine if starting from c2 or spiral by checking lenspiral
	double original_start_x, original_start_y;
	if(path.lenspiral>0) //spiral exists
	{

		original_start_x= path.Spiral[path.lenspiral-1][0];
		original_start_y= path.Spiral[path.lenspiral-1][1];

	//variables for calculating altitude 
		double current_height,current_x,current_y; //contains new altitude and last point after each assignment
		current_height=augmented_spiral.points[augmented_spiral.len_curve-1][4];
		current_x=augmented_spiral.points[augmented_spiral.len_curve-1][0];
		current_y=augmented_spiral.points[augmented_spiral.len_curve-1][1];

	// alpha is the angle between the airspeed heading and wind heading
	//theta is the angle between the actual direction of movement and airspeed
	
	   	double airspeed_heading = rnwy_heading; //heading of second element (just to be safe) of C2
		double alpha= fabs(airspeed_heading-wind_heading);
	
		double w_perp= (wind_velocity) * sin(alpha); //component of windspeed perpendicular to airspeed or CROSSWIND

		double theta= atan(w_perp/(airspeed + ((wind_velocity) * cos(alpha))));

		double movement_heading=airspeed_heading-theta; //actual direction of motion wrt ground

		double original_distance= horizontal(rnwy_x, rnwy_y, original_start_x, original_start_y); //original distance travelled wrt windmass, length of original SLS

		//first point from the last point of augmented_C1= last point in augmented_c1:
		augmented_extended.points[0][0]=augmented_spiral.points[augmented_spiral.len_curve-1][0];
		augmented_extended.points[0][1]=augmented_spiral.points[augmented_spiral.len_curve-1][1];
		augmented_extended.points[0][2]=rnwy_heading; //heading of aircraft in original SLS
		augmented_extended.points[0][4]=augmented_spiral.points[augmented_spiral.len_curve-1][4];
		augmented_extended.len_curve=augmented_extended.len_curve+1;	

		Pair parallel_end= along_heading_at_distance(augmented_extended.points[0][0], augmented_extended.points[0][1], airspeed_heading, original_distance);

		double time_shift=fabs(original_distance/ (airspeed + ((wind_velocity) * cos(alpha))));
		double shift_of_end= fabs(time_shift* w_perp);
		Pair end_aug_extended=along_heading_at_distance(parallel_end.x , parallel_end.y , wind_heading , shift_of_end);

		//last point in augmented SLS
		augmented_extended.points[1][0]=end_aug_extended.x;
		augmented_extended.points[1][1]=end_aug_extended.y;
		augmented_extended.points[1][2]=rnwy_heading; //heading of aircraft in original SLS

		augmented_extended.points[1][4]=0.0;		
		
		augmented_extended.len_curve=augmented_extended.len_curve+1;	
		//-----------------PILOT INSTRUCTIONS
		FILE *instructions_file;
		instructions_file = fopen("pilot_instructions.txt", "a");
		fprintf(instructions_file,"Dirty configuration straight glide for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",time_shift,augmented_extended.points[1][0],augmented_extended.points[1][1],augmented_extended.points[1][2],azmth(augmented_extended.points[1][2]),augmented_extended.points[1][4]*364173.0);
		fclose(instructions_file);
		sprintf(augmented_extended.instructions,"Dirty configuration straight glide for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",time_shift,augmented_extended.points[1][0],augmented_extended.points[1][1],augmented_extended.points[1][2],azmth(augmented_extended.points[1][2]),augmented_extended.points[1][4]*364173.0);

		//-----------------SAVED INSTRUCTIONS

	}
	else //no spiral, starts from C2
	{

		original_start_x= path.C2[path.lenc2-1][0];
		original_start_y= path.C2[path.lenc2-1][1];

	//variables for calculating altitude 
		double current_height,current_x,current_y; //contains new altitude and last point after each assignment
		current_height=augmented_C2.points[augmented_C2.len_curve-1][4];
		current_x=augmented_C2.points[augmented_C2.len_curve-1][0];
		current_y=augmented_C2.points[augmented_C2.len_curve-1][1];

	// alpha is the angle between the airspeed heading and wind heading
	//theta is the angle between the actual direction of movement and airspeed
	
	   	double airspeed_heading = rnwy_heading; //heading of second element (just to be safe) of C2
		double alpha= fabs(airspeed_heading-wind_heading);
	
		double w_perp= (wind_velocity) * sin(alpha); //component of windspeed perpendicular to airspeed or CROSSWIND

		double theta= atan(w_perp/(airspeed + ((wind_velocity) * cos(alpha))));

		double movement_heading=airspeed_heading-theta; //actual direction of motion wrt ground

		double original_distance= horizontal(rnwy_x, rnwy_y, original_start_x, original_start_y); //original distance travelled wrt windmass, length of original SLS

		//first point from the last point of augmented_C1= last point in augmented_c1:
		augmented_extended.points[0][0]=augmented_C2.points[augmented_C2.len_curve-1][0];
		augmented_extended.points[0][1]=augmented_C2.points[augmented_C2.len_curve-1][1];
		augmented_extended.points[0][2]=rnwy_heading; //heading of aircraft in original SLS
		augmented_extended.points[0][4]=augmented_C2.points[augmented_C2.len_curve-1][4];
		augmented_extended.len_curve=augmented_extended.len_curve+1;	

		Pair parallel_end= along_heading_at_distance(augmented_extended.points[0][0], augmented_extended.points[0][1], airspeed_heading, original_distance);

		double time_shift=fabs(original_distance/ (airspeed + ((wind_velocity) * cos(alpha))));
		double shift_of_end= fabs(time_shift* w_perp);
		Pair end_aug_extended=along_heading_at_distance(parallel_end.x , parallel_end.y , wind_heading , shift_of_end);

		//last point in augmented SLS
		augmented_extended.points[1][0]=end_aug_extended.x;
		augmented_extended.points[1][1]=end_aug_extended.y;
		augmented_extended.points[1][2]=rnwy_heading; //heading of aircraft in original SLS

		augmented_extended.points[1][4]=0.0;
		augmented_extended.len_curve=augmented_extended.len_curve+1;	
		//-----------------PILOT INSTRUCTIONS
		FILE *instructions_file;
		instructions_file = fopen("pilot_instructions.txt", "a");
		fprintf(instructions_file,"Dirty configuration straight glide for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",time_shift,augmented_extended.points[1][0],augmented_extended.points[1][1],augmented_extended.points[1][2],azmth(augmented_extended.points[1][2]),augmented_extended.points[1][4]*364173.0);
		fclose(instructions_file);
		sprintf(augmented_extended.instructions,"Dirty configuration straight glide for %f seconds. Should reach --> {%f,%f,(%f->Rad wrt E=0, %f->Deg wrt N=0),%f}\n",time_shift,augmented_extended.points[1][0],augmented_extended.points[1][1],augmented_extended.points[1][2],azmth(augmented_extended.points[1][2]),augmented_extended.points[1][4]*364173.0);
		//-----------------SAVED INSTRUCTIONS
	}

	augmented_extended.extended=true;
	return augmented_extended; //CONTAINS ONLY TWO POINTS
}

//function that saves augmented path in file
void save_wind_in_file(Curve augmented_curve_A, Curve augmented_SLS, Curve augmented_curve_B, Curve augmented_spiral, Curve augmented_extended, int filename, char alphabet)
{
	int i;
	char file_name[20];
	sprintf(file_name, "augmented.csv"); //generating new file name
	//saving in file for testing---
	FILE *output_file, *end_point_file;
	output_file = fopen(file_name, "w"); 
	end_point_file=fopen("end.csv","w");
	fprintf(output_file, "%c\n", alphabet);
	for(i=0;i<augmented_curve_A.len_curve;i++)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", augmented_curve_A.points[i][0],augmented_curve_A.points[i][1],augmented_curve_A.points[i][2],augmented_curve_A.points[i][4]*364173);

	}	
	//
	for(i=0;i<augmented_SLS.len_curve;i++)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", augmented_SLS.points[i][0],augmented_SLS.points[i][1],augmented_SLS.points[i][2],augmented_SLS.points[i][4]*364173);

	}
	fprintf(end_point_file, "%f,%f,%f,%f\n", augmented_SLS.points[0][0],augmented_SLS.points[0][1],augmented_SLS.points[0][2],augmented_SLS.points[0][4]*364173);	
	//
	for(i=1;i<augmented_curve_B.len_curve;i++)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", augmented_curve_B.points[i][0],augmented_curve_B.points[i][1],augmented_curve_B.points[i][2],augmented_curve_B.points[i][4]*364173);

	}	
	fprintf(end_point_file, "%f,%f,%f,%f\n", augmented_curve_B.points[0][0],augmented_curve_B.points[0][1],augmented_curve_B.points[0][2],augmented_curve_B.points[0][4]*364173);	
	fprintf(end_point_file, "%f,%f,%f,%f\n", augmented_curve_B.points[augmented_curve_B.len_curve-1][0],augmented_curve_B.points[augmented_curve_B.len_curve-1][1],augmented_curve_B.points[augmented_curve_B.len_curve-1][2],augmented_curve_B.points[augmented_curve_B.len_curve-1][4]*364173);	
	//
	if(augmented_spiral.spiral)
	{

		for(i=0;i<augmented_spiral.len_curve;i++)
		{
			fprintf(output_file, "%f,%f,%f,%f\n", augmented_spiral.points[i][0],augmented_spiral.points[i][1],augmented_spiral.points[i][2],augmented_spiral.points[i][4]*364173);

		}	
	fprintf(end_point_file, "%f,%f,%f,%f\n", augmented_spiral.points[augmented_spiral.len_curve-1][0],augmented_spiral.points[augmented_spiral.len_curve-1][1],augmented_spiral.points[augmented_spiral.len_curve-1][2],augmented_spiral.points[augmented_spiral.len_curve-1][4]*364173);	
	}
	//
	if(augmented_extended.extended)
	{
		for(i=0;i<augmented_extended.len_curve;i++)
		{
			fprintf(output_file, "%f,%f,%f,%f\n", augmented_extended.points[i][0],augmented_extended.points[i][1],augmented_extended.points[i][2],augmented_extended.points[i][4]*364173);

		}	
	}

	fclose(end_point_file);
	fclose(output_file);
}
//========================END OF WIND MODELLING FUNCTIONS=================================================
//Mathematical function that calculates new altitude
double heightS(double last_height, double distance, double Rg_straight)
{
        double heigt= last_height - (distance/Rg_straight);
        return heigt;  
}
//Mathematical function that calculates new altitude for banked turns
double heightBC(double last_height, double distance, int angle, double Rg_straight)
{	
	double Rg;
	if(angle==20)
		Rg=Rg_straight*cos(20*PI/180);
	if(angle==30)
		Rg=Rg_straight*cos(30*PI/180);
	if(angle==45)
		Rg=Rg_straight*cos(45*PI/180);
        double heigt= last_height - (distance/Rg);
        return heigt;  

}

//Assigns altitude at each point of DP
Seg assign_altitude(Seg parts, double last_height, double last_x, double last_y, int angle, double Rg_straight)
{
	int i;
	double current_height,current_x,current_y; //contains new altitude and last point after each assignment
        current_height=last_height;
        current_x=last_x;
        current_y=last_y;  

        //Altitudes for C1
	for(i=0; i<parts.lenc1;i++) 
	{
        	double distance= horizontal(current_x, current_y,parts.C1[i][0],parts.C1[i][1]);
		parts.C1[i][4]=heightBC(current_height, distance, angle, Rg_straight);
                current_height= parts.C1[i][4]; //updating current
		current_x=parts.C1[i][0];
		current_y=parts.C1[i][1];                   
	}
	
	//Altitudes for SLS
	for(i=0; i<parts.lensls;i++)
	{
        	double distance= horizontal(current_x, current_y,parts.SLS[i][0],parts.SLS[i][1]);
		parts.SLS[i][4]=heightS(current_height, distance, Rg_straight);
                current_height= parts.SLS[i][4]; //updating current
		current_x=parts.SLS[i][0];
		current_y=parts.SLS[i][1];	
	}
	
        //Altitudes for C2
	for(i=0; i<parts.lenc2;i++) 
	{
        	double distance= horizontal(current_x, current_y,parts.C2[i][0],parts.C2[i][1]);
		parts.C2[i][4]=heightBC(current_height, distance, angle, Rg_straight);
                current_height= parts.C2[i][4]; //updating current
		current_x=parts.C2[i][0];
		current_y=parts.C2[i][1];
	}
        return parts;                          
}
//new print function
void print_trajectory(Seg path, int angle, double rnwy_x, double rnwy_y, double rnwy_heading)
{
	int i,n;
	char file_name[20];
	sprintf(file_name, "%d.csv",angle); //generating new file name		
        FILE *output_file, *end_point_file;
    	output_file = fopen(file_name, "w");
	end_point_file=fopen("ends.csv","w"); 
	for(i=0;i<path.lenc1;i++)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", path.C1[i][0],path.C1[i][1],path.C1[i][2],(path.C1[i][4]*364173));
	}	

	for(i=0;i<path.lensls;i++)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", path.SLS[i][0],path.SLS[i][1],path.SLS[i][2],(path.SLS[i][4]*364173));
	}
	fprintf(end_point_file, "%f,%f,%f,%f\n", path.SLS[0][0],path.SLS[0][1],path.SLS[0][2],(path.SLS[0][4]*364173));
	for(i=0;i<path.lenc2;i++)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", path.C2[i][0],path.C2[i][1],path.C2[i][2],(path.C2[i][4]*364173));
	}
	fprintf(end_point_file, "%f,%f,%f,%f\n", path.C2[0][0],path.C2[0][1],path.C2[0][2],(path.C2[0][4]*364173));
	fprintf(end_point_file, "%f,%f,%f,%f\n", path.C2[path.lenc2-1][0],path.C2[path.lenc2-1][1],path.C2[path.lenc2-1][2],(path.C2[path.lenc2-1][4]*364173));
	if(path.lenspiral>0)
	{
	fprintf(end_point_file, "%f,%f,%f,%f\n", path.Spiral[path.lenspiral-1][0],path.Spiral[path.lenspiral-1][1],path.Spiral[path.lenspiral-1][2],(path.Spiral[path.lenspiral-1][4]*364173));

		for(i=0;i<path.lenspiral;i++)
		{
			fprintf(output_file, "%f,%f,%f,%f\n", path.Spiral[i][0],path.Spiral[i][1],path.Spiral[i][2],(path.Spiral[i][4]*364173));
		}
	}
	if(path.extended)
	{
		fprintf(output_file, "%f,%f,%f,%f\n", rnwy_x,rnwy_y,rnwy_heading,0.0000);
	}

	fclose(end_point_file);
	fclose(output_file);
}


//generates circles for spiral
Seg generate_spiral(Seg path, double radius, int angle, double Rg_straight)
{
	path.lenspiral=0;// initializing
	//printf("initialized\n");

	double step= 2*PI/50; //50 points per spiral circle
	//get starting theta from last point of dubins
	double last_x= path.C2[path.lenc2-1][0];
	double last_y= path.C2[path.lenc2-1][1];
	double last_altitude= path.C2[path.lenc2-1][4];
	Pair centre= find_centre(path.C2[path.lenc2-1][0], path.C2[path.lenc2-1][1], path.C2[path.lenc2-2][0], path.C2[path.lenc2-2][1], path.C2[path.lenc2-3][0], path.C2[path.lenc2-3][1]);//centre of turn

	double start_theta= acos((last_x-centre.x)/radius); //angle of starting point wrt centre 

	//generate circles
	int i, integral_turns=0, last_integral_i=0;
	step=-step; //for clockwise turn, make positive for anti clockwise turns	
	
	for(i=0;i<300;i++)
	{
		path.Spiral[i][0]=centre.x+(radius*cos((i*step)-start_theta)); //check - start or + start
		path.Spiral[i][1]=centre.y+(radius*sin((i*step)-start_theta));	
		path.Spiral[i][2]=path.C2[path.lenc2-1][2]+fabs(i*step); //MIGHT NEED TO CHANGE> ROUGH CALCULATION 

		//assign altitude
		double distance= horizontal(last_x,last_y,path.Spiral[i][0],path.Spiral[i][1]);
		path.Spiral[i][4]=heightBC(last_altitude, distance, angle, Rg_straight);

		//DELETING NOT INTEGRAL PART		
		if(path.Spiral[i][4]<=0)
		{
			path.lenspiral=last_integral_i;
			break;
		}				

		if((i) % 51 == 0) //integral turns
		{

			last_integral_i=i;
		}		

		last_x=path.Spiral[i][0];
		last_y=path.Spiral[i][1];
		last_altitude=path.Spiral[i][4];
		path.lenspiral=path.lenspiral+1;
	}
	path.spiral_centre= centre;
	path.spiral_start_angle= start_theta;
		
	return path;
}

//Function to find extended runway segment //MIGHT NEED FIXING
Seg find_extended_runway(Seg path, double rnwy_x, double rnwy_y, double rnwy_heading , double init_x, double init_y, double init_heading, double start_altitude, int angle, double min_radius, double interval, double Rg_straight, double Rg_dirty)
{
	double end_altitude;
	if(path.lenspiral>0)
	{
		 end_altitude=path.Spiral[path.lenspiral-1][4];
	}
	else
	{
		 end_altitude=path.C2[path.lenc2-1][4];
	}
	double q2[]={rnwy_x,rnwy_y,rnwy_heading};	
	double q1[]={init_x,init_y,init_heading};
	if(end_altitude>0.000137) //50 feet
	{
		double current_x=rnwy_x;
		double current_y=rnwy_y;
		double heading=rnwy_heading;
		double heading_opp= heading+PI;
		while(end_altitude>0.000137) //more than 50 feet
		{

	//finding new point at d feet away (using d=50 to find discrete points 50 feet apart)
			Pair new_point=along_heading(current_x, current_y, heading_opp);
			double q3[]={new_point.x,new_point.y,heading};
	//loss of altitude in SL from new to runway
			double distance=horizontal(new_point.x, new_point.y, rnwy_x, rnwy_y);
			double loss=(distance/Rg_dirty);
	//create a path for that point check if end_alti-loss in straight line distance D <50 and >-10		
			Result dubins=demo(q1,q3,min_radius, interval); //sending configs to demo to generate DP
			Seg dubin_parts=split(dubins);//sending DP to split into segments
			//return dubin_parts;
			dubin_parts=assign_altitude(dubin_parts, start_altitude, init_x, init_y, angle, Rg_straight);//Send parts to assign_altitude() to get alti for each point
			Seg path_with_spiral= generate_spiral(dubin_parts,min_radius,angle, Rg_straight);

			int flag=0;
			if(path_with_spiral.lenspiral>0)
			{	flag=1;
				end_altitude= path_with_spiral.Spiral[path_with_spiral.lenspiral-1][4];				
			}
			else 
			{
				end_altitude= path_with_spiral.C2[path_with_spiral.lenc2-1][4];				
			}

			if((end_altitude-loss)<0.000137)
			{
				path_with_spiral.extended=true;
				return path_with_spiral;
			}
			else
			{
				current_x=new_point.x;
				current_y=new_point.y;
			} 	
		}//end of while
	}//end of if
	else
	{
		return path;
	}
}
