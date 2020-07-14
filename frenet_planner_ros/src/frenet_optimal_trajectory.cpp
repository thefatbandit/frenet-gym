#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include <sstream>

namespace plt = matplotlibcpp;
static int flag_for_display_paths = 0;

vecD FrenetPath::get_d_ddd()
{
	return d_ddd;
}
double FrenetPath::get_cf()
{
	return cf;
}
vecD FrenetPath::get_d()
{
	return d;
}

vecD FrenetPath::get_yaw()
{
	return yaw;
}

void FrenetPath::calc_lat_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, double Ti, double di, double di_d)
{
	quintic lat_qp(c_d, c_d_d, c_d_dd, di, di_d, 0.0, Ti); 
	for(double te = 0.0; te <= Ti + DT; te += DT)
	{
		t.push_back(te);
		d.push_back(lat_qp.calc_point(te));
		d_d.push_back(lat_qp.calc_first_derivative(te));
		d_dd.push_back(lat_qp.calc_second_derivative(te));
		d_ddd.push_back(lat_qp.calc_third_derivative(te));
	}
}

void FrenetPath::calc_lon_paths(double c_speed, double s0, double Ti, FrenetPath &fp, double tv, double Jp)
{
	// Normal Implementation
	quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);	//s_dd is set to const. 0 (i.e. not being sampled) 
	for(auto const& te : t) 
	{
		s.push_back(lon_qp.calc_point(te));
		s_d.push_back(lon_qp.calc_first_derivative(te));
		s_dd.push_back(lon_qp.calc_second_derivative(te));
		s_ddd.push_back(lon_qp.calc_third_derivative(te));
	}

	//https://www.geeksforgeeks.org/std-inner_product-in-cpp/
	double Js = inner_product(s_ddd.begin(), s_ddd.end(), s_ddd.begin(), 0);
	double ds = pow((TARGET_SPEED - s_d.back()), 2);

	cd = (KJ*Jp + KT*Ti + KD*d.back()*d.back());
	// cout<<"Jp : "<<Jp<<" Ti : "<<Ti<<" d^2 : "<<d.back()*d.back()<<endl;
	// cout<<"Js : "<<Js<<" Ti : "<<Ti<<" Speed around target speed : "<<ds<<endl;
	cv = (KJ*Js + KT*Ti + KD_V*ds);
	cf = (KLAT*cd + KLON*cv);
}

void FrenetPath::calc_lon_paths_quintic_poly(double c_speed, double s0, double Ti, FrenetPath &fp, double ts, double tv, double Jp)
{
	quintic lon_qp(s0, c_speed, 0.0, s0 + ts, tv, 0.0, Ti);	// s_dd is not being sampled
	for(auto const& te : t) 
	{
		s.push_back(lon_qp.calc_point(te));
		// cout<<"lon_qp.calc_point(te) : "<<lon_qp.calc_point(te)<<endl;
		s_d.push_back(lon_qp.calc_first_derivative(te));
		s_dd.push_back(lon_qp.calc_second_derivative(te));
		s_ddd.push_back(lon_qp.calc_third_derivative(te));
	}
	//https://www.geeksforgeeks.org/std-inner_product-in-cpp/
	double Js = inner_product(s_ddd.begin(), s_ddd.end(), s_ddd.begin(), 0);
	double ds = pow((TARGET_SPEED - s_d.back()), 2);

	cd = (KJ*Jp + KT*Ti + KD*d.back()*d.back());
	cv = (KJ*Js + KT*Ti + KD_V*ds);
	cf = (KLAT*cd + KLON*cv);
}

void get_limits_d(FrenetPath lp, double *lower_limit_d, double *upper_limit_d)
{
	vecD d_sampling = lp.get_d();
	if(d_sampling.size() == 0)
	{
		*lower_limit_d = -MAX_ROAD_WIDTH;
		*upper_limit_d = MAX_ROAD_WIDTH + D_ROAD_W;
	}
	else
	{
		*lower_limit_d = d_sampling.back() - MAX_SHIFT_D;
		*upper_limit_d = d_sampling.back() + MAX_SHIFT_D + D_ROAD_W;
	}
}

FrenetPath calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, FrenetPath lp)
{


	double lower_limit_d, upper_limit_d;
	lower_limit_d = -MAX_ROAD_WIDTH;
	upper_limit_d = MAX_ROAD_WIDTH + D_ROAD_W;
	get_limits_d(lp, &lower_limit_d, &upper_limit_d);//IF not required to sample around previous sampled d then comment this line.

	FrenetPath fp;
	/*
	Parameters to be subscribed:
		di, Ti, di_d, tv(target_vel) ,s(optional)
	*/
	double tv = twist.linear.x;
	double Ti = twist.linear.y;
	double di = twist.linear.z;
	double di_d = twist.angular.x;
	//s0 = action.angular.y;

	fp.calc_lat_paths(c_speed, c_d, c_d_d, c_d_dd, s0, Ti, di, di_d);
	vecD d_ddd_vec = fp.get_d_ddd();
	double Jp = inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0);
	fp.calc_lon_paths(c_speed, s0, Ti, fp, tv, Jp);
	return fp;
}

void FrenetPath::adding_global_path(Spline2D csp)
{
	for(int i = 0; i < s.size(); i++)
	{
		double ix, iy;
		csp.calc_position(&ix, &iy, s[i]);

		if(ix == NONE)
		{
			break;
		}
		double iyaw = csp.calc_yaw(s[i]);
		double di = d[i];
		double fx = ix - di*sin(iyaw);
		double fy = iy + di*cos(iyaw);
		x.push_back(fx);
		y.push_back(fy);
	}
	for(int i = 0; i < x.size() - 1; i++)
	{
		double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];
		yaw.push_back(atan2(dy, dx));
		ds.push_back(sqrt(dx*dx + dy*dy));
	}
	if(s.size() - x.size() != 0)//TO remove paths whose predicted s goes out of bounds of global path.
	{
		return;
	}
	for(int i = 0; i < yaw.size() - 1; i++){
		c.push_back((yaw[i + 1] - yaw[i]) / ds[i]);
	}
}

// convert to global frame 
FrenetPath calc_global_paths(FrenetPath fp, Spline2D csp)
{
	fp.adding_global_path(csp);	
	return fp;
}	

vector<geometry_msgs::Point32> transformation(vector<geometry_msgs::Point32> fp, geometry_msgs::Pose cp, double px, double py, double pyaw)
{
	vector<geometry_msgs::Point32> new_fp(fp.size());
	tf::Quaternion qb(cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w);
	tf::Matrix3x3 mb(qb);

	double broll, bpitch, byaw;
	mb.getRPY(broll, bpitch, byaw);

	double bx, by;
	bx = cp.position.x;
	by = cp.position.y;

	double x, y, theta;
	theta = pyaw - byaw;
	x = px - bx;
	y = py - by;
	
	for(int i = 0; i < new_fp.size(); i++)
	{
		new_fp[i].x = (fp[i].x - bx)* cos(theta) + (fp[i].y - by) * sin(theta) + x + bx;
		new_fp[i].y = -(fp[i].x - bx) * sin(theta) + (fp[i].y - by) * cos(theta) + y + by;
	}
	return new_fp;
}


double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

bool point_obcheck(geometry_msgs::Point32 p, double obst_r)
{
	int xlower, ylower, xupper, yupper;
	auto it = lower_bound(ob_x.begin(), ob_x.end(), p.x);
	if (ob_x.size() == 0)
		return 0;
	if (it == ob_x.begin()) 
		xlower = xupper = it - ob_x.begin();//no smaller value  than val in vector
	else if (it == ob_x.end()) 
		xupper = xlower = (it-1)- ob_x.begin();//no bigger value than val in vector
	else
	{
    	xlower = (it-1) - ob_x.begin();
    	xupper = it - ob_x.begin();
	}
	double dist1 = dist(p.x,p.y, ob_x[xlower], ob_y[xlower]);
	double dist2 = dist(p.x, p.y, ob_x[xupper], ob_y[xupper]);
	
	if(min(dist1, dist2) < obst_r)
		return 1;
	it = lower_bound(ob_y.begin(), ob_y.end(), p.y);
	if (it == ob_y.begin()) 
		ylower = yupper = it - ob_y.begin();//no smaller value  than val in vector
	else if (it == ob_y.end()) 
		yupper = ylower = (it-1)- ob_y.begin();//no bigger value than val in vector
	else
	{
		ylower = (it-1) - ob_y.begin();
		yupper = it - ob_y.begin();
	}
	dist1 = dist(p.x,p.y, ob_x[ylower], ob_y[ylower]);
	dist2 = dist(p.x, p.y, ob_x[yupper], ob_y[yupper]);
	if(min(dist1, dist2) < obst_r)
		return 1;
	return 0;	 
}

// check for specified velocity, acceleration and curvature constraints
int check_path(FrenetPath fp, double bot_yaw, double yaw_error, double obst_r)
{
	vecD path_yaw = fp.get_yaw();					//BOT CHECKING YAW. PLEASE ADD BACK
	if (abs(path_yaw[0] - bot_yaw)> yaw_error) //15 deg
	{
		cout << "Yaw out of range" << endl;
		return 1;
	}
	return 0;
}

void FrenetPath::plot_path()
{
	plt::plot(x,y);
	plt::pause(0.001);
}
void FrenetPath::plot_velocity_profile()
{
	plt::plot(t, s_d);
	plt::pause(0.001);
}

void display_paths(vector<FrenetPath> fplist)
{
	plt::ion();
	plt::show();
	int count=0;
	for(auto &fp : fplist)
	{
		if(count%50 == 0 && flag_for_display_paths){
			cout<<"!!"<<endl;
			fp.plot_path();
		}	
		count++;
	}
	flag_for_display_paths = 1;
}

// generates the path and returns the bestpath
FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, FrenetPath lp, double bot_yaw)
{
	FrenetPath fp = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, lp);
	fp = calc_global_paths(fp, csp);
	int yaw_flag = check_path(fp, bot_yaw, 0.261799, 4.0);
	// For showing the bestpath
	double cf;
	if(false)
	{
		plt::ion();
		plt::show();
		fp.plot_path();
	}

	if(false)
	{
		plt::ion();
		plt::show();
		fp.plot_velocity_profile();
	}
	cf = fp.get_cf();
	return fp;
}