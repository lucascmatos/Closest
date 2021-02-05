#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h> //lucasmatos

#include <math.h>
#define PI 3.14159265 //lucasmatos

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_close_point_;
ros::Publisher pub_close_point_marker_;
ros::Publisher pub_collision_status_; //lucasmatos

void callback(const PointCloud::ConstPtr &msg)
{
	//printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
	//------------lucasmatos------------BEGIN
	double distance_limit;
	ros::param::get("/distance_limit", distance_limit);
	double angle_limit;
	ros::param::get("/angle_limit", angle_limit);
	double distance_yellow;
	ros::param::get("distance_yellow", distance_yellow);
	double distance_red;
	ros::param::get("distance_red", distance_red);
	//This Variable is used to improve the obstacle detection verifying if the neighboring points are classified as collision points
	int surface_points_limit;
	ros::param::get("surface_points_limit", surface_points_limit);

	double d2D1 = 0;
	double d2D2 = 0;
	double height1 = 0;
	double height2 = 0;
	//double difx[2] = {0, 0};
	//double dify[2] = {0, 0};
	//double difz[2] = {0, 0};
	//int count_dif = 0;
	//int count_pair = 0;
	double diference = 0;
	double angle = 0;
	//Values to control de LED
	int collision = 0;
	int yellow = 0;
	int red = 0;
	int row = msg->height;
	int count_row = 0;
	int controlevel = 0;
	int control = 0;
	double D_square;
	int old_row = 0;
	int new_row = 0;
	int collision_surface = 0;
	//This variable makes sure that we don't compare row's too far away from each other	
	int line = 0;
	//------------lucasmatos------------END
	static float px_filt = 0.0;
	static float py_filt = 0.0;
	static float pz_filt = 0.0;
	static float alpha = 0.3;
	static float signal = -1;

	int count = 0;
	float D_square_min = 1000 * 1000;
	float p_min[3] = {0, 0, 0};
	BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
	{
		count++;
		if (controlevel <= row)
		{			
			//std::cout << " COunt row :" << count_row << "\n";
			count_row++;
			if (pt.x != 0 and pt.y != 0 and pt.z != 0)
			{
				//float D_square = (pt.x - 0.2 * signal) * (pt.x - 0.2 * signal) + pt.y * pt.y + pt.z * pt.z; //no need to take the square root
				//if (D_square < 0.2 + 0.1)
				//{
				//	D_square = 10000.0;
				//}
				// if(D_square > 0.3){
				//-------------lucasmatos-----------BEGIN
				if (control = 0)
				{
					d2D1 = (pt.x - 0.2 * signal) * (pt.x - 0.2 * signal) + pt.y * pt.y;
					height1 = pt.z;
					control = 1;
				}
				d2D2 = (pt.x - 0.2 * signal) * (pt.x - 0.2 * signal) + pt.y * pt.y;
				height2 = pt.z;
				diference = sqrt((d2D2 - d2D1) * (d2D2 - d2D1));
				if (diference > distance_limit)
				{
					//std::cout << " Diference  =" << diference << "\n";
					diference = 0;
					angle = atan2(sqrt((height2 - height1) * (height2 - height1)), sqrt((d2D2 - d2D1) * (d2D2 - d2D1))) * 180 / PI;
					d2D1 = d2D2;
					height1 = height2;
					d2D2 = 0;
					height2 = 0;
					//Compare the same point in two diferents row's to check for collision
					if (angle > angle_limit)
					{
						line++;
						if (old_row == 0)
						{
							old_row = count_row;
						}
						if (old_row != 0)
						{
							new_row = count_row;
						}
						//
						//Every time that a collision poiint is detected, the variable collision_surface add 1 to itself
						//
						if (old_row == new_row)
						{
							collision_surface++;
							old_row = 0;
							new_row = 0;							
							//std::cout<<"Collision surface ="<<collision_surface<<"\n";
						}
						//
						//If collision surface its equal a surface points limit, the point is classified as a collision point
						//and the point is published as closest point
						//
						if (collision_surface == surface_points_limit and line<7)
						{							
							printf("Line : %d\n",line);
							line = 0;
                            collision_surface =0;
							D_square = (pt.x - 0.2 * signal) * (pt.x - 0.2 * signal) + pt.y * pt.y + pt.z * pt.z;
							if (D_square < 0.2 + 0.1)
							{
								D_square = 10000.0;
							}
							//std::cout << angle << "\n";
							if (D_square != 0)
							{
								if (D_square < D_square_min)
								{
									//Print for debug
									//std::cout << "Angulo " << angle << " Distancia " << D_square << "\n";
									D_square_min = D_square;
									p_min[0] = pt.x;
									p_min[1] = pt.y;
									p_min[2] = pt.z;
								}
								angle = 0;
							}
							if (sqrt(D_square) < distance_yellow)
							{
								//printf("Distancia amarelo %f \n",sqrt(D_square));
								yellow = 1;
							}
							if (sqrt(D_square) < distance_red)
							{
								red = 1;
							}
							//points_pair = points_pair + 1;
						}
						else
						{
							line = 0;
						}
						
					}

					//If the surface angle is above the limite save the D_min value

					//Print for debug the angle
					//printf(" Angle %f \n",angle);
				}
				else if (count_row == row)
				{
					controlevel++;
					d2D2 = 0;
					d2D1 = 0;
					height2 = 0;
					height1 = 0;
					diference = 0;
					control = 0;
					count_row = 0;
				}
				else
				{
					controlevel = 0;
				}				
				//Print for debug
				//printf(" difx  %f \n ",difx[0]);
				//printf("%f \n", difx[1]);
				//printf(" dify  %f \n ",dify[0]);
				//printf("%f \n", dify[1]);
			}
		}

		// }

		//-------------lucasmatos-----------END

		//if(D_square<D_square_min){
		//	if(pt.z>-0.2){
		//		D_square_min = D_square;
		//		p_min[0] = pt.x;
		//		p_min[1] = pt.y;
		//		p_min[2] = pt.z;
		//	}
		//}
		/*if(count % 10000 == 0){
			printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		}*/
		//printf("%f\n",D_square_min);
	}

	//-----------lucasmatos-------------BEGIN
	//Change the color of LED according to the danger
	if (yellow == 1 and red == 0)
	{
		collision = 1;
	}
	if (red == 1)
	{
		collision = 2;
	}
	if (yellow == 0 and red == 0)
	{
		collision = 0;
	}
	//printf("Collision %i \n",collision);

	std_msgs::Int8 collision_status;
	collision_status.data = collision;
	pub_collision_status_.publish(collision_status);

	//-----------lucasmatos-----------END
	//printf("p_close = [%.4f, %.4f, %.4f,]\n", p_min[0], p_min[1], p_min[2]);
	//std::cout << "p_min = [" << p_min[0] << ", " << p_min[1] << ", " << p_min[2] << "]" << '\n';
	int flag = 1;
	if (p_min[0] * p_min[0] + p_min[1] * p_min[1] + p_min[2] * p_min[2] < 0.3 * 0.3)
	{
		flag = 0;
		//std::cout << "meleca"
		//		  << "\n";
	}
	//else
	//{
	//std::cout << "meleca_fail"
	//		  << "\n";
	//}
	//if ((collision == 2 || pub==1) && flag==1)
	if (flag == 1)
	{
		//std::cout << "meleca2"
		//		  << "\n";
		px_filt = (1 - alpha) * px_filt + alpha * (signal * p_min[0]);
		py_filt = (1 - alpha) * py_filt + alpha * (signal * p_min[1]);
		pz_filt = (1 - alpha) * pz_filt + alpha * (p_min[2]);
		//Publish point with minimum distance
		geometry_msgs::Point close_point;
		close_point.x = px_filt;
		close_point.y = py_filt;
		close_point.z = pz_filt;
		pub_close_point_.publish(close_point);

		//Publish a marker with the closest point
		visualization_msgs::Marker close_point_marker;
		close_point_marker.header.frame_id = "/os1_sensor";
		close_point_marker.header.stamp = ros::Time::now();
		close_point_marker.ns = "basic_shapes";
		close_point_marker.type = visualization_msgs::Marker::SPHERE;
		if (px_filt != 0 and py_filt != 0 and pz_filt != 0)
		{
			close_point_marker.pose.position.x = px_filt;
		}
		close_point_marker.pose.position.y = py_filt;
		close_point_marker.pose.position.z = pz_filt;
		close_point_marker.scale.x = 0.12;
		close_point_marker.scale.y = 0.12;
		close_point_marker.scale.z = 0.12;
		close_point_marker.pose.orientation.x = 0.0;
		close_point_marker.pose.orientation.y = 0.0;
		close_point_marker.pose.orientation.z = 0.0;
		close_point_marker.pose.orientation.w = 1.0;
		close_point_marker.color.r = 1.0;
		close_point_marker.color.g = 1.0;
		close_point_marker.color.b = 1.0;
		close_point_marker.color.a = 1.0;
		close_point_marker.lifetime = ros::Duration(0.1);
		pub_close_point_marker_.publish(close_point_marker);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "closest_node");

	ros::NodeHandle nh;
	pub_close_point_ = nh.advertise<geometry_msgs::Point>("closest_point", 1);
	pub_close_point_marker_ = nh.advertise<visualization_msgs::Marker>("closest_point_marker", 1);
	//Collsision status by lucasmatos
	pub_collision_status_ = nh.advertise<std_msgs::Int8>("collision_status", 1);

	ros::Subscriber sub = nh.subscribe<PointCloud>("/os1_cloud_node/points", 1, callback); //Subscribe to filter topic /os1_cloud_node/points
	ros::spin();
}
