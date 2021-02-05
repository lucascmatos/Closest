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
	int threshold;
	ros::param::get("/threshold", threshold);
	int angle_limit;
	ros::param::get("/angle_limit", threshold);
	int paircollision_limit;
	ros::param::get("paircollision_limit", paircollision_limit);
	float distance_limit;
	ros::param::get("distance_limit", distance_limit);
	float distance_yellow;
	ros::param::get("distance_yellow", distance_yellow);
	float distance_red;
	ros::param::get("distance_red", distance_red);
	int pub;
	ros::param::get("pub", pub); // Flag that allow to always publish the closest point

	double d2pair[] = {0, 0};
	double heightpair[] = {0, 0};
	double difx[2] = {0, 0};
	double dify[2] = {0, 0};
	double difz[2] = {0, 0};
	int count_dif = 0;
	int count_pair = 0;
	double angle = 0;
	//Values to control de LED
	int collision = 0;
	int yellow = 0;
	int red = 0;
	int row = msg->height;
	int count_row = 0;
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
		if (count_row <= row)
		{
			count_row++;
			std::cout << pt.x << "\n";
		}
		else
		{
			std::cout<<"New row \n\n\n";
			count_row = 0;
		}
		if (pt.x != 0 and pt.y != 0 and pt.z != 0)
		{
			//float D_square = (pt.x - 0.2 * signal) * (pt.x - 0.2 * signal) + pt.y * pt.y + pt.z * pt.z; //no need to take the square root
			//if (D_square < 0.2 + 0.1)
			//{
			//	D_square = 10000.0;
			//}
			// if(D_square > 0.3){
			//-------------lucasmatos-----------BEGIN
			difx[count_dif] = (pt.x - 0.2 * signal);
			dify[count_dif] = pt.y;
			difz[count_dif] = pt.z;

			if (count_dif == 1)
			{
				if ((sqrt((difx[1] - difx[0]) * (difx[1] - difx[0])) < 0.2) && (sqrt((dify[1] - dify[0]) * (dify[1] - dify[0])) < 0.2))
				{
					//std::cout<<"Dif x :"<<sqrt((difx[1] - difx[0])*(difx[1] - difx[0]))
					//        <<" Dif y :"<<sqrt((dify[1] - dify[0])*(dify[1] - dify[0]))
					//        <<"\n";
					d2pair[0] = sqrt((difx[0]) * (difx[0]) + dify[0] * dify[0]);
					heightpair[0] = difz[0];
					d2pair[1] = sqrt((difx[1]) * (difx[1]) + dify[1] * dify[1]);
					heightpair[1] = difz[1];
					//Calculates the surface angle
					angle = atan2(sqrt((heightpair[0] - heightpair[1]) * (heightpair[0] - heightpair[1])), sqrt((d2pair[0] - d2pair[1]) * (d2pair[0] - d2pair[1]))) * 180 / PI;
					//std::cout<<angle<<"\n";
					if (angle > 75)
					{
						float D_square = difx[1] * difx[1] + dify[1] * dify[1] + difz[1] * difz[1];
						if (D_square < 0.2 + 0.1)
						{
							D_square = 10000.0;
						}
						//std::cout << angle << "\n";
						if (D_square != 0)
						{
							if (D_square < D_square_min)
							{
								if (angle > 75)
								{
									D_square_min = D_square;
									p_min[0] = difx[1];
									p_min[1] = dify[1];
									p_min[2] = difz[1];
								}
							}
							count_pair++;
							angle = 0;
						}
						if (count_pair > 3)
						{
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
						count_pair = 0;
					}

					//If the surface angle is above the limite save the D_min value

					//Print for debug the angle
					//printf(" Angle %f \n",angle);
				}
				count_dif = 0;
				difx[0] = difx[1];
				dify[0] = dify[1];
			}

			//Print for debug
			//printf(" difx  %f \n ",difx[0]);
			//printf("%f \n", difx[1]);
			//printf(" dify  %f \n ",dify[0]);
			//printf("%f \n", dify[1]);
			count_dif++; // Makes sure that we take two diferent points from the cloud to certify that we are on the same surface
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
	//Restart the values for the next rotation of the laser
	memset(difx, 0, sizeof(difx));
	memset(dify, 0, sizeof(dify));
	memset(d2pair, 0, sizeof(d2pair));
	memset(heightpair, 0, sizeof(heightpair));

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
		std::cout << "meleca"
				  << "\n";
	}
	else
	{
		std::cout << "meleca_fail"
				  << "\n";
	}
	//if ((collision == 2 || pub==1) && flag==1)
	if (flag == 1)
	{
		std::cout << "meleca2"
				  << "\n";
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
		close_point_marker.pose.position.x = px_filt;
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

	ros::Subscriber sub = nh.subscribe<PointCloud>("/os1_cloud_node/points", 1, callback); //Subscribe to filter topic
	ros::spin();
}
