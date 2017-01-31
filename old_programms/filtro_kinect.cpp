#include <ros/ros.h>
#include <ros/package.h>
#include <kinect/kinectbody.h>
#include <math.h>
#include <stdlib.h>
#include <cmath>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <deque>

using namespace std;

//################CLASS#################################
class KinectPoints
{
    public:
    KinectPoints()
    : msg_(new kinect::kinectbody)
    {
        std::cout << msg_->pose.size() << std::endl;
    }
    void readKinectPoints(const kinect::kinectbody::ConstPtr& msg)
    {
        msg_ = msg;
    }
    kinect::kinectbody::ConstPtr getPoints()
    {
        return msg_;
    }
    private:
    kinect::kinectbody::ConstPtr msg_;
}
;
 
int main(int argc, char **argv)
{
    //std::deque<int> container;
    //############################################
    //NODE
    //############################################
    ros::init(argc, argv, "n_filterkinect");
    ros::NodeHandle nh;
    KinectPoints kpoints;
    //############################################
    //Suscriber
    //############################################
    ros::Subscriber sub_1 = nh.subscribe("kinect_data", 1000, &KinectPoints::readKinectPoints, &kpoints);
    ros::Rate iter_rate(1000); // Hz
    unsigned int niter=0, max_iter = 1e3;
    ros::spinOnce();
    iter_rate.sleep();
    std::vector< std::deque<double> > container;
    std::vector< std::vector<double> > P_position;
    std::vector< std::vector<double> > Q_orientation;
    std::vector<double> suma;
    //Here put how much point we need to employ
    suma.resize(36,0);
    container.resize(36);
    P_position.resize(6);
    Q_orientation.resize(6);
    //############################################
    //INICIO DEL PROCESO DE ENVIO
    //############################################
    ros::Publisher pub = nh.advertise<kinect::kinectbody>("kinect_data_filtrada", 1000);
    kinect::kinectbody pdata;
    pub.publish(pdata);
    pdata.pose.resize(6);
    //############################################
    // Tasks and Inverse Kinematics Solver
    //############################################
    unsigned int f = 200;
    Eigen::VectorXd qdes;
    ros::Rate rate(f); // Hz
    //############################################
    //ROS
    //############################################
    int filter=0;
    int window=5;
     
    for(int q=0;q<36;q++){
    	container[q].resize(window,1);
	}
    while(ros::ok())
    {	
        if (kpoints.getPoints()->pose.size() > 0)
        {
	        std::cout<<"Working"<<endl;
	        for (int k=0;k<6;k++){
		        Q_orientation[k].resize(3);
		        tf::Quaternion q(kpoints.getPoints()->pose[k].orientation.x, kpoints.getPoints()->pose[k].orientation.y, kpoints.getPoints()->pose[k].orientation.z, kpoints.getPoints()->pose[k].orientation.w);
		        tf::Matrix3x3 m(q);
		        double roll, pitch, yaw;
		        m.getRPY(roll, pitch, yaw);
		        Q_orientation[k][0]=roll;
		        Q_orientation[k][1]=pitch;
		        Q_orientation[k][2]=yaw;
		    }
		    
		    ///////////////////////////////////////////////////////
		    
		    for(int k=0;k<6;k++)
		    {
			    container[6*k+0].push_back(kpoints.getPoints()->pose[k].position.x);
			    container[6*k+1].push_back(kpoints.getPoints()->pose[k].position.y);
			    container[6*k+2].push_back(kpoints.getPoints()->pose[k].position.z);
			    container[6*k+3].push_back(Q_orientation[k][0]);
			    container[6*k+4].push_back(Q_orientation[k][1]);
			    container[6*k+5].push_back(Q_orientation[k][2]);
		    }
		    for(int i=0;i<36;i++)
			{
				container[i].pop_front();
				//std::cout<<container[i][99]<<endl;
			}
			std::cout<<"inicio"<<endl;
			for(int i=0;i<window;i++)
			{
				std::cout<<container[1][i]<<endl;
			}
			std::cout<<"size: "<<container[1].size()<<endl;
			std::cout<<"fin"<<endl;
			///////////////////////////////////////////////////////
			if(filter<window){
				for(int k=0; k<36; k++)
				{
					for (int q=0; q<window;q++)
					{
						suma[k]=suma[k]+(container[k][q])/window;
						//cout<<"valor"<<suma[k]<<endl;
					}
				}

	        }
	        std::cout<<"filter "<<filter<<endl;
	        ///////////////////////////////////////////////////////
	        if (filter>window){
		        std::cout<<"Sumatory"<<endl;
		        for(int k=0; k<36; k++){
					suma[k]=suma[k]-(container[k].front()-container[k].back())/window;
					std::cout<<suma[k]<<endl;
				}
				//std::cout<<"working_4"<<endl;
				for(int k=0; k<6; k++){
		        pdata.pose[k].position.x=suma[6*k+0];
		        pdata.pose[k].position.y=suma[6*k+1];
		        pdata.pose[k].position.z=suma[6*k+2];
		        pdata.pose[k].orientation.x=suma[6*k+3];
		        pdata.pose[k].orientation.y=suma[6*k+4];
		        pdata.pose[k].orientation.z=suma[6*k+5];
		        pdata.pose[k].orientation.w=0;
		        }
		        pub.publish(pdata);
		    }
	       
	        filter++;
	      
    	}
    	ros::spinOnce();
		rate.sleep(); 
    }
    return 0;
}
