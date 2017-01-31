// Control
// Executing
// ./baxter.sh sim
// roslaunch baxter_gazebo baxter_world.launch
// rosrun baxter_tools enable_robot.py -e
// rosrun baxter..AA
////////////////// Executing
// /home/jose/baxter-workspace/src/baxter_cpp/launch
// roslaunch baxter_cpp l_baxter_cpp.launch
// rosrun baxter_cpp OSIK_baxter
#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h> 
#include <robot-model/robot-model.hpp>
#include <kinect/kinectbody.h>
#include <tf2_msgs/TFMessage.h>
#include <osik-control/math-tools.hpp>		
#include <osik-control/kine-task.hpp>
#include <osik-control/kine-task-pose.hpp>
#include <osik-control/kine-solver-WQP.hpp>
#include <baxter_cpp/robotSpecifics.h>
#include <baxter_cpp/tools.hpp>
#include <math.h>
#include <stdlib.h>
#include <cmath>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <baxter_cpp/markers.hpp>
#include <tf/transform_datatypes.h>
#include <fstream> 
#include <unistd.h>
#include <tf/transform_listener.h>
//#include <bits/stdc++.h>
using namespace osik;
using namespace std;
//################CLASS#################################
int main(int argc, char **argv)
{
    //ofstream myfile;
  	//myfile.open ("example.txt");
    //ofstream cout("informacion_total.txt");
    //std::ofstream outfile ("data_fullxd.txt");
    //############################################
    //INFORMACION DEL BAXTER
    //############################################
    std::string baxter_description = ros::package::getPath("baxter_description");
    std::string model_name = baxter_description + "/urdf/baxter.urdf";
    RobotModel* robot = new RobotModel();
    bool has_floating_base = false;
    if (!robot->loadURDF(model_name, has_floating_base)){
        return -1;
    }
    else{
        std::cout << "Robot " << model_name << " loaded." << std::endl;
    }
    unsigned int ndof_full = robot->ndof();
    std::cout << "Reading initial sensor values ..." << ndof_full<< std::endl;
    // Get the joint names and joint limits
    std::vector<std::string> jnames;
    std::vector<double> qmin, qmax, dqmax;
    jnames = robot->jointNames();
    qmin = robot->jointMinAngularLimits();
    qmax = robot->jointMaxAngularLimits();
    dqmax = robot->jointVelocityLimits();
    //############################################
    //NODE
    //############################################
    ros::init(argc, argv, "n_sendbaxter");
    ros::NodeHandle nh;
	Eigen::VectorXd P_right_wrist;
    Eigen::VectorXd P_left_elbow;
    Eigen::VectorXd P_right_elbow;
    Eigen::VectorXd P_left_wrist;



    for(int i=0;i<jnames.size();i++){
        cout<<jnames[i]<<endl;
    }
    //############################################
    //Suscriber
    //############################################
    //ros::Subscriber sub_1 = nh.subscribe("kinect_data", 1000, &KinectPoints::readKinectPoints, &kpoints);
    //ros::Subscriber sub_1 = nh.subscribe("tf", 1000, &TfPoints::readTfPoints, &tfpoints);
    //JointSensors jsensor;
    //ros::Subscriber sub_2 = nh.subscribe("joint_states", 1000,&JointSensors::readJointSensors, &jsensor);
    //robot/joint_state ~ joint_state
    ros::Rate iter_rate(200); // Hz
    unsigned int niter=0, max_iter = 1e3;
    unsigned int ndof_sensed = 222;
    ros::spinOnce();
    iter_rate.sleep();
    Eigen::VectorXd qsensed = Eigen::VectorXd::Zero(ndof_full);
    Eigen::Vector3d w(0,0,0);
    Eigen::Vector3d v(0,0,0);
    Eigen::MatrixXd ball_position(3,100);
    Eigen::MatrixXd frame_pose(6,100);
    int link_number=
    {
        1
    };
    //Eigen::VectorXd qposition=Eigen::MatrixXd::Constant(15, 1, 0.0);
	std::vector< std::vector<double> > P_received;
    std::vector< std::vector<double> > P_send;
    std::vector< std::vector<double> > Q_orientation;
    //Q_orientation.resize(6);
    //############################################
    //INICIO DEL PROCESO DE ENVIO
    //############################################
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    sensor_msgs::JointState jcmd;
    jcmd.name.resize(19);
    jcmd.position.resize(19);
    jcmd.velocity.resize(19);
    jcmd.effort.resize(19);
    jcmd.name[0]="head_pan";
    jcmd.name[1]="left_s0";
    jcmd.name[2]="left_s1";
    jcmd.name[3]="left_e0";
    jcmd.name[4]="left_e1";
    jcmd.name[5]="left_w0";
    jcmd.name[6]="left_w1";
    jcmd.name[7]="left_w2";
    jcmd.name[8]="right_s0";
    jcmd.name[9]="right_s1";
    jcmd.name[10]="right_e0";
    jcmd.name[11]="right_e1";
    jcmd.name[12]="right_w0";
    jcmd.name[13]="right_w1";
    jcmd.name[14]="right_w2";
    jcmd.name[15]="l_gripper_l_finger_joint";
    jcmd.name[16]="l_gripper_r_finger_joint";
    jcmd.name[17]="r_gripper_l_finger_joint";
    jcmd.name[18]="r_gripper_r_finger_joint";
    for(int j=0; j<19;j++){
        jcmd.position[j]=0.0;
    }
    pub.publish(jcmd);
    //############################################
    // Tasks and Inverse Kinematics Solver
    //############################################
    unsigned int f = 300;
    double dt = static_cast<double>(1.0/f);
    KineSolverWQP solver(robot, qsensed, dt);
    solver.setJointLimits(qmin, qmax, dqmax);
    KineTask* taskrh = new KineTaskPose(robot, 14, "position");
    taskrh->setGain(100.0);
    KineTask* tasklh = new KineTaskPose(robot, 7, "position");
    tasklh->setGain(100.0);
    KineTask* taskre = new KineTaskPose(robot, 11, "position");
    taskre->setGain(100.0);
    KineTask* taskle = new KineTaskPose(robot, 4, "position");
    taskle->setGain(100.0);
    Eigen::VectorXd qdes ;
    ros::Rate rate(f); // Hz
    //############################################
    //PROCESS MARKER
    //############################################
    double red[3] = {
        1.,0.,0.
    }
    ;
    double green[3] = {
        0.,1.,0.
    }
    ;
    double blue[3] = {
        0.,0.,1.
    }
    ;
    double yellow[3] = {
        0.5,0.5,0.
    }
    ;

    double white[3] = {
        1,1,1
    }
    ;

    BallMarker* b_shoulder_left;
    BallMarker* b_shoulder_right;
    BallMarker* b_elbow_right;
    BallMarker* b_elbow_left;
    BallMarker* b_hand_right;
    BallMarker* b_hand_left;

    b_shoulder_left= new BallMarker(nh, white);
    b_shoulder_right= new BallMarker(nh, white);
    b_elbow_right= new BallMarker(nh, yellow);
   	b_elbow_left= new BallMarker(nh, green);
    b_hand_right= new BallMarker(nh, red);
    b_hand_left= new BallMarker(nh, blue);

    solver.pushTask(taskrh);
  	solver.pushTask(tasklh);
  	solver.pushTask(taskre);
  	solver.pushTask(taskle);

    //############################################
    //ROS
    //############################################
    P_received.resize(6);
    P_send.resize(6);

    tf::StampedTransform transform_right_elbow;
    tf::StampedTransform transform_left_elbow;
    tf::StampedTransform transform_right_shoulder;
    tf::StampedTransform transform_left_shoulder;
    tf::StampedTransform transform_right_hand;
    tf::StampedTransform transform_left_hand;
    tf::TransformListener listener;


    while(ros::ok())
    {
 		jcmd.header.stamp = ros::Time::now();
        //############################################
        //BAXTER INFO
        //############################################
        double L1 = 0.39;//Del hombro al codo
        double L2 = 0.63;//Del codo a la mano

        double shoulder_x=-0.10;
        double shoulder_y=-0.37;
        double shoulder_z=0.37;

        for (int k=0;k<6;k++){
				P_received[k].resize(3);
				P_send[k].resize(3);
		}
		std::cout<<"bien"<<std::endl;
		//usleep(1000);
        //############################################
        try
		    {
		    	listener.lookupTransform("/openni_depth_frame", "/right_elbow_1",ros::Time(0), transform_right_elbow);
		    	listener.lookupTransform("/openni_depth_frame", "/left_elbow_1",ros::Time(0), transform_left_elbow);
		    	listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1",ros::Time(0), transform_right_shoulder);
		    	listener.lookupTransform("/openni_depth_frame", "/left_shoulder_1",ros::Time(0), transform_left_shoulder);
		    	listener.lookupTransform("/openni_depth_frame", "/right_hand_1",ros::Time(0), transform_right_hand);
		    	listener.lookupTransform("/openni_depth_frame", "/left_hand_1",ros::Time(0), transform_left_hand);
		    }

		    catch (tf::TransformException ex)
		    {
		      	ROS_ERROR("%s",ex.what());
		      	ros::Duration(2.0).sleep();
		    }


        if (transform_right_shoulder.getOrigin().x()>0.1)
        {
	        std::cout<<"recibe"<<std::endl;
	        std::cout<<"right_shoulder_x: "<<transform_right_shoulder.getOrigin().x()<<std::endl;
			std::cout<<"right_shoulder_y: "<<transform_right_shoulder.getOrigin().y()<<std::endl;
		    std::cout<<"right_shoulder_z: "<<transform_right_shoulder.getOrigin().z()<<std::endl;
		    std::cout<<"left_shoulder_x: "<<transform_left_shoulder.getOrigin().x()<<std::endl;
			std::cout<<"left_shoulder_y: "<<transform_left_shoulder.getOrigin().y()<<std::endl;
		    std::cout<<"left_shoulder_z: "<<transform_left_shoulder.getOrigin().z()<<std::endl;

		    std::cout<<"right_elbow_x: "<<transform_right_elbow.getOrigin().x()<<std::endl;
			std::cout<<"right_elbow_y: "<<transform_right_elbow.getOrigin().y()<<std::endl;
		    std::cout<<"right_elbow_z: "<<transform_right_elbow.getOrigin().z()<<std::endl;
		    std::cout<<"left_elbow_x: "<<transform_left_elbow.getOrigin().x()<<std::endl;
			std::cout<<"left_elbow_y: "<<transform_left_elbow.getOrigin().y()<<std::endl;
		    std::cout<<"left_elbow_z: "<<transform_left_elbow.getOrigin().z()<<std::endl;

		    std::cout<<"right_hand_x: "<<transform_right_hand.getOrigin().x()<<std::endl;
			std::cout<<"right_hand_y: "<<transform_right_hand.getOrigin().y()<<std::endl;
		    std::cout<<"right_hand_z: "<<transform_right_hand.getOrigin().z()<<std::endl;
		    std::cout<<"left_hand_x: "<<transform_left_hand.getOrigin().x()<<std::endl;
			std::cout<<"left_hand_y: "<<transform_left_hand.getOrigin().y()<<std::endl;
		    std::cout<<"left_hand_z: "<<transform_left_hand.getOrigin().z()<<std::endl;


		    P_received[0][1]=transform_right_shoulder.getOrigin().x();
			P_received[0][2]=transform_right_shoulder.getOrigin().y();
		    P_received[0][3]=transform_right_shoulder.getOrigin().z();
		    
		    P_received[3][1]=transform_left_shoulder.getOrigin().x();
			P_received[3][2]=transform_left_shoulder.getOrigin().y();
		    P_received[3][3]=transform_left_shoulder.getOrigin().z();

		    P_received[1][1]=transform_right_elbow.getOrigin().x();
			P_received[1][2]=transform_right_elbow.getOrigin().y();
		    P_received[1][3]=transform_right_elbow.getOrigin().z();
		    
		    P_received[4][1]=transform_left_elbow.getOrigin().x();
			P_received[4][2]=transform_left_elbow.getOrigin().y();
		    P_received[4][3]=transform_left_elbow.getOrigin().z();

		    P_received[2][1]=transform_right_hand.getOrigin().x();
			P_received[2][2]=transform_right_hand.getOrigin().y();
		    P_received[2][3]=transform_right_hand.getOrigin().z();
		    
		    P_received[5][1]=transform_left_hand.getOrigin().x();
			P_received[5][2]=transform_left_hand.getOrigin().y();
		    P_received[5][3]=transform_left_hand.getOrigin().z();

		    double M1= sqrt(pow(P_received[1][1]- P_received[0][1], 2.0) + pow(P_received[1][2]- P_received[0][2], 2.0) + pow(P_received[1][3]- P_received[0][3], 2.0));
            double M2= sqrt(pow(P_received[2][1]- P_received[1][1], 2.0) + pow(P_received[2][2]- P_received[1][2], 2.0) + pow(P_received[2][3]- P_received[1][3], 2.0));
            double Q1 = L1 / M1;
            double Q2 = L2 / M2;

            P_send[0][1] = shoulder_x;
            P_send[0][2] = shoulder_y;
            P_send[0][3] = shoulder_z;

            //Redefinimos P2
            P_send[1][1] = (Q1*(P_received[1][1] - P_received[0][1])+P_send[0][1]);
            P_send[1][2] = (Q1*(P_received[1][2] - P_received[0][2])+P_send[0][2]);
            P_send[1][3] = (Q1*(P_received[1][3] - P_received[0][3])+P_send[0][3]);

            //Redefinimos P2
            P_send[2][1] = (Q2*(P_received[2][1] - P_received[1][1])+P_send[1][1]);
            P_send[2][2] = (Q2*(P_received[2][2] - P_received[1][2])+P_send[1][2]);
            P_send[2][3] = Q2*(P_received[2][3] - P_received[1][3])+P_send[1][3];
            //Redfinimos P1

		    double M3= sqrt(pow(P_received[4][1]- P_received[3][1], 2.0) + pow(P_received[4][2]- P_received[3][2], 2.0) + pow(P_received[4][3]- P_received[3][3], 2.0));
            double M4= sqrt(pow(P_received[5][1]- P_received[4][1], 2.0) + pow(P_received[5][2]- P_received[4][2], 2.0) + pow(P_received[5][3]- P_received[4][3], 2.0));
            double Q3 = L1 / M3;
            double Q4 = L2 / M4;

            P_send[3][1] = shoulder_x;
            P_send[3][2] = -shoulder_y;
            P_send[3][3] = shoulder_z;

            //Redefinimos P2
            P_send[4][1] = (Q3*(P_received[4][1] - P_received[3][1])+P_send[3][1]);
            P_send[4][2] = (Q3*(P_received[4][2] - P_received[3][2])+P_send[3][2]);
            P_send[4][3] = Q3*(P_received[4][3] - P_received[3][3])+P_send[3][3];

            //Redefinimos P2
            P_send[5][1] = (Q4*(P_received[5][1] - P_received[4][1])+P_send[4][1]);
            P_send[5][2] = (Q4*(P_received[5][2] - P_received[4][2])+P_send[4][2]);
            P_send[5][3] = Q4*(P_received[5][3] - P_received[4][3])+P_send[4][3];

//######################### Rotation ############################
            P_send[0][1]=-P_send[0][1];
            P_send[0][2]=-P_send[0][2];
            P_send[0][3]=P_send[0][3];

            P_send[1][1]=-P_send[1][1];
            P_send[1][2]=-P_send[1][2];
            P_send[1][3]=P_send[1][3];

            P_send[2][1]=-P_send[2][1];
            P_send[2][2]=-P_send[2][2];
            P_send[2][3]=P_send[2][3];

            P_send[3][1]=-P_send[3][1];
            P_send[3][2]=-P_send[3][2];
            P_send[3][3]=P_send[3][3];

            P_send[4][1]=-P_send[4][1];
            P_send[4][2]=-P_send[4][2];
            P_send[4][3]=P_send[4][3];

            P_send[5][1]=-P_send[5][1];
            P_send[5][2]=-P_send[5][2];
            P_send[5][3]=P_send[5][3];

            std::cout<<"Valores Obtenidos"<<std::endl;
//######################### ##### ############################
		    P_right_wrist.resize(3);
		    std::cout<<"1"<<std::endl;
		    P_left_wrist.resize(3);
		    std::cout<<"2"<<std::endl;
		    P_right_elbow.resize(3);
		    std::cout<<"3"<<std::endl;
		    P_left_elbow.resize(3);
		    std::cout<<"4"<<std::endl;
		    std::cout<<"Vectores Inicializados"<<std::endl;
            // Red
            P_right_wrist[0]=P_send[5][1];
            P_right_wrist[1]=P_send[5][2];
            P_right_wrist[2]=P_send[5][3];
            //Blue
            P_left_wrist[0]=P_send[2][1];
            P_left_wrist[1]=P_send[2][2];
            P_left_wrist[2]=P_send[2][3];
            //yellow
            P_right_elbow[0]=P_send[4][1];
            P_right_elbow[1]=P_send[4][2];
            P_right_elbow[2]=P_send[4][3];
            //Green
            P_left_elbow[0]=P_send[1][1];
            P_left_elbow[1]=P_send[1][2];
            P_left_elbow[2]=P_send[1][3];

            std::cout<<"task"<<std::endl;
            std::cout<<"P_left_elbow"<<std::endl<<P_left_elbow<<std::endl;
            taskle->setDesiredValue(P_left_elbow);
            std::cout<<"P_left_wrist"<<std::endl<<P_left_wrist<<std::endl;
            tasklh->setDesiredValue(P_left_wrist);
            std::cout<<"P_right_elbow"<<std::endl<<P_right_elbow<<std::endl;
            taskre->setDesiredValue(P_right_elbow);
            std::cout<<"P_right_wrist"<<std::endl<<P_right_wrist<<std::endl;
            taskrh->setDesiredValue(P_right_wrist);
 			if(2<1){
            std::cout<<"solver"<<std::endl;
            solver.getPositionControl(qsensed, qdes);
           	
            for(int j=0; j<jnames.size();j++)
            {
               jcmd.position[j]=qdes[j];
            }
            std::cout<<"publish"<<std::endl;
            pub.publish(jcmd);
            }
            //qsensed = qdes;

//###################################################################
            ball_position(0,0)=P_send[0][1];
            ball_position(1,0)=P_send[0][2];
            ball_position(2,0)=P_send[0][3];

            ball_position(0,1)=P_send[1][1];
            ball_position(1,1)=P_send[1][2];
            ball_position(2,1)=P_send[1][3];
            
            ball_position(0,2)=P_send[2][1];
            ball_position(1,2)=P_send[2][2];
            ball_position(2,2)=P_send[2][3];
            
            ball_position(0,3)=P_send[3][1];
            ball_position(1,3)=P_send[3][2];
            ball_position(2,3)=P_send[3][3];
            
            ball_position(0,4)=P_send[4][1];
            ball_position(1,4)=P_send[4][2];
            ball_position(2,4)=P_send[4][3];
            
            ball_position(0,5)=P_send[5][1];
            ball_position(1,5)=P_send[5][2];
            ball_position(2,5)=P_send[5][3];



            b_shoulder_left->setPose(ball_position.col(0));
	        b_shoulder_right->setPose(ball_position.col(3));
	        b_elbow_right->setPose(ball_position.col(4));
	        b_elbow_left->setPose(ball_position.col(1));
	        b_hand_right->setPose(ball_position.col(5));
	        b_hand_left->setPose(ball_position.col(2));

	        b_shoulder_left->publish();
	        b_shoulder_right->publish();
	        b_elbow_right->publish();
	        b_elbow_left->publish();
	        b_hand_right->publish();
	        b_hand_left->publish();

        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}




/*if (kpoints.getPoints()->pose.size() > 0){
//Programacion que recibe el brazo derecho
//for (int k=0;k<3;k++){
//P[k].resize(3);
//It's necessary reduce P[0][k]=(0,0,0)
//P[k][0] = (-kpoints.getPoints()->pose[k].position.z)-(-kpoints.getPoints()->pose[0].position.z);
//P[k][1] = (-kpoints.getPoints()->pose[k].position.x)-(-kpoints.getPoints()->pose[0].position.x);
//P[k][2] = (kpoints.getPoints()->pose[k].position.y)-(kpoints.getPoints()->pose[0].position.y);
//}
//Construimos los puntos (0,0,0); P1; P2
//Hallamos el modulo M1 de P1 - modulo M2 de (P2-P1)
//double M1= sqrt(pow(P[1][0],2.0)+pow(P[1][1],2.0)+pow(P[1][2],2.0));
//double M2= sqrt(pow(P[2][0]- P[1][0], 2.0) + pow(P[2][1]- P[1][1], 2.0) + pow(P[2][2]- P[1][2], 2.0));
//double Q1 = L1 / M1;
            double Q2 = L2 / M2;
            //Redfinimos P1
            P[0][0] = elbow_x;
            P[0][1] = elbow_y;
            P[0][2] = elbow_z;
            //Redefinimos P2
            P[2][0] = Q2*(P[2][0] - P[1][0]);
            P[2][1] = Q2*(P[2][1] - P[1][1]);
            P[2][2] = Q2*(P[2][2] - P[1][2]);
            //Redfinimos P1
            P[1][0] = P[0][0]+Q1*P[1][0];
            P[1][1] = P[0][1]+Q1*P[1][1];
            P[1][2] = P[0][2]+Q1*P[1][2];
            
            P[2][0] = P[2][0]+P[1][0];
            P[2][1] = P[2][1]+P[1][1];
            P[2][2] = P[2][2]+P[1][2];
            //Programacion que recibe el brazo izquierdo
            for (unsigned k=(P.size()/2);k<P.size();k++){
                P[k].resize(3);
                P[k][0] = (-kpoints.getPoints()->pose[k].position.z)-(-kpoints.getPoints()->pose[3].position.z);
                P[k][1] = (-kpoints.getPoints()->pose[k].position.x)-(-kpoints.getPoints()->pose[3].position.x);
                P[k][2] = (kpoints.getPoints()->pose[k].position.y)-(kpoints.getPoints()->pose[3].position.y);
            }
            //Hallamos el modulo M1 de P4
            double M3 = sqrt(pow(P[4][0], 2.0) + pow(P[4][1], 2.0) + pow(P[4][2], 2.0));
            double M4 = sqrt(pow(P[5][0] - P[4][0], 2.0) + pow(P[5][1] - P[4][1], 2.0) + pow(P[5][2] - P[4][2], 2.0));
            double Q3 = L1 / M3;
            double Q4 = L2 / M4;
            //Construimos los puntos (0,0,0); P4; P5
            P[3][0] = elbow_x;
            P[3][1] = -elbow_y;
            P[3][2] = elbow_z;
            //Redefinimos P5
            P[5][0] = Q4*(P[5][0] - P[4][0]);
            P[5][1] = Q4*(P[5][1] - P[4][1]);
            P[5][2] = Q4*(P[5][2] - P[4][2]);
            //Redfinimos P4
            P[4][0] = P[3][0]+Q3*P[4][0];
            P[4][1] = P[3][1]+Q3*P[4][1];
            P[4][2] = P[3][2]+Q3*P[4][2];
            
            P[5][0] = P[5][0]+P[4][0];
            P[5][1] = P[5][1]+P[4][1];
            P[5][2] = P[5][2]+P[4][2];
            
            //cout<<"error1"<<endl;
            //############################################
        	//trasform Orientation
        	//############################################

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
            //outfile.put(Q_orientation[1][0]);
            P_right_wrist.resize(3);
            P_left_wrist.resize(3);
            P_right_elbow.resize(3);
            P_left_elbow.resize(3);
            
            /*
            //Elbow Izquierdo
            P_left_elbow[3] = Q_orientation[1][0];
            P_left_elbow[4] = Q_orientation[1][1];
            P_left_elbow[5] = Q_orientation[1][2];
            //Left hand
            P_left_wrist[3] = Q_orientation[2][0];
            P_left_wrist[4] = Q_orientation[2][1];
            P_left_wrist[5] = Q_orientation[2][2];
            //Right elbow
            P_right_elbow[3] = Q_orientation[4][0];
            P_right_elbow[4] = Q_orientation[4][1];
            P_right_elbow[5] = Q_orientation[4][2];
            //Right hand
            double x = 0.635163;
            double y= -0.830166;
            double z=0.320976;****
            pub.publish(jcmd);
            P_right_wrist[0] = P[5][0];//0.635163;//
            P_right_wrist[1] = P[5][1];//-0.830166;//
            P_right_wrist[2] = P[5][2];//0.320976;//
            
            valor=valor-0.0001;
            //cout<<valor<<endl;
            //P_right_wrist[3] = Q_orientation[5][0];//1.5708;//
            //P_right_wrist[4] = Q_orientation[5][1];//-0.7854;//
            //P_right_wrist[5] = Q_orientation[5][2];//0;//
            //cout<<"error2"<<endl;

            // Format file: Position-Orientation:	
            for(int q=0;q<6;q++){
            	for(int r=0;r<3;r++){
            		myfile << P[q][r]<<endl;
            	}
            	for(int r=0;r<3;r++){
            		myfile << Q_orientation[q][r]<<endl;
            	}
            }
           

            P_right_elbow[0] = P[4][0];
            P_right_elbow[1] = P[4][1];
            P_right_elbow[2] = P[4][2];
            
            P_left_wrist[0] = P[2][0];
            P_left_wrist[1] = P[2][1];
            P_left_wrist[2] = P[2][2];
            
            P_left_elbow[0] = P[1][0];
            P_left_elbow[1] = P[1][1];
            P_left_elbow[2] = P[1][2];
             /*
            P_right_wrist[3] = Q_orientation[5][0];
            P_right_wrist[4] = Q_orientation[5][1];
            P_right_wrist[5] = Q_orientation[5][2];
            ***
            taskle->setDesiredValue(P_left_elbow);
            //cout<<"P_left_elbow"<<endl;
            //cout<<P_left_elbow.transpose()<<endl;
            tasklh->setDesiredValue(P_left_wrist);
            //cout<<"P_left_wrist"<<endl;
            //cout<<P_left_wrist.transpose()<<endl;
            taskre->setDesiredValue(P_right_elbow);
            //cout<<"P_right_elbow"<<endl;
            //cout<<P_right_elbow.transpose()<<endl;
            taskrh->setDesiredValue(P_right_wrist);
            //cout<<"P_right_wrist"<<endl;
            //cout<<P_right_wrist.transpose()<<endl;
            //cout<<"KineSolver"<<endl;
            solver.getPositionControl(qsensed, qdes);
            //cout<<"publish"<<endl;
            //cout<<"qdes:_"<<endl<<qdes<<endl;
            
            for(int j=0; j<jnames.size();j++){
               jcmd.position[j]=qdes[j];
            }
            pub.publish(jcmd);
            //cout<<"publish"<<endl;
            qsensed = qdes;
            ball_position(0,0)=P[3][0];
            ball_position(1,0)=P[3][1];
            ball_position(2,0)=P[3][2];
            ball_position(0,1)=P[4][0];
            ball_position(1,1)=P[4][1];
            ball_position(2,1)=P[4][2];
            ball_position(0,2)=P[5][0];
            ball_position(1,2)=P[5][1];
            ball_position(2,2)=P[5][2];
            ball_position(0,3)=P[0][0];
            ball_position(1,3)=P[0][1];
            ball_position(2,3)=P[0][2];
            ball_position(0,4)=P[1][0];
            ball_position(1,4)=P[1][1];
            ball_position(2,4)=P[1][2];
            ball_position(0,5)=P[2][0];
            ball_position(1,5)=P[2][1];
            ball_position(2,5)=P[2][2];

            for (int k=0; k<6;k++){
	            for(int j=0; j<3;j++){
	            	frame_pose(j,k)=ball_position(j,k);
	            }
        	}

        	for (int k=0; k<6;k++){
	            for(int j=3; j<6;j++){
	            	frame_pose(j,k)=Q_orientation[k][j-3];
	            }
        	}
            marker_current_right_0->setPose(ball_position.col(0));
	        marker_current_right_1->setPose(ball_position.col(1));
	        marker_current_right_2->setPose(ball_position.col(2));
	        marker_current_left_0->setPose(ball_position.col(3));
	        marker_current_left_1->setPose(ball_position.col(4));
	        marker_current_left_2->setPose(ball_position.col(5));

	        frame_current_right_0->setPose(frame_pose.col(0));
    		frame_current_right_1->setPose(frame_pose.col(1));
    		frame_current_right_2->setPose(frame_pose.col(2));
    		frame_current_left_0->setPose(frame_pose.col(3));
    		frame_current_left_1->setPose(frame_pose.col(4));
    		frame_current_left_2->setPose(frame_pose.col(5));

	        marker_current_right_0->publish();
	        marker_current_right_1->publish();
	        marker_current_right_2->publish();
	        marker_current_left_0->publish();
	        marker_current_left_1->publish();
	        marker_current_left_2->publish();

	        frame_current_right_0->publish();
    		frame_current_right_1->publish();
    		frame_current_right_2->publish();
    		frame_current_left_0->publish();
    		frame_current_left_1->publish();
    		frame_current_left_2->publish();
        }
        */