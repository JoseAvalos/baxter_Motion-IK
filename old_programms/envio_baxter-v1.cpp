//
// Control 
////////////////// Executing 
//  ./baxter.sh sim
//  roslaunch baxter_gazebo baxter_world.launch
//  rosrun baxter_tools enable_robot.py -e
//  rosrun baxter..AA

////////////////// Executing 
// /home/jose/baxter-workspace/src/baxter_cpp/launch
// roslaunch l_baxter_cpp.launch 
//  rosrun baxter_cpp OSIK_baxter


#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#include <robot-model/robot-model.hpp>
#include <kinect/kinectbody.h>
#include <osik-control/math-tools.hpp>
#include <osik-control/kine-task.hpp>
#include <osik-control/kine-task-pose.hpp>
#include <osik-control/kine-solver-WQP.hpp>
#include <baxter_cpp/robotSpecifics.h>
#include <baxter_cpp/tools.hpp>

#include <data_kinect/BodyArray.h>
#include <cmath>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <baxter_cpp/markers.hpp>


using namespace osik;
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
};

int main(int argc, char **argv)
{
  //kinect::kinectbody hola;
  
  //############################################
  //INFORMACION DEL BAXTER
 //*****************************************

  std::string baxter_description = ros::package::getPath("baxter_description");
  std::string model_name = baxter_description + "/urdf/baxter.urdf";
  RobotModel* robot = new RobotModel();
  bool has_floating_base = false;
  

  if (!robot->loadURDF(model_name, has_floating_base)){
    return -1;}
  else{
  
  std::cout << "Robot " << model_name << " loaded." << std::endl;}

  unsigned int ndof_full = robot->ndof();        

  std::cout << "Reading initial sensor values ..." << std::endl;

  // Get the joint names and joint limits
  std::vector<std::string> jnames;
  std::vector<double> qmin, qmax, dqmax;
  jnames = robot->jointNames();
  qmin  = robot->jointMinAngularLimits();
  qmax  = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();
  
  //############################################
  //INICIO DEL PROCESO DE RECEPCION 
  //*****************************************

  ros::init(argc, argv, "n_sendbaxter");
  ros::NodeHandle nh;


  KinectPoints kpoints;

  //Suscriber
  ros::Subscriber sub_1 = nh.subscribe("t_kinectdata", 1000, &KinectPoints::readKinectPoints, &kpoints);
  JointSensors jsensor;
  ros::Subscriber sub_2 = nh.subscribe("robot/joint_states", 1000,&JointSensors::readJointSensors, &jsensor);
  
  //"kinect_points"
  std::cout << "Reading initial sensor values ..." << std::endl;
  ros::Rate iter_rate(1000); // Hz
  unsigned int niter=0, max_iter = 1e3;
  //unsigned int ndof_sensed = 222;


  unsigned int ndof_sensed = jsensor.sensedValue()->position.size();
  ndof_sensed = jsensor.sensedValue()->name.size();
  ros::spinOnce();
  iter_rate.sleep();
  std::cout << "Found " << ndof_sensed << " sensed joints" << std::endl;

  Eigen::VectorXd qsensed = Eigen::VectorXd::Zero(ndof_full);
  
  jsensor.getSensedJointsRBDL(qsensed, ndof_sensed);

  Eigen::VectorXd q=Eigen::MatrixXd::Constant(16, 1, 0.5);
  cout<<q<<endl;
  Eigen::Vector3d w(0,0,0);  
  Eigen::Vector3d v(0,0,0);
  Eigen::MatrixXd qposition(3,20);
  int link_number={1};
  //Eigen::VectorXd qposition=Eigen::MatrixXd::Constant(15, 1, 0.0);

  cout <<"full: "<<ndof_full<<endl;


  cout<<"LINK position"<<endl;

  std::vector< std::vector<double> > P;
  P.resize(6);
  

  //############################################
  //PROCESO MARKER
 //*****************************************
/*
  double red[3] = {1.,0.,0.};
  double green[3] = {0.,1.,0.};
  double blue[3] = {0.,0.,1.};
  double yellow[3] = {0.5,0.5,0.};
 
  BallMarker* marker_current_head;
  BallMarker* marker_current_base;

  BallMarker* marker_current_right_0;
  BallMarker* marker_current_right_1;
  BallMarker* marker_current_right_2;
  BallMarker* marker_current_right_3;
  BallMarker* marker_current_right_4;
  BallMarker* marker_current_right_5;
  BallMarker* marker_current_right_6;

  BallMarker* marker_current_left_0;
  BallMarker* marker_current_left_1;
  BallMarker* marker_current_left_2;
  BallMarker* marker_current_left_3;
  BallMarker* marker_current_left_4;
  BallMarker* marker_current_left_5;
  BallMarker* marker_current_left_6;

  marker_current_base = new BallMarker(nq, red);
  marker_current_head = new BallMarker(nq, blue);


  marker_current_right_0 = new BallMarker(nq, green);
  marker_current_right_1 = new BallMarker(nq, green);
  marker_current_right_2 = new BallMarker(nq, green);
  marker_current_right_3 = new BallMarker(nq, green);
  marker_current_right_4 = new BallMarker(nq, green);
  marker_current_right_5 = new BallMarker(nq, green);
  marker_current_right_6 = new BallMarker(nq, green);

  marker_current_left_0 = new BallMarker(nq, yellow);
  marker_current_left_1 = new BallMarker(nq, yellow);
  marker_current_left_2 = new BallMarker(nq, yellow);
  marker_current_left_3 = new BallMarker(nq, yellow);
  marker_current_left_4 = new BallMarker(nq, yellow);
  marker_current_left_5 = new BallMarker(nq, yellow);
  marker_current_left_6 = new BallMarker(nq, yellow);
*/
//############################################
//INICIO DEL PROCESO DE ENVIO
//*****************************************

  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("t_sendbaxter", 1000);
  
  sensor_msgs::JointState jcmd;

  jcmd.name.resize(15);
  jcmd.position.resize(15);
  jcmd.velocity.resize(15);
  jcmd.effort.resize(15);

  for(int j=0; j<jnames.size();j++){
      jcmd.name[j]=jnames[j];
      //jcmd.position[j]=q[j];
      jcmd.velocity[j]=0.2;
      jcmd.effort[j]=0.2;
  }
  /*
  qposition.col(19)<<0,
  0,
  0;

  //cout<<jnames.size()<<endl;

  for(int i=0; i<jnames.size(); i++){
      //cout<<i<<endl;
      //cout<<i<<".-"<<jnames[i] <<": " <<endl;
      qposition.col(i) =robot->linkPosition(q,i+1,w);
      cout<<""<<qposition.col(i).transpose()<<endl;
      //cout<<i<<endl;
  }
  
  cout<<"ok_names"<<endl;*/

// ************************************
// Tasks and Inverse Kinematics Solver
// ************************************

  // Sampling time
  unsigned int f = 30;   // Frequency
  double dt = static_cast<double>(1.0/f);

  KineSolverWQP solver(robot, qsensed, dt);
  solver.setJointLimits(qmin, qmax, dqmax);

 
  //KineTask* taskrh = new KineTaskPose(robot, RGRIPPER, "position");
  KineTask* taskrh = new KineTaskPose(robot, 7, "position");
  taskrh->setGain(300.0);
  KineTask* tasklh = new KineTaskPose(robot, 14, "position");
  tasklh->setGain(300.0);
  KineTask* taskre = new KineTaskPose(robot, 4, "position");
  taskre->setGain(300.0);
  KineTask* taskle = new KineTaskPose(robot, 11, "position");
  taskle->setGain(300.0);


  Eigen::VectorXd P_right_wrist;
  Eigen::VectorXd P_right_elbow;
  Eigen::VectorXd P_left_wrist;
  Eigen::VectorXd P_left_elbow;

  solver.pushTask(taskrh);
  solver.pushTask(tasklh);
  solver.pushTask(taskre);
  solver.pushTask(taskle);

  Eigen::VectorXd qdes;

  ros::Rate rate(f); // Hz

  //#######################################################
  //############################################
  //ROS
  //*****************************************

  while(ros::ok())
    {
      
    
    std::cout<<"size: "<< kpoints.getPoints()->pose.size() << std::endl;

    /*
    for(int j=0; j<jnames.size();j++){
      qposition.col(j) =robot->linkPosition(q,j+1,w);
    }
    
    marker_current_base->setPose(qposition.col(19) );
    marker_current_head->setPose(qposition.col(0) );

    marker_current_right_0->setPose(qposition.col(1) );
    marker_current_right_1->setPose(qposition.col(2) );
    marker_current_right_2->setPose(qposition.col(3) );
    marker_current_right_3->setPose(qposition.col(4) );
    marker_current_right_4->setPose(qposition.col(5) );
    marker_current_right_5->setPose(qposition.col(6) );
    marker_current_right_6->setPose(qposition.col(7) );
    
    marker_current_left_0->setPose(qposition.col(8) );
    marker_current_left_1->setPose(qposition.col(9) );
    marker_current_left_2->setPose(qposition.col(10) );
    marker_current_left_3->setPose(qposition.col(11) );
    marker_current_left_4->setPose(qposition.col(12) );
    marker_current_left_5->setPose(qposition.col(13) );
    marker_current_left_6->setPose(qposition.col(14) );
    
    marker_current_base->publish();
    marker_current_head->publish();

    marker_current_right_0->publish();
    marker_current_right_1->publish();
    marker_current_right_2->publish();
    marker_current_right_3->publish();
    marker_current_right_4->publish();
    marker_current_right_5->publish();
    marker_current_right_6->publish();
   
    marker_current_left_0->publish();
    marker_current_left_1->publish();
    marker_current_left_2->publish();
    marker_current_left_3->publish();
    marker_current_left_4->publish();
    marker_current_left_5->publish();
    marker_current_left_6->publish();
    
    */

    jcmd.header.stamp = ros::Time::now();

    ///////////////////////////////////////////////////////////////
    //Datos del baxter
    double L1 = 0.37;//Del hombro al codo
    double L2 = 0.37;//Del codo a la mano
    //L2=111
    
    if (kpoints.getPoints()->pose.size() > 0){
      cout<<"ok_body.size"<<endl;
      //Programacion que recibe el brazo derecho
      
      for (int k=0;k<3;k++){    
        P[k].resize(3);
        //A partir de eso P[0][k]=(0,0,0)
        P[k][0] = (-kpoints.getPoints()->pose[k].position.z)-(-kpoints.getPoints()->pose[0].position.z);
        P[k][1] = (-kpoints.getPoints()->pose[k].position.x)-(-kpoints.getPoints()->pose[0].position.x);
        P[k][2] = (kpoints.getPoints()->pose[k].position.y)-(kpoints.getPoints()->pose[0].position.y);
        }
        
      //Construimos los puntos (0,0,0); P1; P2
      //Hallamos el modulo M1 de P1
      //cout<<"ok_4"<<endl;

      double M1= sqrt(pow(P[1][0],2.0)+pow(P[1][1],2.0)+pow(P[1][2],2.0));
      //Hallamos el modulo M2 de (P2-P1)
      double M2= sqrt(pow(P[2][0]- P[1][0], 2.0) + pow(P[2][1]- P[1][1], 2.0) + pow(P[2][2]- P[1][2], 2.0));
      //Determimos la proporcionalidades
      double Q1 = L1 / M1;
      double Q2 = L2 / M2;
      //std::cout<< "M2: "<<M2<<std::endl;
      //Redfinimos P1
      //cout<<"ok_5"<<endl;
      
      P[0][0] = 0.069;
      P[0][1] = 0.0;
      P[0][2] = 0.2735;
      //P[0][1] = 0.098;
      //[0][2] = 0.100
      //Redefinimos P2
      P[2][0] = Q2*(P[2][0] - P[1][0]);
      P[2][1] = Q2*(P[2][1] - P[1][1]);
      P[2][2] = Q2*(P[2][2] - P[1][2]);

      //Redfinimos P1
      P[1][0] = P[0][0]+Q1*P[1][0];
      P[1][1] = P[0][1]+Q1*P[1][1];
      P[1][2] = P[0][2]+Q1*P[1][2];
      //double M4= sqrt(pow(P[1][0],2)+pow(P[1][1],2)+pow(P[1][2],2));
      //std::cout<< "M4: "<<M4<<std::endl;
      P[2][0] = P[2][0]+P[1][0];
      P[2][1] = P[2][1]+P[1][1];
      P[2][2] = P[2][2]+P[1][2];

      //cout<<P[2][2]<<endl;
        
      //Programacion que recibe el brazo izquierdo
      for (unsigned k=(P.size()/2);k<P.size();k++){    
      P[k].resize(3);
      P[k][0] = (-kpoints.getPoints()->pose[k].position.z)-(-kpoints.getPoints()->pose[3].position.z);
      P[k][1] = (-kpoints.getPoints()->pose[k].position.x)-(-kpoints.getPoints()->pose[3].position.x);
      P[k][2] = (kpoints.getPoints()->pose[k].position.y)-(kpoints.getPoints()->pose[3].position.y);
      //std::cout << "k 2: " << k << std::endl;
      } 
      
      //Hallamos el modulo M1 de P4
      double M3 = sqrt(pow(P[4][0], 2.0) + pow(P[4][1], 2.0) + pow(P[4][2], 2.0));
      //Hallamos el modulo M2 de (P2-P1)
      double M4 = sqrt(pow(P[5][0] - P[4][0], 2.0) + pow(P[5][1] - P[4][1], 2.0) + pow(P[5][2] - P[4][2], 2.0));
      double Q3 = L1 / M3;
      double Q4 = L2 / M4;
      //Construimos los puntos (0,0,0); P4; P5

      P[3][0] = 0.00;
      P[3][1] = -0.098;
      P[3][2] = 0.100;
      //P[0][1] = 0.098;
      //[0][2] = 0.100
      //Redefinimos P5
      P[5][0] = Q4*(P[5][0] - P[4][0]);
      P[5][1] = Q4*(P[5][1] - P[4][1]);
      P[5][2] = Q4*(P[5][2] - P[4][2]);

      //Redfinimos P4
      P[4][0] = P[3][0]+Q3*P[4][0];
      P[4][1] = P[3][1]+Q3*P[4][1];
      P[4][2] = P[3][2]+Q3*P[4][2];
      //double M4= sqrt(pow(P[1][0],2)+pow(P[1][1],2)+pow(P[1][2],2));
      //std::cout<< "M4: "<<M4<<std::endl;
      P[5][0] = P[5][0]+P[4][0];
      P[5][1] = P[5][1]+P[4][1];
      P[5][2] = P[5][2]+P[4][2];

      //#######################################################

      P_right_wrist.resize(3);
      P_left_wrist.resize(3);
      P_right_elbow.resize(3);
      P_left_elbow.resize(3);
       
      //Elbow Izquierdo
      P_left_elbow[0] = P[1][0];
      P_left_elbow[1] = P[1][1];
      P_left_elbow[2] = P[1][2];
          
      //Left hand
      P_left_wrist[0] = P[2][0];
      P_left_wrist[1] = P[2][1];
      P_left_wrist[2] = P[2][2];
          
      //Right elbow
      P_right_elbow[0] = P[4][0];
      P_right_elbow[1] = P[4][1];
      P_right_elbow[2] = P[4][2];
          
      //Right hand
      P_right_wrist[0] = P[5][0];
      P_right_wrist[1] = P[5][1];
      P_right_wrist[2] = P[5][2];
        

      taskle->setDesiredValue(P_left_elbow);
      tasklh->setDesiredValue(P_left_wrist);
      taskre->setDesiredValue(P_right_elbow);
      taskrh->setDesiredValue(P_right_wrist);
          
      //cout<<"ok_7"<<endl;
      //cout<<"Next_qsensed:"<<endl<<qsensed<<endl;
      //cout<<"Next_sqdes:"<<endl<<qdes<<endl;
      //cout<<"ok_9"<<endl;

      solver.getPositionControl(qsensed, qdes);
      pub.publish(jcmd);
      qsensed = qdes;
    }
  ros::spinOnce();
  rate.sleep();
  }
  return 0;
}
