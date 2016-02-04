#include <HAL/Messages.pb.h>
//#include <HAL/Utils/Node.h>
#include "JoystickHandler.h"
#include <node/Node.h>
#include "NinjaMsgs.pb.h"
//#include <HAL/Posys/PosysDevice.h>
#include <vector>
#include <HAL/Utils/TicToc.h>

#define GPID_P   1.2
#define GPID_I   0
#define GPID_D   1.5
#define Traj_Block_Size     0.61    //meters
#define Forward_force       0.8
#define PID_I_BuffSize      50

/* ***********************
 * Desired_trajectory to follow
 * 1      :    strait line
 * 2      :    infinity
 */
#define DESIRED_TRAJ          2
// 1      :    strait line
#define AXIS_Y_OFFSET         -3
// 2      :    infinity
#define CIRCLEA_ORIGIN_X     -1
#define CIRCLEA_ORIGIN_Y     1
#define CIRCLEB_ORIGIN_X     1
#define CIRCLEB_ORIGIN_Y     1


double PID_I_Buff[PID_I_BuffSize];
bool g_bError =false;
double joystickAccel,joystickPhi;
double CIRCLE_RADIOUS = 0;

NinjaCommandMsg BuildJoystickStateMsg( JoystickHandler& Joy)
{
  NinjaCommandMsg Cmd;
  static unsigned int time_stamp = 0;
  time_stamp++;

  //printf("Accel: %f      Phi: %f\r",joystickAccel,joystickPhi);
  Cmd.set_speed(joystickAccel);
  Cmd.set_turnrate(joystickPhi);
  return Cmd;
}

std::vector<double> Quaternion2Euler(const std::vector<double>& pose_quat){
  std::vector<double> pose_euler;

  pose_euler.push_back(pose_quat[0]);
  pose_euler.push_back(pose_quat[1]);
  pose_euler.push_back(pose_quat[2]);

  std::vector<double> q;
  q.push_back(pose_quat[6]);
  q.push_back(pose_quat[3]);
  q.push_back(pose_quat[4]);
  q.push_back(pose_quat[5]);

  pose_euler.push_back(std::atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(std::pow(q[1],2)+std::pow(q[2],2))));
  pose_euler.push_back(std::asin(2*(q[0]*q[2]-q[3]*q[1])));
  pose_euler.push_back(std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(std::pow(q[2],2)+std::pow(q[3],2))));

/*  for(int ii=0;ii<6;ii++)
    std::cout << pose_euler[ii] << " , ";
  std::cout << std::endl;
*/
  return pose_euler;
}

std::vector<double> Read_Pose( node::node &posys_node){
  static bool bNodeSubscribedPosys = false;
  std::vector<double> Pose_vector;

  // Subscribe to Posys node
  if( !bNodeSubscribedPosys ){
    if( posys_node.subscribe("posetonode/pose") == false ){
      printf("Couldn't subscribe to posetonode/pose \n");
    }
      bNodeSubscribedPosys = true;
  }
  // Receive Posys State
  hal::PoseMsg Posys;
  if( posys_node.receive("posetonode/pose",Posys) ){
//    printf("*********************************************************\n");
    if(Posys.type() == hal::PoseMsg::Type::PoseMsg_Type_SE3){
//        std::cout << "Posys Id: " << Posys.id() << ". Data: ";
        for (int ii = 0; ii < Posys.pose().data_size(); ++ii) {
          Pose_vector.push_back(Posys.pose().data(ii));
//          std::cout << Posys.pose().data(ii) << " ";
        }
//        std::cout << std::endl;
    }else
        printf("wrong Posys massage type");

  }else
  {
    printf("node.receive(Posys) returned false .......... \n");
    bNodeSubscribedPosys = false;
  }
  return Pose_vector;
}

NinjaStateMsg Read_NinjaState(node::node &Ninja_state){
  static bool bNodeSubscribed = false;

  // Subscribe to ninja node
  if( !bNodeSubscribed ){
    if( Ninja_state.subscribe("ninja_node/state") == false ){
      printf("Couldn't subscribe to nc_node/state \n");
    }
      bNodeSubscribed = true;
  }

  // Receive Ninja's State
  NinjaStateMsg Ninjastate;
  if( Ninja_state.receive("ninja_node/state",Ninjastate) ){
    printf("*********************************************************\n");
    //printf("acc_x: %d  acc_y: %d  acc_z: %d \n",Ninjastate.acc_x(),Ninjastate.acc_y(),Ninjastate.acc_z());
    //printf("Gyro_x: %d  Gyro_y: %d  Gyro_z: %d \n",Ninjastate.gyro_x(),Ninjastate.gyro_y(),Ninjastate.gyro_z());
    //printf("Mag_x: %d  Mag_y: %d  Mag_z: %d \n",Ninjastate.mag_x(),Ninjastate.mag_y(),Ninjastate.mag_z());
//    printf("Enc_lb: %d  Enc_lf: %d  Enc_rb: %d  Enc_rf: %d \n",Ninjastate.enc_lb(),Ninjastate.enc_lf(),Ninjastate.enc_rb(),Ninjastate.enc_rf());
//    printf("adc_steer: %d  adc_lb: %d  adc_lf: %d  adc_rb: %d  adc_rf: %d \n", Ninjastate.adc_steer(),Ninjastate.adc_lb(),Ninjastate.adc_lf(),Ninjastate.adc_rb(),Ninjastate.adc_rf());
  }else
  {
    printf("node.receive(Ninjastate) returned false .......... \n");
    bNodeSubscribed = false;
  }
  return Ninjastate;
}

bool Publish_NinjaCmd( node::node& commander, NinjaCommandMsg& CmdMsg ){
  // Send Command to Ninja
  if( commander.publish("command",CmdMsg) == false ){
    printf("Error publishing message!\n");
    return false;
  }
  return true;
}

void Reset_Buff(double* buff,int buff_size){
  for(int ii; ii<buff_size ; ii++)
    buff[ii] = 0;
}

std::vector<double> Trim_6dof_to_3dof(std::vector<double> &pose_6dof){
  std::vector<double> pose_3dof;
  if(pose_6dof.size()!=6)
    LOG(FATAL) << "Received wrong parameter size";

  pose_3dof.push_back(pose_6dof[0]);  // X
  pose_3dof.push_back(pose_6dof[1]);  // Y
  pose_3dof.push_back(pose_6dof[5]);  // Yaw

  return pose_3dof;
}

double CrossTrackError(std::vector<double> pose_3dof){
  double CTError = 0;
  double polar_r = 0;
  double polar_th_A  = 0;
  double polar_th_B  = 0;
  static int state = 1;

  if(pose_3dof.size()!=3)
    LOG(FATAL) << "Received wrong parameter size";

  if(DESIRED_TRAJ == 1){
    CTError = pose_3dof[1] + AXIS_Y_OFFSET;
  }else if(DESIRED_TRAJ == 2){
    polar_th_A = std::atan2(pose_3dof[1]-CIRCLEA_ORIGIN_Y,pose_3dof[0]-CIRCLEA_ORIGIN_X);
    polar_th_B = std::atan2(pose_3dof[1]-CIRCLEB_ORIGIN_Y,pose_3dof[0]-CIRCLEB_ORIGIN_X);

    switch(state){
    case 1:
            if((pose_3dof[0]>=(((CIRCLEB_ORIGIN_X-CIRCLEA_ORIGIN_X)/2)+CIRCLEA_ORIGIN_X)) && (polar_th_B < 0)){
              state = 4;
            }else{
              polar_r = std::sqrt(std::pow(pose_3dof[0]-CIRCLEA_ORIGIN_X,2)+std::pow(pose_3dof[1]-CIRCLEA_ORIGIN_Y,2));
            }
            break;
    case 2:
            if((pose_3dof[0] < (((CIRCLEB_ORIGIN_X-CIRCLEA_ORIGIN_X)/2)+CIRCLEA_ORIGIN_X)) && (polar_th_A >= 0)){
              state = 1;
            }else{
              polar_r = std::sqrt(std::pow(pose_3dof[0]-CIRCLEA_ORIGIN_X,2)+std::pow(pose_3dof[1]-CIRCLEA_ORIGIN_Y,2));
            }
            break;
    case 3:
            if((pose_3dof[0] < (((CIRCLEB_ORIGIN_X-CIRCLEA_ORIGIN_X)/2)+CIRCLEA_ORIGIN_X)) && (polar_th_A < 0)){
              state = 2;
            }else{
              polar_r = std::sqrt(std::pow(pose_3dof[0]-CIRCLEB_ORIGIN_X,2)+std::pow(pose_3dof[1]-CIRCLEB_ORIGIN_Y,2));
            }
            break;
    case 4:
            if((pose_3dof[0]>=(((CIRCLEB_ORIGIN_X-CIRCLEA_ORIGIN_X)/2)+CIRCLEA_ORIGIN_X)) && (polar_th_B >= 0)){
              state = 3;
            }else{
              polar_r = std::sqrt(std::pow(pose_3dof[0]-CIRCLEB_ORIGIN_X,2)+std::pow(pose_3dof[1]-CIRCLEB_ORIGIN_Y,2));
            }
            break;
    }
/*
    if((pose_3dof[0]<((CIRCLEB_ORIGIN_X-CIRCLEA_ORIGIN_X)/2))){      // Then choose Circle A
      polar_r = std::sqrt(std::pow(pose_3dof[0]-CIRCLEA_ORIGIN_X,2)+std::pow(pose_3dof[1]-CIRCLEA_ORIGIN_Y,2));
    }else{                                                         // Then choose Circle B
      polar_r = std::sqrt(std::pow(pose_3dof[0]-CIRCLEB_ORIGIN_X,2)+std::pow(pose_3dof[1]-CIRCLEB_ORIGIN_Y,2));
    }
*/
    CTError = polar_r - CIRCLE_RADIOUS;
    if((state==4)||(state==3))
      CTError *= -1;

    std::cout << "Error is:" << CTError <<  "State is:" << state << std::endl;
  }else{
    CTError = 0;
    LOG(FATAL) << "Desired Trajectory Not Defined";
  }

  return CTError;
}

NinjaCommandMsg Control_PID( std::vector<double> &pose_6dof ){
  NinjaCommandMsg NinjaCmd;
  static double Prev_CTError = 0;
  static double TicTime = 0;
  static bool Initialize_param = true;
  static unsigned int I_buff_NextIndex = 0;
  double CTError;

  if(Initialize_param){
    Initialize_param = false;
    Reset_Buff(PID_I_Buff,PID_I_BuffSize);
    Prev_CTError = CTError;
    TicTime = hal::Tic();
    NinjaCmd.set_speed(0);
    NinjaCmd.set_turnrate(0);
    return NinjaCmd;
  }

  std::vector<double> pose_3dof = Trim_6dof_to_3dof(pose_6dof);
  CTError = 0 - CrossTrackError(pose_3dof);

  // Calculate P value
  double P_effect = GPID_P * CTError;

  // Calculate D value
  double step_duration = hal::TocMS(TicTime)*0.001;    //in seconds
  double D_effect = GPID_D * (CTError-Prev_CTError) / step_duration;

  // Calculate I value
  if(!(I_buff_NextIndex<PID_I_BuffSize)){
    I_buff_NextIndex = 0;
  }
  PID_I_Buff[I_buff_NextIndex] = CTError;
  double buff_sum = 0;
  for(int ii=0 ; ii<PID_I_BuffSize ; ii++ )
    buff_sum += PID_I_Buff[ii];
  double I_effect = GPID_I * buff_sum;

  double pid_out = P_effect + D_effect; // + I_effect;

  Prev_CTError = CTError;
  TicTime = hal::Tic();

  NinjaCmd.set_speed(Forward_force);
  NinjaCmd.set_turnrate(pid_out);

  return NinjaCmd;
}

/////////////////////////////////////////////////////////////////
int main()
{

    CIRCLE_RADIOUS = 0.5 * std::sqrt(std::pow(CIRCLEB_ORIGIN_X-CIRCLEA_ORIGIN_X,2)-std::pow(CIRCLEB_ORIGIN_Y-CIRCLEA_ORIGIN_Y,2));
//    node::node  commander_node;

    // Initialize Commander Node
//    commander_node.init("commander_node");

//    if( commander_node.advertise("command") == false ){
//      printf("Error setting up publisher!\n");
//    }

    JoystickHandler joystick;
    joystick.InitializeJoystick();

    while(1)
    {
        // Receive Ninja's State
//        NinjaStateMsg Ninjastate = Read_NinjaState(commander_node);

        // Receive Posys
//        std::vector<double> Pose_quaternion = Read_Pose(commander_node);
//        std::vector<double> pose_6dof = Quaternion2Euler(Pose_quaternion);

        //update the joystick and get the accel/phi values
        joystick.UpdateJoystick();
        joystickAccel = (double)joystick.GetAxisValue(1);
        joystickPhi = -(double)joystick.GetAxisValue(2);
//        printf("Axis_acc: %f   Axis_Phi: %f\n",joystickAccel,joystickPhi);

        NinjaCommandMsg cmd;
        if((joystick.IsButtonPressed(4))&&(false)){   // Then go to auto-control
//          cmd = Control_PID(pose_6dof);
          //cmd2 = BuildJoystickStateMsg(joystick);
          //cmd.set_speed(cmd2.speed());
//          Publish_NinjaCmd(commander_node,cmd);
        }else{
          //cmd = BuildJoystickStateMsg(joystick);
          int ii=0;
          if(ii<10)
            ii++;
          else
            ii=0;
          cmd.set_speed((double)ii/10);
          cmd.set_turnrate(joystickPhi);
//          Publish_NinjaCmd(commander_node,cmd);
//          std::cout << "SENT" << std::endl;
        }

        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    joystick.JoinThread();

}
