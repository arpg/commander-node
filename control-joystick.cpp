#include "Messages.pb.h"
#include <HAL/Utils/Node.h>
#include "JoystickHandler.h"
#include <atomic>
#include <Node/Node.h>
#include "NinjaMsgs.pb.h"

bool g_bError =false;
node::node  commander_node;


NinjaCommandMsg BuildJoystickStateMsg( JoystickHandler& Joy)
{
  NinjaCommandMsg Cmd;
  double joystickAccel,joystickPhi;
  joystickAccel = (double)Joy.GetAxisValue(1);
  joystickPhi = (double)Joy.GetAxisValue(2);
  //printf("Accel: %f      Phi: %f\r",joystickAccel,joystickPhi);

  Cmd.set_speed(joystickAccel);
  Cmd.set_turnrate(joystickPhi);
}

int main()
{
    // Initialize Commander Node
    commander_node.init("ninja_commander");

    // Subscribe to nc_node
    commander_node.subscribe("nc_node/state");

    if( commander_node.advertise("command") == false ){
      printf("Error setting up publisher!\n");
    }

    JoystickHandler joystick;
    joystick.InitializeJoystick();

    while(1)
    {
        //update the joystick and get the accel/phi values
        joystick.UpdateJoystick();

        // Send Command to Ninja
        NinjaCommandMsg CmdMsg = BuildJoystickStateMsg(joystick);
        if( commander_node.publish("command",CmdMsg) == false ){
          printf("Error publishing message!\n");
        }

        // Receive Ninja's State
        NinjaStateMsg Ninjastate;
        if( commander_node.receive("nc_node/state",Ninjastate) ){
          printf("Ninja's State is: Acc_x:%d    Acc_y:%d",Ninjastate.acc_x(),Ninjastate.acc_y());
        }


        //sleep 0.1ms
        usleep(100000);
    }

}
