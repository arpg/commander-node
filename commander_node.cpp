#include "Messages.pb.h"
#include <HAL/Utils/Node.h>
#include "JoystickHandler.h"
#include <Node/Node.h>
#include "NinjaMsgs.pb.h"

bool g_bError =false;
node::node  commander_node;
double joystickAccel,joystickPhi;
NinjaCommandMsg CmdMsg;
NinjaStateMsg Ninjastate;


NinjaCommandMsg BuildJoystickStateMsg( JoystickHandler& Joy)
{
  NinjaCommandMsg Cmd;

  //printf("Accel: %f      Phi: %f\r",joystickAccel,joystickPhi);
  Cmd.set_speed(joystickAccel);
  Cmd.set_turnrate(joystickPhi);
  return Cmd;
}

int main()
{
    bool bNodeSubsribed = false;

    // Initialize Commander Node
    commander_node.init("commander_node");

    if( commander_node.advertise("command") == false ){
      printf("Error setting up publisher!\n");
    }

    JoystickHandler joystick;
    joystick.InitializeJoystick();

    while(1)
    {
        //update the joystick and get the accel/phi values
        joystick.UpdateJoystick();
        joystickAccel = (double)joystick.GetAxisValue(1);
        joystickPhi = (double)joystick.GetAxisValue(2);

        // Subscribe to ninja node
        if( !bNodeSubsribed ){
          if( commander_node.subscribe("ninja_node/state") == false ){
            printf("Couldn't subscribe to nc_nide/state");
        }else{
          bNodeSubsribed = true;
          }
        }

        // Send Command to Ninja
        CmdMsg = BuildJoystickStateMsg(joystick);
        if( commander_node.publish("command",CmdMsg) == false ){
          printf("Error publishing message!\n");
        }

        // Receive Ninja's State
        NinjaStateMsg Ninjastate;
        if( commander_node.receive("ninja_node/state",Ninjastate) ){
          printf("Ninja's State is: Acc_x:%d    Acc_y:%d \n",Ninjastate.acc_x(),Ninjastate.acc_y());
        }

        // Sleep 10ms
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}
