#include "Messages.pb.h"
#include <HAL/Utils/Node.h>
#include "JoystickHandler.h"
#include <atomic>


bool g_bError =false;
int g_nRpcControlCount = 0;

int main()
{
    JoystickHandler joystick;
    joystick.InitializeJoystick();

    while(1)
    {
        //update the joystick and get the accel/phi values
        joystick.UpdateJoystick();
        double joystickAccel,joystickPhi;
        joystickAccel = (double)joystick.GetAxisValue(1);
        joystickPhi = (double)joystick.GetAxisValue(2);
        printf("ACC is: %f    Phi is: %f \n",joystickAccel,joystickPhi);

        //sleep 0.1ms
        usleep(1000);
    }

}
