#include "Messages.pb.h"
#include "../Hermes1.0/Node.h"
#include "JoystickHandler.h"
#include "CarPlannerCommon.h"
#include "BulletCarModel.h"
#include <atomic>

#define DEFAULT_ACCEL_COEF 5.65
#define DEFAULT_STEERING_COEF -650
#define DEFAULT_ACCEL_OFFSET 245.35
#define DEFAULT_STEERING_OFFSET 255

enum ControlMaster
{
    eJoystickControl,
    eProgramControl,
    eInvalid
};

bool g_bError =false;
ControlMaster g_eMaster = eJoystickControl;
rpg::Node g_CommandRpcNode(5002);
int g_nRpcControlCount = 0;

//atomic variables for the accelerations and steering values given by the RPC
std::atomic<double> m_gLastAccel;
std::atomic<double> m_gLastPhi;
std::atomic<int> m_nProgramCount;

void ProgramControlRpc( CommandMsg& Req, CommandReply& Rep, void* userData )
{    
    if(g_eMaster == eProgramControl){
        try{
            m_gLastAccel = Req.accel();
            m_gLastPhi = Req.phi();
        }catch(...){
            std::cout << "Error occured when sending packet." << std::endl;
            //g_bError = true;
        }
    }
    m_nProgramCount++;
}

const char *GetControlTypeString(ControlMaster eMaster)
{
    if(eMaster == eInvalid){
        return "invalid";
    }else if(eMaster == eProgramControl){
        return "program";
    }else{
        return "joystick";
    }
}

int main()
{
    printf("Initializing Mocha Daemon.\n");
    m_gLastAccel = DEFAULT_ACCEL_OFFSET;
    m_gLastPhi = DEFAULT_STEERING_OFFSET;
    m_nProgramCount = 0;
    double dUpdateTime = Tic();

    JoystickHandler joystick;

    joystick.InitializeJoystick();


    g_CommandRpcNode.Register("ProgramControlRpc",&ProgramControlRpc,NULL);

    CommandMsg Req;
    CommandReply Rep;

    while(1)
    {
        //update the joystick and get the accel/phi values
        joystick.UpdateJoystick();
        double joystickAccel,joystickPhi;
        joystickAccel = (((double)joystick.GetAxisValue(1)/JOYSTICK_AXIS_MAX)*-40.0);
        joystickPhi = (((double)joystick.GetAxisValue(2)/(double)JOYSTICK_AXIS_MAX) * (MAX_SERVO_ANGLE*M_PI/180.0)*0.5);

        joystickAccel = joystickAccel*DEFAULT_ACCEL_COEF + DEFAULT_ACCEL_OFFSET;
        joystickPhi = joystickPhi*DEFAULT_STEERING_COEF + DEFAULT_STEERING_OFFSET;

        //if the user pressed the button, switch control from joystick to program and vice versa
        ControlMaster eNewMaster = g_eMaster;
        if(joystick.IsButtonPressed(7)){
            eNewMaster = eJoystickControl;
        }else if(joystick.IsButtonPressed(5)){
            eNewMaster = eProgramControl;
        }

        if(eNewMaster != g_eMaster){
            printf("Switching to %s control.\n",GetControlTypeString(eNewMaster));
            fflush(stdout);
            g_eMaster = eNewMaster;
        }

        try{
            //only send if commands have changed
            if(g_eMaster == eJoystickControl){
                //if(joystickAccel != m_gLastAccel || joystickPhi != m_gLastPhi){
                    //now depending on the selected mode, call the RPC on the car with the commands
                    m_gLastPhi = joystickPhi;
                    m_gLastAccel = joystickAccel;
                    Req.set_accel(g_bError ? DEFAULT_ACCEL_OFFSET : joystickAccel);
                    Req.set_phi(g_bError ? DEFAULT_STEERING_OFFSET : joystickPhi);
                    g_CommandRpcNode.Call("herbie:5001","ControlRpc",Req,Rep,100);
                    g_nRpcControlCount++;
                //}
            }else{
                Req.set_accel(g_bError ? DEFAULT_ACCEL_OFFSET : (double)m_gLastAccel);
                Req.set_phi(g_bError ? DEFAULT_STEERING_OFFSET : (double)m_gLastPhi);
                g_CommandRpcNode.Call("herbie:5001","ControlRpc",Req,Rep,100);
                g_nRpcControlCount++;
            }
        }catch(...){
            std::cout << "Error occured when sending packet." << std::endl;
            //g_bError = true;
        }

        //print out the statistics
        double dDuration = Toc(dUpdateTime);
        if(dDuration > 1.0 && g_bError == false){
            dUpdateTime = Tic();
            printf("Receiving %.2f/s program calls and sending %.2f/s control calls. Current control: %s\n",
                   (double)m_nProgramCount/dDuration, (double)g_nRpcControlCount/dDuration,GetControlTypeString(g_eMaster));
            fflush(stdout);
            m_nProgramCount = 0;
            g_nRpcControlCount = 0;
        }

        //sleep 0.1ms
        usleep(1000);
    }

}
