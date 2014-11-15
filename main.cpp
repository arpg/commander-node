#include <pangolin/pangolin.h>
#include <HAL/Utils/GetPot>
#include <HAL/IMU/IMUDevice.h>

#include <HAL/Utils/Node.h>

#include "Command.pb.h"
//#include "JoystickHandler.h"
#include <iostream>
#include <boost/bind.hpp>

pangolin::DataLog theLog;  //vector of points
pangolin::DataLog AccLog;  //vector of points
pangolin::DataLog GyroLog;  //vector of points
pangolin::DataLog MagLog;  //vector of points

#define MaxThrottle 500
#define MinThrottle 0
#define MaxSteering 500
#define MinSteering 0

struct NinjaCtrlSigs
{
    int steering;
    int throttle;
}CarCtrlVar={(MaxThrottle-MinThrottle)/2,(MaxSteering-MinSteering)/2};


#define MIN_ACCEL 500
#define MAX_ACCEL 500
#define MIN_PHI 0
#define MAX_PHI 500

#define DEFAULT_ACCEL_COEF 5.65
#define DEFAULT_STEERING_COEF -650
#define DEFAULT_ACCEL_OFFSET 245.35
#define DEFAULT_STEERING_OFFSET 255

//JoystickHandler theGamepad;

using namespace pangolin;
int KeyCounter = 0;

rpg::Node Node;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Handler(pb::ImuMsg& IMUdata)
{
    const pb::VectorMsg& pbAcc = IMUdata.accel();
    const pb::VectorMsg& pbGyr = IMUdata.gyro();
    const pb::VectorMsg& pbMag = IMUdata.mag();

//    std::vector< float > AccData,GyroData,MagData;
//    AccData.push_back( pbAcc.data(0) );
//    AccData.push_back( pbAcc.data(1) );
//    AccData.push_back( pbAcc.data(2) );
//    GyroData.push_back( pbGyr.data(0) );
//    GyroData.push_back( pbGyr.data(1) );
//    GyroData.push_back( pbGyr.data(2) );
//    MagData.push_back( pbMag.data(0) );
//    MagData.push_back( pbMag.data(1) );
//    MagData.push_back( pbMag.data(2) );
    AccLog.Log(pbAcc.data(0),pbAcc.data(1),pbAcc.data(2));
    GyroLog.Log(pbGyr.data(0),pbGyr.data(1),pbGyr.data(2));
    MagLog.Log(pbMag.data(0),pbMag.data(1),pbMag.data(2));
}

void GlobalKeyHook(const int KeyStatus)
{
    const int KeyStep = 10;
    KeyCounter = 0;
    switch(KeyStatus)
    {
        case pangolin::PANGO_KEY_UP:
            CarCtrlVar.throttle = std::min(CarCtrlVar.throttle+KeyStep,MaxThrottle);
            break;
        case pangolin::PANGO_KEY_DOWN:
            CarCtrlVar.throttle = std::max(CarCtrlVar.throttle-KeyStep,MinThrottle);
            break;
        case pangolin::PANGO_KEY_LEFT:
            CarCtrlVar.steering =  std::min(CarCtrlVar.steering+KeyStep,MaxSteering);
            break;
        case pangolin::PANGO_KEY_RIGHT:
            CarCtrlVar.steering =  std::max(CarCtrlVar.steering-KeyStep,MinSteering);
            break;
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{

    GetPot clArgs( argc, argv );

    ///-------------------- INIT IMU
    hal::IMU theIMU( clArgs.follow("", "-imu") );
    theIMU.RegisterIMUDataCallback(IMU_Handler);

    ///-------------------- INIT GAMEPAD
//    if(theGamepad.InitializeJoystick()) {
//        std::cout << "Successfully initialized gamepad." << std::endl;
//    } else {
//        std::cerr << "Failed to initialized gamepad." << std::endl;
//    }

    ///-------------------- INIT PANGOLIN

    // Load configuration data
    pangolin::ParseVarsFile("app.cfg");

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateWindowAndBind("Main",640,480);
    glClearColor(0.0f,0.0f,0.0f,1.0f);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //pangolin::OpenGlMatrix proj = ProjectionMatrix(640,480,420,420,320,240,0.1,1000);

    const int UI_WIDTH = 180;
    const int UI_HIGHT = 480;


    // OpenGL 'view' of data. We might have many views of the same data.
    pangolin::Plotter AccPlot(&AccLog,0,300,-2000,2000,60,0.5);
    AccPlot.SetBounds(2.0/3.0, 1.0, Attach::Pix(UI_WIDTH), 1.0);

    pangolin::Plotter GyroPlot(&GyroLog,300,600,-2000,2000,60,0.5);
    GyroPlot.SetBounds(1.0/3.0, 2.0/3.0, Attach::Pix(UI_WIDTH), 1.0);

    pangolin::Plotter MagPlot(&MagLog,300,600,-2000,2000,60,0.5);
    MagPlot.SetBounds(0.0, 1.0/3.0, Attach::Pix(UI_WIDTH), 1.0);

    pangolin::DisplayBase()
            .AddDisplay(AccPlot)
            .AddDisplay(GyroPlot)
            .AddDisplay(MagPlot);

    // Optionally add named labels
    std::vector<std::string> vLabels;
    vLabels.push_back(std::string("x"));
    vLabels.push_back(std::string("y"));
    vLabels.push_back(std::string("z"));
    AccLog.SetLabels(vLabels);
    GyroLog.SetLabels(vLabels);
    MagLog.SetLabels(vLabels);

    // A Panel is just a View with a default layout and input handling
    View& d_panel = pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, Attach::Pix(UI_WIDTH));

    // Demonstration of how we can register a keyboard hook to alter a Var
//    pangolin::RegisterKeyPressCallback('w', SetVarFunctor<double>("ui.Throttle", 50) );

    pangolin::RegisterKeyPressCallback( 'w', boost::bind(GlobalKeyHook,PANGO_KEY_UP) );
    pangolin::RegisterKeyPressCallback( 's', boost::bind(GlobalKeyHook,PANGO_KEY_DOWN) );
    pangolin::RegisterKeyPressCallback( 'a', boost::bind(GlobalKeyHook,PANGO_KEY_LEFT) );
    pangolin::RegisterKeyPressCallback( 'd', boost::bind(GlobalKeyHook,PANGO_KEY_RIGHT) );

    // Demonstration of how we can register a keyboard hook to trigger a method
//    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'r', boost::bind(GlobalKeyHook, "You Pushed ctrl-r!" ) );

    // set up a publisher
    if( Node.Publish("CarControl", 6002) == false ) {
        printf("Error setting publisher.\n");
    }


    while(!pangolin::ShouldQuit()) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        printf("Accel: %f      Phi: %f\r",joystickAccel,joystickPhi);
//////////////////////
        CommandMsg Req;
        //CommandReply Rep;
        Req.set_accel(CarCtrlVar.throttle);
        Req.set_phi(CarCtrlVar.steering);
//        Node.Call("localhost:5001","ControlRpc", Req, Rep, 1);
        //Node.Call(clArgs.follow("","-Comm"),Req, Rep, 1);
        // set up a publisher


//////////////////////

//        static Var<bool> a_button("ui.A Button",false,false);
//        static Var<double> a_double("ui.A Double",3,0,5);
        static Var<int> NinjaThrottle("ui.Throttle",50,0,100);
        static Var<int> NinjaSteering("ui.Steering",50,0,100);
        static Var<bool> EnableCar("ui.Enable NinjaCar",false,true);
//        static Var<CustomType> any_type("ui.Some Type",(CustomType){0,1.2,"Hello"});
//        if( Pushed(a_button) )
//          NULL;//cout << "You Pushed a button!" << endl;
        if( (!EnableCar) || (KeyCounter>20))
        {
            CarCtrlVar.throttle = (MaxThrottle-MinThrottle)/2;
            CarCtrlVar.steering = (MaxSteering-MinSteering)/2;
        }

        KeyCounter++;

        if ( Node.Write( "CarControl", Req ) == false )
            printf("Error sending message.\n");



        NinjaThrottle = (CarCtrlVar.throttle-MinThrottle)*100/MaxThrottle;
        NinjaSteering = (CarCtrlVar.steering-MinSteering)*100/MaxSteering;

        // Render some stuff
        glColor3f(1.0,1.0,1.0);

        // Swap frames and Process Events
        // and update pangolin GUI
        pangolin::FinishFrame();

        // sleep
        usleep(10000);
    }
    return 0;
}

