#ifndef ALVIUM_CAMERA_H
#define ALVIUM_CAMERA_H


#include "VimbaCPP/Include/VimbaCPP.h"

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
//


//////////////////////////GLOBAL VARIABLES (Needed by .cpp file)

//Variables that need to be used both in the main and the source file
//extern bool StartReceiving;

extern unsigned char* ptr_raw_frame; //For taking the raw image data
extern int img_count;          //For FPS count

//For accessing camera object and changing features at runtime
extern AVT::VmbAPI::CameraPtrVector cameras ; 
extern AVT::VmbAPI::FeaturePtr pFeature ;



/////////////////////////////////
///CLASSES FOR FPS COUNTING

class QTime;

/*class to calculate frames per second*/
class FPSCounter
{
    double          m_CurrentFPS;
    QTime*          m_Timer;
    bool            m_IsRunning;
    VmbInt64_t      m_LastFrameID;
    bool            m_Valid;
public:
    double  CurrentFPS()    const { return m_CurrentFPS; }

    FPSCounter( );
        
    void count( VmbInt64_t id );
    
    bool isValid() const {return m_Valid;}
    void stop()
    {
        m_CurrentFPS    = 0.0;
        m_LastFrameID   = 0;
        m_Valid         = false;
        m_IsRunning     = false;
    }
    bool isRunning() const { return m_IsRunning; }

    ~FPSCounter( );
};



///////////////////////////FRAME OBSERVER CLASS

//define observer that reacts on new frames
class FrameObserver : public AVT::VmbAPI::IFrameObserver
{
    FPSCounter m_FPSReceived;  //For FPS count (Fps received)
    FPSCounter m_FPSCamera;     //For FPS count (Fps Camera)
    
public :


    // In your contructor call the constructor of the base class
    // and pass a camera object
    FrameObserver ( AVT::VmbAPI::CameraPtr pCamera );
    
    void FrameReceived ( const AVT::VmbAPI::FramePtr pFrame ); //Method that gets called at every received frame
    
    void Print_FPS(); //method for FPS print
};


///For Startup (Find camera, open camera, reading buffer size...) and starting acquisition
VmbErrorType Open_and_Start_Acquisition();


///For Stopping camera acquisition, closing camera and Shutting down system
VmbErrorType Stop_Acquisition_and_Close();


#endif  //ALVIUM_CAMERA_H

