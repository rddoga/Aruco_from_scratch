#ifndef ALVIUM_CAMERA_H
#define ALVIUM_CAMERA_H


#include "VimbaCPP/Include/VimbaCPP.h"

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <chrono>


//////////////////////////GLOBAL VARIABLES (Needed by .cpp file)

//Variables that need to be used both in the main and the source file
extern bool StartReceiving;
extern unsigned char* ptr_raw_frame; //For taking the raw image data
extern int img_count;          //For FPS count

///////////////////////////FRAME OBSERVER CLASS

//define observer that reacts on new frames
class FrameObserver : public AVT::VmbAPI::IFrameObserver
{

public :
    static float total_time, start; //


    // In your contructor call the constructor of the base class
    // and pass a camera object
    FrameObserver ( AVT::VmbAPI::CameraPtr pCamera );
    
    void FrameReceived ( const AVT::VmbAPI::FramePtr pFrame ); //Method that gets called at every received frame
    
    static void Print_FPS(); //static method for FPS print
};


///For Startup (Find camera, open camera, reading buffer size...) and starting acquisition
VmbErrorType Open_and_Start_Acquisition();


///For Stopping camera acquisition, closing camera and Shutting down system
VmbErrorType Stop_Acquisition_and_Close();


#endif  //ALVIUM_CAMERA_H

