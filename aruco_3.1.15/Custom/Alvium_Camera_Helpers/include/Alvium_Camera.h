#ifndef ALVIUM_CAMERA_H
#define ALVIUM_CAMERA_H


#include "VimbaCPP/Include/VimbaCPP.h"

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <QTime>


//////////////////////////GLOBAL VARIABLES (Needed by .cpp file)

//Variables that need to be used both in the main and the source file
extern bool StartReceiving;
extern unsigned char* ptr_raw_frame; //For taking the raw image data
extern int img_count;          //For FPS count



/////////////////////////////////
///CLASSES FOR FPS COUNTING

#ifdef _WIN32
/*high resolution timer for windows*/
class PrecisionTimer
{
    const LARGE_INTEGER     m_Frequency;
    LARGE_INTEGER           m_Time;
    static LARGE_INTEGER getFrq()
    {
        LARGE_INTEGER tmp;
        QueryPerformanceFrequency( & tmp );
        return tmp;
    }
public:
    PrecisionTimer()
        : m_Frequency( getFrq() )
    {
    }
    LARGE_INTEGER Frequency() const { return m_Frequency; }
    LARGE_INTEGER Ticks() const
    {
        LARGE_INTEGER tmp;
        QueryPerformanceCounter( &tmp );
        return tmp;
    }
    void start()
    {
        QueryPerformanceCounter( &m_Time );
    }
    double elapsed() const
    {
        LARGE_INTEGER tmpTime;
        QueryPerformanceCounter( &tmpTime );
        tmpTime.QuadPart -= m_Time.QuadPart;
        return (tmpTime.QuadPart*1000.0)/ m_Frequency.QuadPart;
    }
};

typedef PrecisionTimer timer_type;
#else
typedef QTime timer_type;
#endif

/*class to calculate frames per second*/
class FPSCounter
{
    double          m_CurrentFPS;
    timer_type      m_Timer;
    bool            m_IsRunning;
    VmbInt64_t      m_LastFrameID;
    bool            m_Valid;
public:
    double  CurrentFPS()    const { return m_CurrentFPS; }

    FPSCounter( )
        : m_IsRunning( false )
        , m_LastFrameID( 0 )
        , m_Valid( false )
    {

    }
    void count( VmbInt64_t id )
    {
        if( ! m_IsRunning )
        {
            m_LastFrameID   = id;
            m_IsRunning     = true;
            m_Valid         = false;
            m_Timer.start();
            return;
        }
        double      time_ms     = m_Timer.elapsed();
        VmbInt64_t  delta_id = id - m_LastFrameID;
        if( (time_ms > 1000  && delta_id != 0) )
        {
            m_CurrentFPS    = (delta_id*1000) / time_ms;
            m_LastFrameID   = id;
            m_Valid         = true;
            m_Timer.start();
        }
    }
    bool isValid() const {return m_Valid;}
    void stop()
    {
        m_CurrentFPS    = 0.0;
        m_LastFrameID   = 0;
        m_Valid         = false;
        m_IsRunning     = false;
    }
    bool isRunning() const { return m_IsRunning; }


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

