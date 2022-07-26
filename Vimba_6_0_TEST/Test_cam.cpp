#include "VimbaCPP/Include/VimbaCPP.h"
//#include "ApiController.h"
//#include "ProgramConfig.h"

//#include "v4l2_helper.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core_c.h>
#include <QTime>

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
//#include <mutex>



using namespace std;
using namespace AVT::VmbAPI;

//std::mutex mtx;           // mutex for critical section

///////////////////////////

float ExposureTime = 8000.0f; //To set/get the exposure time on the trackbar

int imgSizeX = 4024, imgSizeY = 3036;
//frame pointer
cv::Mat raw_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);
bool StartReceiving = false;

//unsigned char* ptr_receive;
//VmbUint32_t width, height;// ImageMemSize;
//VmbPixelFormatType PixFormat;
///////////////////////////
///////////////////////////


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


///////////////////////////////



//define observer that reacts on new frames
class FrameObserver : public IFrameObserver
{
    FPSCounter m_FPSReceived;  //For FPS count (Fps received)
    FPSCounter m_FPSCamera;     //For FPS count (Fps Camera)
    
public :

    int img_count;          //For FPS count
    
    
    // In your contructor call the constructor of the base class
    // and pass a camera object
    FrameObserver ( CameraPtr pCamera ) : IFrameObserver ( pCamera ), img_count(0)
    {
        // Put your initialization code here
    }
    
    void FrameReceived ( const FramePtr pFrame )
    {
        VmbFrameStatusType eReceiveStatus;
        if( VmbErrorSuccess == pFrame -> GetReceiveStatus ( eReceiveStatus ) )
        {
            if ( VmbFrameStatusComplete == eReceiveStatus )
            {
                StartReceiving = true; //Start the display when we start receiving
                //Starting point

                //mtx.lock(); //Lock the Resources 
                            
                //cout << "Frame received successfully !!" << endl;
                pFrame->GetImage(raw_frame.data); //Getting raw image
                
                //mtx.unlock(); //Unlock the Resources 
                            
                //pFrame->GetHeight(height);
                //pFrame->GetWidth(width);
                //pFrame->GetPixelFormat(PixFormat);
                //pFrame->GetImageSize(ImageMemSize);
                //cout << width << " x " << height << endl;
                //cout << PixFormat << "\t\t" << ImageMemSize << endl;
                //cout << sizeof(ptr_receive)/sizeof(ptr_receive[0]) << endl;
                //cout <<yuyv_frame.data[0] << endl;
                
                img_count++;
                m_FPSReceived.count( img_count );
                VmbUint64_t camera_frame_id;
                pFrame->GetFrameID( camera_frame_id );
                m_FPSCamera.count( camera_frame_id ); 
        
                if( m_FPSReceived.isValid() )
                {
                    if(img_count%5 == 0)// Printing current fps
                    {
                        cout << "Received : " << m_FPSReceived.CurrentFPS() << " FPS" << endl;
                        cout << "Camera : " << m_FPSCamera.CurrentFPS() << " FPS" << endl;
                    }
                }
                
                // Put your code here to react on a successfully received frame
            }
            else
            {
                cout << "ERR : Image not successfully received !!" << endl;
                // Put your code here to react on an unsuccessfully received frame
            }
        }
        // When you are finished copying the frame , re - queue it
        m_pCamera -> QueueFrame ( pFrame );
    }
    
};

//float FrameObserver::total_time = 0.0f, FrameObserver::start = clock(); //
//int FrameObserver::img_count = 0;          //For FPS count


/////////////////////////

int main(int argc, char* argv[])
{    
    /*
    VimbaSystem &system = VimbaSystem::GetInstance(); //Create a reference to the VimbaSystem singleton
    
    /////////
    VmbVersionInfo_t version;
    system.QueryVersion(version); //Get version of Vimba C++ API
    
    cout << "Version : " << version.major << "." << version.minor << "." << version.patch << endl;
    */
    
    //Create and resize window
    cv::namedWindow("Img", cv::WINDOW_NORMAL);
    cv::resizeWindow("Img", 1920, 1080);
    
    //Create trackbar window to change the Exposure Time
    cv::namedWindow("Trackbar", cv::WINDOW_GUI_EXPANDED);
    cv::moveWindow("Trackbar",0,750);
    
    //Create trackbar on thje window
    cv::createTrackbar("Exposure time", "Trackbar", NULL, 20000);
    cv::setTrackbarPos("Exposure time", "Trackbar", ExposureTime);
    
    //Original image, that gets the cam frame from the pointer
    //cv::Mat originalImage;
    //IplImage* Ipl;
    
    cout << "START" << endl;
    /////////
    cout << "Hello" << endl;
    VmbErrorType err;
    // Every Vimba function returns an error code that
    // should always be checked for VmbErrorSuccess
    
    
    cout << "Getting VimbaSystem singleton" << endl;
    VimbaSystem &sys = VimbaSystem :: GetInstance ();
    // A reference to the VimbaSystem singleton
    
    cout << "creating camera vector" << endl;
    CameraPtrVector cameras ;
    // A list of known cameras
    
    cout << "creating frame vector" << endl;
    FramePtrVector frames (15); // A list of frames for streaming . We chose
    // to queue 3 frames .
    
    //IFrameObserverPtr pObserver ( new FrameObserver ( cameras[0] ) );
    // Our implementation
    // of a frame observer
    
    FeaturePtr pFeature ;
    // Any camera feature
    
    VmbInt64_t nPLS;
    // The payload size of one frame

    cout << "Starting up system" << endl;
    err = sys.Startup ();

    
    
    cout << "Getting available cameras" << endl;
    err = sys.GetCameras( cameras );
    
    cout << "Opening camera" << endl;
    err = cameras[0]->Open( VmbAccessModeFull );
        
    std::string Name;
    if( VmbErrorSuccess == cameras[0]->GetName( Name ))
        cout << Name << endl;
    else
        cout << "Not recognizing camera" << endl;

    
    //Creating the Frame observer pointer with the custom object (and the camera)
    IFrameObserverPtr pObserver(new FrameObserver (cameras[0]) );    
    

    cout << "Getting buffer" << endl;
    
    err = cameras[0]-> GetFeatureByName ( "PayloadSize", pFeature );
    err = pFeature->GetValue( nPLS );


    
    for (FramePtrVector :: iterator iter = frames . begin (); frames .end () != iter; ++ iter )
    {
        cout << "Allocating and announcing frames "<< endl;
        ( *iter ). reset( new Frame ( nPLS ) );
        err = ( *iter )-> RegisterObserver ( pObserver );//IFrameObserverPtr (new FrameObserver ( cameras[0] ))
        err = cameras [0]-> AnnounceFrame ( *iter );
        
    }
    
    
    //////Setting up some features 
    
    bool writable1, writable2, readable1, val1;
    VmbInt64_t H, W;
    
    /*err = cameras[0]->GetFeatureByName ( "BinningVerticalMode", pFeature );
    err = pFeature -> SetValue("Average");
    err = pFeature -> IsWritable(writable1);
    cout << "Writable? : "<<  writable1 << endl;*/
    
    /*err = cameras[0]->GetFeatureByName ( "BinningVertical", pFeature );
    err = pFeature -> SetValue(2); //dividing by 2 the original vertical resolution (the horizontal one is then cut in half automatically, giving a final 4 times smaller image)
    err = pFeature -> IsWritable(writable2);
    err = pFeature->GetValue (val1);
    cout << "Binning V value : " << val1 << "\t" << endl;
    cout << "Writable? : "<<  writable2 << endl;*/
    
    /*err = cameras[0]->GetFeatureByName ( "BinningHorizontal", pFeature );
    //err = pFeature -> SetValue(2); //dividing by 2 the original vertical resolution (the horizontal one is then cut in half automatically, giving a final 4 times smaller image)
    err = pFeature -> IsWritable(writable2);
    err = pFeature -> IsReadable(readable1);
    err = pFeature -> GetValue (val1);
    cout << "Binning H value : " << val1 << "\t" << endl;
    cout << "Writable? : "<<  writable2 << endl;
    cout << "Readable? : "<<  readable1 << endl;*/

   /* err = cameras[0]->GetFeatureByName ( "Height", pFeature );
    err = pFeature -> SetValue(1080); //set the wanted height
    err = pFeature -> GetValue(H);
   // err = pFeature -> IsWritable(writable1);
    
    err = cameras[0]->GetFeatureByName ( "Width", pFeature );
    err = pFeature -> SetValue(1920); //set the wanted width
    err = pFeature -> GetValue(W); 
    cout << "Img resolution : " << W << "x" << H << endl;*/
     
    /*double framerate, min, max;
    bool framerateEnable, writable, Readable;
    bool available1, available2, available3;
    StringVector ExposureMode;
    
    //err = cameras[0]->GetFeatureByName ( "AcquisitionMode", pFeature );
    //err = pFeature -> SetValue("Continuous");
    
    err = cameras[0]->GetFeatureByName ( "AcquisitionFrameRateEnable", pFeature );
    
    err = pFeature -> IsWritable(writable);//SetValue(true);
    err = pFeature -> IsReadable(Readable);//SetValue(true);
    err = pFeature -> GetValue(framerateEnable);

    cout << "Enable custom framerate : " << framerateEnable  << endl;
    cout << "Customizable : " << writable << endl;
    cout << "Readable : " << Readable << endl;
    
    err = cameras[0]->GetFeatureByName ( "AcquisitionFrameRate", pFeature );
    err = pFeature -> GetValue(framerate);
    err = pFeature -> GetRange( min, max );
    
    cout << "Default Frame rate : " << framerate << endl;
    cout << "(min : " << min << " / max : " << max << " )" << endl;
    //err = pFeature -> SetValue(30.0);
    err = pFeature -> GetValue(framerate);
    cout << "New Frame rate : " << framerate << endl;*/
    
    /*err = cameras[0]->GetFeatureByName( "ExposureAuto", pFeature );
    err = pFeature -> GetValues(ExposureMode);
    err = pFeature -> IsValueAvailable( "Off", available1 );
    err = pFeature -> IsValueAvailable( "Once", available2 );
    err = pFeature -> IsValueAvailable( "Continuous", available3 );
    cout << "Exposure Auto available : "<< available1 << "\t" << available2 << "\t" << available3 << endl;
    cout << "Exposure Auto : "<< ExposureMode.at(0) << "\t" << ExposureMode.at(1) << "\t" << ExposureMode.at(2) << endl;;*/
          
            
    /*err = cameras[0]->GetFeatureByName ( "IntensityControllerTarget", pFeature );
    err = pFeature -> SetValue(20.0f);
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure auto" << endl;
        return err;
    }*/
    
    /*err = cameras[0]->GetFeatureByName ( "ExposureAuto", pFeature );
    err = pFeature -> SetValue("Continuous");
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure auto" << endl;
        return err;
    }*/
    /*double framerate, min, max;
    err = cameras[0]->GetFeatureByName ( "Gain", pFeature );
    err = pFeature -> SetValue(20.0f);
    err = pFeature -> GetRange( min, max );
    cout << "(min : " << min << " / max : " << max << " )" << endl;
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure time" << endl;
        return err;
    }*/
    
    /*err = cameras[0]->GetFeatureByName ( "ExposureTime", pFeature );
    err = pFeature -> SetValue(5000.0f);
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure time" << endl;
        return err;
    }*/
    
    
    
    
   /* err = cameras[0]->GetFeatureByName ( "GainAuto", pFeature );
    err = pFeature -> SetValue("Continuous");
    if(VmbErrorSuccess != err){
        cout << "Could not set gain auto" << endl;
        return err;
    }*/
    
////////

    err = cameras [0]-> StartCapture ();
    cout << "Starting capture" << endl;

    for (FramePtrVector :: iterator iter = frames . begin (); frames .end () != iter;   ++ iter )
    {
        cout << "Queueing frames" << endl;
        err = cameras[0]-> QueueFrame ( *iter);
    }
    

    
    
    err = cameras[0]->GetFeatureByName ( "AcquisitionStart", pFeature );//Starting image acquisition
    if(err != VmbErrorSuccess){
        cout << "Couldn't Start Image acquisition !" << endl;
    }
    err = pFeature -> RunCommand ();
    if(err != VmbErrorSuccess){
        cout << "Couldn't Start Image acquisition !" << endl;
    }
///////////////



    cout  << "Starting program" << endl;
 
    // Program runtime ...
    
    
    
    int key = 0;
    while(key != 27){
    
        key = cv::waitKey(1) & 0xFF;
        
        if(StartReceiving){   
            //cout << "Avant affichage" << endl;
            //getting the image with the right colors in originalImage matrix
            //cv::cvtColor(raw_frame, originalImage, cv::); // or COLOR_YUV2BGR_YUYV
            
            //Ipl->imageData = (char *)ptr_receive;
            //originalImage = cv::cvarrToMat(	Ipl, true); //converting ipl image to cv::Mat (with data copy)
            // = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);
            
            ExposureTime = cv::getTrackbarPos("Exposure time", "Trackbar");
            
            /*double framerate, min, max;
            err = cameras[0]->GetFeatureByName ( "Gain", pFeature );
            err = pFeature -> SetValue(20.0f);
            err = pFeature -> GetRange( min, max );
            cout << "(min : " << min << " / max : " << max << " )" << endl;
            if(VmbErrorSuccess != err){
                cout << "Could not set exposure time" << endl;
                return err;
            }*/
            
            err = cameras[0]->GetFeatureByName ( "ExposureTime", pFeature );
            err = pFeature -> SetValue(ExposureTime);
            if(VmbErrorSuccess != err){
                cout << "Could not set exposure time" << endl;
                return err;
            }/**/
    
            //Show each image
            
            cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC1);
            //resize down for printing the image
            cv::resize(raw_frame, resized_down, resized_down.size(), 0, 0, cv::INTER_LINEAR);
            
            // Send each image out to ffmpeg, in order to stream the video
            /*string out((char*)resized_down.data, resized_down.total() * resized_down.elemSize());
            cout << out; */
            
            cv::imshow("Img", resized_down);
            //cout << "Apres affichage" << endl;
            
        }
    }   
    
    
    cout << "Stopping program"<< endl;
    // When finished , tear down the acquisition chain , close the camera and Vimba

    err = cameras[0]->GetFeatureByName ( "AcquisitionStop", pFeature );//Stopping image acquisition
    err = pFeature -> RunCommand ();
    err = cameras[0]-> EndCapture ();
    err = cameras[0]-> FlushQueue ();
    err = cameras[0]-> RevokeAllFrames ();
    
    for( FramePtrVector :: iterator iter= frames .begin (); frames .end ()!= iter; ++ iter)
    {
        // Unregister the frame observer / callback
        (* iter)-> UnregisterObserver ();
    }

    err = cameras[0]-> Close ();
    err = sys. Shutdown ();

    cout << "STOP" << endl;
    
    return err;
    
}
