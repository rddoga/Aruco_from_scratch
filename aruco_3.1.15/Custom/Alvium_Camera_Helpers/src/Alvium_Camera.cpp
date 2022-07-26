#include "Alvium_Camera.h"

#include <QTime>


using namespace std;
using namespace AVT::VmbAPI;

//std::mutex mtx;           // mutex for critical section
//pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//////////////////Global variables
//bool StartReceiving = false;
VmbUchar_t* ptr_raw_frame = NULL; //Pointer that will be catching the frame
int img_count = 0;          //For FPS count



///////////////////////

VmbErrorType err;
// Every Vimba function returns an error code that
// should always be checked for VmbErrorSuccess


//cout << "Getting VimbaSystem singleton" << endl;
VimbaSystem &sys = VimbaSystem :: GetInstance ();
// A reference to the VimbaSystem singleton

//cout << "creating camera vector" << endl;
CameraPtrVector cameras ;
// A list of known cameras

//cout << "creating frame vector" << endl;
FramePtrVector frames (6); // A list of frames for streaming . We chose
// to queue 3 frames .


FeaturePtr pFeature ;
// Any camera feature

VmbInt64_t nPLS;
// The payload size of one frame


/////////////Functions for the fps counting class

//Constructor
FPSCounter::FPSCounter() : m_IsRunning( false ), m_LastFrameID( 0 ) , m_Valid( false )
    {
        m_Timer = new QTime;
    }
    

void FPSCounter::count( VmbInt64_t id )
    {
        if( ! m_IsRunning )
        {
            m_LastFrameID   = id;
            m_IsRunning     = true;
            m_Valid         = false;
            m_Timer->start();
            return;
        }
        double      time_ms     = m_Timer->elapsed();
        VmbInt64_t  delta_id = id - m_LastFrameID;
        if( (time_ms > 1000  && delta_id != 0) )
        {
            m_CurrentFPS    = (delta_id*1000) / time_ms;
            m_LastFrameID   = id;
            m_Valid         = true;
            m_Timer->start();
        }
    }
  
  //Destuctor (needed for the QTime pointer)
FPSCounter::~FPSCounter()
    {
        delete m_Timer;
    }



///////////////////////////FRAME OBSERVER CLASS

FrameObserver::FrameObserver ( CameraPtr pCamera ) : IFrameObserver ( pCamera )
{
    // Put your initialization code here
}
    
void FrameObserver::FrameReceived ( const FramePtr pFrame ) //Method that gets called at every received frame
{
    VmbFrameStatusType eReceiveStatus;
    if( VmbErrorSuccess == pFrame -> GetReceiveStatus ( eReceiveStatus ) )
    {
        if ( VmbFrameStatusComplete == eReceiveStatus )
        {
            //StartReceiving = true; //Start the display when we start receiving
            
            //For counting the FPS
            //total_time += ((float)clock() - start)/CLOCKS_PER_SEC;
            //start = clock();
            
            //cout << "Frame received successfully !!" << endl;
            //ptr_raw_frame = new VmbUchar_t();
            //cout << "image before mutex" <<endl;
            //pthread_mutex_lock(&mutex); //Lock the Resources 
            
            pFrame->GetImage(ptr_raw_frame); //Getting raw image
            //cout << "IMAGE" <<endl;
            //pthread_mutex_unlock(&mutex); //Unlock the resources
            //cout << "image after mutex" <<endl;
            //cout << ptr_raw_frame;
            
            unsigned int width, height;
            pFrame->GetHeight(height);
            pFrame->GetWidth(width);

            //cout << width << " x " << height << endl;/**/
            
            img_count++;
            m_FPSReceived.count( img_count );
            VmbUint64_t camera_frame_id;
            pFrame->GetFrameID( camera_frame_id );
            m_FPSCamera.count( camera_frame_id ); 
    
            if( m_FPSReceived.isValid() )
            {
                if(img_count%30 == 0)// Printing current fps
                {
                    Print_FPS();
                }
            }/**/
            
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


void FrameObserver::Print_FPS() //method for FPS print
{
    cout << "Received : " << m_FPSReceived.CurrentFPS() << " FPS" << endl;
    cout << "Camera : " << m_FPSCamera.CurrentFPS() << " FPS" << endl;
}

//float FrameObserver::total_time = 0.0f, FrameObserver::start = clock(); //For FPS count






//////////////////////////////OTHER FUNCTIONS



///For Startup (Find camera, open camera, reading buffer size...) and starting acquisition
VmbErrorType Open_and_Start_Acquisition()
{
 
    /////////

    cout << "Starting up system" << endl;
    err = sys.Startup ();
    if(err != VmbErrorSuccess)
        return err;

    cout << "Getting available cameras" << endl;
    err = sys.GetCameras( cameras );
    if(err != VmbErrorSuccess)
        return err;
    
    cout << "Opening camera" << endl;
    err = cameras[0]->Open( VmbAccessModeFull );
    if(err != VmbErrorSuccess)
        return err;
        
    std::string Name;
    if( VmbErrorSuccess == cameras[0]->GetName( Name )){
        cout << Name << endl;
    }else{
        cout << "Not recognizing camera" << endl;
        return err;
    }
    
    //Creating the Frame observer pointer with the custom object (and the camera)
    IFrameObserverPtr pObserver(new FrameObserver (cameras[0]) );    
        
    
    cout << "Getting buffer" << endl;
    
    err = cameras[0]-> GetFeatureByName ( "PayloadSize", pFeature );
    err = pFeature->GetValue( nPLS );
    if(err != VmbErrorSuccess)
        return err;


    
    for (FramePtrVector :: iterator iter = frames . begin (); frames .end () != iter; ++ iter )
    {
        cout << "Allocating and announcing frames "<< endl;
        ( *iter ). reset( new Frame ( nPLS ) );
        err = ( *iter )-> RegisterObserver ( pObserver );
        if(err != VmbErrorSuccess)
            return err;
            
        err = cameras [0]-> AnnounceFrame ( *iter );
        if(err != VmbErrorSuccess)
            return err;
        
    }
    
    ////////Setting up some features
    VmbInt64_t H, W, val;
    bool writable1, writable2;
    //For printing readouts
    ///Reducing resolution
    

    
    
    /*err = cameras[0]->GetFeatureByName ( "Height", pFeature );
    err = pFeature -> SetValue(1080); //set the wanted height (/!\ NE RESIZE PAS L IMAGE, PREND JUSTE LE NOMBRE DE PIXELS VOULUS)
    err = pFeature -> GetValue(H);
   // err = pFeature -> IsWritable(writable1);
    
    err = cameras[0]->GetFeatureByName ( "Width", pFeature );
    err = pFeature -> SetValue(1920); //set the wanted width (/!\ NE RESIZE PAS L IMAGE, PREND JUSTE LE NOMBRE DE PIXELS VOULUS)
    err = pFeature -> GetValue(W); 
  //  err = pFeature -> IsWritable(writable2);*/
    
   // cout << "Img resolution : " << W << "x" << H << endl;
    
    
    //Fixing exposure time and gain (for higher framerates)
   
    /*err = cameras[0]->GetFeatureByName ( "ExposureMode", pFeature );
    err = pFeature -> SetValue("Timed");
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure mode" << endl;
        return err;
    }*/
    
    /*err = cameras[0]->GetFeatureByName ( "IntensityControllerTarget", pFeature );
    err = pFeature -> SetValue(40.0f);
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure auto" << endl;
        return err;
    }*/
    
    /*err = cameras[0]->GetFeatureByName ( "GainAuto", pFeature );
    err = pFeature -> SetValue("Continuous");
    if(VmbErrorSuccess != err){
        cout << "Could not set auto gain" << endl;
        return err;
    } */
    
    /*err = cameras[0]->GetFeatureByName ( "ExposureAuto", pFeature );
    err = pFeature -> SetValue("Continuous");
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure auto" << endl;
        return err;
    }*/
    
    /*err = cameras[0]->GetFeatureByName ( "ExposureTime", pFeature );
    err = pFeature -> SetValue(168.0f);
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure time" << endl;
        return err;
    } */
    
    /*err = cameras[0]->GetFeatureByName ( "LineStatusAll", pFeature );
    err = pFeature -> GetValue(val);
    cout << val << endl;
    if(VmbErrorSuccess != err){
        cout << "Could not get lines status" << endl;
        return err;
    } */
    
   /* err = cameras[0]->GetFeatureByName ( "ContrastEnable", pFeature );
    //err = pFeature -> IsWritable(writable1);
    //cout << writable1 << endl;
    err = pFeature -> SetValue(true);
    if(VmbErrorSuccess != err){
        cout << "Could not set contrast" << endl;
        return err;
    }
    
    err = cameras[0]->GetFeatureByName ( "Sharpness", pFeature );
    //err = pFeature -> IsWritable(writable1);
    //cout << writable1 << endl;
    err = pFeature -> SetValue(8);
    if(VmbErrorSuccess != err){
        cout << "Could not set sharpness" << endl;
        return err;
    }*/
    
    
    /*bool available1, available2, available3;
    err = cameras[0]->GetFeatureByName ( "SensorBitDepth", pFeature );
    err = pFeature -> IsValueAvailable( "Bpp8", available1 );
    err = pFeature -> IsValueAvailable( "Bpp10", available2 );
    err = pFeature -> IsValueAvailable( "Bpp12", available3 );
    cout << "Sensorbit depth available : "<< available1 << "\t" << available2 << "\t" << available3 << endl;
    err = pFeature -> IsWritable( available1 );
    cout << "writable : " << available1 << endl;*/
    
        
    
    /*err = pFeature -> SetValue("Bpp8");
    if(VmbErrorSuccess != err){
        cout << "Could not set SensorBitDepth" << endl;
        return err;
    }*/
    
    /*string whatever;
    err = cameras[0]->GetFeatureByName ("ExposureActiveMode", pFeature );
    err = pFeature -> GetValue(whatever);
    if(VmbErrorSuccess != err){
        cout << "Could not get exposure active mode " << endl;
        return err;
    }
    cout << whatever << endl;*/
    /**/err = cameras[0]->GetFeatureByName ( "BinningVerticalMode", pFeature );
    err = pFeature -> SetValue("Average"); //Summing/Averaging values when binning adjacent pixels
    err = pFeature -> IsWritable(writable1);
    
    err = cameras[0]->GetFeatureByName ( "BinningVertical", pFeature );
    err = pFeature -> SetValue(2); //dividing by 2 the original vertical resolution (the horizontal one is then divided automatically)
    err = pFeature -> IsWritable(writable2);
    cout << writable1 << "\t" << writable2 << endl;
    
    
    /**/double framerate, min, max;
    err = cameras[0]->GetFeatureByName ( "Gain", pFeature );
    err = pFeature -> SetValue(20.0f);
    err = pFeature -> GetRange( min, max );
    cout << "(min : " << min << " / max : " << max << " )" << endl;
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure time" << endl;
        return err;
    }
    
    err = cameras[0]->GetFeatureByName ( "ExposureTime", pFeature );
    err = pFeature -> SetValue(6000.0f);
    if(VmbErrorSuccess != err){
        cout << "Could not set exposure time" << endl;
        return err;
    }
    
    
    
    /*err = cameras[0]->GetFeatureByName ( "Height", pFeature );
    err = pFeature -> SetValue(1080); //set the wanted height
    err = pFeature -> GetValue(H);
   // err = pFeature -> IsWritable(writable1);
    
    err = cameras[0]->GetFeatureByName ( "Width", pFeature );
    err = pFeature -> SetValue(1920); //set the wanted width
    err = pFeature -> GetValue(W); 
    cout << "Img resolution : " << W << "x" << H << endl;*/
    
    
       
    
    ///////////////////////
    ///////////////////////
    cout << "Starting capture" << endl;
    err = cameras [0]-> StartCapture ();
    if(err != VmbErrorSuccess)
        return err;
    

    for (FramePtrVector :: iterator iter = frames . begin (); frames .end () != iter;   ++ iter )
    {
        cout << "Queueing frames" << endl;
        err = cameras[0]-> QueueFrame ( *iter);
        if(err != VmbErrorSuccess)
            return err;
    }
    
    
    ////////////
    err = cameras[0]->GetFeatureByName ( "AcquisitionStart", pFeature );//Starting image acquisition
    err = pFeature -> RunCommand ();
    if(err != VmbErrorSuccess)
        return err;
    
    
    cout  << "Starting program" << endl;
 
    return err;
    
}
    
     

///For Stopping camera acquisition, closing camera and Shutting down system
VmbErrorType Stop_Acquisition_and_Close()
{

    cout << "Stopping program" << endl;
    // When finished , tear down the acquisition chain , close the camera and Vimba

    err = cameras[0]->GetFeatureByName ( "AcquisitionStop", pFeature );//Stopping image acquisition
    err = pFeature -> RunCommand ();
    if(err != VmbErrorSuccess){
        cout << "Couldnt stop acquisition" << endl;
        return err;  
    }
        
    err = cameras[0]-> EndCapture ();
    if(err != VmbErrorSuccess){
        cout << "Couldnt end capture" << endl;
        return err;  
    }

        
    err = cameras[0]-> FlushQueue ();
    if(err != VmbErrorSuccess) {
        cout << "Couldnt flush queue" << endl;
        return err;  
    }
        
    err = cameras[0]-> RevokeAllFrames ();
    if(err != VmbErrorSuccess){
        cout << "Couldnt revoke frames" << endl;
        cout << err << endl;
        
        return err;  
    }
        
    
    for( FramePtrVector :: iterator iter= frames .begin (); frames .end ()!= iter; ++ iter)
    {
        // Unregister the frame observer / callback
        (* iter)-> UnregisterObserver ();
    }

    err = cameras[0]-> Close ();
    if(err != VmbErrorSuccess){
        cout << "Couldnt close camera" << endl;
        return err;  
    }
        
    err = sys. Shutdown ();
    if(err != VmbErrorSuccess){
        cout << "Couldnt shutdown system" << endl;
        return err;  
    }

    cout << "STOP" << endl;
    
    return err;

}
