#include "Alvium_Camera.h"

using namespace std;
using namespace AVT::VmbAPI;


//////////////////Global variables
bool StartReceiving = false;
unsigned char* ptr_raw_frame;
int img_count = 0;          //For FPS count

//cout << "Hello" << endl;
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
FramePtrVector frames (10); // A list of frames for streaming . We chose
// to queue 3 frames .


FeaturePtr pFeature ;
// Any camera feature

VmbInt64_t nPLS;
// The payload size of one frame



    
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
            StartReceiving = true; //Start the display when we start receiving
            
            //For counting the FPS
            total_time += ((float)clock() - start)/CLOCKS_PER_SEC;
            start = clock();
            
            cout << "Frame received successfully !!" << endl;
            pFrame->GetImage(ptr_raw_frame); //Getting raw image
            
            /*unsigned int width, height;
            pFrame->GetHeight(height);
            pFrame->GetWidth(width);

            cout << width << " x " << height << endl;*/
            img_count++;
            
            
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


void FrameObserver::Print_FPS() //static method for FPS print
{
    cout << "Average FPS : " << img_count/total_time << " FPS" << endl;
    img_count = 0;
    total_time = 0.0f;
}

float FrameObserver::total_time = 0.0f, FrameObserver::start = clock(); //For FPS count






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
        
    
    cout << "Getting buffer" << endl;
    
    err = cameras[0]-> GetFeatureByName ( "PayloadSize", pFeature );
    err = pFeature->GetValue( nPLS );
    if(err != VmbErrorSuccess)
        return err;


    
    for (FramePtrVector :: iterator iter = frames . begin (); frames .end () != iter; ++ iter )
    {
        cout << "Allocating and announcing frames "<< endl;
        ( *iter ). reset( new Frame ( nPLS ) );
        err = ( *iter )-> RegisterObserver ( IFrameObserverPtr (new FrameObserver ( cameras[0] )) );
        if(err != VmbErrorSuccess)
            return err;
            
        err = cameras [0]-> AnnounceFrame ( *iter );
        if(err != VmbErrorSuccess)
            return err;
        
    }
    
    ////////Setting up some features
    VmbInt64_t H, W;
    bool writable1, writable2;
    
    ///Reducing resolution
    
   /* err = cameras[0]->GetFeatureByName ( "BinningVerticalMode", pFeature );
    err = pFeature -> SetValue("Sum");
    err = pFeature -> IsWritable(writable1);
    
    err = cameras[0]->GetFeatureByName ( "BinningVertical", pFeature );
    err = pFeature -> SetValue(2); //dividing by 2 the original vertical resolution (the horizontal one is then divided automatically)
    err = pFeature -> IsWritable(writable2);
    
    
    err = cameras[0]->GetFeatureByName ( "Height", pFeature );
    err = pFeature -> SetValue(1080); //set the wanted height (/!\ NE RESIZE PAS L IMAGE, PREND JUSTE LE NOMBRE DE PIXELS VOULUS)
    err = pFeature -> GetValue(H);
   // err = pFeature -> IsWritable(writable1);
    
    err = cameras[0]->GetFeatureByName ( "Width", pFeature );
    err = pFeature -> SetValue(1920); //set the wanted width (/!\ NE RESIZE PAS L IMAGE, PREND JUSTE LE NOMBRE DE PIXELS VOULUS)
    err = pFeature -> GetValue(W); 
  //  err = pFeature -> IsWritable(writable2);*/
    
    cout << "Img resolution : " << W << "x" << H << endl;
    
    ////////
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

    cout << "Stopping program"<< endl;
    // When finished , tear down the acquisition chain , close the camera and Vimba

    err = cameras[0]->GetFeatureByName ( "AcquisitionStop", pFeature );//Stopping image acquisition
    err = pFeature -> RunCommand ();
    if(err != VmbErrorSuccess)
        return err;
        
    err = cameras[0]-> EndCapture ();
    if(err != VmbErrorSuccess)
        return err;
        
    err = cameras[0]-> FlushQueue ();
    if(err != VmbErrorSuccess)
        return err;
        
    err = cameras[0]-> RevokeAllFrames ();
    if(err != VmbErrorSuccess)
        return err;
        
    
    for( FramePtrVector :: iterator iter= frames .begin (); frames .end ()!= iter; ++ iter)
    {
        // Unregister the frame observer / callback
        (* iter)-> UnregisterObserver ();
    }

    err = cameras[0]-> Close ();
    if(err != VmbErrorSuccess)
        return err;
        
    err = sys. Shutdown ();
    if(err != VmbErrorSuccess)
        return err;

    cout << "STOP" << endl;
    
    return err;

}