#ifndef ENABLE_GL_DISPLAY
#define ENABLE_GL_DISPLAY
#endif

#ifndef ENABLE_GPU_UPLOAD
#define ENABLE_GPU_UPLOAD
#endif

#include "VimbaCPP/Include/VimbaCPP.h"
//#include "ApiController.h"
//#include "ProgramConfig.h"

//#include "v4l2_helper.h"

#include "aruco.h"
#include "calibrator.h"

#include "Alvium_Camera.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/core/core_c.h>
#include <opencv2/core/cuda.hpp>

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>



using namespace std;
using namespace AVT::VmbAPI;
using namespace cv;
using namespace aruco;

///////////////////////////


int imgSizeX = 2008, imgSizeY = 1518;
//frame pointer
cv::Mat raw_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);
//bool StartReceiving = false;
//int img_count;//For FPS count
//unsigned char* ptr_raw_frame; //For gettting the frame pointer

CameraParameters TheCameraParameters;
MarkerDetector TheMarkerDetector;
vector<vector<aruco::Marker>> allMarkers;
string TheOutCameraParams;
aruco::CameraParameters camp;  // camera parameters estimated
Calibrator calibrator;


aruco::CameraParameters cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr=0, aruco::MarkerMap *inmmap=0);

/************************************
 *
 *  FUNCTIONS / CLASSES
 *
 ************************************/
 
//For handling cam Startup/Shutdown errors
class Cam_Exception : public std::exception
{
    const char* cam_err;
public:

    Cam_Exception(const char* _cam_err) : cam_err(_cam_err) {}
    
    const char* what() const throw()
    {
        return cam_err;
    }
};



/////////////////////////
/************************************
 *
 * MAIN
 *
 ************************************/
 
 
int main(int argc, char* argv[])
{    

    try
    {
    
#if defined(ENABLE_GL_DISPLAY) && defined(ENABLE_GPU_UPLOAD)
            std::cout << "Using CUDA\n";
            cuda::GpuMat gpu_frame;
#else
            std::cout << "Not using CUDA\n";
#endif

#ifdef ENABLE_GL_DISPLAY
            std::cout << "Using openGL\n";
            namedWindow("in", cv::WINDOW_OPENGL);
#else
            std::cout << "Not using openGL\n";
            namedWindow("in", cv::WINDOW_NORMAL);
#endif/**/

        //Create and resize window
        //cv::namedWindow("Img", cv::WINDOW_NORMAL);
        cv::resizeWindow("in", 1920, 1080);
        
        
        //configure the calibrator
        calibrator.setParams(cv::Size(imgSizeX, imgSizeY), 0.04, "");
        
        /*aruco::MarkerDetector::Params &params= TheMarkerDetector.getParameters();
        
        params.cornerRefinementM = aruco::CORNER_LINES; //Corner refinement method
        params.maxThreads = -1; //Max threads used in parallel
        params.lowResMarkerSize = 5; //minimum size of a marker in the low resolution image
        params.NAttemptsAutoThresFix = 2; //number of times that tries a random threshold in case of THRES_AUTO_FIXED
        params.error_correction_rate = 1; */
        
        // set specific parameters for this configuration
        TheMarkerDetector.setDictionary("ARUCO_MIP_36h12");
        TheMarkerDetector.setDetectionMode(aruco::DM_NORMAL);
            
        int New_img_received = 0; //For checking when we receive a new image
            
        int cpt_img = 0; //For counting the real fps shown (not the fps of the camera)
            
        
        cout << "START" << endl;
        
        //Starting up system
        if(VmbErrorSuccess != Open_and_Start_Acquisition() ){
            throw Cam_Exception("Openning / Start Acquisition error (See Alvium_Camera.cpp file ) !");
        }
        
        /*
        
        //////Setting up some features 
        bool writable1, writable2, val1;
        VmbInt64_t H, W;
        
        /*err = cameras[0]->GetFeatureByName ( "BinningVerticalMode", pFeature );
        err = pFeature -> SetValue("Sum");
        //err = pFeature -> IsWritable(writable1);
        
        err = cameras[0]->GetFeatureByName ( "BinningVertical", pFeature );
        err = pFeature -> SetValue(2); //dividing by 2 the original vertical resolution (the horizontal one is then cut in half automatically, giving a final 4 times smaller image)
        //err = pFeature -> IsWritable(writable2);
        err = pFeature->GetValue (val1);
        cout << "Writable? : " << val1 << "\t" << endl;*/

        /*err = cameras[0]->GetFeatureByName ( "Height", pFeature );
       // err = pFeature -> SetValue(1080); //set the wanted height
        err = pFeature -> GetValue(H);
       // err = pFeature -> IsWritable(writable1);
        
        err = cameras[0]->GetFeatureByName ( "Width", pFeature );
       // err = pFeature -> SetValue(1920); //set the wanted width
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
        cout << "New Frame rate : " << framerate << endl;
        */
        //err = cameras[0]->GetFeatureByName ( "ExposureAuto", pFeature );
        //err = pFeature -> SetValue("Off");
        
        //err = pFeature -> IsValueAvailable( "Off", available1 );
        //err = pFeature -> IsValueAvailable( "Once", available2 );
        //err = pFeature -> IsValueAvailable( "Continuous", available3 );
        //cout << "Exposure Auto available : "<< available1 << "\t" << available2 << "\t" << available3 << endl;
        //cout << "Exposure Auto : "<< ExposureMode.at(0) << "\t" << ExposureMode.at(1) << "\t" << ExposureMode.at(2) << endl;
    ////////
        
        
        int key = 0;
        cv::TickMeter tm_full, tm_resize1, tm_resize2, tm_resize3, tm_detect, tm_end;
        
        while(key != 27){
        
            tm_full.start();
            
            tm_resize1.start();
            key = cv::waitKey(1) & 0xFF;
            
            if(!StartReceiving){//Check if we started receiving
                continue;
            }
             
            
            //Filling image data
            raw_frame.data = ptr_raw_frame;
            //cout << "Avant affichage" << endl;
            tm_resize1.stop();
            
            tm_resize3.start();
            
            cv::Mat imgtmp = cv::Mat(1080, 1920, CV_8UC1); 
                        
            //cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC3);
            //resize down for printing the image
            //cv::resize(raw_frame, imgtmp, imgtmp.size(), 0, 0, cv::INTER_NEAREST);
            cv::cvtColor(raw_frame, imgtmp, cv::COLOR_GRAY2BGR ); // for printing colored features on the image, and for better marker detection (because needs color image for input)
            
            tm_resize3.stop();
            
            tm_detect.start();
            //Detect markers
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(imgtmp);
            
            // draw help
            cv::putText(imgtmp, "'a' add current image for calibration", cv::Point(10,40), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(imgtmp, "'esc' save and quit", cv::Point(10,80), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(imgtmp, calibrator.getInfo(), cv::Point(10,120), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            
            // print markers from the board
            for (auto m: detected_markers)
                m.draw(imgtmp, Scalar(0, 0, 255), 1);
            
            
            if (key == 'a')
                calibrator.addView(detected_markers);
            tm_detect.stop();
            

            tm_end.start();  
            if(New_img_received == img_count){ //Check if a new image has been received
                tm_full.stop();
                tm_end.stop();
                continue;
            }
            //If so, update the image received checker and show the last image (from the frame pointer)
            New_img_received = img_count;
            
            
            cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC3);
            //resize down for printing the image
            cv::resize(imgtmp, resized_down, imgtmp.size(), 0, 0, cv::INTER_NEAREST);
            
#if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
		    gpu_frame.upload(resized_down);
		    imshow("in", gpu_frame);
#else
		    imshow("in", resized_down);
#endif

            //cv::imshow("Img", resized_down);
            //cout << "Apres affichage" << endl;
            

            //cout << "Comp time = " << (float)(stop - start)/CLOCKS_PER_SEC << " s" << endl;
            
            cpt_img++;
            tm_end.stop();
            tm_full.stop();
        }   
        
        cout << "Stopping program" << endl;
        // When finished , tear down the acquisition chain , close the camera and Vimba

        aruco::CameraParameters camp;
        if (calibrator.getCalibrationResults(camp) && calibrator.getNumberOfViews() >= 8)
        {
            camp.saveToFile("../cam_calibration_new.yml");
            cout << "results saved to cam_calibration_new.yml\n";
        }
        else
            cerr << "Could not obtain calibration\n";
                

        if(VmbErrorSuccess != Stop_Acquisition_and_Close()){
            throw  Cam_Exception("Closing / Stop Acquisition error (See Alvium_Camera.cpp file ) !");        
        }      
        

        cout << "Average image resizing time 1: " << tm_resize1.getAvgTimeSec()*1000 << " ms" << endl;
        //cout << "Average image resizing time 2: " << tm_resize2.getAvgTimeSec()*1000 << " ms" << endl;
        cout << "Average image resizing time 3: " << tm_resize3.getAvgTimeSec()*1000 << " ms" << endl;
        cout << "Average detection time: " << tm_detect.getAvgTimeSec()*1000 << " ms" << endl;
        cout << "Average end computation time: " << tm_end.getAvgTimeSec()*1000 << " ms" << endl;
        cout << "Average full computation time: " << tm_full.getAvgTimeSec()*1000 << " ms" << endl;
        //cout << "Average FPS: " << tm.getFPS() << endl;

        cout << "STOP" << endl;
        
    }
    catch(std::exception& ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
    
}
