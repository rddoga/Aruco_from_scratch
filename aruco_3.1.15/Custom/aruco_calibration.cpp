/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

// to be commented depending on system

#ifndef ENABLE_GL_DISPLAY
#define ENABLE_GL_DISPLAY
#endif

#ifndef ENABLE_GPU_UPLOAD
#define ENABLE_GPU_UPLOAD
#endif

//#define OLD_CAMERA 1 //For knowing if we are using the old or the new camera (COMMENT IF USING NEW CAMERA)

#ifdef OLD_CAMERA
#include "v4l2_helper.h"

#else
#include "Alvium_Camera.h"

#endif

#include "aruco.h"
#include "calibrator.h"


#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <sstream>
#include <thread>
#include <mutex>

using namespace std;
using namespace cv;
using namespace aruco;


#ifdef OLD_CAMERA
unsigned int imgSizeX = 1920; // x pixel image size
unsigned int imgSizeY = 1080; // y pixel image size
#else
unsigned int imgSizeX = 4024; // x pixel image size
unsigned int imgSizeY = 3036; // y pixel image size
#endif

CameraParameters TheCameraParameters;
MarkerDetector TheMarkerDetector;
vector<vector<aruco::Marker>> allMarkers;
string TheOutCameraParams;
aruco::CameraParameters camp;  // camera parameters estimated
Calibrator calibrator;

#ifdef OLD_CAMERA
cv::Mat yuyv_frame(imgSizeY, imgSizeX, CV_8UC2);    // raw image
cv::Mat originalImage(imgSizeX, imgSizeY, CV_8UC3); // converted image
cv::Mat image(imgSizeX, imgSizeY, CV_8UC3);         // saved image (old camera)
#else
cv::Mat raw_frame(imgSizeY, imgSizeX, CV_8UC1);    // raw image
cv::Mat originalImage(imgSizeX, imgSizeY, CV_8UC1); // copied image
#endif

bool exiting = false;

//aruco::CameraParameters cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr=0, aruco::MarkerMap *inmmap=0);

/************************************
 *
 *  FUNCTIONS
 *
 ************************************/

#ifdef OLD_CAMERA
void FrameGetter()
{
    /*TimerAvrg timerGetImage;
    TimerAvrg timerConv;*/

    unsigned char* ptr_cam_frame;
    int bytes_used;

    while(!exiting)
    {
        //timerGetImage.start();

        if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0)
        {
            fprintf(stderr, "Failed to get image : %m\n");
            break;
        }
        yuyv_frame.data = ptr_cam_frame;
        cv::cvtColor(yuyv_frame, originalImage, cv::COLOR_YUV2BGR_UYVY); // or COLOR_YUV2BGR_YUYV
        if (helper_release_cam_frame() < 0)
        {
            fprintf(stderr, "Failed to release image : %m\n");
            break;
        }

        //timerGetImage.stop();
    }

    // exiting program
    if (helper_deinit_cam() < 0)
        fprintf(stderr, "Failed to deinitialise camera : %m\n");

    //std::cout << "Average image catch time : "<< timerGetImage.getAvrg() * 1000 << " ms /!\\ keep in mind this is multithreaded\n";
}
#endif

/************************************
 *
 * MAIN
 *
 ************************************/

int main(int argc, char** argv)
{
    try
    {
        std::cout << "Aruco calibration program using CUDA acceleration and openGL when available\n";

        if (argc > 1)
        {
            cerr << "Usage: no argument required\n";
            return -1;
        }

#ifdef OLD_CAMERA
        if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            fprintf(stderr, "Failed to open video : %m\n");
            return -1;
        }
#endif


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
#endif

        cv::resizeWindow("in", 1920, 1080);

        //configure the calibrator
        calibrator.setParams(cv::Size(imgSizeX, imgSizeY), 0.04, "");

        // set specific parameters for this configuration
        TheMarkerDetector.setDictionary("ARUCO_MIP_36h12");
        TheMarkerDetector.setDetectionMode(aruco::DM_NORMAL);

        /*TimerAvrg timerFull;
        TimerAvrg timerDetect;
        TimerAvrg timerDisplay;*/

        char key = 0;
        int waitKeyTime = 1;
	  
#ifdef OLD_CAMERA 
	// capture until press ESC
        std::thread t1(FrameGetter);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
#else          
        //NEW CAMERA
        //Startup and start image acquisition
        if(VmbErrorSuccess != Open_and_Start_Acquisition() ){
            //throw Cam_Exception("Openning / Start Acquisition error !");
            cout << "Openning / Start Acquisition error !" << endl;
            return -1;
        }
#endif   

        cv::TickMeter tm_full, tm_resize1, tm_resize2, tm_resize3, tm_detect, tm_end;
        
        while (key != 27)
        {
            tm_full.start();
            
            //timerFull.start();
            key = cv::waitKey(waitKeyTime);   // wait for key to be pressed
            
            
            
#ifdef OLD_CAMERA
            // get frame
            image = originalImage.clone();
            
            tm_detect.start();
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(image);
            tm_detect.stop();
            
            // print markers from the board
            //timerDisplay.start();
            for (auto m: detected_markers)
                m.draw(image, Scalar(0, 0, 255), 1);

            
            // draw help
            cv::putText(image, "'a' add current image for calibration", cv::Point(10,40), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(image, "'esc' save and quit", cv::Point(10,80), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(image, calibrator.getInfo(), cv::Point(10,120), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            
    #if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
		        gpu_frame.upload(image);
		        imshow("in", gpu_frame);
    #else
		        imshow("in", image);
    #endif
    
#else //IF new camera


            if(ptr_raw_frame == NULL){//Check if we started receiving
                continue;
            }
            
            //Filling image data
            raw_frame.data = ptr_raw_frame;
            originalImage = raw_frame.clone();
            
            cv::Mat imgtmp = cv::Mat(3036, 4024, CV_8UC1);
            // for printing colored features on the image, and for better marker detection (because needs color image for input)
            cv::cvtColor(originalImage, imgtmp, cv::COLOR_GRAY2BGR ); 
            
            cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC3);
            //resize down for printing the image
            cv::resize(imgtmp, resized_down, resized_down.size(), 0, 0, cv::INTER_NEAREST);
            
            // detect
            tm_detect.start();
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(resized_down);
            tm_detect.stop();
            
            // print markers from the board
            //timerDisplay.start();
            for (auto m: detected_markers)
                m.draw(resized_down, Scalar(0, 0, 255), 1);

            
            // draw help
            cv::putText(resized_down, "'a' add current image for calibration", cv::Point(10,40), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(resized_down, "'esc' save and quit", cv::Point(10,80), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(resized_down, calibrator.getInfo(), cv::Point(10,120), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);

            
    #if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
		        gpu_frame.upload(resized_down);
		        imshow("in", gpu_frame);
    #else
		        imshow("in", resized_down);
    #endif
      
#endif

            //timerDisplay.stop();

            
            if (key == 'a')
                calibrator.addView(detected_markers);

            tm_full.stop();
            
        } ////////////END OF WHILE LOOP

        aruco::CameraParameters camp;
        if (calibrator.getCalibrationResults(camp))
        {
            camp.saveToFile("../cam_calibration_3.yml");
            cout << "results saved to cam_calibration.yml\n";
        }
        else
            cerr << "Could not obtain calibration\n";
	        
	    cv::destroyAllWindows();

        std::cout << "\n";
        cout << "Final error= " << calibrator.getReprjError() << "\n";

#ifdef  OLD_CAMERA        
        exiting = true;
        t1.join();
#else           
        //NEW CAMERA
        //Stop image acquisition and shutdown camera and API
        if(VmbErrorSuccess != Stop_Acquisition_and_Close() ){
            //throw Cam_Exception("Closing / Stop Acquisition error !");/**/
            cout << "Closing / Stop Acquisition error !" << endl;
            return -1;
        }
#endif  

        cout << "Average detection time: " << tm_detect.getAvgTimeSec()*1000 << " ms" << endl;
        //cout << "Average end computation time: " << tm_end.getAvgTimeSec()*1000 << " ms" << endl;
        cout << "Average full computation time: " << tm_full.getAvgTimeSec()*1000 << " ms" << endl;

    }
    catch (std::exception& ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}

