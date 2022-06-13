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

/*#ifndef ENABLE_GL_DISPLAY
#define ENABLE_GL_DISPLAY
#endif

#ifndef ENABLE_GPU_UPLOAD
#define ENABLE_GPU_UPLOAD
#endif*/


#include "aruco.h"
#include "calibrator.h"

#include "Alvium_Camera.h"

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

///Variables from the camera helpers files
//unsigned char* ptr_raw_frame;
//bool StartReceiving;
//int img_count;          //For FPS count

CameraParameters TheCameraParameters;
MarkerDetector TheMarkerDetector;
vector<vector<aruco::Marker>> allMarkers;
string TheOutCameraParams;
aruco::CameraParameters camp;  // camera parameters estimated
Calibrator calibrator;
unsigned int imgSizeX = 4024; // x pixel image size
unsigned int imgSizeY = 3036; // y pixel image size
//cv::Mat yuyv_frame(imgSizeY, imgSizeX, CV_8UC2);    // raw image
//cv::Mat originalImage(imgSizeX, imgSizeY, CV_8UC3); // converted image
cv::Mat image(imgSizeY, imgSizeX, CV_8UC1);         // saved image
bool exiting = false;

aruco::CameraParameters cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr=0, aruco::MarkerMap *inmmap=0);

/************************************
 *
 *  FUNCTIONS
 *
 ************************************/


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
        

        //Image that gets data from the frame pointer
        cv::Mat raw_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);
        
            
        //Startup and start image acquisition
        VmbErrorType err = Open_and_Start_Acquisition();
        if(err != VmbErrorSuccess){
            throw "Openning / Start Acquisition error (See Alvium_Camera.cpp file ) !";
        }
        
        int New_img_received = 0; //For checking when we receive a new image
        
        int cpt_img = 0; //For counting the real fps shown (not the fps of the camera)

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

        //cv::resizeWindow("in", 1920, 1080);
    
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
	    
	// capture until press ESC
        //std::thread t1(FrameGetter);
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        clock_t start, stop;
        
        start = clock();
        while (key != 27)
        {
            //timerFull.start();
            image.data = ptr_raw_frame;
            // get frame
            //raw_frame.copyTo(image);
            
            if(!StartReceiving){ //Check if we started receiving
                continue;
            }
            
            cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC1);
            //resize down
            cv::resize(image, resized_down, resized_down.size(), 0, 0, cv::INTER_LINEAR);
            
            if(FrameObserver::total_time > 1.0f)
            {
                FrameObserver::Print_FPS();
            }
            
            
            // detect
            //timerDetect.start();
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(resized_down);
            //timerDetect.stop();
            
            // print markers from the board
            //timerDisplay.start();
            for (auto m: detected_markers)
                m.draw(resized_down, Scalar(0, 0, 255), 1);
            // draw help
            cv::putText(resized_down, "'a' add current image for calibration", cv::Point(10,40), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(resized_down, "'esc' save and quit", cv::Point(10,80), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(resized_down, calibrator.getInfo(), cv::Point(10,120), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
/*
#if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
		    gpu_frame.upload(image);
		    imshow("in", gpu_frame);
#else*/

//#endif

            key = cv::waitKey(waitKeyTime);   // wait for key to be pressed
            if (key == 'a')
                calibrator.addView(detected_markers);
                
           
            if(New_img_received == img_count){ //Check if a new image has been received
                continue;
            }
            //If so, update the image received checker and show the last image (from the frame pointer)
            New_img_received = img_count;
            
            
            //cv::resizeWindow("in", 1920, 1080);
            
#if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
            gpu_frame.upload(image);
            cv::imshow("in", gpu_frame);
#else
            cv::imshow("in", image);
#endif
            cpt_img++;
            //timerDisplay.stop();
           // timerFull.stop();
        }


//////////////////////////////

        stop = clock();        
        
        aruco::CameraParameters camp;
        if (calibrator.getCalibrationResults(camp))
        {
            camp.saveToFile("cam_calibration_new.yml");
            cout << "results saved to cam_calibration_new.yml\n";
        }
        else
            cerr << "Could not obtain calibration\n";
	    
	cv::destroyAllWindows();
        
        //Stop image acquisition and shutdown camera and API
        err = Stop_Acquisition_and_Close();

        if(err != VmbErrorSuccess)
            throw "Closing / Stop Acquisition error (See Alvium_Camera.cpp file ) !";
        
        std::cout << "\n";
        cout << "Final error= " << calibrator.getReprjError() << "\n";
        //exiting = true;
        //t1.join();
        //std::cout << "Detection : "<< timerDetect.getAvrg() * 1000 << " ms\n";
        //std::cout << "Display : "<< timerDisplay.getAvrg() * 1000 << " ms\n";
        std::cout << "Average Real FPS : "<< (float)cpt_img/((float)(stop-start)/CLOCKS_PER_SEC) << " fps\n";
    }
    catch (std::exception& ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}

