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


#include "aruco.h"
#include "calibrator.h"
#include "v4l2_helper.h"
#include "utils.h"

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

CameraParameters TheCameraParameters;
MarkerDetector TheMarkerDetector;
vector<vector<aruco::Marker>> allMarkers;
string TheOutCameraParams;
aruco::CameraParameters camp;  // camera parameters estimated
Calibrator calibrator;
unsigned int imgSizeX = 1920; // x pixel image size
unsigned int imgSizeY = 1080; // y pixel image size
cv::Mat yuyv_frame(imgSizeY, imgSizeX, CV_8UC2);    // raw image
cv::Mat originalImage(imgSizeX, imgSizeY, CV_8UC3); // converted image
cv::Mat image(imgSizeX, imgSizeY, CV_8UC3);         // saved image
bool exiting = false;

aruco::CameraParameters cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr=0, aruco::MarkerMap *inmmap=0);

/************************************
 *
 *  FUNCTIONS
 *
 ************************************/

void FrameGetter()
{
    TimerAvrg timerGetImage;
    TimerAvrg timerConv;

    unsigned char* ptr_cam_frame;
    int bytes_used;

    while(!exiting)
    {
        timerGetImage.start();

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

        timerGetImage.stop();
    }

    // exiting program
    if (helper_deinit_cam() < 0)
        fprintf(stderr, "Failed to deinitialise camera : %m\n");

    std::cout << "Average image catch time : "<< timerGetImage.getAvrg() * 1000 << " ms /!\\ keep in mind this is multithreaded\n";
}

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

        if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            fprintf(stderr, "Failed to open video : %m\n");
            return -1;
        }

/*#if defined(ENABLE_GL_DISPLAY) && defined(ENABLE_GPU_UPLOAD)
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
#endif*/

        cv::resizeWindow("in", 1920, 1080);

        //configure the calibrator
        calibrator.setParams(cv::Size(imgSizeX, imgSizeY), 0.04, "");

        // set specific parameters for this configuration
        TheMarkerDetector.setDictionary("ARUCO_MIP_36h12");
        TheMarkerDetector.setDetectionMode(aruco::DM_NORMAL);

        TimerAvrg timerFull;
        TimerAvrg timerDetect;
        TimerAvrg timerDisplay;

        char key = 0;
        int waitKeyTime = 1;
	    
	// capture until press ESC
        std::thread t1(FrameGetter);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        while (key != 27)
        {
            timerFull.start();

            // get frame
            image = originalImage.clone();

            // detect
            timerDetect.start();
            vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(image);
            timerDetect.stop();
            
            // print markers from the board
            timerDisplay.start();
            for (auto m: detected_markers)
                m.draw(image, Scalar(0, 0, 255), 1);
            // draw help
            cv::putText(image, "'a' add current image for calibration", cv::Point(10,40), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(image, "'esc' save and quit", cv::Point(10,80), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
            cv::putText(image, calibrator.getInfo(), cv::Point(10,120), FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
/*
#if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
		    gpu_frame.upload(image);
		    imshow("in", gpu_frame);
#else*/
		    imshow("in", image);
//#endif
            timerDisplay.stop();

            key = cv::waitKey(waitKeyTime);   // wait for key to be pressed
            if (key == 'a')
                calibrator.addView(detected_markers);

            timerFull.stop();
        }

        aruco::CameraParameters camp;
        if (calibrator.getCalibrationResults(camp))
        {
            camp.saveToFile("cam_calibration_3.yml");
            cout << "results saved to cam_calibration.yml\n";
        }
        else
            cerr << "Could not obtain calibration\n";
	    
	cv::destroyAllWindows();

        std::cout << "\n";
        cout << "Final error= " << calibrator.getReprjError() << "\n";
        exiting = true;
        t1.join();
        std::cout << "Detection : "<< timerDetect.getAvrg() * 1000 << " ms\n";
        std::cout << "Display : "<< timerDisplay.getAvrg() * 1000 << " ms\n";
        std::cout << "Average FPS : "<< 1./timerFull.getAvrg() << " fps\n";
    }
    catch (std::exception& ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}

