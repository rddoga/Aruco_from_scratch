

#include "Alvium_Camera.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core_c.h>


#include <iostream>
/*#include <string>
#include <cstring>
#include <unistd.h>*/




using namespace std;
using namespace AVT::VmbAPI;


///////////////////////////

int imgSizeX = 4024, imgSizeY = 3036;
//frame pointer
cv::Mat raw_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);


int main(int argc, char* argv[])
{  

    //Startup and start image acquisition
    if(VmbErrorSuccess != Open_and_Start_Acquisition() ){
        //throw Cam_Exception("Openning / Start Acquisition error !");
        cout << "Openning / Start Acquisition error !" << endl;
        return -1;
    }

    int key = 0;

    cv::TickMeter tm_full, tm_resize1, tm_resize2, tm_resize3, tm_detect, tm_end;

    while(key != 27){
        
		tm_full.start();
        key = cv::waitKey(1) & 0xFF; 
        
        //NEW CAMERA
        if(ptr_raw_frame == NULL){//Check if we started receiving
            continue;
        }
         
        raw_frame.data = ptr_raw_frame; //Getting raw frame (of the current image ?)      
        
        //cout << "Avant affichage" << endl;
        //getting the image with the right colors in originalImage matrix
        //cv::cvtColor(raw_frame, originalImage, cv::); // or COLOR_YUV2BGR_YUYV
        
        //Ipl->imageData = (char *)ptr_receive;
        //originalImage = cv::cvarrToMat(	Ipl, true); //converting ipl image to cv::Mat (with data copy)
        // = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);
        
        //Show each image
       
        
        cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC1);
        //resize down for printing the image
        cv::resize(raw_frame, resized_down, resized_down.size(), 0, 0, cv::INTER_LINEAR);
        
        // Send each image out to ffmpeg, in order to stream the video
        /*string out((char*)resized_down.data, resized_down.total() * resized_down.elemSize());
        cout << out; */
        
        cv::imshow("Img", resized_down);
        //cout << "Apres affichage" << endl;
		tm_full.stop();
    }   
    
    if(VmbErrorSuccess != Stop_Acquisition_and_Close() ){
        //throw Cam_Exception("Closing / Stop Acquisition error !");/**/
        cout << "Closing / Stop Acquisition error !" << endl;
        return -1;
    }

	cout << "Average full computation time: " << tm_full.getAvgTimeSec()*1000 << " ms" << endl;
    
    return 0;   
}


