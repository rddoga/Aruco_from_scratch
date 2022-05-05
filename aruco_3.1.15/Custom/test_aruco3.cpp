#include "v4l2_helper.h"
#include "utils.h"
#include "i2c_helper.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/cuda.hpp>
#include "aruco.h"

#include <vector>
//#include <tuple>
#include <iostream>
#include <chrono>

using namespace std;
//using namespace aruco;

int imgSizeX = 1920, imgSizeY = 1080;
aruco::MarkerDetector MDetector;


int main(int argc, char** argv)
{
    int cpt = 0;
    unsigned char* ptr_cam_frame; //permettra de prendre l'image
    int bytes_used, key = 0;
    TimerAvrg timerImg, timerFull, timerComputation;
    
    clock_t start, stop, sum = 0;
    
    //frame pointer
    cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
    //Original image, that gets the cam frame from the pointer
    cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);
    //copied image, for printing the markers on the screen
    cv::Mat imageCopy;
    
    if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            cout << "Failed to open video" << endl;
            return -1;
        }
        
    while (key != 27){
        
        
        // start timers
        timerFull.start();
        timerImg.start();
        start = clock();
        //getting cam frame
        if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0)
        {
            cout << "Failed to get image" << endl;
            break;
        }

        //getting frame pointer
        yuyv_frame.data = ptr_cam_frame;
        
        //gettting the image with the right colors in originalImage matrix
        cv::cvtColor(yuyv_frame, originalImage, cv::COLOR_YUV2BGR_UYVY); // or COLOR_YUV2BGR_YUYV
        
        //release cam frame
        if (helper_release_cam_frame() < 0)
        {
            cout << "Failed to release image" << endl;
            break;
        }
        stop = clock();
        cout << ((float) stop - start)/CLOCKS_PER_SEC << endl;
        sum += (((float) stop - start)/CLOCKS_PER_SEC) * 1000;
        
        timerImg.stop();
        timerComputation.start();
       
        //Copying image for showing the detected markers
        originalImage.copyTo(imageCopy);
        
        
        MDetector.setDictionary("ARUCO_MIP_16h3");
        //detecting markers
        vector<aruco::Marker> markers=MDetector.detect(imageCopy);
        
        if (markers.size() > 0){
            for(size_t i=0;i<markers.size();i++){
                //draw in the image
                markers[i].draw(imageCopy, cv::Scalar(0, 0, 255), 2);
            }
        }
        
        key = cv::waitKey(1) & 0xFF;
        
        if(cpt%20 == 0)
            cout << "getting frame" << endl;
        
        //Show image
        cv::imshow("Img", imageCopy);
        cpt++;
        
        timerComputation.stop();
        timerFull.stop();
    }
    
    if (helper_deinit_cam() < 0)
        cout << "Failed to deinitialise camera" << endl;
   
   
   
    cout << "Average image catch time : " << timerImg.getAvrg() * 1000 << " ms" << endl;
    cout << "Average image catch time (timer 2): " << (float)sum/cpt << " ms" << endl;
    cout << "Average computation time : " << timerComputation.getAvrg() * 1000 << " ms"<< endl;
    cout << "Average total time : " << timerFull.getAvrg() * 1000 << " ms" << endl;
    cout << "Real average FPS : "<< 1./timerFull.getAvrg() << " fps" << endl;
    
    return 0;
    //printf("hello\n");
    
}
