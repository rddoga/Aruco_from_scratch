
#include "v4l2_helper.h"
#include "utils.h"
#include "i2c_helper.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/cuda.hpp>/**/
#include <stdio.h>


//using namespace std;


int main(int argc, char** argv)
{
    int imgSizeX = 1920, imgSizeY = 1080, cpt = 0;
    unsigned char* ptr_cam_frame;
    int bytes_used, key = 0;
    cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
    cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);
    
    if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            printf("Failed to open video \n");
            return -1;
        }
        
    while (key != 27){
        
        if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0)
        {
            printf("Failed to get image \n");
            break;
        }
        
        key = cv::waitKey(1) & 0xFF;
        
        yuyv_frame.data = ptr_cam_frame;
        cv::cvtColor(yuyv_frame, originalImage, cv::COLOR_YUV2BGR_UYVY); // or COLOR_YUV2BGR_YUYV
        
        if(cpt%20 == 0)
            printf("getting frame \n");
            
        cv::imshow("Tool", originalImage);
            
        if (helper_release_cam_frame() < 0)
        {
            printf("Failed to release image \n");
            break;
        }
        cpt++;
    }
    
    if (helper_deinit_cam() < 0)
        printf("Failed to deinitialise camera \n");
   
    
    
    
    return 0;
    //printf("hello\n");
    
}
