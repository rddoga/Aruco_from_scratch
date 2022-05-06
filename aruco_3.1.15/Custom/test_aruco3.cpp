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
#include <thread>
#include <stdexcept>

using namespace std;
//using namespace aruco;

int imgSizeX = 1920, imgSizeY = 1080;
bool exiting = false; //For the image getter to know when to stop getting the image
aruco::MarkerDetector MDetector; //Detector object
aruco::CameraParameters camera; //Camera object for saving camera parameters

/***************************************************/
/*                                                 */
/******************** FUNCTIONS ********************/
/*                                                 */
/***************************************************/

void FrameGetter(cv::Mat &originalImage)
{
    TimerAvrg timerGetImage;
    clock_t start, stop;
    float sum_img = 0.0f;
    
    unsigned char* ptr_cam_frame;
    int bytes_used, cpt = 0;
    cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);//Raw image

    while(!exiting)
    {
        start = clock();
        //timerGetImage.start();
        
        //getting cam frame
        if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0)
        {
            fprintf(stderr, "Failed to get image : %m\n");
            break;
        }
        yuyv_frame.data = ptr_cam_frame; //Getting raw image
        
        //gettting the image with the right colors in originalImage matrix
        cv::cvtColor(yuyv_frame, originalImage, cv::COLOR_YUV2BGR_UYVY); // or COLOR_YUV2BGR_YUYV
        
        //release cam frame
        if (helper_release_cam_frame() < 0)
        {
            fprintf(stderr, "Failed to release image : %m\n");
            break;
        }

        //timerGetImage.stop();
        stop = clock();
        cout << ((float) stop - start)/CLOCKS_PER_SEC << endl;
        sum_img += (((float) stop - start)/CLOCKS_PER_SEC);
        cpt++;
    }

    // exiting program
    if (helper_deinit_cam() < 0)
        fprintf(stderr, "Failed to deinitialise camera : %m\n");

    //std::cout << "Average image catch time                   : "<< timerGetImage.getAvrg() * 1000 << " ms /!\\ keep in mind this is done in a separate thread\n";
    std::cout << "Average image catch time (timer 2): " << (sum_img/cpt * 1000) << " ms  /!\\ keep in mind this is done in a separate thread" << std::endl;
    cout << "Average base FPS (timer 2) : "<< (float)cpt/sum_img << " fps" << endl << endl;
}

/**********************************************/
/*                                            */
/******************** MAIN ********************/
/*                                            */
/**********************************************/

int main(int argc, char** argv)
{
    try
    {
        int cpt = 0, Mdetected = 0;
        unsigned char* ptr_cam_frame; //permettra de prendre l'image
        int bytes_used, key = 0;
        TimerAvrg timerFull, timerGetImage, timerComputation;
        
        clock_t start, stop;
        float sum_comp = 0.0f;
        float sum_img = 0.0f;
        
        //frame pointer
        cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
        //Original image, that gets the cam frame from the pointer
        cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);
        //copied image, for printing the markers on the screen
        cv::Mat imageCopy;
        
        //Set dictionary and detection mode
        MDetector.setDictionary("ARUCO_MIP_36h12");
        MDetector.setDetectionMode(aruco::DM_VIDEO_FAST);
        
        camera.readFromXMLFile("cam_calibration_3.yml");
            
        if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            cout << "Failed to open video" << endl;
            return -1;
        }
            
        // multithreaded image getter
        //std::thread t1(FrameGetter, std::ref(originalImage));
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
        while (key != 27){
            
            
            timerFull.start();
            start = clock();
            timerGetImage.start();
            
            //getting cam frame
            if (helper_get_cam_frame(&ptr_cam_frame, &bytes_used) < 0)
            {
                fprintf(stderr, "Failed to get image : %m\n");
                break;
            }
            yuyv_frame.data = ptr_cam_frame; //Getting raw image
            
            //gettting the image with the right colors in originalImage matrix
            cv::cvtColor(yuyv_frame, originalImage, cv::COLOR_YUV2BGR_UYVY); // or COLOR_YUV2BGR_YUYV
            
            //release cam frame
            if (helper_release_cam_frame() < 0)
            {
                fprintf(stderr, "Failed to release image : %m\n");
                break;
            }

            timerGetImage.stop();
            stop = clock();
            cout << ((float) stop - start)/CLOCKS_PER_SEC << endl;
            sum_img += (((float) stop - start)/CLOCKS_PER_SEC);
            
            // start timers
            
            start = clock();   
            
        //    timerImg.stop();
            timerComputation.start();
           
            //Copying image for showing the detected markers
            originalImage.copyTo(imageCopy);
            
            
            //detecting markers
            vector<aruco::Marker> markers=MDetector.detect(imageCopy);
            
            if (markers.size() > 0){
                Mdetected++;
                for(size_t i=0;i<markers.size();i++){
                    //draw in the image
                    markers[i].draw(imageCopy, cv::Scalar(0, 0, 255), 1, false);
                }
            }
            
            /*for(auto m:markers){
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camera, m.Rvec, m.Tvec, 0.1);
                aruco::CvDrawingUtils::draw3dCube(imageCopy, m, camera, 1);
                //cout<<m.Rvec<<" "<<m.Tvec<<endl;
            }*/

            
            key = cv::waitKey(1) & 0xFF;
            
            //if(cpt%20 == 0)
            //    cout << "getting frame" << endl;
            
            //Show image
            cv::imshow("Img", imageCopy);
            cpt++;
            
            timerComputation.stop();
            timerFull.stop();
            
            stop = clock();
            float inter = ((float) stop - start)/CLOCKS_PER_SEC;
            cout << "main : " << inter << endl;
            sum_comp += inter;
        }
       
        // exiting program
        if (helper_deinit_cam() < 0)
            fprintf(stderr, "Failed to deinitialise camera : %m\n");
        
        exiting = true; //Indicate that we have to stop the image getter thread
        //t1.join();
       
       // cout << "Average image catch time : " << timerImg.getAvrg() * 1000 << " ms" << endl;
        //cout << "Average image catch time (timer 2): " << (float)sum/cpt << " ms" << endl;
        std::cout << "Average image catch time : "<< timerGetImage.getAvrg() * 1000 << " ms " << endl;
        cout << "Average computation time : " << timerComputation.getAvrg() * 1000 << " ms"<< endl;
        cout << "Average total time : " << timerFull.getAvrg() * 1000 << " ms" << endl;
        cout << "Real average FPS : "<< 1./timerFull.getAvrg() << " fps" << endl;
        
        std::cout << "Average image catch time (timer 2): " << (sum_img/cpt) * 1000 << " ms  /!\\ keep in mind this is done in a separate thread" << std::endl;
        cout << "Average base FPS (timer 2) : "<< cpt/sum_img << " fps" << endl << endl;
        std::cout << "Average full time (timer 2): " << ((sum_comp+sum_img)/cpt) * 1000 << " ms" << std::endl;
        cout << "Real average FPS (timer 2) : "<< cpt/(sum_comp+sum_img) << " fps" << endl << endl;
        
        cout << "MArker catch frequency : " << (float)Mdetected/cpt << endl;
    }
    catch(std::exception& ex)
    {
        cout << "Exception :" << ex.what() << "\n";
    }
    return 0;
    //printf("hello\n");
    
}
