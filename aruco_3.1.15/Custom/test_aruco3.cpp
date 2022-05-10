#include "v4l2_helper.h"
#include "utils.h"
#include "i2c_helper.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/cuda.hpp>
#include "aruco.h"
#include "markermap.h"

#include <vector>
//#include <tuple>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>

using namespace std;
//using namespace aruco;

/***************************************************/
/*                                                 */
/******************** VARIABLES ********************/
/*                                                 */
/***************************************************/

int imgSizeX = 1920, imgSizeY = 1080;
float zOffset = 350; //position of the tool cursor on the z axis (in the tool markermap basis). Value will be changed by trackbar

bool exiting = false; //For the image getter to know when to stop getting the image
bool OneMarkerValid = false;

float sum_img = 0.0f, sum_comp = 0.0f;
//vector< vector< cv::Point2f > > DrawnImgPoints; //Remembering drawn points
vector< cv::Point2f > DrawnImgPoints; //Remembering drawn points

aruco::MarkerDetector MDetector; //Detector object
aruco::CameraParameters camera; //Camera object for saving camera parameters
aruco::MarkerMapPoseTracker MMTracker; //tracker for estimating the pose of the markermap



/***************************************************/
/*                                                 */
/******************** FUNCTIONS ********************/
/*                                                 */
/***************************************************/

/*void FrameGetter(cv::Mat &originalImage)
{
    TimerAvrg timerGetImage;
    clock_t start, stop;
    
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
    cout << "Average base FPS (timer 2) : "<< cpt/sum_img << " fps" << endl << endl;
}*/

/**********************************************/
/*                                            */
/******************** MAIN ********************/
/*                                            */
/**********************************************/

int main(int argc, char** argv)
{
    try
    {
        aruco::MarkerMap mmap; //main MarkerMap
        int cpt = 0, Mdetected = 0;
        unsigned char* ptr_cam_frame; //permettra de prendre l'image
        int bytes_used, key = 0;
        TimerAvrg timerFull, timerGetImage, timerComputation;
        string posMarkerMapfile = "stacked.yml";
    
        clock_t start, stop;
        //float sum_img = 0.0f;

        //frame pointer
        cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
        
        //Original image, that gets the cam frame from the pointer
        cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);
        //copied image, for printing the markers on the screen
        cv::Mat imageCopy;
        //empty img used for displaying the trackbar
        cv::Mat TrackbarImg = cv::Mat::zeros(1, 500, CV_8UC1);


        //Set dictionary and detection mode
        MDetector.setDictionary("ARUCO_MIP_16h3");
        MDetector.setDetectionMode(aruco::DM_FAST);
        
        camera.readFromXMLFile("cam_calibration_3.yml");

        mmap.readFromFile(posMarkerMapfile); //Read 3d markers position of the markermap from file
        MMTracker.setParams(camera, mmap, 0.02); //Set Tracker params
        
        //Initialise camera
        if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            cout << "Failed to open video" << endl;
            return -1;
        }
        
        
        cv::namedWindow("Tool", cv::WINDOW_GUI_EXPANDED);
        cv::namedWindow("Trackbars", cv::WINDOW_GUI_EXPANDED);
        
        cv::resizeWindow("Tool", 1920, 1080);
        //cv::resizeWindow("Trackbars", 100, 100);
        cv::moveWindow("Trackbars",0,0);
        //Creating trackbars
        cv::createTrackbar("Calibration Z offset", "Trackbars", NULL, 2000);
        //cv::createTrackbar("Track 2", "Trackbars", NULL, 2000);
        
        cv::setTrackbarPos("Calibration Z offset", "Trackbars", zOffset);

        // multithreaded image getter
        //std::thread t1(FrameGetter, std::ref(originalImage));
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
        while (key != 27){
            
            
           // timerFull.start();
            
            start = clock();
            
            //timerGetImage.start();
            //////////////IF NOT MULTITHREADED////////////////////////
            
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

            
            stop = clock();
            timerGetImage.stop();
            //cout << ((float) stop - start)/CLOCKS_PER_SEC << endl;
            sum_img += (((float) stop - start)/CLOCKS_PER_SEC);
            /**/
            
            //timerImg.stop();
            //timerComputation.start();
            
            
            start = clock();   
            //Copying image for showing the detected markers
            originalImage.copyTo(imageCopy);
            
            
            //detecting markers
            vector<aruco::Marker> markers=MDetector.detect(imageCopy,camera,0.04);
            //vector<aruco::Marker> markers=MDetector.detect(imageCopy);
            
            if(MMTracker.estimatePose(markers)){
                OneMarkerValid = true;
            }else{
                OneMarkerValid = false;
            }
            
            //if (MMTracker.isValid())
              //  cout<<MMTracker.getRvec()<<" "<<MMTracker.getTvec()<<endl;
                
            if (markers.size() > 0){
                Mdetected++;
                for(size_t i=0;i<markers.size();i++){
                    //draw in the image
                    markers[i].draw(imageCopy, cv::Scalar(0, 0, 255), 1, false);
                }
            }
            
            for(auto m:markers){
                //aruco::CvDrawingUtils::draw3dAxis(imageCopy, camera, m.Rvec, m.Tvec, 0.1);
                //aruco::CvDrawingUtils::draw3dCube(imageCopy, m, camera, 1);
                //cout<<m.Rvec<<" "<<m.Tvec<<endl;
            }/**/
            
            key = cv::waitKey(1) & 0xFF;
            
            if(OneMarkerValid)
            {
                //Draw tool axis
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camera, MMTracker.getRvec(), MMTracker.getTvec(), 0.02, 2);
                
                zOffset = cv::getTrackbarPos("Calibration Z offset", "Trackbars") / 10000.;//Take value from trackbar
                
                vector< cv::Point2f > imagePoint;
                cv::Mat objectPoint(2, 3, CV_32FC1);
                objectPoint.at<float>(0, 0) = 0;
                objectPoint.at<float>(0, 1) = 0;
                objectPoint.at<float>(0, 2) = 0;
                objectPoint.at<float>(1, 0) = 0;
                objectPoint.at<float>(1, 1) = 0;
                objectPoint.at<float>(1, 2) = -zOffset;
                cv::projectPoints(objectPoint, MMTracker.getRvec(), MMTracker.getTvec(), camera.CameraMatrix, camera.Distorsion, imagePoint); //Project the 3d points in 2d
                cv::drawMarker(imageCopy, imagePoint.at(1), cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); 
                
                
                cv::Mat objectPoint_41 = cv::Mat(4, 1, CV_32FC1);
                objectPoint_41.at<float>(0, 0) = objectPoint.at<float>(1, 0);
                objectPoint_41.at<float>(1, 0) = objectPoint.at<float>(1, 1);
                objectPoint_41.at<float>(2, 0) = objectPoint.at<float>(1, 2);
                objectPoint_41.at<float>(3, 0) = 1;
                
                cv::Mat Real_coordinates = cv::Mat(4, 1, CV_32FC1);
                Real_coordinates = MMTracker.getRTMatrix() * objectPoint_41;
                cout << "RT : " << MMTracker.getRTMatrix() << endl;
                cout << "in : " << objectPoint_41 << endl;
                cout << "out : " << Real_coordinates << endl;
                string R_coordinates = "( " + to_string(Real_coordinates.at<float>(0, 0)) + "; " + to_string(Real_coordinates.at<float>(1, 0)) + "; " + to_string(Real_coordinates.at<float>(2, 0)) + " )";
                cv::putText(imageCopy, "Coordinates of the tool top :", cv::Point(5, 800), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
                cv::putText(imageCopy, R_coordinates, cv::Point(5, 850), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
                
                
                if(key == 'r' | key == 'R'){
                    //std::vector<cv::Point2f> imagePoints;
                    //cv::Mat mat3Dpoints = cv::Mat(markers[0].get3DPoints(), true); //get the matrix out of the 3d point array
                    //cv::projectPoints(mat3Dpoints, markers[0].Rvec, markers[0].Tvec, camera.CameraMatrix, camera.Distorsion, imagePoints); //Project the 3d points in 2d
                    
                    DrawnImgPoints.push_back(imagePoint.at(1) );//Remember the new point
                }
                
                /*if(DrawnImgPoints.size() > 0)
                {
                    for (int i=0; i < DrawnImgPoints.size(); i++){
                        //aruco::CvDrawingUtils::drawXYcross(imageCopy, camera, markers[0].Rvec, markers[0].Tvec, 0.005);///2 * imgSizeX / 1920 // adapting ui size to image size 
                        for(int j=0; j < DrawnImgPoints.at(i).size(); j++){
                            cv::drawMarker(imageCopy, DrawnImgPoints.at(i).at(j), cv::Scalar(0, 255, 0) );//Draw all the points as a cross
                        }
                    }
                }*/
                
            }
            
            if(DrawnImgPoints.size() > 0)
            {
                for (int i=0; i < DrawnImgPoints.size(); i++){
                    //aruco::CvDrawingUtils::drawXYcross(imageCopy, camera, markers[0].Rvec, markers[0].Tvec, 0.005);///2 * imgSizeX / 1920 /* adapting ui size to image size */  
                    cv::drawMarker(imageCopy, DrawnImgPoints.at(i), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw all the points as a cross
                    //cv::line(imageCopy, DrawnImgPoints.at(i).at(0), DrawnImgPoints.at(i).at(1), cv::Scalar(0, 0, 255, 255), 1);
                }
            }/**/
            
                            
            //if(cpt%20 == 0)
            //    cout << "getting frame" << endl;
            
            //Show image
            cv::imshow("Tool", imageCopy);
            cv::imshow("Trackbars", TrackbarImg );
            cpt++;
            
            
            
            stop = clock();
            float inter = ((float) stop - start)/CLOCKS_PER_SEC;
            //cout << "main : " << inter << endl;
            sum_comp += inter;
            
            //timerComputation.stop();
            //timerFull.stop();
        }
        // exiting program
        if (helper_deinit_cam() < 0)
            fprintf(stderr, "Failed to deinitialise camera : %m\n");
        
        exiting = true; //Indicate that we have to stop the image getter thread
        //t1.join(); //Wait for the other thread to finish
       
       // cout << "Average image catch time : " << timerImg.getAvrg() * 1000 << " ms" << endl;
        //cout << "Average image catch time (timer 2): " << (float)sum/cpt << " ms" << endl;
        //std::cout << "Average image catch time : "<< timerGetImage.getAvrg() * 1000 << " ms " << endl;
        //cout << "Average computation time : " << timerComputation.getAvrg() * 1000 << " ms"<< endl;
        //cout << "Average total time : " << timerFull.getAvrg() * 1000 << " ms" << endl;
        //cout << "Real average FPS : "<< 1./timerFull.getAvrg() << " fps" << endl;
        
        std::cout << "Average image catch time (timer 2): " << (sum_img/cpt) * 1000 << " ms  /!\\ keep in mind this is done in a separate thread" << std::endl;
        cout << "Average base FPS (timer 2) : "<< cpt/sum_img << " fps" << endl << endl;
        std::cout << "Average computation time (timer 2): " << ((sum_comp)/cpt) * 1000 << " ms" << std::endl;
        
        std::cout << "Average full time (timer 2): " << ((sum_comp+sum_img)/cpt) * 1000 << " ms" << std::endl;
        cout << "Real average FPS (timer 2) : "<< cpt/(sum_comp+sum_img) << " fps" << endl << endl;
        
        cout << "MArker catch frequency : " << (float)Mdetected*100/cpt << " % of the time." << endl;
    }
    catch(std::exception& ex)
    {
        cout << "Exception :" << ex.what() << "\n";
    }
    return 0;
    //printf("hello\n");
    
}
