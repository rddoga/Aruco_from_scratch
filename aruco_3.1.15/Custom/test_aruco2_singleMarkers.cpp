
#include "v4l2_helper.h"
#include "utils.h"
#include "i2c_helper.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/mat.hpp>
#include "aruco.hpp" //Lib pour aruco2!!
#include "aruco_samples_utility.hpp"


#include <vector>
//#include <tuple>
#include <iostream>
#include <chrono>


using namespace std;
//using namespace aruco;

//For custom dection parameters
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//creating variables
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250); //Define dictionary
    
cv::Mat cameraMatrix, distCoeffs; 
std::vector<int> markerIds; //Ids of the detected markers
std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates; //All four corners of each marker (+ rejected markers)

//Sauvegarder les positions et ids des markers pour utiliser l'option board
/*void saveTofile(cv::FileStorage& fs){
        //fs << "aruco_bc_dict" << "DICT_4X4_250";
        fs << "aruco_bc_nmarkers" << (int)markerIds.size();
        //fs << "aruco_bc_mInfoType" << (int)mInfoType;
        fs << "aruco_bc_markers"
           << "[";
        for (size_t i = 0; i < markerIds.size(); i++)
        {
            fs << "{:"
               << "id" << markerIds.at(i);

            fs << "corners"
               << "[:";
            for (size_t c = 0; c < markerCorners.at(i).size(); c++)
                fs << markerCorners.at(i)[c];
            fs << "]";
            fs << "}";
        }
        fs << "]";
        cout << "File board saved " << endl;
}*/

///////////////////////////////////////
//Modification of the detection parameters, for better performance in the detection
void modify_parameters()
{
    
    //Less steps for initial thresholding
    parameters->adaptiveThreshWinSizeMax = 13;//Same value for both implies only 1 thresholding operation at that window size (improving speed of processing)
    parameters->adaptiveThreshWinSizeMin = 13;
    //parameters->adaptiveThreshWinSizeStep = 10;
    
    //Min and Max marker size considered
    parameters->minMarkerPerimeterRate = 0.05;
    parameters->maxMarkerPerimeterRate = 1.0;
    
    //Maximum error that the polygonal approximation can produce
    parameters->polygonalApproxAccuracyRate = 0.05;
    
    //Number of pixels in each marker cell (in the obtained image after removing perspective distortion)
    parameters->perspectiveRemovePixelPerCell = 8;
    
    //Percentage of pixels not analysed at the border of each of these cells
    parameters->perspectiveRemoveIgnoredMarginPerCell = 0.4;
    
    //Maximuml erroneous bits in the borders
    parameters->maxErroneousBitsInBorderRate = 0.1;
    
    //Corner refinement method (for a more precise localisation). Disabled by default, this line enables the subpix method
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    
    //////////////////////////////////////////
}

int main(int argc, char** argv)
{
    int imgSizeX = 1920, imgSizeY = 1080, cpt = 0;
    unsigned char* ptr_cam_frame; //permettra de prendre l'image
    int bytes_used, key = 0;
    TimerAvrg timerImg, timerFull, timerComputation;
    string calib_filename = "calib.yml";///"/home/rddoga/Desktop/Positionnement/tagsCode/aruco/aruco-3/calibration.yml"
    string markers_for_board_file = "Markers_for_board.yml";
    
    //bool saved = false; //For saving the marker corner positions and ids (NOT USEFULL HERE)
    
    //frame pointer
    cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
    //Original image, that gets the cam frame from the pointer
    cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);/**/
    //copied image, for printing the markers on the screen
    cv::Mat imageCopy;
       
    /*
    ///////////////////////Create Marker images and store them///////////////////////////////
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);//"ARUCO_MIP_36h12" ""
    for (int i=0; i<50;i++){
        string filename = "marker" + to_string(i) + ".png";
        cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
        cv::imwrite(filename, markerImage);
    }*/
    
    
    //Modification of the detection parameters, for better performance in the detection
    modify_parameters();
    
    // You can read camera parameters from tutorial_camera_params.yml
    readCameraParameters(calib_filename, cameraMatrix, distCoeffs); // This function is located in detect_markers.cpp
        
        
    //initialisation camera
    if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            cout << "Failed to open video" << endl;
            return -1;
        }
        
    while (key != 27){
    
        // start timers
        timerFull.start();
        timerImg.start();
        
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
        
        timerImg.stop();
        timerComputation.start();
        
        //copying image
        originalImage.copyTo(imageCopy);
        
        //detecting markers
        cv::aruco::detectMarkers(originalImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        //Drawing markers to the output image (and the rejected candidates)
        if (markerIds.size() > 0){
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            //if(rejectedCandidates.size() > 0)
            //   cv::aruco::drawDetectedMarkers(imageCopy, rejectedCandidates, cv::noArray(), cv::Scalar(255, 0, 255)); // Print rejected candidates if needed
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            //Estimate the pose of a marker
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i<markerIds.size(); i++)
                cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
        }
        
        
        /////////////////////////////////////////
        //Saving marker positions and ids the first time they are all detected at the same time (USELESS HERE)
        /////////////////////////////////////////
        /*if (!saved && (markerIds.size() == 20) ){
        
            cv::FileStorage fs(markers_for_board_file, cv::FileStorage::WRITE);
            saveTofile(fs);
            saved = true;
        }*/


        key = cv::waitKey(1) & 0xFF;
        
        //output in the terminal
        if(cpt%20 == 0)
            cout << "getting frame" << endl;
            
        cpt++;
        
        //output the image on the screen
        cv::imshow("Img", imageCopy);
        
        timerComputation.stop();
        timerFull.stop();
    }

    //deinitialise camera
    if (helper_deinit_cam() < 0)
        cout << "Failed to deinitialise camera" << endl;
   
    //printouts in the terminal
    cout << "Average image catch time : " << timerImg.getAvrg() * 1000 << " ms" << endl;
    cout << "Average computation time : " << timerComputation.getAvrg() * 1000 << " ms"<< endl;
    cout << "Average total time : " << timerFull.getAvrg() * 1000 << " ms" << endl;
    cout << "Real average FPS : "<< 1./timerFull.getAvrg() << " fps" << endl;
    
    
    return 0;
    //printf("hello\n");
    
}
