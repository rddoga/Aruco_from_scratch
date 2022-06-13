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

//For custom detection parameters
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//Get predefined dictionary
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);//"ARUCO_MIP_16h3" ""

// camera parameters are read from somewhere
cv::Mat cameraMatrix, distCoeffs;

//Parameters for initializing the board
std::vector<int> markerIds;
std::vector<std::vector<cv::Point3f> > objPoints; //Vector for saving the corners of the markers (in the board)
    
    
//Lecture du fichier avec les positions et ids des markers (NOT USED HERE)
void readFromFile(cv::FileStorage& fs)
{
    int aux = 0;
    // look for the nmarkers
    if (fs["aruco_bc_nmarkers"].name() != "aruco_bc_nmarkers")
        throw cv::Exception(81818, "readFromFile", "invalid file type", __FILE__, __LINE__);
    fs["aruco_bc_nmarkers"] >> aux;
    objPoints.resize(aux);
    markerIds.resize(aux);
    //fs["aruco_bc_mInfoType"] >> mInfoType;
    cv::FileNode markers = fs["aruco_bc_markers"];
    int i = 0;
    for (cv::FileNodeIterator it = markers.begin(); it != markers.end(); ++it, i++)
    {
        markerIds.at(i) = (*it)["id"];
        cv::FileNode FnCorners = (*it)["corners"];
        //std::vector<cv::Point3f> corner_points;
        //objPoints.push_back(corner_points);
        for (cv::FileNodeIterator itc = FnCorners.begin(); itc != FnCorners.end(); ++itc)
        {
            vector<float> coordinates3d;
            (*itc) >> coordinates3d;
            if (coordinates3d.size() != 3)
                throw cv::Exception(81818, "readFromFile", "invalid file type 3", __FILE__, __LINE__);
            cv::Point3f point(coordinates3d[0] * 1000, coordinates3d[1] * 1000, coordinates3d[2] * 1000);
            objPoints.at(i).push_back(point);
        }
    }
    //if (fs["aruco_bc_dict"].name() == "aruco_bc_dict")
    //    fs["aruco_bc_dict"] >> dictionary;
}

////////////////////////////////////////////////////
//VECTOR FOR MAKING THE ARUCO_MIP_16H3 DICTIONARY
vector<uint64_t> codes={0x5867UL,0x8b03UL,0x2537UL,0xb6c7UL,0xe45UL,0x161UL,0x219UL,0x859bUL,0x87UL,0xc93fUL,0x905fUL,0x3e73UL,0x6ab7UL,0x1bafUL,0x6f0fUL,0x23d3UL,0x47a5UL,0x8cf7UL,0x83cfUL,0x9205UL,0x29a5UL,0x8033UL,0x857dUL,0xa4afUL,0x422fUL,0x1d07UL,0x4ee3UL,0x64c5UL,0xaa7fUL,0x4b75UL,0x34dbUL,0x926UL,0x262UL,0x501dUL,0x415UL,0x6201UL,0x2064UL,0x2d5UL,0x10bUL,0x9427UL,0xc16bUL,0xa603UL,0x911UL,0x1043UL,0x87bUL,0xccfUL,0x162bUL,0x9ab3UL,0x30b7UL,0xad0bUL,0x60a6UL,0x3845UL,0xce2bUL,0xadc7UL,0x612UL,0x4253UL,0x9cc3UL,0xc23UL,0x409bUL,0x8e87UL,0x98e5UL,0x20f1UL,0xa807UL,0x1bfUL,0x7023UL,0xdf1UL,0x2957UL,0x26b3UL,0xd80fUL,0x4076UL,0x233bUL,0x32e3UL,0x7a85UL,0x4349UL,0xc857UL,0x41c6UL,0x3813UL,0x6d97UL,0x324fUL,0xe3fUL,0x47ffUL,0x2217UL,0xdd6fUL,0x48b1UL,0x8b95UL,0xd9c7UL};
//87 elements

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


///////////////////////////////////////////////////
//For getting a matrix from a hexa number (used for creating the dictionary) MARCHE PAS !!!
/*cv::Mat generateMatFromHexaCode(uint64_t code, int size)
{
    cv::Mat Mat(size, size, CV_8UC1);
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            Mat.at<uint8_t>(size - i - 1, size - j - 1) = code%2;
            code >>= 1;
        }
    }
    return Mat;
}*/



///////////////////////////////////////
/////////////    MAIN    //////////////
///////////////////////////////////////
int main(int argc, char** argv)
{
    int imgSizeX = 1920, imgSizeY = 1080, cpt = 0;
    unsigned char* ptr_cam_frame; //permettra de prendre l'image
    int bytes_used, key = 0;
    TimerAvrg timerImg, timerFull, timerComputation;
    string calib_filename = "calib.yml";///"/home/rddoga/Desktop/Positionnement/tagsCode/aruco/aruco-3/calibration.yml"
    //string markers_for_board_file = "stacked.yml";//"Markers_for_board.yml";
    
    //frame pointer
    cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
    //Original image, that gets the cam frame from the pointer
    cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);/**/
    //copied image, for printing the markers on the screen
    cv::Mat imageCopy;


    //////////////////////////CREATE AND SAVE CUSTOM DICTIONARY (FOR USING ARUCO_MIP_16H3)////////////////////////////
    /////////////////////////MARCHE PAS !!!!!!!!!!!!!!!!!!!///////////////////////////
    /*cv::Ptr<cv::aruco::Dictionary> dictionary;
    // markers of 4x4 bits
    dictionary->markerSize = 4;
    // maximum number of bit corrections
    dictionary->maxCorrectionBits = 3;
    
    // lets create a dictionary of 50 markers
    for(int i=0; i<50; i++)
    {
        // assume generateMarkerBits() generates a new marker in binary format, so that
        // markerBits is a 4x4 matrix of CV_8UC1 type, only containing 0s and 1s
        cv::Mat markerBits = generateMatFromHexaCode( codes.at(i), 4);
        cv::Mat markerCompressed = cv::aruco::Dictionary::getByteListFromBits(markerBits);
        // add the marker as a new row
        dictionary->bytesList.push_back(markerCompressed);
    }*/
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    //Modification of the detection parameters, for better performance in the detection
    modify_parameters();
    
    
    // You can read camera parameters from tutorial_camera_params.yml (from opencv_contrib/modules/aruco/samples)
    readCameraParameters( calib_filename, cameraMatrix, distCoeffs); // This function is located in detect_board.cpp
    
    ////////////////////////NOT USED HERE!!////////////////
    //Taking marker positions and ids from file
    /*cv::FileStorage fs(markers_for_board_file, cv::FileStorage::READ);
    readFromFile(fs);*/
    ////////////////////////////////////////////////////
    
    //////////////////MARCHE PAS!!////////////////
    // assume we have a function to create the board object
    //cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(objPoints, dictionary, markerIds);
    
    
    ////////////////////////////////////////////////
    // To use tutorial sample, you need read custome dictionaty from tutorial_dict.yml
    //readDictionary(filename, dictionary); // This function is located in detect_board.cpp
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(4, 5, 0.04, 0.01, dictionary);
    
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
        
        
        //creating variables and copying image
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        originalImage.copyTo(imageCopy);
        
        //detecting markers
        cv::aruco::detectMarkers(originalImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        
        // if at least one marker detected
        if(markerIds.size() > 0) {
            //Draw detected markers
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            
            cv::Vec3d rvec, tvec;
            int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rvec, tvec);
            
            if(cpt%20 == 0)
                    cout << "Nb markers on board : " << valid << endl;
            // if at least one board marker detected
            if(valid > 0)
                cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
        }
    
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
}
