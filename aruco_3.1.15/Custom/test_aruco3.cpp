//Comment if needed
#ifndef ENABLE_GL_DISPLAY
#define ENABLE_GL_DISPLAY
#endif

#ifndef ENABLE_GPU_UPLOAD
#define ENABLE_GPU_UPLOAD
#endif


/***************Different modes****************/
//#define OLD_CAMERA 1 //For knowing if we are using the old or the new camera (COMMENT IF USING NEW CAMERA)
#define OFFLINE_TEST 1 //For knowing if we are testing the detection on opencv directly or with the server communication (COMMENT IF TESTING ONLINE)


#ifdef OLD_CAMERA
#include "v4l2_helper.h"

#else
#include "Alvium_Camera.h"

#endif


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/cuda.hpp>
#include "aruco.h"
#include "markermap.h"

#ifndef OFFLINE_TEST // (dont need webserver if we are offline testing)
#include "crow.h"
#endif

#include <vector>
#include <tuple>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
//#include <stdexcept> 

using namespace std;
using namespace cv;
//using namespace aruco;

/***************************************************/
/*                                                 */
/******************** VARIABLES ********************/
/*                                                 */
/***************************************************/

///Variables from the camera helpers files
//unsigned char* ptr_raw_frame;
//bool StartReceiving = false;


#ifdef OLD_CAMERA
int imgSizeX = 1920, imgSizeY = 1080;
#else
int imgSizeX = 4024, imgSizeY = 3036;
#endif

////////////////Variables used for communicating with the server//////////////

float ExposureTime = 9000.0f; //Exposure time, can be changed through the webserver
float Gain = 20.0f; //Gain, can be changed through the webserver
int Threshold = -1; //Detection Threshold, can be changed through the webserver
int8_t Output_Video = 1; //For knowing if we are outputing the video as binary image or color image (By default, color image is sent) (0 no image, 1 color image, 2 binary image)

float zOffset = 350; //position of the tool cursor on the z axis (in the tool markermap basis). Value will be changed by trackbar

bool Register_P = false; //Becomes true when we receive a request from the server ordering to register a point
bool Register_M = false; //Becomes true when we receive a request from the server ordering to register an error margin

bool RegOrigin = false; //Becomes true when we receive a request from the server ordering to register the origin of the relative Basis
bool RegXAxis = false; //Becomes true when we receive a request from the server ordering to register the X axis of the relative Basis
bool RegYAxis = false; //Becomes true when we receive a request from the server ordering to register the Y axis of the relative Basis

bool RemoveLastRegistered = false; //Becomes true when we receive a request from the server ordering to delete the last registered point

bool Detecting = false; //Becomes true when we receive a request from the server ordering to check if a point is detected by the tooltip
int Point_Detected; //Detected point to send to the server


//////////////////////// Image that gets data from the frame pointer 
////////////////////////  AS GLOBAL VARIABLES (NEEDED)

#ifdef OLD_CAMERA
        //OLD CAMERA
        //Raw frame
        cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
        //Original image, that gets the cam frame from the pointer
        cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);
        //copied image, for printing the markers on the screen
        cv::Mat imageCopy;
#else
        //NEW CAMERA
        //raw frame
        cv::Mat raw_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);   
        //copied image
        cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC1);
#endif  

////////////////Variables used locally for the detection program//////////////

bool exiting = false; //For the image getter/webserver to know when to stop 
bool OneMarkerValid = false; //Check if the tool markermap is found
bool OneRelativeMarkerValid = false; //Check if the relative MarkerMap is found

bool IsStatic = false; //Check if we want static positionning
bool IsStaticSet = false; //Do we need to set static basis?

bool definePlane = true; //Do we need to define a plane (i.e are we in static mode?)
bool Calib_points = false; //Are we in calibration mode?
//bool Detect_points = false; //Are we in detection mode?
bool RegisterErrMargin = false; //Knowing if we are registering a point or its error margin (in calibration mode)
bool print_relative_coord = true; //Do we print relative coordinates or camera coordinates?



float sum_img = 0.0f, sum_comp = 0.0f;


vector< cv::Point2f > DrawnImgPoints; //Remembering drawn points
vector< cv::Point2f > ImagePoint; //2d projected point on the image (of the current tool tip)

///////Used for setting the static basis (in Static mode)///////
std::vector<cv::Point3d> planePoints;   // vector to hold plane points (if definedPlane is true)
std::vector<cv::Point2f> DrawplanePoints;   // vector to hold 2D plane points for drawing (if definedPlane is true)
const cv::Point3d upwardDirection = cv::Point3d(0, 0, 1); //For finding the upward direction

cv::Mat rodrigued(3, 3, CV_32F);                    // static relative basis rotation 3x3 matrix
cv::Mat relativeRvec(1, 3, CV_32F);                 // static relative basis rotation 1x3 matrix
cv::Mat relativeTvec(1, 3, CV_32F);                 // static relative basis location (translation)
cv::Mat relativeRTmat(4, 4, CV_32F);                // static relative basis homogeneous matrix
cv::Mat relativeRTmatInv(4, 4, CV_32F);             // static relative basis invert matrix

///////Used for the registered points and the coordinates in different basis///////
cv::Mat Real_coordinates; //For the coordinates of the tool tip (in the camera basis)
cv::Mat Relative_coordinates = cv::Mat::zeros(4, 1, CV_32F); //For the coordinates of the tool tip in the relative basis (either static or the relative markermap) 
vector< std::tuple< cv::Mat, cv::Mat, double, double > > registeredPoints; //3d coordinates in camera basis, 3D coordinates in relative basis, error margin (circle), error margin in pixels (circle) of the registered points

//vector< cv::Mat > Points_tool_coord; //Coordinates of the points in the tool basis

////////Aruco objects for detection//////
aruco::MarkerDetector MDetector; //Detector object
aruco::CameraParameters camParameters; //Camera object for saving camera parameters
aruco::MarkerMapPoseTracker MMTracker; //tracker for estimating the pose of the markermap
aruco::MarkerMapPoseTracker MMRelativeTracker; //Tracker for the relative merkermap

//aruco::MarkerMap mmap; //main MarkerMap //!\\ DOESNT WORK - THROWS SEGMENTATION FAULT !!


/***************************************************/
/*                                                 */
/******************** FUNCTIONS ********************/
/*                                                 */
/***************************************************/



//For handling cam Startup/Shutdown errors (DOESNT WORK !!)
/*class Cam_Exception : public std::exception
{
    const char* cam_err;
public:

    Cam_Exception(const char* _cam_err) : cam_err(_cam_err) {}
    
    const char* what() const throw()
    {
        return cam_err;
    }
};*/



/*
 * Load the configuration (MMap, Detector, Camera parameters...)
 */
void ParseConfig(const char *path, aruco::MarkerMap* mmap, aruco::MarkerMap* mmRelative)
{
    std::string line;
    std::ifstream file;
    file.open(path);

    if(!file.is_open())
    {
        std::cerr << "Error while opening : " << path << "\n";
        return;
    }

    std::cout << "\nParsing config file :\n";
    float markersSize = 1;

    while(getline(file, line))
    {
        char *linec = &line[0];
        char *lineType = std::strtok(linec, ":"); // get the first line element to get its type

        if (lineType == NULL || linec[0] == '#') // found a comment or an invalid line, ignoring it
            continue;
            
        // getting tags size
        if (strcmp(lineType, "markers_size") == 0)
        {
            markersSize = std::stod(std::strtok(nullptr, ", ()\n"));
            std::cout << "\tmarkers size = " << markersSize << "m\n";
        }

        // getting image size
        if (strcmp(lineType, "image_size") == 0)
        {
            imgSizeX = std::stoi(std::strtok(nullptr, ", ()\n"));
            imgSizeY = std::stoi(std::strtok(nullptr, ", ()\n"));
            std::cout << "\timage size = " << imgSizeX << "x" << imgSizeY << "\n";
        }

        // getting camera parameters
        if (strcmp(lineType, "camera_parameters") == 0)
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            camParameters.readFromXMLFile(name);
            camParameters.resize(cv::Size(imgSizeX, imgSizeY));
            std::cout << "\tcamera parameters = " << name << "\n";
            continue;
        }

        // getting tool markermap
        if (strcmp(lineType, "tool_markermap") == 0)
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            mmap->readFromFile(name);
            std::cout << "\ttool markermap = " << name << "\n";
            continue;
        }

        /////////////// getting detector configuration (NOT USED, SLOWS DOWN APPLICATION (WHY??) )/////////////////
        /*if (strcmp(lineType, "detector_config") == 0)
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            MDetector.loadParamsFromFile(name);
            std::cout << "\tdetector config = " << name << "\n";
            continue;
        }*/
       
        // getting is_static
        if (strcmp(lineType, "is_static") == 0 )//&& markerMapConfigRelatives.size() == 0)  just check we did not provide relative markermaps
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            if (strcmp(name, "true") == 0)
            {
                IsStatic = true;
                //DPCxOffset = 0;
                //DPCyOffset = 0;
                //DPCzOffset = 0;
                std::cout << "\tIsStatic = true\n";
            }
            else
                std::cout << "\tIsStatic = false\n";
            continue;
        }
        /*
        if (strcmp(lineType, "define_static_plane") == 0 && IsStatic) // check we are in static mode
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            if (strcmp(name, "true") == 0)
            {
                definePlane = true;
                std::cout << "\tdefinePlane = true\n";
            }
            else
                std::cout << "\tdefinePlane = false\n";
            continue;
        }*/

        // getting relative markermaps
        if (strcmp(lineType, "relative_markermap") == 0 && !IsStatic) // just check we do not want static positionning
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            mmRelative->readFromFile(name); // first token
            /*markerMapConfigRelatives.push_back(mmRelative);

            cv::Mat newT(1, 3, CV_32F);
            newT.at<float>(0) = std::stod(std::strtok(nullptr, ", ()\n")); // second token
            newT.at<float>(1) = std::stod(std::strtok(nullptr, ", ()\n")); // third token
            newT.at<float>(2) = std::stod(std::strtok(nullptr, ", ()\n")); // fourth token
            trackerRelativesT.push_back(newT);

            cv::Mat newR(1, 3, CV_32F);
            newR.at<float>(0) = std::stod(std::strtok(nullptr, ", ()\n")); // fifth token
            newR.at<float>(1) = std::stod(std::strtok(nullptr, ", ()\n")); // sixth token
            newR.at<float>(2) = std::stod(std::strtok(nullptr, ", ()\n")); // seventh token
            trackerRelativesR.push_back(newR);*/

            std::cout << "\trelative markermap = " << name << endl; // ", Toffset (in units not m) : " << newT << ", Roffset (in radians) : " << newR << "\n";
            continue;
        }
    }

    std::cout << "\n";

    // set dictionnary
    MDetector.setDictionary(mmap->getDictionary() );
    
    // setting up tool markermap config
    if (mmap->isExpressedInPixels())
        *mmap = mmap->convertToMeters(markersSize);

    if (camParameters.isValid() && mmap->isExpressedInMeters() )
        MMTracker.setParams(camParameters, *mmap);
    
    //Setting up relative markermap config
    if (mmRelative->isExpressedInPixels())
        *mmRelative = mmRelative->convertToMeters(markersSize);
        
    if (camParameters.isValid() && mmRelative->isExpressedInMeters() )
        MMRelativeTracker.setParams(camParameters, *mmRelative);
    
    
    /*
    // setting up relative markermaps configs
    for (int i = 0; i < markerMapConfigRelatives.size(); i++)
    {
        if (markerMapConfigRelatives[i].isExpressedInPixels())
            markerMapConfigRelatives[i] = markerMapConfigRelatives[i].convertToMeters(markersSize);
    }

    for (int i = 0; i < markerMapConfigRelatives.size(); i++)
    {
        if (camParameters.isValid() && markerMapConfigRelatives[i].isExpressedInMeters())
        {
            MarkerMapPoseTracker tRelative;
            tRelative.setParams(camParameters, markerMapConfigRelatives[i]);
            trackerRelatives.push_back(tRelative);
        }
    }*/
}

//Normalise vectors
void normaliseVec(cv::Point3d &p)
{
    p = p / cv::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

////Set Static Basis
void SetOrigin(const char &key, cv::Mat imageCopy)
{
    // print the needed messages on the image, depending on which state we are in
    cv::putText(imageCopy, "Static mode, press O to", cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
    
    if (definePlane && planePoints.size() == 0)
        cv::putText(imageCopy, "Place origin", cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
    else if (definePlane && planePoints.size() == 1)
        cv::putText(imageCopy, "Place x axis", cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
    else if (definePlane && planePoints.size() == 2)
        cv::putText(imageCopy, "Place y axis", cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
    
    
    /////////Check first if we can find the tool markermap
    if(!OneMarkerValid)
    {
        //Disabling the flags, because we will need to wait for another request from the server
        RegOrigin = false;
        RegXAxis = false;
        RegYAxis = false;
        return;
    }
    
    //3D coordinates in the tool markermap basis
    /*cv::Mat objectPoint(1, 3, CV_32FC1);

    objectPoint.at<float>(0, 0) = 0;
    objectPoint.at<float>(0, 1) = 0;
    objectPoint.at<float>(0, 2) = -zOffset; //for translating the tool tip on the z axis as needed
    
    //Project the 3d points in 2d
    std::vector<cv::Point2f> imgPoint;
    cv::projectPoints(objectPoint, MMTracker.getRvec(), MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, imgPoint); */
        
        
    // register Origin and Axis when R pressed (Or when request from the server is received)
    if ( (key == 'o' | key == 'O') || (definePlane && planePoints.size() == 0 && RegOrigin) || (definePlane && planePoints.size() == 1 && RegXAxis) || (definePlane && planePoints.size() == 2 && RegYAxis) )
    {
        
        
        //For having a 3 channels Mat (instead of 4) of the 3d coordinates of the points (in the camera basis)
        cv::Point3d coordinates;
        coordinates.x = Real_coordinates.at<float>(0, 0);
        coordinates.y = Real_coordinates.at<float>(1, 0);
        coordinates.z = Real_coordinates.at<float>(2, 0);

        //Saving 2D point
        DrawplanePoints.push_back(ImagePoint.at(0));
        
        //Saving 3D current point (in the CAMERA basis) 
        planePoints.push_back(coordinates);
        
        if (definePlane && planePoints.size() == 1)
        {
            
        }
        else if (definePlane && planePoints.size() == 2)
        {

        }
        else if (definePlane && planePoints.size() >= 3)
        {
            cv::Point3d Xvec = (planePoints[1] - planePoints[0]); // x given vector
            normaliseVec(Xvec);
            cout << Xvec << endl;
            cv::Point3d Yvec = (planePoints[2] - planePoints[0]); // y given vector, for now y is coplanar with x but not necessarily orthogonal
            normaliseVec(Yvec);
            cout << Yvec << endl;
            cv::Point3d Zvec = Xvec.cross(Yvec);                  // z normal vector to x and y
            normaliseVec(Zvec);
            cout << Zvec << endl;
            cv::Point3d correctedY = Zvec.cross(Xvec);            // new y vector normal to x and z so that they are all perpendicular
            normaliseVec(correctedY);
            cout << correctedY << endl;
            
            // explanation here : https://math.stackexchange.com/questions/1246679/expression-of-rotation-matrix-from-two-vectors
            /*cv::Point3d n = Xvec.cross(correctedY);
            normaliseVec(n);
            cout << n << endl;*/
            
            cv::Point3d n = Zvec;
            
            // change n direction depending on the one we registered while placing the first point
            /*double dotn = n.dot(upwardDirection);
            std::cout << dotn << "\n";
            if (dotn > 0)
                n = -n; // this to point upwards on a right handed basis*/
                
            /*cv::Point3d nxa = n.cross(Xvec);
            normaliseVec(nxa);
            cout << nxa << endl;*/
            
            cv::Point3d nxa = correctedY;
            
            
            //Setting the Translation relative to the camera (being equal to the first registered point of the basis)
            relativeTvec.at<float>(0) = planePoints[0].x;
            relativeTvec.at<float>(1) = planePoints[0].y;
            relativeTvec.at<float>(2) = planePoints[0].z;
            
            //Finding 3x3 rotation matrix
            cv::Mat newRot(3, 3, CV_32F);
            newRot.at<float>(0, 0) = Xvec.x;
            newRot.at<float>(1, 0) = Xvec.y;
            newRot.at<float>(2, 0) = Xvec.z;
            newRot.at<float>(0, 1) = nxa.x;
            newRot.at<float>(1, 1) = nxa.y;
            newRot.at<float>(2, 1) = nxa.z;
            newRot.at<float>(0, 2) = n.x;
            newRot.at<float>(1, 2) = n.y;
            newRot.at<float>(2, 2) = n.z;
            
            //saving relative Rvec
            cv::Rodrigues(newRot, relativeRvec);
            
             // get RT matrix
            cv::Rodrigues(relativeRvec, rodrigued);
            relativeRTmat.at<float>(0, 0) = rodrigued.at<float>(0, 0);
            relativeRTmat.at<float>(1, 0) = rodrigued.at<float>(1, 0);
            relativeRTmat.at<float>(2, 0) = rodrigued.at<float>(2, 0);
            relativeRTmat.at<float>(3, 0) = 0;
            relativeRTmat.at<float>(0, 1) = rodrigued.at<float>(0, 1);
            relativeRTmat.at<float>(1, 1) = rodrigued.at<float>(1, 1);
            relativeRTmat.at<float>(2, 1) = rodrigued.at<float>(2, 1);
            relativeRTmat.at<float>(3, 1) = 0;
            relativeRTmat.at<float>(0, 2) = rodrigued.at<float>(0, 2);
            relativeRTmat.at<float>(1, 2) = rodrigued.at<float>(1, 2);
            relativeRTmat.at<float>(2, 2) = rodrigued.at<float>(2, 2);
            relativeRTmat.at<float>(3, 2) = 0;
            relativeRTmat.at<float>(0, 3) = relativeTvec.at<float>(0);
            relativeRTmat.at<float>(1, 3) = relativeTvec.at<float>(1);
            relativeRTmat.at<float>(2, 3) = relativeTvec.at<float>(2);
            relativeRTmat.at<float>(3, 3) = 1;

            relativeRTmatInv = relativeRTmat.inv();
            
            IsStaticSet = true;
            Calib_points = true;
        }
    }
    
    
    
        
    if (definePlane && planePoints.size() == 1)
    {
        cv::drawMarker(imageCopy, DrawplanePoints.at(0), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw point as a cross
        cv::line(imageCopy, DrawplanePoints.at(0), ImagePoint.at(0), cv::Scalar(0, 165, 255), 2);
    }
    
    else if (definePlane && planePoints.size() == 2)
    {
        cv::drawMarker(imageCopy, DrawplanePoints.at(0), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw point as a cross
        cv::line(imageCopy, DrawplanePoints.at(0), DrawplanePoints.at(1), cv::Scalar(0, 165, 255), 2);
        cv::line(imageCopy, DrawplanePoints.at(0), ImagePoint.at(0), cv::Scalar(0, 165, 255), 2);
    }
    else if (definePlane && planePoints.size() >= 3)
    {
        
    }
    
    //Setting the flags back to false (because we want only on registerd point at a time)
    RegOrigin = false;
    RegXAxis = false;
    RegYAxis = false;
}



//For registering points and error margin
void Calibration_Points(const char &key,  cv::Mat imageCopy)
{
    
    //Check if we want to register a point
    if( (key == 'r' | key == 'R' | Register_P == true) && !RegisterErrMargin )
    {
        
        
        //For having a 3 channels Mat (instead of 4) of the coordinates in the camera basis
        cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
        coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
        coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
        coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);
        
        
        //Same (for relative basis)
        cv::Mat Curr_Rel_coordinates = cv::Mat(3, 1, CV_32FC1);
        Curr_Rel_coordinates.at<float>(0, 0) = Relative_coordinates.at<float>(0, 0);
        Curr_Rel_coordinates.at<float>(1, 0) = Relative_coordinates.at<float>(1, 0);
        Curr_Rel_coordinates.at<float>(2, 0) = Relative_coordinates.at<float>(2, 0);
            
        //if(IsStatic)
        //{
            //Then find the coordinates in the relative basis
            //Relative_coordinates = relativeRTmat.inv() * Real_coordinates;
        //}

        
        //remember the new 3D point (in both coordinates)
        auto tup = std::make_tuple(coordinates, Curr_Rel_coordinates, 0.0f, 0.0f);
        registeredPoints.push_back(tup);
        
        //Remember the new 2D projected point
        DrawnImgPoints.push_back(ImagePoint.at(0) );
        RegisterErrMargin = true;
        
        //Put registering flags back to false (because we need only one registration per request) 
        Register_P = false;
        Register_M = false;
    
    }
    
    //Check if we want to register an error margin
    if( (key == 'e' | key == 'E' | Register_M == true) && RegisterErrMargin )
    {
        
        
        //For having a 3 channels Mat (instead of 4)
        /*cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
        coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
        coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
        coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);*/
        
        cv::Mat Curr_Rel_coordinates = cv::Mat(3, 1, CV_32FC1);
        Curr_Rel_coordinates.at<float>(0, 0) = Relative_coordinates.at<float>(0, 0);
        Curr_Rel_coordinates.at<float>(1, 0) = Relative_coordinates.at<float>(1, 0);
        Curr_Rel_coordinates.at<float>(2, 0) = Relative_coordinates.at<float>(2, 0);
        
        //Finding the error circle radius
        //float disterror = norm(coordinates, std::get<0>(registeredPoints.back()), cv::NORM_L2, cv::noArray() );
        
        float disterror = norm(Curr_Rel_coordinates, std::get<1>(registeredPoints.back()), cv::NORM_L2, cv::noArray() );
        
        
        //remember the error circle radius
        std::get<2>(registeredPoints.back()) = disterror;
        
        //Variables for finding the 2D radius
        vector <cv::Point2f> center;
        
        //cv::projectPoints( std::get<1>(registeredPoints.back()), IsStatic ? relativeRvec : MMRelativeTracker.getRvec(), IsStatic ? relativeTvec : MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, center); //Project the 3d points in 2d
        center.push_back(DrawnImgPoints.back() );
        vector <cv::Point2f> border;
        //cv::projectPoints(Curr_Rel_coordinates, IsStatic ? relativeRvec : MMRelativeTracker.getRvec(), IsStatic ? relativeTvec : MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, border); //Project the 3d points in 2d
        border.push_back(ImagePoint.at(0) );
        
        //Finding the circle radius
        float disterror2D = norm(center, border, cv::NORM_L2);
        
        std::get<3>(registeredPoints.back()) = disterror2D;
        
        RegisterErrMargin = false;
        
        //Put registering flags back to false (because we need only one registration per request) 
        Register_P = false;
        Register_M = false;
    
    }
    
    //Check if we want to remove the last saved point
    if(key == 'x' | key == 'X' | RemoveLastRegistered)
    {
        //Stop registering error margin of that last point (if we are in that phase)
        RegisterErrMargin = false;
        
        //Delete the last registered point and its 2D projection (if there exists registered points)
        if(registeredPoints.size() > 0){
            registeredPoints.pop_back();
            DrawnImgPoints.pop_back();
        }
        
        //Put remove flag back to false (because we need only one per request) 
        RemoveLastRegistered = false;
    }
    /*if(DrawnImgPoints.size() > 0)
    {
        for (int i=0; i < DrawnImgPoints.size(); i++){
            //aruco::CvDrawingUtils::drawXYcross(imageCopy, camParameters, markers[0].Rvec, markers[0].Tvec, 0.005);///2 * imgSizeX / 1920 // adapting ui size to image size 
            for(int j=0; j < DrawnImgPoints.at(i).size(); j++){
                cv::drawMarker(imageCopy, DrawnImgPoints.at(i).at(j), cv::Scalar(0, 255, 0) );//Draw all the points as a cross
            }
        }
    }*/
}


void Detection_Points(cv::Mat imageCopy)
{
    //For having a 3 channels Mat (instead of 4)
    /*cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
    coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
    coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
    coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);*/
    
    cv::Mat Curr_Rel_coordinates = cv::Mat(3, 1, CV_32FC1);
    Curr_Rel_coordinates.at<float>(0, 0) = Relative_coordinates.at<float>(0, 0);
    Curr_Rel_coordinates.at<float>(1, 0) = Relative_coordinates.at<float>(1, 0);
    Curr_Rel_coordinates.at<float>(2, 0) = Relative_coordinates.at<float>(2, 0);
    
    //return to zero
    Point_Detected = 0;
    
    for(int i = 0; i < registeredPoints.size(); i++)
    {
        //Finding the distance between the tool and the registered point
        float disterror = norm(Curr_Rel_coordinates, std::get<1>(registeredPoints.at(i)), cv::NORM_L2, cv::noArray() );
        
        //If the tool tip appears to be inside the error radius of the current point, write in green, if not write in red
        if( disterror < std::get<2>(registeredPoints.at(i)) )
        {
            cv::putText(imageCopy, to_string(i), cv::Point(1800, 50*(i+2)), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 255, 0), 2);
            
            //For informing the server which point has be detected
            Point_Detected = i+1;
        }
        else
        {
            cv::putText(imageCopy, to_string(i), cv::Point(1800, 50*(i+2)), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 0, 255), 2);
        }  
    }
    
    //Indicating to the server that we are done detecting, the informations can be used
    Detecting = false;
    
}





//Launch Crow WebServer asynchronously (if not offline testing)

#ifndef OFFLINE_TEST

void Launch_Crow_WebServer(){

    crow::SimpleApp app; //define your crow application
    
    //app.loglevel(crow::LogLevel::Debug); //Afficher différents niveaux de log
    std::cout << std::system("pwd") << std::endl;//Check the directory of the executable (from linux command "pwd")
    
    //define your endpoint at the root directory
    CROW_ROUTE(app, "/test")([]{ // 

        //crow::mustache::context ctx;
        
        auto page = crow::mustache::load_text("view-stream.html"); //html page
    
        return page; 
    });
    
    
    //Route for handling requests and responses between client and server
    CROW_ROUTE(app, "/DataEx").methods(/*crow::HTTPMethod::POST,*/ crow::HTTPMethod::GET)([](const crow::request& req){ 
    
        std::string var = req.raw_url;
        std::string Body = req.body;
        cout << "Raw url received: " <<  var << endl;
        cout << "req body : " << Body << endl; 
        
        //Checking if we changed the wanted camera output 
        if(req.url_params.get("output") != nullptr )
        {
            //Print radio button value from URL
            const char* out = req.url_params.get("output");
            cout << "msg from client: " << out << endl; 
            
            if(strcmp(out, "Real") == 0){ //Output Color image
                Output_Video = 1;
            }
            else if(strcmp(out, "Bin") == 0){ //Output Binary image
                Output_Video = 2;
            }
            else if(strcmp(out, "Inactive") == 0){ //Don't output image
                Output_Video = 0;
            }
        }
        
        //Checking if we changed the Exposure time, Gain or Threshold through the slider
        if(req.url_params.get("Exposure_Time") != nullptr )
        {
            ExposureTime = std::stof(req.url_params.get("Exposure_Time"));
        }
        if(req.url_params.get("Gain") != nullptr )
        {
            Gain = std::stof(req.url_params.get("Gain"));
        }
        if(req.url_params.get("Threshold") != nullptr )
        {
            Threshold = std::stof(req.url_params.get("Threshold"));
        }
        
        //Checking if we want to register a point
        if(req.url_params.get("RegPoint") != nullptr)
        {
            Register_P = true;
            
            //While the point hasnt been registered, wait
            while(Register_P);
            
            //Save Relative coordinates and id of the last registered point as a json string, in order to send it to the client
             crow::json::wvalue last_point({
             {"id", registeredPoints.size()}, 
             {"action", "addition"},
             {"coordinates", crow::json::wvalue::list( { (double)std::get<1>(registeredPoints.back()).at<float>(0, 0) * 100, (double)std::get<1>(registeredPoints.back()).at<float>(1, 0) * 100, (double)std::get<1>(registeredPoints.back()).at<float>(2, 0) * 100 } )  }
             });
             
             //crow::json::wvalue last_point( crow::json::wvalue::list( { std::get<1>(registeredPoints.back()).at<float>(0, 0) * 100, std::get<1>(registeredPoints.back()).at<float>(1, 0) * 100, std::get<1>(registeredPoints.back()).at<float>(2, 0) * 100 } ) );
            //Send response
            return crow::response( last_point );
            
        }
        //Checking if we want to register an error margin
        if(req.url_params.get("RegErrMargin") != nullptr)
        {
            Register_M = true;
            
            //While the err margin hasnt been registered, wait
            while(Register_M);
            
            //Save error margin and id of the last registered point as a json string, in order to send it to the client
             crow::json::wvalue last_point_err({
             {"id", registeredPoints.size()}, 
             {"action", "addition err"},
             {"errorMargin", (double)std::get<2>(registeredPoints.back()) * 100}
             });
             
            //Send response
            return crow::response( last_point_err );
             
        }
        
        //Checking if we want to remove the last saved point
        if(req.url_params.get("RemoveLastP") != nullptr)
        {
            RemoveLastRegistered = true;
            
            //While the point hasnt been erased, wait
            while(RemoveLastRegistered);
            
            //Send the size +1 to the client for him to know which point to remove
            crow::json::wvalue last_point({
            {"id",registeredPoints.size() + 1},
            {"action", "deletion"}
            });
            //Send response
            return crow::response( last_point );
        } 
        
        
        
        //Checking if we want to register a point or an axis for the relative basis
        if(req.url_params.get("RegOrigin") != nullptr)
        {
            RegOrigin = true;
            
            //While the origin hasnt been registered, wait
            while(RegOrigin);
            
            //Send response to client, for him to know which point has been registered
            return crow::response( to_string(planePoints.size()) );
        }
        if(req.url_params.get("RegXAxis") != nullptr)
        {
            RegXAxis = true;
            
            //While the X axis hasnt been registered, wait
            while(RegXAxis);
            
            //Send response to client, for him to know which point has been registered
            return crow::response( to_string(planePoints.size()) );
        }
        if(req.url_params.get("RegYAxis") != nullptr)
        {
            RegYAxis = true;
            
            //While the Y axis hasnt been registered, wait
            while(RegYAxis);
            
            //Send response to client, for him to know which point has been registered
            return crow::response( to_string(planePoints.size()) );
        }
        
        //Check if we are detecting a point in detection mode (checking continuously, each 3000ms for example)
        if(req.url_params.get("Detected") != nullptr)
        {
            Detecting = true;
            
            cout << "while loop" << endl;
            //Wait for the Detection to be finished
            while(Detecting);
            
            cout << "Creating var" << endl;
            crow::json::wvalue last_point({
            {"id",Point_Detected},
            {"action", "CheckDetect"}
            });
            
            cout << "sending response" << endl;
             //Send response to client, for him to know which point has been detected
            return crow::response( last_point );
        }
        /*
        //cout << Body.find("output") << "   " << Body.find("Real") << "   " << Body.find("Bin") << endl;
        
        
        //Checking if we find "output" ind the request body
        if( Body.find("output") != string::npos  )
        {
            //Print radio button value from URL
            //const char* out = Body.c_str();
            //cout << "msg from client: " << out << endl; 
            
            //Then, checking if we find "Real" or "Bin", for knowing which image to display
            
            if( Body.find("Real") != string::npos ){  //strcmp(out, "Real") == 0
                cout << "FOund Real" << endl;
                Output_Video = false;
            }
            else if( Body.find("Bin") != string::npos ){  //strcmp(out, "Bin") == 0
                cout << "FOund Bin" << endl;
                Output_Video = true;
            }
        }       
        else
        {
            cout << "No output chosen" << endl;
        } */
       // crow::mustache::context ctx ({{"person", name}}); // 

        return crow::response(200);
    });
    
    
    /*//Route for responses (from server to client) to send infos about the basis
    CROW_ROUTE(app, "/Resp/BasisInfo")
    ([](crow::response& res) {
        cout << "Hi" << endl;
        //Send the number of registered point for the relative basis to the client
        res.write( to_string(planePoints.size()) );
        //res.write( to_string(img_count) );
        res.end();
    });
    
    //Route for responses (from server to client) to send the coordinates of each registered point
    CROW_ROUTE(app, "/Resp/Table")
    ([](crow::response& res) {
        cout << "Hi" << endl;
        res.end();
    });*/
    
    //set the port, set the app to run asynchronously (value is stored in a variable because for some reason it doesn't run asynchronously otherwise)
    std::future< void > runner = app.port(18080).run_async();
    
    
    //Wait for the bool "exiting" to be set to true by the main thread, then we can stop the app and join the threads
    while(!exiting);
    app.stop();

}
#endif



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


/*string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}*/



/**********************************************/
/*                                            */
/******************** MAIN ********************/
/*                                            */
/**********************************************/


/*
int main(int argc, char* argv[])
{  
    aruco::MarkerMap* mmap = new aruco::MarkerMap; //Tool (main) MarkerMap
    aruco::MarkerMap* mmRelative = new aruco::MarkerMap; //Relative MarkerMap
    
    ParseConfig(argv[1], mmap, mmRelative);
    
    //raw frame
      
        
//Use opengl and cuda for speedup
#if defined(ENABLE_GL_DISPLAY) && defined(ENABLE_GPU_UPLOAD)
    std::cout << "Using CUDA\n";
    cv::cuda::GpuMat gpu_frame;
#else
    std::cout << "Not using CUDA\n";
#endif

//Creation of images to display
#ifdef ENABLE_GL_DISPLAY
    std::cout << "Using openGL\n";
    cv::namedWindow("Tool", cv::WINDOW_OPENGL);
    //cv::namedWindow("Thresh", cv::WINDOW_OPENGL);
#else
    std::cout << "Not using openGL\n";
    cv::namedWindow("Tool", cv::WINDOW_NORMAL);
    //cv::namedWindow("Thresh", cv::WINDOW_NORMAL);
#endif  

    //Startup and start image acquisition
    if(VmbErrorSuccess != Open_and_Start_Acquisition() ){
        //throw Cam_Exception("Openning / Start Acquisition error !");
        cout << "Openning / Start Acquisition error !" << endl;
        return -1;
    }

    namedWindow("Tool", cv::WINDOW_NORMAL);
    
    cv::resizeWindow("Tool", 1920, 1080);


    //empty img used for displaying the trackbar
    cv::Mat TrackbarImg = cv::Mat::zeros(1, 500, CV_8UC1);


    //Set dictionary and detection mode
    //MDetector.setDictionary("ARUCO_MIP_16h3");
    MDetector.setDetectionMode(aruco::DM_FAST);
    
    
    //Changing manually the parameters of the detector
    aruco::MarkerDetector::Params &params= MDetector.getParameters();
    
    params.cornerRefinementM = aruco::CORNER_LINES; //Corner refinement method
    params.maxThreads = -1; //Max threads used in parallel
    params.lowResMarkerSize = 5; //minimum size of a marker in the low resolution image
    params.NAttemptsAutoThresFix = 1; //number of times that tries a random threshold in case of THRES_AUTO_FIXED
    params.error_correction_rate = 1; 
    
    cv::namedWindow("Trackbars", cv::WINDOW_GUI_EXPANDED);
        
    //
    
    //cv::resizeWindow("Trackbars", 100, 100);
    cv::moveWindow("Trackbars",0,750);
    //Creating trackbars
    cv::createTrackbar("Calibration Z offset", "Trackbars", NULL, 2000);
    //cv::createTrackbar("Track 2", "Trackbars", NULL, 2000);
    
    cv::setTrackbarPos("Calibration Z offset", "Trackbars", zOffset);


    //Start calibration right away if we are in dynamic mode
    if(!IsStatic)
        Calib_points = true;
        
            
    int Current_Image = 0;
    
    int key = 0;
    cv::TickMeter tm_full, tm_comp1, tm_comp2, tm_resize3, tm_detect, tm_end;
    
    while(key != 27){
        
        tm_full.start();
        tm_comp1.start(); 
        key = cv::waitKey(1) & 0xFF; 
        
        
        //If this image has already been processed, skip it
        if(Current_Image == img_count){
            continue;
        }
        Current_Image = img_count;
        
        //NEW CAMERA
        if(ptr_raw_frame == NULL){//Check if we started receiving
            continue;
        }

        raw_frame.data = ptr_raw_frame; //Getting raw frame (of the current image ?)      
        
        //resize down for printing the image
        cv::Mat resized_down = cv::Mat(1080, 1920, CV_8UC1);
        cv::resize(raw_frame, resized_down, resized_down.size(), 0, 0, cv::INTER_NEAREST);
        
        // creating color image for printing colored features on the image, and for better marker detection (because needs color image for input)
        cv::Mat img_color = cv::Mat(1080, 1920, CV_8UC3);
        cv::cvtColor(resized_down, img_color, cv::COLOR_GRAY2BGR ); 
       tm_comp1.stop();
        
        //Detect markers
        tm_detect.start();
        vector<aruco::Marker> detected_markers = MDetector.detect(img_color);
        tm_detect.stop();
        
        tm_comp2.start();
        //draw Markers
        for (auto m: detected_markers)
            m.draw(img_color, Scalar(0, 0, 255), 1);

            
        // draw help
        cv::putText(img_color, "'a' add current image for calibration", cv::Point(10,40), FONT_HERSHEY_SIMPLEX, 1 * 1920 / 1920., cv::Scalar(125,255,255), 2);
        cv::putText(img_color, "'esc' save and quit", cv::Point(10,80), FONT_HERSHEY_SIMPLEX, 1 * 1920 / 1920., cv::Scalar(125,255,255), 2);

#if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
        gpu_frame.upload(img_color);//resized_down
        cv::imshow("Tool", gpu_frame);
#else
        cv::imshow("Tool", img_color);//resized_down
#endif
        
        tm_comp2.stop();    
        tm_full.stop();
    }   
        
    cv::destroyAllWindows();
    
    if(VmbErrorSuccess != Stop_Acquisition_and_Close() ){
        //throw Cam_Exception("Closing / Stop Acquisition error !");
        cout << "Closing / Stop Acquisition error !" << endl;
        return -1;
    }
    
    cout << "Average comp 1 time: " << tm_comp1.getAvgTimeSec()*1000 << " ms" << endl;
    cout << "Average detection time: " << tm_detect.getAvgTimeSec()*1000 << " ms" << endl;
    cout << "Average comp 2 time: " << tm_comp2.getAvgTimeSec()*1000 << " ms" << endl;
    cout << "Average full computation time: " << tm_full.getAvgTimeSec()*1000 << " ms" << endl;
        
    return 0;   
}*/





int main(int argc, char** argv)
{
    //try
    //{
        aruco::MarkerMap* mmap = new aruco::MarkerMap; //Tool (main) MarkerMap
        aruco::MarkerMap* mmRelative = new aruco::MarkerMap; //Relative MarkerMap

         
        int cpt = 0, Mdetected = 0;
        unsigned char* ptr_cam_frame; //permettra de prendre l'image
        int bytes_used, key = -1, lastkey = -1;
        //TimerAvrg timerFull, timerGetImage, timerComputation;
        //string posMarkerMapfile = "stacked.yml";

        
        
#ifndef OFFLINE_TEST
        //Used for streaming video to web server
        //to hand opencv image by GStreamer
        cv::VideoWriter writer;
        
        //Pipeline for sending frames to server
        //string pipeline = "appsrc ! videoconvert ! videoscale ! video/x-raw,width=960,height=540 ! x264enc bitrate=256 ! video/x-h264,profile=\"high\" ! mpegtsmux ! hlssink playlist-root=http://10.5.83.185:8080 location=/home/rddoga/Desktop/hlstest/segment_%05d.ts playlist-location=/home/rddoga/Desktop/hlstest/playlist.m3u8 target-duration=5 max-files=5 ";

        //GStreamer pipeline for sending and encoding images to server
        string out = "appsrc ! videoconvert ! videoscale ! video/x-raw, framerate=24/1 ! avenc_mpeg1video bitrate=1500000 ! mpegtsmux ! curlhttpsink location=http://127.0.0.1:8080/rddoga"; //      "http://localhost:8080/rddoga""./test_video.avi"
        
        int codec = cv::VideoWriter::fourcc('P','I','M','1');
        cv::Size frame_size(640, 480);
        double fps = 24.0;
        
        //open the writer
        writer.open(out, cv::CAP_GSTREAMER, 0, fps, frame_size);//cv::CAP_FFMPEG
        
        if (!writer.isOpened()) {
            std::cout << "Could not open output file" << std::endl;
            exit(1); // or error handling
        }
#endif      

        
        //empty img used for displaying the trackbar
        cv::Mat TrackbarImg = cv::Mat::zeros(1, 500, CV_8UC1);


        //Set dictionary and detection mode
        //MDetector.setDictionary("ARUCO_MIP_16h3");
        MDetector.setDetectionMode(aruco::DM_FAST);
        
        
        //Changing manually the parameters of the detector
        aruco::MarkerDetector::Params &params= MDetector.getParameters();
        
        params.cornerRefinementM = aruco::CORNER_LINES; //Corner refinement method
        params.maxThreads = -1; //Max threads used in parallel
        params.lowResMarkerSize = 5; //minimum size of a marker in the low resolution image
        params.NAttemptsAutoThresFix = 1; //number of times that tries a random threshold in case of THRES_AUTO_FIXED
        params.error_correction_rate = 1; 
        
        
        
        //camParameters.readFromXMLFile("cam_calibration_3.yml");

        //mmap.readFromFile(posMarkerMapfile); //Read 3d markers position of the markermap from file
        //MMTracker.setParams(camParameters, mmap, 0.02); //Set Tracker params
        
        //Load configuration
        ParseConfig(argv[1], mmap, mmRelative);
        
#ifdef OLD_CAMERA        
        //OLD CAMERA
        //Initialise camera
        if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            cout << "Failed to open video" << endl;
            return -1;
        }
#endif        
      
        
#ifdef OFFLINE_TEST
    //Use opengl and cuda for speedup
    #if defined(ENABLE_GL_DISPLAY) && defined(ENABLE_GPU_UPLOAD)
            std::cout << "Using CUDA\n";
            cv::cuda::GpuMat gpu_frame;
    #else
            std::cout << "Not using CUDA\n";
    #endif
    
    //Creation of images to display
    #ifdef ENABLE_GL_DISPLAY
            std::cout << "Using openGL\n";
            cv::namedWindow("Tool", cv::WINDOW_OPENGL);
            //cv::namedWindow("Thresh", cv::WINDOW_OPENGL);
    #else
            std::cout << "Not using openGL\n";
            cv::namedWindow("Tool", cv::WINDOW_NORMAL);
            //cv::namedWindow("Thresh", cv::WINDOW_NORMAL);
    #endif
    
        //Resizing for better display
       // cv::resizeWindow("Thresh", 1920, 1080);
        cv::resizeWindow("Tool", 1920, 1080);
#endif 
        
//        cv::namedWindow("Tool", cv::WINDOW_GUI_EXPANDED);
        cv::namedWindow("Trackbars", cv::WINDOW_GUI_EXPANDED);
        
        //
        
        //cv::resizeWindow("Trackbars", 100, 100);
        cv::moveWindow("Trackbars",0,750);
        //Creating trackbars
        cv::createTrackbar("Calibration Z offset", "Trackbars", NULL, 2000);
        //cv::createTrackbar("Track 2", "Trackbars", NULL, 2000);
        
        cv::setTrackbarPos("Calibration Z offset", "Trackbars", zOffset);


        //Start calibration right away if we are in dynamic mode
        if(!IsStatic)
            Calib_points = true;
            
        // multithreaded image getter
        ///std::thread t1(FrameGetter, std::ref(originalImage));
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));

    //Launching web server in another thread (if testing online)
#ifndef OFFLINE_TEST        
        std::thread t1(Launch_Crow_WebServer);
#endif
        
        
#ifndef OLD_CAMERA            
        //NEW CAMERA
        //Startup and start image acquisition
        if(VmbErrorSuccess != Open_and_Start_Acquisition() ){
            //throw Cam_Exception("Openning / Start Acquisition error !");
            cout << "Openning / Start Acquisition error !" << endl;
            return -1;
        }
#endif        

        int Current_Image = 0;
        
        /////Timers
        cv::TickMeter tm_full, tm_image, tm_copy, tm_detect, tm_comp1, tm_comp2, tm_comp3;
                   
        /////////////////////////////////////////////////            
        ///////////////MAIN WHILE LOOP///////////////////
        /////////////////////////////////////////////////
        while (key != 27){
            
            tm_full.start();
            // timerFull.start();
            
            //start = clock();
            
            //timerGetImage.start();
            //////////////IF NOT MULTITHREADED////////////////////////
            
            key = cv::waitKey(1) & 0xFF; //Registering pressed key
            
#ifdef OLD_CAMERA       
            tm_image.start();      
            // OLD CAMERA
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
            tm_image.stop();                  
#else           
            
            //NEW CAMERA
            tm_copy.start();
            
            //If this image has already been processed, skip it
            if(Current_Image == img_count){
                continue;
            }
            Current_Image = img_count;
        
            //Changing features updated through webserver (if needed)
            VmbErrorType err = cameras[0]->GetFeatureByName ( "Gain", pFeature );
            err = pFeature -> SetValue(Gain);
            //err = pFeature -> GetRange( min, max );
            if(VmbErrorSuccess != err){
                cout << "Could not set Gain" << endl;
                return err;
            }
            
            err = cameras[0]->GetFeatureByName ( "ExposureTime", pFeature );
            err = pFeature -> SetValue(ExposureTime);
            if(VmbErrorSuccess != err){
                cout << "Could not set exposure time" << endl;
                return err;
            }
            
            if(ptr_raw_frame == NULL){//Check if we started receiving
                continue;
            }
             
            raw_frame.data = ptr_raw_frame; //Getting raw frame (of the current image ?) 
            //originalImage = raw_frame.clone(); //Copying image to free the image pointer
            
            //DEALLOCATE FRAME POINTER MEMORY AFTER COPYING THE IMAGE
            //delete ptr_raw_frame;          
#endif            
            
            //Setting up threshold, if changed through webserver, before detection
            if(Threshold > 0){
                params.ThresHold = Threshold;
            }

#ifdef OLD_CAMERA                 
            //OLD CAMERA   
            tm_copy.start();
            //Copying image for showing the detected markers
            originalImage.copyTo(imageCopy);
            tm_copy.stop();
#else            
            //NEW CAMERA
            
            // First resizing image to HD
            cv::Mat imgtmp = cv::Mat(1080, 1920, CV_8UC3);

            cv::resize(raw_frame, imgtmp, imgtmp.size(), 0, 0, cv::INTER_NEAREST);
            
            //Copy image to the right format
            cv::Mat imageCopy = cv::Mat(1080, 1920, CV_8UC3);
            
            cv::cvtColor(imgtmp, imageCopy, cv::COLOR_GRAY2BGR ); // for printing colored features on the image, and for better marker detection (because needs color image for input)
            tm_copy.stop();
#endif
            
            tm_detect.start();
            //detecting markers
            vector<aruco::Marker> markers=MDetector.detect(imageCopy,camParameters,0.02);
            //vector<aruco::Marker> markers=MDetector.detect(imageCopy);
            
            
            //get thresholded image
            cv::Mat imageBin = MDetector.getThresholdedImage();
            cv::cvtColor(imageBin, imageBin, cv::COLOR_GRAY2BGR ); // for sending the image
            


            tm_detect.stop();
            
            tm_comp1.start();
            //Check if we can find the tool markermap
            if(MMTracker.estimatePose(markers)){
                OneMarkerValid = true;
            }else{
                OneMarkerValid = false;
            }
            
            
            //Check if we can find the relative markermap
            if(!IsStatic){
                if(MMRelativeTracker.estimatePose(markers) ){
                    OneRelativeMarkerValid = true;
                }else{
                    OneRelativeMarkerValid = false;
                }
            }
            
            
            //if (MMTracker.isValid())
              //  cout<<MMTracker.getRvec()<<" "<<MMTracker.getTvec()<<endl;
                
            if (markers.size() > 0){
                Mdetected++;
                for(size_t i=0;i<markers.size();i++){
                    //draw in the image
                    markers[i].draw(imageCopy, cv::Scalar(0, 0, 255), 2, false);
                }
            }
            
            for(auto m:markers){
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camParameters, m.Rvec, m.Tvec, 0.1);
                aruco::CvDrawingUtils::draw3dCube(imageCopy, m, camParameters, 1);
                //cout<<m.Rvec<<" "<<m.Tvec<<endl;
            }
            tm_comp1.stop();
            
            
            tm_comp2.start();
            //For Preventing registration of many points at once if a key is pressed too long
            if(lastkey == key && key != -1){
                lastkey = key;
                key = -1;
            }
            else
            {
                lastkey = key;
            }
            
            
            ///Deciding what coordinates we want to print out
            if(key == 'z' | key == 'Z'){
                print_relative_coord = !print_relative_coord;
                //cout << "CHANGE " << endl;
            }
                
            //Draw relative markermap axis
            if(!IsStatic && OneRelativeMarkerValid )
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camParameters, MMRelativeTracker.getRvec(), MMRelativeTracker.getTvec(), 0.02, 2);
                    
            
            /////////////DRAWING TOOL TIP AND AXIS EACH FRAME, AND GETTING 3D POSITION OF THE TOOL TIP (IN EACH BASIS)//////////////
            if(OneMarkerValid)
            {
                //Draw tool axis
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camParameters, MMTracker.getRvec(), MMTracker.getTvec(), 0.02, 2);

                
                //Take value from trackbar
                zOffset = cv::getTrackbarPos("Calibration Z offset", "Trackbars") / 10000.;
                
                //3d coordinates of the tool tip (in the tool basis)
                cv::Mat objectPoint(1, 3, CV_32F);
                objectPoint.at<float>(0, 0) = 0;
                objectPoint.at<float>(0, 1) = 0;
                objectPoint.at<float>(0, 2) = -zOffset; //for translating the tool tip on the z axis as needed
                //vector <cv::Point2f> imagePoint;//Vector needed for drawing the point
                //cv::projectPoints(objectPoint, MMTracker.getRvec(), MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, ImagePoint); //Project the 3d points in 2d
                //cv::drawMarker(imageCopy, ImagePoint.at(0), cv::Scalar(0, 120, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); //Draw cross for the tool tip on each frame
                
                //Coordinates of the tooltip in the markermap basis (4x1 matrix needed for swaping basis)
                cv::Mat objectPoint_41 = cv::Mat(4, 1, CV_32F);
                objectPoint_41.at<float>(0, 0) = objectPoint.at<float>(0, 0);
                objectPoint_41.at<float>(1, 0) = objectPoint.at<float>(0, 1);
                objectPoint_41.at<float>(2, 0) = objectPoint.at<float>(0, 2);
                objectPoint_41.at<float>(3, 0) = 1;
                
                //calculating the 3d coordinates in the camera basis
                Real_coordinates = MMTracker.getRTMatrix() * objectPoint_41;
                
                cv::Mat Rel_objectPoint = cv::Mat(1, 3, CV_32F); 
                
                
                //Then calculate the coordinates in the relative basis (and draw the TOOL TIP, either in relative or in camera coordinates)
                
                //If static mode
                if(IsStatic && IsStaticSet)
                {
                    Relative_coordinates = relativeRTmatInv * Real_coordinates;

                    Rel_objectPoint.at<float>(0, 0) = Relative_coordinates.at<float>(0, 0);
                    Rel_objectPoint.at<float>(0, 1) = Relative_coordinates.at<float>(1, 0);
                    Rel_objectPoint.at<float>(0, 2) = Relative_coordinates.at<float>(2, 0);
                    
                    cv::projectPoints(Rel_objectPoint, relativeRvec, relativeTvec, camParameters.CameraMatrix, camParameters.Distorsion, ImagePoint); //Project the 3d points in 2d
                    cv::drawMarker(imageCopy, ImagePoint.at(0), cv::Scalar(0, 120, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); //Draw cross for the tool tip on each frame
                }
                //If dynamic
                else if (!IsStatic && OneRelativeMarkerValid )
                {
                    Relative_coordinates = MMRelativeTracker.getRTMatrix().inv() * Real_coordinates;
                    //cout << "RT2 : " << MMRelativeTracker.getRTMatrix().inv() << endl;
                    
                    Rel_objectPoint.at<float>(0, 0) = Relative_coordinates.at<float>(0, 0);
                    Rel_objectPoint.at<float>(0, 1) = Relative_coordinates.at<float>(1, 0);
                    Rel_objectPoint.at<float>(0, 2) = Relative_coordinates.at<float>(2, 0);
                    
                    cv::projectPoints(Rel_objectPoint, MMRelativeTracker.getRvec(), MMRelativeTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, ImagePoint); //Project the 3d points in 2d
                    cv::drawMarker(imageCopy, ImagePoint.at(0), cv::Scalar(0, 120, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); //Draw cross for the tool tip on each frame
                }
                else
                {
                    cv::projectPoints(objectPoint, MMTracker.getRvec(), MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, ImagePoint); //Project the 3d points in 2d
                    cv::drawMarker(imageCopy, ImagePoint.at(0), cv::Scalar(0, 120, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); //Draw cross for the tool tip on each frame
                }
                
                //cout << "RT1 : " << MMTracker.getRTMatrix() << endl;
                //cout << "in : " << objectPoint_41 << endl;
                //cout << "out : " << Real_coordinates << endl << endl;
                
                //cout << "RTbase : " << relativeRTmat << endl;
                
                //cout << "in : " << Real_coordinates << endl;
                //cout << "out : " << Relative_coordinates << endl << endl;
                
                
                if(Calib_points){
                
                    //Choosing if we print Relative or Camera coordinates
                    string R_coordinates = print_relative_coord ? ("Relative : ( " + to_string(Relative_coordinates.at<float>(0, 0)*100) + "; " + to_string(Relative_coordinates.at<float>(1, 0)*100) + "; " + to_string(Relative_coordinates.at<float>(2, 0)*100) + " )" ) : ( "Camera : ( " + to_string(Real_coordinates.at<float>(0, 0)*100) + "; " + to_string(Real_coordinates.at<float>(1, 0)*100) + "; " + to_string(Real_coordinates.at<float>(2, 0)*100) + " )" );
                    
                    //Print tool coordinates
                    //string Tool_coordinates = "Tool : ( " + to_string(Relative_coordinates.at<float>(0, 0)*100) + "; " + to_string(Relative_coordinates.at<float>(1, 0)*100) + "; " + to_string(Relative_coordinates.at<float>(2, 0)*100) + " )" ;
                    
                    //printing out the coordinates of the tool tip
                    cv::putText(imageCopy, "Coordinates of the tool tip :", cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
                    cv::putText(imageCopy, R_coordinates, cv::Point(5, 150), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
                }
            }
            
            tm_comp2.stop();
            
            
           
            /////////////////IF THE ORIGIN IS NOT SET (and we want static positionning), SET IT ////////////////////////
            if(!IsStaticSet && IsStatic)
            {
                SetOrigin(key, imageCopy);
            }
            
            ///////////DRAW THE BASIS OF THE WORKING SURFACE///////////
            if(IsStaticSet && IsStatic)
                
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camParameters, relativeRvec, relativeTvec, 0.07, 2);
            
            
            ////////////DETECT POINTS (At any time)
            cv::putText(imageCopy, "Detected Points", cv::Point(1800, 50), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(255, 0, 255), 2);
            if(OneMarkerValid)
            {
                Detection_Points(imageCopy);
            }
                    
                    
            /////////////////IF THE ORIGIN IS SET (OR WE DONT WANT STATIC POSITIONNING), DO THE CALIBRATION / DETECTION ///////////////////
            if(IsStaticSet || !IsStatic)
            {
                
                ///////CHANGE MODE IF THE ORIGIN WAS SET
                
                /*if(key == 'd' | key == 'D')
                {
                    if(Calib_points){
                        Detect_points = true;
                        Calib_points = false;
                    }
                }*/
                
                if(key == 'c' | key == 'C')
                {
                    //if(Detect_points){
                        Calib_points = true;
                    //    Detect_points = false;
                    //}
                }
                  
                //////IF WE ARE IN CALIBRATION MODE
                if(Calib_points)
                {
                    cv::putText(imageCopy, "Calibration Mode", cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(255, 0, 255), 2);
                    
                    //Check if we can find the tool markermap (plus the relative markermap if we dont want static positionning)
                    if(OneMarkerValid )
                    {
                        Calibration_Points(key, imageCopy);
                        
                        //Drawing the moving error circle of the current point
                        if(RegisterErrMargin)
                        {
                            //For having a 3 channels Mat (instead of 4)
                            //cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
                            //coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
                            //coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
                            //coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);

                            //cv::Mat Curr_Rel_coordinates = cv::Mat(3, 1, CV_32FC1);
                            //Curr_Rel_coordinates.at<float>(0, 0) = Relative_coordinates.at<float>(0, 0);
                            //Curr_Rel_coordinates.at<float>(1, 0) = Relative_coordinates.at<float>(1, 0);
                            //Curr_Rel_coordinates.at<float>(2, 0) = Relative_coordinates.at<float>(2, 0);
            
            
                            //Variables for finding the 2D radius
                            vector <cv::Point2f> center;
                            center.push_back(DrawnImgPoints.back() );
                            //cv::projectPoints( std::get<1>(registeredPoints.back()), IsStatic ? relativeRvec : MMRelativeTracker.getRvec(), IsStatic ? relativeTvec : MMRelativeTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, center); //Project the 3d points in 2d

                            vector <cv::Point2f> border;
                            border.push_back(ImagePoint.at(0) );
                            //cv::projectPoints(Curr_Rel_coordinates, IsStatic ? relativeRvec : MMRelativeTracker.getRvec(), IsStatic ? relativeTvec : MMRelativeTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, border); //Project the 3d points in 2d

                            //Finding the circle radius
                            float radius = norm(center, border, cv::NORM_L2);
                            
                            //Draw circle around the last point, following the tool tip
                            cv::circle(imageCopy, center.at(0), radius, cv::Scalar(0, 0, 255), 2);
                        }
                        
                    }
                }
                
                ///////////// DETECT POINTS
                //if(Detect_points)
               //{
                    
               // }
                
                
                
                ////////////////////DRAWING///////////////
                //Drawing each registered point and error circle (each frame)
                
                //cout << DrawnImgPoints.size() << endl;
                if(DrawnImgPoints.size() > 0 && ( IsStatic || (!IsStatic && OneRelativeMarkerValid) ) )
                {
                    for (int i=0; i < registeredPoints.size(); i++){

                        //cv::Mat rvec, tvec;
                        //rvec.create(3, 1, CV_32F);
                        //rvec = cv::Mat::zeros(3, 1, CV_32F);
                        //tvec.create(3, 1, CV_32F);
                        //tvec = cv::Mat::zeros(3, 1, CV_32F);
                    
                         //Project point from RELATIVE COORDINATES (so that it can follow the moving basis)
                        vector <cv::Point2f> imagePoint;
                        cv::projectPoints(std::get<1>(registeredPoints.at(i)), IsStatic ? relativeRvec : MMRelativeTracker.getRvec(), IsStatic ? relativeTvec : MMRelativeTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, imagePoint); //Project the 3d points in 2d
                        
                        //Draw point
                        cv::drawMarker(imageCopy, imagePoint.at(0) , cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw all the points as a cross

                        //Draw error circle
                        if(std::get<3>(registeredPoints.at(i)) > 0.0f)
                            cv::circle(imageCopy, imagePoint.at(0) , std::get<3>(registeredPoints.at(i)), cv::Scalar(128, 0, 255), 2);
                        
                        
                        
                        //Write coordinates on the image (in the relative basis) if not detection mode
                        if(/*!Detect_points && */OneMarkerValid){
                            string Rel_coordinates = "P" + to_string(i+1) + " : ( " + to_string( (std::get<1>(registeredPoints.at(i))).at<float>(0, 0) *100) + "; " + to_string( (std::get<1>(registeredPoints.at(i))).at<float>(1, 0) *100) + "; " + to_string( (std::get<1>(registeredPoints.at(i))).at<float>(2, 0) *100) + " )" ;
                            cv::putText(imageCopy, Rel_coordinates, cv::Point(5, 50*(i+4)), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(255, 0, 0), 2);
                            
                            
      //////////////////JUST FOR DEBUG (TO DELETE LATER)///////////////////////  
                          
                            //For having a 3 channels Mat (instead of 4) of the coordinates in the camera basis
                            cv::Mat Real_pt_coord = cv::Mat(4, 1, CV_32FC1);
                            Real_pt_coord.at<float>(0, 0) = std::get<0>(registeredPoints.at(i)).at<float>(0, 0);
                            Real_pt_coord.at<float>(1, 0) = std::get<0>(registeredPoints.at(i)).at<float>(1, 0);
                            Real_pt_coord.at<float>(2, 0) = std::get<0>(registeredPoints.at(i)).at<float>(2, 0);
                            Real_pt_coord.at<float>(3, 0) = 1;
                            
                            //Find the coordinates of the point in the tool basis
                            cv::Mat point_tool_coordinates = MMTracker.getRTMatrix().inv() * Real_pt_coord;
            
                            //Write coordinates in the tool basis (for debug)
                            //string str_tool_coordinates = "P" + to_string(i+1) + " : ( " + to_string( point_tool_coordinates.at<float>(0, 0) *100) + "; " + to_string( point_tool_coordinates.at<float>(1, 0) *100) + "; " + to_string( point_tool_coordinates.at<float>(2, 0) *100) + " )" ;
                            //cv::putText(imageCopy, str_tool_coordinates, cv::Point(5, 50*(i+4)), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(255, 0, 0), 2);
                        }
                    }
                }          
            }        
            cpt++;

#ifdef OFFLINE_TEST //If we are testing offline
      
           // cv::Mat imgPrint = cv::Mat(1080, 1920, CV_8UC3);

           // cv::resize(imageCopy, imgPrint, imgPrint.size(), 0, 0, cv::INTER_NEAREST);
             tm_comp3.start();
    #if (defined ENABLE_GL_DISPLAY) && (defined ENABLE_GPU_UPLOAD)
                gpu_frame.upload(imageCopy);//resized_down
                cv::imshow("Tool", gpu_frame);
    #else
                cv::imshow("Tool", imageCopy);//resized_down
    #endif   
            
            //Resizing and showing imageBin with opencv  
            //cv::resize(imageBin, imageBin, cv::Size(1920, 1080), 0, 0, cv::INTER_NEAREST);
            //cv::imshow("Thresh", imageBin);
            
#else //If we are sending to the server        
            cv::Mat imgPrint = cv::Mat(480, 640, CV_8UC3);            
            //resize down for printing the image on the client
            
            //Checking if we want to output video to the server
            if(Output_Video != 0)
            {
                //If so, Checking if we want to output the binary image or the color image (to the server)
                if(Output_Video == 1)
                {
                    //Color image output
                    cv::resize(imageCopy, imgPrint, imgPrint.size(), 0, 0, cv::INTER_NEAREST);
                }
                else if(Output_Video == 2) 
                {   //Binary image output
                    cv::resize(imageBin, imgPrint, imgPrint.size(), 0, 0, cv::INTER_NEAREST);
                }
                
                //then hand image to websocket by Gstreamer 
                writer.write(imgPrint);
            }
#endif         
            
            tm_comp3.stop();
            
            tm_full.stop();
            
        } 
        
        /////////////END WHILE LOOP//////////////

#ifdef OLD_CAMERA             
        // exiting program
        if (helper_deinit_cam() < 0)
            fprintf(stderr, "Failed to deinitialise camera : %m\n");
#else            
        //NEW CAMERA
        //Stop image acquisition and shutdown camera and API
        if(VmbErrorSuccess != Stop_Acquisition_and_Close() ){
            //throw Cam_Exception("Closing / Stop Acquisition error !");
            cout << "Closing / Stop Acquisition error !" << endl;
            return -1;
        }
#endif       

#ifndef OFFLINE_TEST
        exiting = true; //Indicate that we have to stop the image getter thread
        t1.join(); //Wait for the other thread to finish
#endif
       
        //Clearing all windows
        cv::destroyAllWindows();
        
        //Closing Video Writer
        //writer.release();

        
        delete mmap; //delete the mmap pointer 
        delete mmRelative; //delete the mmRelative pointer 

        

#ifdef OLD_CAMERA                    
        cout << "Average image catch time (old camera): " << tm_image.getAvgTimeSec() * 1000 << " ms" << std::endl;
#endif         
        cout << "Average image copy time : " << tm_copy.getAvgTimeSec() * 1000 << " ms" << std::endl;
        cout << "Average detection time : " << tm_detect.getAvgTimeSec() * 1000 << " ms" << std::endl;
        cout << "Average comp 1 time : " << tm_comp1.getAvgTimeSec() * 1000 << " ms" << std::endl;
        cout << "Average comp 2 time : " << tm_comp2.getAvgTimeSec() * 1000 << " ms" << std::endl;
        cout << "Average comp 3 time : " << tm_comp3.getAvgTimeSec() * 1000 << " ms" << std::endl;
        cout << "Average full computation time : " << tm_full.getAvgTimeSec() * 1000 << " ms" << std::endl;  
        
        //std::cout << "Average full time (timer 2): " << ((sum_comp+sum_img)/cpt) * 1000 << " ms" << std::endl;
        cout << "Real average FPS  : "<< tm_full.getFPS() << " fps" << endl << endl;
        
        cout << "MArker catch frequency : " << (float)Mdetected*100/cpt << " % of the time." << endl;
        
    //}
    //catch(std::exception& ex)
    //{
    //    cout << "Exception : " << ex.what() << "\n";
    //}
       
    return 0;
    
}


