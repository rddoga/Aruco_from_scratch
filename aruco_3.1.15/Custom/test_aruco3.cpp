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
#include <tuple>
#include <iostream>
#include <fstream>
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
bool OneMarkerValid = false; //Check if the tool markermap is found
bool IsStaticSet = false; //Do we need to set static basis?
bool definePlane = true; //Do we need to define a plane (i.e are we in static mode?)
bool Calib_points = false; //Are we in calibration mode?
bool Detect_points = false; //Are we in detection mode?
bool RpressedOnce = false; //Knowing if we are registering a point or its error margin (in calibration mode)

float sum_img = 0.0f, sum_comp = 0.0f;
//vector< vector< cv::Point2f > > DrawnImgPoints; //Remembering drawn points

vector< cv::Point2f > DrawnImgPoints; //Remembering drawn points
vector< cv::Point2f > imagePoint; //2d projected point on the image (of the current tool tip)
//std::vector<cv::Point3d> planePoints;   // vector to hold plane points (if definedPlane is true)

std::vector<cv::Point3d> planePoints;   // vector to hold plane points (if definedPlane is true)
std::vector<cv::Point2f> DrawplanePoints;   // vector to hold 2D plane points for drawing (if definedPlane is true)
const cv::Point3d upwardDirection = cv::Point3d(0, 0, 1); //For finding the upward direction

cv::Mat Real_coordinates = cv::Mat(4, 1, CV_32FC1); //For the coordinates of the tool tip
vector< std::tuple< cv::Mat, double, double > > registeredPoints; //3d coordinates, error margin (circle), error margin in pixels (circle) of the registered points

cv::Mat rodrigued(3, 3, CV_32F);                    // master point rotation 3x3 matrix
cv::Mat relativeRvec(1, 3, CV_32F);                 // master point rotation
cv::Mat relativeTvec(1, 3, CV_32F);                 // master point location
cv::Mat relativeRTmat(4, 4, CV_32F);                // master point homogeneous matrix


aruco::MarkerDetector MDetector; //Detector object
aruco::CameraParameters camParameters; //Camera object for saving camera parameters
aruco::MarkerMapPoseTracker MMTracker; //tracker for estimating the pose of the markermap
//aruco::MarkerMap mmap; //main MarkerMap //!\\ DOESNT WORK - THROWS SEGMENTATION FAULT !!


/***************************************************/
/*                                                 */
/******************** FUNCTIONS ********************/
/*                                                 */
/***************************************************/

/*
 * Load the configuration (MMap, Detector, Camera parameters...)
 */
void ParseConfig(const char *path, aruco::MarkerMap* mmap)
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
       /*
        // getting is_static
        if (strcmp(lineType, "is_static") == 0 && markerMapConfigRelatives.size() == 0) // just check we did not provide relative markermaps
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            if (strcmp(name, "true") == 0)
            {
                isStatic = true;
                DPCxOffset = 0;
                DPCyOffset = 0;
                DPCzOffset = 0;
                std::cout << "\tisStatic = true\n";
            }
            else
                std::cout << "\tisStatic = false\n";
            continue;
        }
        
        if (strcmp(lineType, "define_static_plane") == 0 && isStatic) // check we are in static mode
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
        }

        // getting relative markermaps
        if (strcmp(lineType, "relative_markermap") == 0 && !isStatic) // just check we do not want static positionning
        {
            char *name = std::strtok(nullptr, ",; ()\n");
            MarkerMap mmRelative;
            mmRelative.readFromFile(name); // first token
            markerMapConfigRelatives.push_back(mmRelative);

            cv::Mat newT(1, 3, CV_32F);
            newT.at<float>(0) = std::stod(std::strtok(nullptr, ", ()\n")); // second token
            newT.at<float>(1) = std::stod(std::strtok(nullptr, ", ()\n")); // third token
            newT.at<float>(2) = std::stod(std::strtok(nullptr, ", ()\n")); // fourth token
            trackerRelativesT.push_back(newT);

            cv::Mat newR(1, 3, CV_32F);
            newR.at<float>(0) = std::stod(std::strtok(nullptr, ", ()\n")); // fifth token
            newR.at<float>(1) = std::stod(std::strtok(nullptr, ", ()\n")); // sixth token
            newR.at<float>(2) = std::stod(std::strtok(nullptr, ", ()\n")); // seventh token
            trackerRelativesR.push_back(newR);

            std::cout << "\trelative markermap = name : " << name << ", Toffset (in units not m) : " << newT << ", Roffset (in radians) : " << newR << "\n";
            continue;
        }*/
    }

    std::cout << "\n";

    // set dictionnary
    MDetector.setDictionary(mmap->getDictionary());
    
    // setting up tool markermap config
    if (mmap->isExpressedInPixels())
        *mmap = mmap->convertToMeters(markersSize);

    if (camParameters.isValid() && mmap->isExpressedInMeters())
        MMTracker.setParams(camParameters, *mmap);
    
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
    if(OneMarkerValid)
    {
        //3D coordinates in the tool markermap basis
        cv::Mat objectPoint(1, 3, CV_32FC1);

        objectPoint.at<float>(0, 0) = 0;
        objectPoint.at<float>(0, 1) = 0;
        objectPoint.at<float>(0, 2) = -zOffset; //for translating the tool tip on the z axis as needed
        
        //Project the 3d points in 2d
        std::vector<cv::Point2f> imgPoint;
        cv::projectPoints(objectPoint, MMTracker.getRvec(), MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, imgPoint); 
            
            
        // register origin when R pressed
        if (key == 'o' || key == 'O')
        {
            //For having a 3 channels Mat (instead of 4) of the 3d coordinates of the points (in the camera basis)
            cv::Point3d coordinates;
            coordinates.x = Real_coordinates.at<float>(0, 0);
            coordinates.y = Real_coordinates.at<float>(1, 0);
            coordinates.z = Real_coordinates.at<float>(2, 0);

            //Saving 2D point
            DrawplanePoints.push_back(imgPoint.at(0));
            
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
                cv::Point3d correctedY = Xvec.cross(Zvec);            // new y vector normal to x and z so that they are all perpendicular
                normaliseVec(correctedY);
                cout << correctedY << endl;
                
                // explanation here : https://math.stackexchange.com/questions/1246679/expression-of-rotation-matrix-from-two-vectors
                cv::Point3d n = Xvec.cross(correctedY);
                normaliseVec(n);
                cout << n << endl;
                
                //cv::Point3d n = Zvec;?????????
                
                // change n direction depending on the one we registered while placing the first point
                double dotn = n.dot(upwardDirection);
                std::cout << dotn << "\n";
                if (dotn > 0)
                    n = -n; // this to point upwards on a right handed basis
                    
                cv::Point3d nxa = n.cross(Xvec);
                normaliseVec(nxa);
                cout << nxa << endl;
                
                //cv::Point3d nxa = correctedY;????????
                
                
                //Setting the Translation relative to the camera (being equal to the first registered point of the basis)
                relativeTvec.at<float>(0) = planePoints[0].x;
                relativeTvec.at<float>(1) = planePoints[0].y;
                relativeTvec.at<float>(2) = planePoints[0].z;
                
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

                
                IsStaticSet = true;
                Calib_points = true;
            }
        }
        
            
        if (definePlane && planePoints.size() == 1)
        {
            cv::drawMarker(imageCopy, DrawplanePoints.at(0), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw point as a cross
            cv::line(imageCopy, DrawplanePoints.at(0), imgPoint.at(0), cv::Scalar(0, 165, 255), 2);
        }
        
        else if (definePlane && planePoints.size() == 2)
        {
            cv::drawMarker(imageCopy, DrawplanePoints.at(0), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw point as a cross
            cv::line(imageCopy, DrawplanePoints.at(0), DrawplanePoints.at(1), cv::Scalar(0, 165, 255), 2);
            cv::line(imageCopy, DrawplanePoints.at(0), imgPoint.at(0), cv::Scalar(0, 165, 255), 2);
        }
        else if (definePlane && planePoints.size() >= 3)
        {
            
        }
        
    }
}


//For calibrating points
void Calibration_Points(const char &key)
{
    if(key == 'r' | key == 'R'){
        
        if(!RpressedOnce)
        {
            //std::vector<cv::Point2f> imagePoints;
            //cv::Mat mat3Dpoints = cv::Mat(markers[0].get3DPoints(), true); //get the matrix out of the 3d point array
            //cv::projectPoints(mat3Dpoints, markers[0].Rvec, markers[0].Tvec, camParameters.CameraMatrix, camParameters.Distorsion, imagePoints); //Project the 3d points in 2d
            cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
            coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
            coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
            coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);
            
            //remember the new 3D point
            auto tup = std::make_tuple(coordinates, 0.0f, 0.0f);
            registeredPoints.push_back(tup);
            
            DrawnImgPoints.push_back(imagePoint.at(1) );//Remember the new 2D point
            RpressedOnce = true;
        }
        else
        {
            //For having a 3 channels Mat (instead of 4)
            cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
            coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
            coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
            coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);
            
            //Finding the error circle radius
            float disterror = norm(coordinates, std::get<0>(registeredPoints.back()), cv::NORM_L2, cv::noArray() );
            
            //remember the error circle radius
            std::get<1>(registeredPoints.back()) = disterror;
            
            //Variables for finding the 2D radius
            vector <cv::Point2f> center;
            center.push_back(DrawnImgPoints.back() );
            vector <cv::Point2f> border;
            border.push_back(imagePoint.at(1) );
            
            //Finding the circle radius
            float disterror2D = norm(center, border, cv::NORM_L2);
            
            std::get<2>(registeredPoints.back()) = disterror2D;
            
            RpressedOnce = false;
        }
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
    cv::Mat coordinates = cv::Mat(3, 1, CV_32FC1);
    coordinates.at<float>(0, 0) = Real_coordinates.at<float>(0, 0);
    coordinates.at<float>(1, 0) = Real_coordinates.at<float>(1, 0);
    coordinates.at<float>(2, 0) = Real_coordinates.at<float>(2, 0);
    
    for(int i = 0; i < registeredPoints.size(); i++)
    {
        //Finding the distance between the tool and the registered point
        float disterror = norm(coordinates, std::get<0>(registeredPoints.at(i)), cv::NORM_L2, cv::noArray() );
        
        //If the tool tip appears to be inside the error radius of the current point, write in green, if not write in red
        if( disterror < std::get<1>(registeredPoints.at(i)) )
        {
            cv::putText(imageCopy, to_string(i), cv::Point(5, 50*(i+2)), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 255, 0), 2);
        }
        else
        {
            cv::putText(imageCopy, to_string(i), cv::Point(5, 50*(i+2)), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 0, 255), 2);
        }  
    }
}



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
        aruco::MarkerMap* mmap = new aruco::MarkerMap;
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
        //MDetector.setDictionary("ARUCO_MIP_16h3");
        MDetector.setDetectionMode(aruco::DM_FAST);
        
        //camParameters.readFromXMLFile("cam_calibration_3.yml");

        //mmap.readFromFile(posMarkerMapfile); //Read 3d markers position of the markermap from file
        //MMTracker.setParams(camParameters, mmap, 0.02); //Set Tracker params
        
        //Load configuration
        ParseConfig(argv[1], mmap);
        
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
        cv::moveWindow("Trackbars",0,900);
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
            vector<aruco::Marker> markers=MDetector.detect(imageCopy,camParameters,0.04);
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
            
            /*for(auto m:markers){
                //aruco::CvDrawingUtils::draw3dAxis(imageCopy, camParameters, m.Rvec, m.Tvec, 0.1);
                //aruco::CvDrawingUtils::draw3dCube(imageCopy, m, camParameters, 1);
                //cout<<m.Rvec<<" "<<m.Tvec<<endl;
            }*/
            
            key = cv::waitKey(1) & 0xFF;
            
            /////////////DRAWING TOOL TIP AND AXIS EACH FRAME, AND GETTING 3D POSITION OF THE TOOL TIP//////////////
            if(OneMarkerValid)
            {
                //Draw tool axis
                aruco::CvDrawingUtils::draw3dAxis(imageCopy, camParameters, MMTracker.getRvec(), MMTracker.getTvec(), 0.02, 2);
                //Take value from trackbar
                zOffset = cv::getTrackbarPos("Calibration Z offset", "Trackbars") / 10000.;
                
                //2d projected point on the image
                cv::Mat objectPoint(2, 3, CV_32FC1);
                objectPoint.at<float>(0, 0) = 0;
                objectPoint.at<float>(0, 1) = 0;
                objectPoint.at<float>(0, 2) = 0;
                objectPoint.at<float>(1, 0) = 0;
                objectPoint.at<float>(1, 1) = 0;
                objectPoint.at<float>(1, 2) = -zOffset; //for translating the tool tip on the z axis as needed
                cv::projectPoints(objectPoint, MMTracker.getRvec(), MMTracker.getTvec(), camParameters.CameraMatrix, camParameters.Distorsion, imagePoint); //Project the 3d points in 2d
                cv::drawMarker(imageCopy, imagePoint.at(1), cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); //Draw cross for the tool tip on each frame
                
                //Coordinates of the tooltip in the markermap basis (4x1 matrix needed for swaping basis)
                cv::Mat objectPoint_41 = cv::Mat(4, 1, CV_32FC1); 
                objectPoint_41.at<float>(0, 0) = objectPoint.at<float>(1, 0);
                objectPoint_41.at<float>(1, 0) = objectPoint.at<float>(1, 1);
                objectPoint_41.at<float>(2, 0) = objectPoint.at<float>(1, 2);
                objectPoint_41.at<float>(3, 0) = 1; 
                
                //calculating the 3d coordinates in the camera basis
                Real_coordinates = MMTracker.getRTMatrix() * objectPoint_41;
                
                //cout << "RT : " << MMTracker.getRTMatrix() << endl;
                //cout << "in : " << objectPoint_41 << endl;
                //cout << "out : " << Real_coordinates << endl;
                if(Calib_points){
                    string R_coordinates = "( " + to_string(Real_coordinates.at<float>(0, 0)*100) + "; " + to_string(Real_coordinates.at<float>(1, 0)*100) + "; " + to_string(Real_coordinates.at<float>(2, 0)*100) + " )";
                    
                    //printing out the coordinates of the tool tip
                    cv::putText(imageCopy, "Coordinates of the tool tip :", cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
                    cv::putText(imageCopy, R_coordinates, cv::Point(5, 150), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
                }
            }
            
            /////////////////IF THE ORIGIN IS NOT SET, SET IT ////////////////////////
            if(!IsStaticSet)
            {
                SetOrigin(key, imageCopy);
            }
            
            /////////////////IF THE ORIGIN IS SET, DO THE CALIBRATION / DETECTION ///////////////////
            if(IsStaticSet)
            {
                ///////////DRAW THE BASIS OF THE WORKING SURFACE///////////
                
                //First draw origin
                cv::Mat Origin = cv::Mat(1, 3, CV_32FC1);
                Origin.at<float>(0, 0) = 0.;
                Origin.at<float>(0, 1) = 0.;
                Origin.at<float>(0, 2) = 0.;

                //::Mat X1 = transp(relativeRTmat) * cv::Mat(1,0,0,1);
                
                vector< cv::Point2f > imgPoint;
                cv::projectPoints(Origin, relativeRvec, relativeTvec, camParameters.CameraMatrix, camParameters.Distorsion, imgPoint); //Project the 3d points in 2d
                cv::drawMarker(imageCopy, imgPoint.at(0), cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 20, 2 ); //Draw origin of other basis
                //cv::line(imageCopy, DrawplanePoints.at(0), DrawplanePoints.at(1), cv::Scalar(0, 165, 255), 2);
                
                
                ///////CHANGE MODE IF THE ORIGIN WAS SET
                
                if(key == 'd' | key == 'D')
                {
                    if(Calib_points){
                        Detect_points = true;
                        Calib_points = false;
                    }
                }
                
                if(key == 'c' | key == 'C')
                {
                    if(Detect_points){
                        Calib_points = true;
                        Detect_points = false;
                    }
                }
                  
                //////IF WE ARE IN CALIBRATION MODE
                if(Calib_points)
                {
                    cv::putText(imageCopy, "Calibration Mode", cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(255, 0, 255), 2);
                    
                    if(OneMarkerValid)
                    {
                        Calibration_Points(key);
                        
                        //Drawing the moving error circle of the current point
                        if(RpressedOnce)
                        {
                            vector <cv::Point2f> center;
                            center.push_back(DrawnImgPoints.back() );
                            vector <cv::Point2f> border;
                            border.push_back(imagePoint.at(1) );
                            
                            //Finding the circle radius
                            float radius = norm(center, border, cv::NORM_L2);
                            
                            //Draw circle around the last point, following the tool tip
                            cv::circle(imageCopy, DrawnImgPoints.back(), radius, cv::Scalar(0, 0, 255), 2);
                        }
                        
                    }
                }
                
                /////////////IF WE ARE IN DETECTION MODE
                if(Detect_points)
                {
                    cv::putText(imageCopy, "Detection Mode", cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(255, 0, 255), 2);
                    if(OneMarkerValid)
                    {
                        Detection_Points(imageCopy);
                    }
                }
                
                //Drawing each point and error circle (each frame)
                if(DrawnImgPoints.size() > 0)
                {
                    for (int i=0; i < DrawnImgPoints.size(); i++){
                        //aruco::CvDrawingUtils::drawXYcross(imageCopy, camParameters, markers[0].Rvec, markers[0].Tvec, 0.005);///2 * imgSizeX / 1920 /* adapting ui size to image size */  
                        cv::drawMarker(imageCopy, DrawnImgPoints.at(i), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 20, 2 );//Draw all the points as a cross
                        //cv::line(imageCopy, DrawnImgPoints.at(i).at(0), DrawnImgPoints.at(i).at(1), cv::Scalar(0, 0, 255, 255), 1);
                        if(std::get<2>(registeredPoints.at(i)) > 0.0f)
                            cv::circle(imageCopy, DrawnImgPoints.at(i), std::get<2>(registeredPoints.at(i)), cv::Scalar(128, 0, 255), 2);
                    }
                }/**/
                
            }            
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
        
        delete mmap; //delete le pointeur mmap
       
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
