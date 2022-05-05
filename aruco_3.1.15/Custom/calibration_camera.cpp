#include "v4l2_helper.h"
//#include "calibrator.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "popt_pp.h"
#include <sys/stat.h>

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;
using namespace cv;


//aruco::Calibrator calibrator;
vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;

bool doesExist (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

void setup_calibration(int board_width, int board_height, int num_imgs, 
                       float square_size, char* imgs_directory, char* imgs_filename,
                       char* extension) {
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

cout << num_imgs << endl;

  for (int k = 1; k <= num_imgs; k++) {
  
    char img_file[100];
    sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, k, extension);
    
    //cout << imgs_directory << imgs_filename << k << extension;
    if(!doesExist(img_file)){
        cout << " ERR" << endl;
        continue;
    }
    img = imread(img_file, IMREAD_COLOR);
    cv::cvtColor(img, gray, COLOR_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, board_size, corners,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    if (found)
    {
      cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                   TermCriteria(TermCriteria::MAX_ITER|TermCriteria::EPS, 30, 0.1));
      drawChessboardCorners(gray, board_size, corners, found);
    }
    
    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      cout << k << ". Found corners!" << endl;
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2 );
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char const **argv)
{
    
    int imgSizeX = 1920, imgSizeY = 1080, cpt = 0, nbImgSaved = 0;
    
    //configure the calibrator (aruco 3)
    //calibrator.setParams(cv::Size(imgSizeX, imgSizeY), 0.04, "");
    
    
    ////////////////For saving images live from the camera/////////////////
    
    /*unsigned char* ptr_cam_frame; //permettra de prendre l'image
    int bytes_used, key = 0;
    
    cv::Mat yuyv_frame = cv::Mat(imgSizeY, imgSizeX, CV_8UC2);
    //Original image, that gets the cam frame from the pointer
    cv::Mat originalImage = cv::Mat(imgSizeY, imgSizeX, CV_8UC3);
    //copied image, for printing the markers on the screen
    cv::Mat imageCopy;
    
    //initialisation camera
    if(helper_init_cam("/dev/video0", imgSizeX, imgSizeY, V4L2_PIX_FMT_UYVY, IO_METHOD_USERPTR) < 0) // or V4L2_PIX_FMT_YUYV
        {
            cout << "Failed to open video" << endl;
            return -1;
        }
        
    while (key != 27){
    
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
        
        
        key = cv::waitKey(1) & 0xFF;
        
        if(key == 'a' | key == 'A'){
            nbImgSaved++;
            std::string filename = "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/images_chessboard/chess_calib_" + to_string(nbImgSaved) + ".jpg";
            if(!imwrite(filename, originalImage) ){
                cerr << "Failed to store image" << endl;
            }   
                 
        }
        
        //output in the terminal
        if(cpt%20 == 0)
            cout << "getting frame" << endl;
        
        string string_nb_img = "currently " + to_string( nbImgSaved) + " images";
        cv::putText(originalImage, "Press a to save an image for the calibration (minimum 10)", cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920, cv::Scalar(0, 230, 0), 2);
        cv::putText(originalImage, string_nb_img, cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
        cv::putText(originalImage, "'esc' : save and quit", cv::Point(10,120), cv::FONT_HERSHEY_SIMPLEX, 1 * imgSizeX / 1920., cv::Scalar(125,255,255), 2);
        
         //output the image on the screen
        cv::imshow("Img", originalImage);
        
        //release cam frame
        if (helper_release_cam_frame() < 0)
        {
            cout << "Failed to release image" << endl;
            break;
        }
        cpt++;
    }*/
    
    
    //////////////////////Starting calibration///////////////////////
    
    int board_width = 10, board_height = 6;
    int nb_img_calib = 11;//Nb de fichiers dans le dossier
    float square_size;
    char* imgs_directory = (char*)"/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/images_chessboard/";
    char* imgs_filename = (char*)"chess_calib_";
    char* out_file = (char*)"calib.yml";
    char* extension = (char*)"jpg";

   
    
    setup_calibration(board_width, board_height, nb_img_calib, 20.6, imgs_directory
                    , imgs_filename, extension);

    cout << img.size() <<  endl;
    printf("Starting Calibration\n");
    Mat K;
    Mat D;
    vector< Mat > rvecs, tvecs;
    int flag = 0;
    flag |= CALIB_FIX_K4;
    flag |= CALIB_FIX_K5;
    calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

    cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

    FileStorage fs(out_file, FileStorage::WRITE);
    
    //fs << "board_width" << board_width;
    //fs << "board_height" << board_height;
    fs << "image_width" << imgSizeX;
    fs << "image_height" << imgSizeY;
    fs << "camera_matrix" << K;
    fs << "distortion_coefficients" << D;
    fs << "square_size" << square_size;/**/
    cout << "results saved to "<< out_file << endl;
    cout << "Done Calibration\n";
    
    ////////////////Calib with aruco3///////////////:
    /*aruco::CameraParameters camp;
    if (calibrator.getCalibrationResults(camp))
    {
        camp.saveToFile("cam_calib.yml");
        
        
    }
    else
    {
        cout << "could not obtain calibration" << endl;
    }*/
    
    
    return 0;
}

