#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>
#include "tools_mem_used.h"
#include "tools_logger.hpp"



void  Calibrate::img_cbk(const sensor_msgs::ImageConstPtr &msg)
{

    cv::Mat image = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
    cv::Vec3d  rvec ;
    cv::Vec3d tvec ; 

    Calibrate::detectCharucoBoardWithCalibrationPose(image ,  rvec ,  tvec ) ; 
    cv::imshow(" image1",image);
    cv::waitKey(1);
    if (rvec[0] != 0 || rvec[1] !=0 || rvec[2]!=0  || tvec[0] != 0 ||  tvec[1] !=0 || tvec[2] !=0)
    { 
    std::cout << "translation_camera:" <<  tvec << std::endl;
    std::cout << "rotation_camera:" <<  rvec << std::endl;
    }




}


void  Calibrate::img_cbk_calibrate(const sensor_msgs::ImageConstPtr &msg)
{

    cv::Mat img = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
    if (frame_number <300)
    {
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds ; 
        Calibrate::detectCharucoBoardWithoutCalibration(img,  charucoCorners , charucoIds) ; 

        if(charucoCorners.size() >= 8 && charucoIds.size() > 0 )
        {
        allCharucoCorners.push_back(charucoCorners) ; 
        allCharucoIds.push_back(charucoIds) ; 
        }


        frame_number+=1; 

    }


    else
    {

        std::cout << img.size() << std::endl;
        calibrate_camera_in(img);
        std::cout << "nding" << std::endl;
    }







}

void Calibrate::detectCharucoBoardWithoutCalibration(cv::Mat image ,  std::vector<cv::Point2f> &charucoCorners , std::vector<int> &charucoIds)
{
    //cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    //cv::Ptr<cv::aruco::CharucoBoard> board = new cv::aruco::CharucoBoard(cv::Size(5, 7), 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    
    cv::aruco::detectMarkers(image, dict, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            //cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charuco, charucoCorners, charucoIds);
            // if at least one charuco corner detected
            //if (charucoIds.size() > 0)
                //cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
        }
        //cv::imshow("out", imageCopy);
        //char key = (char)cv::waitKey(30);
        //if (key == 27)
            //break;
}






void Calibrate::calibrate_camera_in(cv::Mat &image)
{


//cv::Ptr< cv::aruco::Dictionary > dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//cv::Ptr< cv::aruco::CharucoBoard > charuco = cv::aruco::CharucoBoard::create(7, 5, 0.04, 0.02, dict);


cv::Size imgSize = image.size(); 



// Detect charuco board from several viewpoints and fill allCharucoCorners and allCharucoId

// After capturing in several viewpoints, start calibration
cv::Mat cameraMatrix, distCoeffs;
std::vector<cv::Mat> rvecs, tvecs;
int calibrationFlags = 0; // Set calibration flags (same than in calibrateCamera() function)
double repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charuco, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);


std::cout << distCoeffs << std::endl;
std::cout << cameraMatrix << std::endl ; 




}


void  Calibrate::detectCharucoBoardWithCalibrationPose(cv::Mat &image  ,cv::Vec3d &rvec , cv::Vec3d &tvec )
{
    //bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);

    cv::Mat imageCopy ;

        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        //cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        image.copyTo(imageCopy);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(image, dict, markerCorners, markerIds, params);
        // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charuco, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected



                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charuco, cameraMatrix, distCoeffs, rvec, tvec);
                }
            }
            //cv::imshow("out", imageCopy);
            //char key = (char)cv::waitKey(30);
                
}