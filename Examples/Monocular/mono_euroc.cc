/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include<System.h>
#include <unistd.h>

using namespace std;


int main(int argc, char **argv)
{
    string voca_path = "/home/zsk/visual_slam/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    string yaml_path = "/home/zsk/visual_slam/ORB_SLAM2/Examples/config/MyCamera.yaml";



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voca_path,yaml_path,ORB_SLAM2::System::MONOCULAR,true);

    cout << "begin" << '\n';
    cv::VideoCapture capture(0);
    cv::Mat im;
    auto start = chrono::system_clock::now();
    while(capture.isOpened() == true)
    {
         capture >> im;
         cv::Mat image_resized;
         cv::resize(im, image_resized, cv::Size(640, 480));

         // Rotate image by 180 degrees
         cv::Mat image_flip;
         cv::flip(image_resized, image_flip, -1);

         auto now = chrono::system_clock::now();
         auto timestamp =chrono::duration_cast<chrono::milliseconds>(now - start);

         // Pass the image to the SLAM system
         SLAM.TrackMonocular(image_flip, double(timestamp.count())/1000.0);
         cout<<SLAM.getTracker()->mState<<endl;
         cout<<SLAM.getTracker()->mCurrentFrame.mvpMapPoints.size()<<endl;
         cout<<SLAM.getTracker()->mnMatchesInliers<<endl;



    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

