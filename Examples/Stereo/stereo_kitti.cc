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
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<cv::Mat> &vGTPoses);

void LoadVelodyne(const string &strPathToSequence, const string &strSettingPath, cv::Mat cur_pose, const int num_id, pcl::PointCloud<pcl::PointXYZ> &GTVelodyne);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    vector<cv::Mat> vGTPoses;

    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps, vGTPoses);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    cv::Mat gtPose;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        gtPose = vGTPoses[ni];
        double tframe = vTimestamps[ni];

        pcl::PointCloud<pcl::PointXYZ> gtVelodyne;
        LoadVelodyne(string(argv[3]),string(argv[2]), gtPose, ni, gtVelodyne);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe,gtPose,gtVelodyne);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<cv::Mat> &vGTPoses)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    fTimes.close();

    ifstream fGTPoses;
    string strPathGTPoseFile = strPathToSequence + "/poses.txt";
    fGTPoses.open(strPathGTPoseFile.c_str());
    while(!fGTPoses.eof())
    {
        string s;
        getline(fGTPoses,s);

        if(!s.empty())
        {
            cv::Mat Tgt = cv::Mat::eye(4, 4, CV_32F);
            Tgt.at<float>(3,3) = 1.0;
            for(int i=0; i<3; i++){
                for(int j=0; j<4; j++){
                int len = s.find(" ");
	            Tgt.at<float>(i,j) = atof(s.substr(0, len).c_str());
	            s = s.erase(0, len + 1);
                }
	        }
            vGTPoses.push_back(Tgt);
        }
    }
    fGTPoses.close();

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void LoadVelodyne(const string &strPathToSequence, const string &strSettingPath, cv::Mat cur_pose, const int num_id, pcl::PointCloud<pcl::PointXYZ> &GTVelodyne)
{

    //Load Tcv
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    cv::Mat Tcv(4,4,CV_32F);
    Tcv.at<float>(0,0) = fSettings["Camera.ctv00"];
    Tcv.at<float>(0,1) = fSettings["Camera.ctv01"];
    Tcv.at<float>(0,2) = fSettings["Camera.ctv02"];
    Tcv.at<float>(0,3) = fSettings["Camera.ctv03"];
    Tcv.at<float>(1,0) = fSettings["Camera.ctv10"];
    Tcv.at<float>(1,1) = fSettings["Camera.ctv11"];
    Tcv.at<float>(1,2) = fSettings["Camera.ctv12"];
    Tcv.at<float>(1,3) = fSettings["Camera.ctv13"];
    Tcv.at<float>(2,0) = fSettings["Camera.ctv20"];
    Tcv.at<float>(2,1) = fSettings["Camera.ctv21"];
    Tcv.at<float>(2,2) = fSettings["Camera.ctv22"];
    Tcv.at<float>(2,3) = fSettings["Camera.ctv23"];
    Tcv.at<float>(3,0) = 0.0;
    Tcv.at<float>(3,1) = 0.0;
    Tcv.at<float>(3,2) = 0.0;
    Tcv.at<float>(3,3) = 1.0;


    // load point cloud
    stringstream ss;
    ss << setfill('0') << setw(6) << num_id;
    string strPathVeloFile = strPathToSequence + "/velodyne/" + ss.str() + ".bin";

    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    FILE *stream;
    stream = fopen (strPathVeloFile.c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;
    for (int32_t i=0; i<num; i++) {
    pcl::PointXYZ pt;
    pt.x = *px; pt.y = *py; pt.z = *pz; //pt.intensity = *pr;
    GTVelodyne.points.push_back(pt);
    px+=4; py+=4; pz+=4; //pr+=4;
    }
    fclose(stream);

    // transform point cloud
    cv::Mat start_mat = cur_pose*Tcv;
    Eigen::Matrix4f eigenPose;
    eigenPose<<start_mat.at<float>(0,0),start_mat.at<float>(0,1),start_mat.at<float>(0,2),start_mat.at<float>(0,3),
               start_mat.at<float>(1,0),start_mat.at<float>(1,1),start_mat.at<float>(1,2),start_mat.at<float>(1,3),
               start_mat.at<float>(2,0),start_mat.at<float>(2,1),start_mat.at<float>(2,2),start_mat.at<float>(2,3),
               start_mat.at<float>(3,0),start_mat.at<float>(3,1),start_mat.at<float>(3,2),start_mat.at<float>(3,3);
    pcl::transformPointCloud (GTVelodyne, GTVelodyne, eigenPose);
 


}
