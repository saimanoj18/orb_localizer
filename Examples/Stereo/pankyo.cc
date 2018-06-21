#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

void LoadVelodyne(const string &strPathToSequence, const string &strSettingPath, vector<cv::Mat> &vGTPoses,  pcl::PointCloud<pcl::PointXYZ>::Ptr &GTVelodyne, cv::Mat camIntrinsic);
void SegmentVelodyne(cv::Mat &vGTPose,  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> oc, pcl::PointCloud<pcl::PointXYZ>::Ptr &GTVelodyne, pcl::PointCloud<pcl::PointXYZ> &curVelodyne, int image_width, int image_height, cv::Mat camIntrinsic);

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
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vgtVelodyne (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (32.0f);

    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    LoadVelodyne(string(argv[3]), string(argv[2]),vGTPoses, vgtVelodyne, K);
    //set scale
    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    float scale1 = fSettings["Camera.scale1"];
    float scale2 = fSettings["Camera.scale2"];
    //set octree
    octree.setInputCloud (vgtVelodyne);
    octree.addPointsFromInputCloud ();

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

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(5);
        clahe->apply(imLeft,imLeft);
        clahe->apply(imRight,imRight);

        gtPose = vGTPoses[ni];
        double tframe = vTimestamps[ni];


        pcl::PointCloud<pcl::PointXYZ> gtVelodyne;
        SegmentVelodyne(gtPose, octree, vgtVelodyne, gtVelodyne, imLeft.cols, imLeft.rows, K);


        cv::resize(imLeft, imLeft, cv::Size(), scale1, scale2);
        cv::resize(imRight, imRight, cv::Size(), scale1, scale2);

        

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe,gtPose,gtVelodyne);


        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        double tttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        vTimesTrack[ni]=ttrack;
        cout<<"********time passed: "<<ttrack<<", "<<tttrack<<endl;

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
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
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

void LoadVelodyne(const string &strPathToSequence, const string &strSettingPath, vector<cv::Mat> &vGTPoses, pcl::PointCloud<pcl::PointXYZ>::Ptr &GTVelodyne, cv::Mat camIntrinsic)
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

    camIntrinsic.at<float>(0,0) = fSettings["Camera.fx"];
    camIntrinsic.at<float>(0,2) = fSettings["Camera.cx"];
    camIntrinsic.at<float>(1,1) = fSettings["Camera.fy"];
    camIntrinsic.at<float>(1,2) = fSettings["Camera.cy"];

    //Load gt poses
    ifstream fGTPoses;
    string strPathGTPoseFile = strPathToSequence + "/poses.txt";
    fGTPoses.open(strPathGTPoseFile.c_str());
    bool start = true;
    cv::Mat start_mat = cv::Mat::eye(4, 4, CV_64F);
    vGTPoses.clear();
    while(!fGTPoses.eof())
    {
        string s;
        getline(fGTPoses,s);

        if(!s.empty())
        {
            cv::Mat Tgt = cv::Mat::eye(4, 4, CV_32F);
            cv::Mat res_mat = cv::Mat::eye(4, 4, CV_32F);
            Tgt.at<float>(3,3) = 1.0;
            for(int i=0; i<3; i++){
                for(int j=0; j<4; j++){
                int len = s.find(" ");
	            Tgt.at<float>(i,j) = atof(s.substr(0, len).c_str());
	            s = s.erase(0, len + 1);
                }
	        }
            if(start)
            {
                start_mat = Tgt*Tcv.inv();
                res_mat = cv::Mat::eye(4, 4, CV_32F);
                start = false;
            }
            else
            {
                res_mat = start_mat.inv()*Tgt*Tcv.inv();
            }
            vGTPoses.push_back(res_mat);

//            cv::Mat saved_pose = res_mat;
//            ofstream poses_file("GT_poses.txt", std::ios::app);
//            if (poses_file.is_open()){
//                poses_file << saved_pose.at<float>(0,0) << ' ';
//                poses_file << saved_pose.at<float>(0,1) << ' ';
//                poses_file << saved_pose.at<float>(0,2) << ' ';
//                poses_file << saved_pose.at<float>(0,3) << '\n';
//                poses_file << saved_pose.at<float>(1,0) << ' ';
//                poses_file << saved_pose.at<float>(1,1) << ' ';
//                poses_file << saved_pose.at<float>(1,2) << ' ';
//                poses_file << saved_pose.at<float>(1,3) << '\n';
//                poses_file << saved_pose.at<float>(2,0) << ' ';
//                poses_file << saved_pose.at<float>(2,1) << ' ';
//                poses_file << saved_pose.at<float>(2,2) << ' ';
//                poses_file << saved_pose.at<float>(2,3) << '\n';
//            }
//            poses_file.close();
        }
    }
    fGTPoses.close();

    //Load global map from .las
    ifstream fGTVelodyne;
    string strPathGTPointFile = strPathToSequence +"/sick_pointcloud2.las";
    
    if (!liblas::Open(fGTVelodyne, strPathGTPointFile))
    {
        throw std::runtime_error(std::string("Can not open ") + strPathGTPointFile);
    }

    liblas::Reader reader(fGTVelodyne);
    liblas::Header const& h = reader.GetHeader();        
    GTVelodyne->width = h.GetPointRecordsCount()/100+1;
    GTVelodyne->height = 1;
    GTVelodyne->points.resize (GTVelodyne->width * GTVelodyne->height);
    int count = 0;
    int count2 = 0;
    while (reader.ReadNextPoint())
    {        
        liblas::Point const& p = reader.GetPoint();
        if(count2%100==0){
        GTVelodyne->points[count].x = p[0];
        GTVelodyne->points[count].y = p[1];
        GTVelodyne->points[count].z = p[2];
        count++;
        }
        count2++;
    }
    Eigen::Matrix4f eigenPose;
    eigenPose<<start_mat.at<float>(0,0),start_mat.at<float>(0,1),start_mat.at<float>(0,2),start_mat.at<float>(0,3),
               start_mat.at<float>(1,0),start_mat.at<float>(1,1),start_mat.at<float>(1,2),start_mat.at<float>(1,3),
               start_mat.at<float>(2,0),start_mat.at<float>(2,1),start_mat.at<float>(2,2),start_mat.at<float>(2,3),
               start_mat.at<float>(3,0),start_mat.at<float>(3,1),start_mat.at<float>(3,2),start_mat.at<float>(3,3);
    pcl::transformPointCloud (*GTVelodyne, *GTVelodyne, eigenPose.inverse());

    cout<<start_mat<<endl;

}

void SegmentVelodyne(cv::Mat &vGTPose,  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> oc, pcl::PointCloud<pcl::PointXYZ>::Ptr &GTVelodyne, pcl::PointCloud<pcl::PointXYZ> &curVelodyne, int image_width, int image_height, cv::Mat camIntrinsic)
{
    //extract local map
    pcl::PointXYZ searchPoint;
    searchPoint.x = vGTPose.at<float>(0,3);
    searchPoint.y = vGTPose.at<float>(1,3);
    searchPoint.z = vGTPose.at<float>(2,3);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    oc.radiusSearch (searchPoint, 50.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    //pt cloud
    curVelodyne.clear();
    curVelodyne.width = pointIdxRadiusSearch.size();///50+1;
    curVelodyne.height = 1;
    curVelodyne.points.resize (curVelodyne.width * curVelodyne.height);

//    //range image
//    cv::Mat K = 0.5*camIntrinsic;
//    K.at<float>(0,0) = K.at<float>(0,0)*0.5;
////    K.at<float>(1,1) = K.at<float>(0,0)*2.0;
//    int width = image_width*0.5;
//    int height = image_height*0.5;
//    cv::Mat range_image = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);


    int count = 0;
    int index = 0;
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
//        if(i%50==0){
        curVelodyne.points[count].x = GTVelodyne->points[ pointIdxRadiusSearch[i] ].x;
        curVelodyne.points[count].y = GTVelodyne->points[ pointIdxRadiusSearch[i] ].y;
        curVelodyne.points[count].z = GTVelodyne->points[ pointIdxRadiusSearch[i] ].z;
        count++;
//        }
    }


}
