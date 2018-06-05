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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap), //mpSystem(pSys),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), needRelocalize(false)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            if(DetectLocalize())
            {
                if(ComputeSE3()){
                    Localize();
                }

            }
        }    
   
        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLocalize()
{

    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.back();
        mlpLoopKeyFrameQueue.pop_back();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+3)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    return true;


}

bool LoopClosing::ComputeSE3()
{
    // create depth, dx, dy, info
    float basefx = mpCurrentKF->mbf;
    float fx = mpCurrentKF->fx;
    float fy = mpCurrentKF->fy;
    float cx = mpCurrentKF->cx;
    float cy = mpCurrentKF->cy;
    int width = mpCurrentKF->mnMaxX;
    int height = mpCurrentKF->mnMaxY;
    int u, v;

    float* depth = new float[width*height]();
    float* depth_gradientX = new float[width*height]();
    float* depth_gradientY = new float[width*height]();
    float* depth_info = new float[width*height]();

    double d_var = 0.00;
    double d_limit = 100.0;
    double matching_thres = basefx*( 1.0/(d_limit/16.0) + d_var/((float)(d_limit/16.0)*(d_limit/16.0)*(d_limit/16.0)) );

    /////////////////////////depth image generation/////////////////////////////
    cv::Mat depth_image = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
    for(size_t i=0; i<width*height;i++)
    {
        u = i%width;
        v = i/width;
        
        int16_t d = mpCurrentKF->mDispImg.at<int16_t>(v,u);            
        if(d==0 || d!=d || d<d_limit) d = 0; //
        depth[i] = basefx*( 1.0/((float)d/16.0) + d_var/((float)(d/16.0)*(d/16.0)*(d/16.0)) );

        //depth image            
        depth_image.at<float>(v,u) = depth[i];
    }

    /////////////////////////depth gradient generation//////////////////////////
    cv::Mat dgx_image = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
    cv::Mat dgy_image = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
    cv::Scharr(depth_image, dgx_image, CV_32FC1, 1, 0);
    cv::Scharr(depth_image, dgy_image, CV_32FC1, 0, 1);
    for(size_t i=0; i<width*height;i++)
    {
        u = i%width;
        v = i/width;

        //depth gradient
        depth_gradientX[i] = dgx_image.at<float>(v,u)/32.0f;
        depth_gradientY[i] = dgy_image.at<float>(v,u)/32.0f;

        //depth info
        float info_denom = sqrt(depth_gradientX[i]*depth_gradientX[i]+depth_gradientY[i]*depth_gradientY[i]);
        if (!isfinite(info_denom)) depth_info[i] = 0;
        else if (info_denom<0.0001) depth_info[i] = 0;
        else depth_info[i] = 10.0/info_denom;

//        if(depth_info[i]<5.0) depth_info[i] = 0.0;
    }

    /////////////////////////optimization//////////////////////////
    const float deltaHuber = sqrt(10);
    //solver initialization
    g2o::SparseOptimizer optimizer;

    g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* blockSolver = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    optimizer.setAlgorithm(solver);

    // SET SIMILARITY VERTEX
    g2o::VertexDepth * vSim3 = new g2o::VertexDepth();
    vSim3->_fix_scale= true;
//    cv::Mat Tcw = mpCurrentKF->GetPose();
//    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(0,0,0);
    g2o::Sim3 g2oS_init(R,t,1.0);
    vSim3->setEstimate(g2oS_init);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point[0] = cx;
    vSim3->_principle_point[1] = cy;
    vSim3->_focal_length[0] = fx;
    vSim3->_focal_length[1] = fy;
    vSim3->_width = width;
    vSim3->_height = height;
    vSim3->ImageD = depth;
    vSim3->ImageGx = depth_gradientX;
    vSim3->ImageGy = depth_gradientY;
    vSim3->ImageInfo = depth_info;
    optimizer.addVertex(vSim3);

    //Set map point vertices
    int numpts = mpCurrentKF->mGtVelodyne.points.size();
    Eigen::Matrix<double, 1, 1> info;
    info << 0.0f;
    Eigen::Matrix4d camcoordinate = Converter::toMatrix4d(mpCurrentKF->GetPose());

    int index = 1;
    vector<g2o::EdgeXYZDepth*> vpEdges;
    vpEdges.reserve(numpts);
    for(size_t i=0; i<numpts;i++)
    {
        // Set map points
        Eigen::Vector3d pts( mpCurrentKF->mGtVelodyne.points[i].x, mpCurrentKF->mGtVelodyne.points[i].y, mpCurrentKF->mGtVelodyne.points[i].z);
//        pts << mpCurrentKF->mGtVelodyne.points[i].x, mpCurrentKF->mGtVelodyne.points[i].y, mpCurrentKF->mGtVelodyne.points[i].z;

//        Eigen::Vector3d xyz = vSim3->estimate().map(pts);
//        Eigen::Vector3d xyz = Converter::TransformPt(camcoordinate,pts);
        Eigen::Vector3d xyz = camcoordinate.block<3,3>(0,0)*pts+camcoordinate.block<3,1>(0,3);
        Eigen::Vector2d Ipos( vSim3->cam_map(xyz) );
        int i_idx = ((int)Ipos[1])*vSim3->_width+((int)Ipos[0]);
        
        
        if ( xyz[2]>0.0f && isfinite(xyz[2]) && xyz[2]<matching_thres && xyz[2]<30.0f ){//
                if (Ipos[0]<vSim3->_width && Ipos[0]>=0 && Ipos[1]<vSim3->_height && Ipos[1]>=0 && depth_info[i_idx]>5.0)
                {
                    // SET PointXYZ VERTEX
                    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                    vPoint->setEstimate(xyz);
                    vPoint->setId(index);
                    vPoint->setFixed(true);
//                    vPoint->setMarginalized(true);
                    optimizer.addVertex(vPoint);
                    
                    // Set Edges
                    g2o::EdgeXYZDepth* e01 = new g2o::EdgeXYZDepth();
                    e01->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(index)));
                    e01->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e01->setMeasurement(1.0f);
                    info << depth_info[i_idx];
                    e01->setInformation(info);
                    g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
                    rk1->setDelta(deltaHuber);
                    e01->setRobustKernel(rk1);

                    optimizer.addEdge(e01);
                    vpEdges.push_back(e01);

                    index++;
                }
        }

    }
    
    cout<<index<<endl;
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();

    int g2oresult = optimizer.optimize(100);
    cout<<g2oresult<<endl;

    // Check inliers
    double index2=0;
    double sum_chi2 = 0;
    for(size_t i=0; i<vpEdges.size();i++)
    {
        g2o::EdgeXYZDepth* e12 = vpEdges[i];
        if(e12->chi2()<10)
        {
            index2++;
            sum_chi2 = sum_chi2 + e12->chi2();
        }
    }
    cout<<index2<<endl;

    // Recover optimized Sim3
    cv::Mat Tcw = mpCurrentKF->GetPose();
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    g2o::Sim3 g2oS_prev(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0);
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    mg2oScw = vSim3_recov->estimate()*g2oS_prev;

    g2o::SparseBlockMatrixXd spinv;
    Eigen::Matrix<double,7,7> marginal_cov;
    if(optimizer.computeMarginals(spinv, optimizer.vertex(0))){
        marginal_cov = spinv.block(0,0)->eval().inverse();
        mInformation = Converter::toCvMat(marginal_cov);
    }

    delete [] depth;
    delete [] depth_gradientX;
    delete [] depth_gradientY;
    delete [] depth_info;
 

    matching_err = optimizer.activeRobustChi2()/(double) index;
    cout<<"activeRobustChi2() "<<matching_err<<endl;

    mInformation = 0.000000001*mInformation;///matching_err;
    cout << mInformation <<endl;

    // add partial pose
    Eigen::Matrix3d eigR = mg2oScw.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = mg2oScw.translation();
    double s = mg2oScw.scale();
    eigt *=(1./s); //[R t/s;0 1]
    cv::Mat correctedTcw = Converter::toCvSE3(eigR,eigt);
    cout<<correctedTcw<<endl;
    mpCurrentKF->mPartialPose.push_back(std::pair<cv::Mat, cv::Mat>(correctedTcw,mInformation));
    mpCurrentKF->mCurPose = correctedTcw;
    mpCurrentKF->mCurCov = mInformation; 

    if(index2>1000 && matching_err<500 ){
         return true;
    }
    else{
        if(index2<=1000)matching_err =10000;
        return false;
    }    


}


void LoopClosing::Localize()
{
    //Modify 'CorrectLoop' for the localization
    //Camera poses are relocated

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();


    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
                
                // add partial pose
                Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                double s = g2oCorrectedSiw.scale();
                eigt *=(1./s); //[R t/s;0 1]
                cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt); 
                pKFi->mPartialPose.push_back(std::pair<cv::Mat, cv::Mat>(correctedTiw, mInformation));
                pKFi->mCurPose = correctedTiw;
                pKFi->mCurCov = mInformation;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, mbFixScale);
    mpMap->InformNewBigChange();



    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();  

    mLastLoopKFid = mpCurrentKF->mnId;       

}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
