/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    Viewer* mpViewer;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                vector<MapPoint*> &vpMatchedMapPoints);


    /**
     * @brief 通过将闭环时相连关键帧的MapPoints投影到这些关键帧中，进行MapPoints检查与替换
     * @param[in] CorrectedPosesMap 关联的当前帧组中的关键帧和相应的纠正后的位姿
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints);
    /**
     * @brief 通过将地图融合时相连关键帧的MapPoints投影到这些关键帧中，进行MapPoints检查与替换
     * @param[in] vConectedKFs 关联的当前帧组中的关键帧
     * @param[in] vpMapPoints  关联的当匹配组中的关键帧观测地图点
     */
    void SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void CheckObservations(set<KeyFrame*> &spKFsMap1, set<KeyFrame*> &spKFsMap2);
    void printReprojectionError(set<KeyFrame*> &spLocalWindowKFs, KeyFrame* mpCurrentKF, string &name);

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    /// 关键帧数据库
    KeyFrameDatabase* mpKeyFrameDB;
    /// 词袋模型中的大字典
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    /// 一个队列, 其中存储了参与到回环检测的关键帧 (当然这些关键帧也有可能因为各种原因被设置成为bad,这样虽然这个关键帧还是存储在这里但是实际上已经不再实质性地参与到回环检测的过程中去了)
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;    

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    /// 连续性阈值,构造函数中将其设置成为了3
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    /// 当前关键帧,其实称之为"当前正在处理的关键帧"更加合适
    KeyFrame* mpCurrentKF;
    KeyFrame* mpLastCurrentKF;
    // 最终检测出来的,和当前关键帧形成闭环的闭环关键帧
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    Map* mpLastMap;         /// 最新的map

    bool mbLoopDetected;        // 是否检测到闭环
    int mnLoopNumCoincidences;  // 对候选组位姿优化后，候选组匹配出的MapPoints对CKF的共视帧同样有效的共视帧数目
    int mnLoopNumNotFound;      // 在找到闭环候选帧后连续匹配闭环不成功的次数
    KeyFrame* mpLoopLastCurrentKF;   /// 上一个检测闭环的关键帧
    g2o::Sim3 mg2oLoopSlw;           // last KF 在检测到闭环时在闭环世界坐标系的位姿 world -> lastKF
    g2o::Sim3 mg2oLoopScw;
    /// 最终检测出来的,和当前关键帧形成闭环的闭环关键帧
    KeyFrame* mpLoopMatchedKF;
    /// 在闭环帧及其共视帧中的闭环MapPoints
    std::vector<MapPoint*> mvpLoopMPs;
    /// 在闭环帧及其共视帧中的重投影匹配的闭环MapPoints
    std::vector<MapPoint*> mvpLoopMatchedMPs;
    /// 是否检测到合并闭环
    bool mbMergeDetected;
    /// 对候选组位姿优化后，候选组匹配出的MapPoints对CKF的共视帧同样有效的共视帧数目
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;             // 在找到闭环候选帧后连续匹配闭环不成功的次数
    KeyFrame* mpMergeLastCurrentKF;     /// 上一个检测合并闭环的关键帧
    // 在NewDetectCommonRegions()中为last KF 在检测到闭环地图合并时在合并地图世界坐标系的位姿 world2 -> lastKF
    // 在Run中为CKF world2 -> CKF
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;             // 用于合并的闭环匹配帧的世界位姿   world1 -> matched
    g2o::Sim3 mg2oMergeScw;             // world2 ->current
    /// 最终检测出来的,和当前关键帧形成闭环的地图合并关键帧
    KeyFrame* mpMergeMatchedKF;
    /// 在合并帧及其共视帧中的合并MapPoints
    std::vector<MapPoint*> mvpMergeMPs;
    /// 在合并帧及其共视帧中的重投影匹配的合并MapPoints
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;       /// 两地图相对位姿 world1 -> world2
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;



    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;  //0:闭环  1:地图融合
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
