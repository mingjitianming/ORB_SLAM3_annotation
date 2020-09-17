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


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;
        // Set of KeyFrames and MapPoints, in this version the set serializator is not working
        //ar & mspKeyFrames;
        //ar & mspMapPoints;

        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;

        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnLastLoopKFid;
    }

public:
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);

    void AddMapPoint(MapPoint* pMP);
    /**
     * @brief 从地图中擦除地图点
     * 
     * @param[in] pMP 地图点
     */
    void EraseMapPoint(MapPoint* pMP);
    /**
     * @brief 从地图中删除关键帧
     * @detials 实际上这个函数中目前仅仅是删除了在std::set中保存的地图点的指针,并且删除后
     * 之前的地图点所占用的内存其实并没有得到释放
     * @param[in] pKF 关键帧
     */
    void EraseKeyFrame(KeyFrame* pKF);
     /**
     * @brief 设置参考地图点
     * @detials 一般是指,设置当前帧中的参考地图点; 这些点将用于DrawMapPoints函数画图
     * 
     * @param[in] vpMPs 地图点们
     */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void RotateMap(const cv::Mat &R);
    void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel=false, const cv::Mat t=cv::Mat::zeros(cv::Size(1,3),CV_32F));

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    // 虽然是一个向量，但是实际上值保存了最初始的关键帧
    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    
    ///当更新地图时的互斥量.回环检测中和局部BA后更新全局地图的时候会用到这个
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;
    /// 下一map id
    static long unsigned int nNextId;

protected:
    ///当前map id
    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;  //关键帧集合

    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;
    //参考地图点 localmap points
    std::vector<MapPoint*> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;  // 初始关键帧id
    long unsigned int mnMaxKFid;   /// 最大关键帧 id
    long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;  // 当前地图是否正在使用
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;  // 是否使用imu
    bool mbIMU_BA1;     // 前5秒数据，用来初始化有先验的imu
    bool mbIMU_BA2;     // 前15秒数据，用来初始化先验为0的imu

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM3

#endif // MAP_H
