//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

/**
@前端的处理逻辑
@1、前端本身有初始化、正常追踪、追踪丢失三种状态
@2、在初始化状态中，根据左右目之间的光流匹配，寻找可以三角化的地图点，成功时建立初始地图
@3、追踪阶段中，前端计算上一帧的特征点到当前帧的光流，根据光流结果计算位姿，该计算只使用左图图像
@4、如果追踪到的点较少，就判定当前帧为关键帧，进行以下关键帧处理
    (1)提取新的特征点
    (2)找到这些点在右图中的对应点，用三角化来建立新的路标点
    (3)将新的关键帧和路标点加入地图，并触发一次后端优化
@5、如果追踪丢失，就重置前端系统，重新初始化
*/
namespace myslam {

Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame; //得到原工厂函数创建的新的一帧
    //初始化状态下status_是INITING 后来的状态会不断的切换  status_是前端本身默认构造就有的成员变量 且默认构造的默认值为INITING 
    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit(); //尝试使用保存在current_frame_(当前帧的指针)中的立体图像来初始化前端
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();      //正常状态下的跟踪
            break;
        case FrontendStatus::LOST:
            Reset();      //跟踪丢失 进行重置
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    if (last_frame_)		//如果存在上一帧
    {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());	//设置当前帧的位姿 relative_motion_是SE3 存在一个初始的默认值 之后是逐帧进行更新的
    }

    int num_track_last = TrackLastFrame();					//返回跟踪到的特征点
    tracking_inliers_ = EstimateCurrentPose();					//估计当前帧的位姿 根据位姿优化返回跟踪到的内点(路标点形成的特征点数量-外围特征点的数量)

    if (tracking_inliers_ > num_features_tracking_) {
        // 成功跟踪
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // 跟踪失败
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // 丢失
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();								//加入关键帧序列 实际上在函数内部需要判断是不是一个关键帧
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();	//当前帧与上一帧的相对位姿的更新

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // 依然有足够的特征点 不需要加入关键帧
        return false;
    }
    // 当前帧是一个关键帧
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);			//在地图中插入这个关键帧序列

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();				//将左图特征点添加进路标点维护的特征点序列
    DetectFeatures();  						//发现新特征点(可能包含旧的特征点) 所有的特征点被保存在当前帧中

    // 在右图中进行特征点匹配
    FindFeaturesInRight();
    // 三角化形成路标点
    TriangulateNewPoints();					//返回三角化得到的路标点的数量
    // 后端更新一次地图
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints() 
{	
    SE3 current_frame_left_pose = camera_left_->pose() * current_frame_->Pose();			//现在的"相机坐标系" (原相机外参*帧相对于原相机的位姿)
    SE3 current_frame_right_pose = camera_right_->pose() * current_frame_->Pose();
    std::vector<SE3> poses{current_frame_left_pose,current_frame_right_pose}; 
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) 
    {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) 
	{
            //左图的特征点未关联地图点且存在右图匹配点，尝试三角化  	expired 用于检测所管理的对象是否已经释放, 如果已经释放, 返回 true
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) 
	    {
                auto new_map_point = MapPoint::CreateNewMappoint();		//三角化创建一个新的路标点
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>() ) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  					//相机位姿顶点
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());					//估计值是当前帧的位姿T
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;					// 仅估计位姿的一元边
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) 
    {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_left_[i]);			//只放入当前帧中形成了路标点的特征点
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K ,camera_left_->pose());		//利用左相机求帧的相对位姿 形成统一
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));		//测量的误差 左图像素误差
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    for (int iteration = 0; iteration < 4; ++iteration) {
        
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());				//优化并设置当前帧的位姿

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();						//外围特征点的weak_ptr(指向路标点)的置为空
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame() {
    // 使用LK光流估计当前帧左图像中的特征点数量
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) 
    {
        if (kp->map_point_.lock()) 
	{
            // 利用三角化的世界坐标系下的点直接计算
            auto mp = kp->map_point_.lock();	//路标点的shared_ptr
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());		//遍历找出上一帧左图中所有可以形成路标点的特征点 转换为当前帧的特征点像素坐标
            kps_last.push_back(kp->position_.pt);					//放入上一帧的特征点的像素坐标序列
            kps_current.push_back(cv::Point2f(px[0], px[1]));				//放入当前帧的追踪到的特征点的像素坐标序列
        } else 
	{
            kps_last.push_back(kp->position_.pt);					//否则 直接设置为相同
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;										//利用OPENCV自带的光流公式直接计算求解
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) 
	{
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));		//有参构造 利用当前帧的shared_ptr和关键点类来构造新的特征点(指向帧的weak_ptr，KeyPoint)
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;   //这个新构造的特征点指向路标点的weak_ptr用上一帧的特征点的weak_ptr直接进行复制构造
            current_frame_->features_left_.push_back(feature);			//在当前帧维护的左图特征点序列中放入该特征点
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();		//在当前帧中检测左图像中的特征关键点并将其保存在当前帧中
    int num_coor_features = FindFeaturesInRight(); 	//在当前帧的右图中找到匹配的特征点 返回匹配的特征点的数量
    if (num_coor_features < num_features_init_) 
    {
        return false;    //两幅立体图像匹配得到的特征点数量小于一定阈值 则不能进行建模
    }

    bool build_map_success = BuildInitMap();		//利用单张图来初始化地图 成功返回1 
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_)   					//观测者存储有帧以及地图的shared_ptr
	{
            viewer_->AddCurrentFrame(current_frame_);   //观测者模块还没有解耦
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255); ////CV_8UC1代表灰度图像 创建一个灰度值0-255的图像矩阵 在追踪分支中发现新特征点中
    //下面是对当前帧中维护的特征点序列进行遍历操作 ---画出特征点位置的边框
    for (auto &feat : current_frame_->features_left_) 
    {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);  //从左图中检测到特征点并放入keypoints序列中
    int cnt_detected = 0;
    for (auto &kp : keypoints) 
    {
        current_frame_->features_left_.push_back(Feature::Ptr(new Feature(current_frame_, kp))); //利用检测到的特征点来构造特征点类 并把该特征点实例类放入当前帧维护的特征点类序列中
        cnt_detected++; //递增放入的特征点类的数量  					         //在追踪检测分支中 如果原有特征点不够 就再放入新的从左图中找到的检测点
    }			

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}
/**
 *weak_ptr被设计为与shared_ptr共同工作，可以从一个shared_ptr或者另一个weak_ptr对象构造，获得资源的观测权。但weak_ptr没有共享资源，它的构造不会引起指针引用计数的增加。
 *使用weak_ptr的成员函数use_count()可以观测资源的引用计数，另一个成员函数expired()的功能等价于use_count()==0,但更快，表示被观测的资源(也就是shared_ptr的管理的资源)已经不复存在。
 *@weak_ptr可以使用成员函数lock()从被观测的shared_ptr获得一个可用的shared_ptr对象从而操作资源。但当expired()==true的时候，lock()函数将返回一个存储空指针的shared_ptr
*/
int Frontend::FindFeaturesInRight() {
    // 使用LK光流来估计右侧图像中的对应得到的特征点
    std::vector<cv::Point2f> kps_left, kps_right; //注意这里的元素是特征点的位置 而不是特征点类
    for (auto &kp : current_frame_->features_left_)  //从当前帧维护的左图特征点序列中进行遍历
    { 
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();  //kp->map_point_是指向路标点类的弱引用指针 
	//当创建一个weak_ptr时，要用一个shared_ptr来初始化它 不能使用weak_ptr直接访问对象，而必须调用lock。此函数检查weak_ptr指向的对象是否仍存在。如果存在
	//lock返回一个指向共享对象的shared_ptr。与任何其它shared_ptr类似，只要此shared_ptr存在，它所指向的底层对象也就会一直存在
        if (mp)   //存在路标点 可以直接结算出右图追踪得到的特征点坐标      在特征点不够重新检测的这个分支中 原特征点是存在对应路标点的 因此可以通过坐标变换进行解算
	{
            // 使用投影点作为初始预测
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else 
	{
            // 还没有三角化的路标点 因此先将右侧特征点坐标设置为与左侧的路标点坐标相同
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;  //uchar类型只能保存0-255的整形数字 多了会溢出再从0开始累加
    Mat error; //opencv调用LK光流
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;  //status是跟踪的状态 为1代表追踪成功 为空代表失控
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i])  //成功跟踪得到特征点
	{
            cv::KeyPoint kp(kps_right[i], 7); //7是特征点的半径
            Feature::Ptr feat(new Feature(current_frame_, kp)); //利用检测到的特征点来构造特征点类 
            feat->is_on_left_image_ = false; //标志位 这个特征点实例是在右图提取的而不是左图
            current_frame_->features_right_.push_back(feat);    //并把该特征点实例类放入当前帧维护的右图图像特征点类序列中
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr); //没有追踪得到，在当前帧维护的右图特征点序列中放入空指针
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;   //返回两图成功匹配得到的特征点的数量
}

bool Frontend::BuildInitMap() 
{
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()}; //初始化前端的左相机是数据集维护的相机0 右相机是数据集维护的相机序列的相机1
    //将这两个经数据集初始化的相机的位姿提取出来放入该创建的位姿序列中
    size_t cnt_init_landmarks = 0; //路标点数量
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) 
    {
        if (current_frame_->features_right_[i] == nullptr) continue; //之前右图没有匹配到的特征点均向特征点序列中放入了空指针
        // 利用路标点来创建空指针
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),  		//points存储的实际上是归一化坐标 深度的默认值为1
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
	//pixel2camera是从像素坐标转换为相机坐标系 默认的深度值是1
	//points序列的元素是Vec3三维向量 这里分别存储了匹配的一对特征点的左目相机下的相机和右目相机下的坐标
        Vec3 pworld = Vec3::Zero();
	//下面是用三角化公式来求出路标点  分别传入左相机和右相机的位姿
	//将工厂函数产出的智能指针放入类的容器 确保资源引用数始终>=1从而不被析构
        if (triangulation(poses, points, pworld) && pworld[2] > 0) 
	{
            auto new_map_point = MapPoint::CreateNewMappoint();  		//创建一个新的路标点指针(路标点默认构造) 还没有填入数据
            new_map_point->SetPos(pworld);              			//设置这个新创建的路标点类实例的坐标
            new_map_point->AddObservation(current_frame_->features_left_[i]);   //将这个用来三角化的左图的特征点放入路标点实例维护的特征点的序列中 且将持有数递增1
            new_map_point->AddObservation(current_frame_->features_right_[i]);  //将这个用来三角化的左图的特征点放入路标点实例维护的特征点的序列中 且将持有数递增1
            current_frame_->features_left_[i]->map_point_ = new_map_point;      //利用一个shared_ptr来初始化一个weak_ptr 防止发生循环引用
            current_frame_->features_right_[i]->map_point_ = new_map_point;	//相当于特征点持有的弱引用指针指向路标点
            cnt_init_landmarks++;						//三角化得到的路标点递增1
            map_->InsertMapPoint(new_map_point);				//将这个路标点插入地图
        }
    }
    current_frame_->SetKeyFrame();						//当前帧设置为关键帧 加入关键帧的标志位以及工厂函数分配的关键帧的id号
    map_->InsertKeyFrame(current_frame_);					//在地图中把当前帧也设置为关键帧
    backend_->UpdateMap();							//触发地图更新 启动优化   后端启动一次BA优化 同时优化路标点的位姿和关键帧的位姿

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

bool Frontend::Reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

}  // namespace myslam