//
// Created by gaoxiang on 19-5-2.
//

#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

namespace myslam {

Backend::Backend() {
    backend_running_.store(true);                                                       //原子量 相当于原子的写
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));		//后端的主线程 std::bind返回一个绑定的函数对象 通过后端类的隐式this指针来调用
}

//函数作用：唤醒后端优化线程
void Backend::UpdateMap()    
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
    //std::condition_variable map_update_; 是类中的条件变量
    //当std::condition_variable对象的某个wait函数被调用的时候 它使用 std::unique_lock(通过std::mutex)来锁住当前线程 
    //当前线程会一直被阻塞 直到另外一个线程在相同的std::condition_variable对象上调用了 notification函数来唤醒当前线程 --随机唤醒一个在等待的线程
}

void Backend::Stop() {
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();		//.join()等待子线程Backendloop执行完之后，主线程才可以继续执行下去，此时主线程会释放掉执行完后的子线程资源
}

void Backend::BackendLoop() 
{
    while (backend_running_.load()) 	//load是原子的读方法     该线层是一个循环
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);							//调用wait睡眠这个线程  直到有其他线程通过notify来唤醒 

        /// 后端仅优化激活的关键帧和路标点
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();  		//获得活跃的关键帧的哈希表
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();	//获得活跃的路标点的哈希表
        Optimize(active_kfs, active_landmarks);					//BA优化
    }
}

void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks) 				//参数：地图中活跃的关键帧的哈希表与活跃的路标点的哈希表
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;				//6-3 位姿与路标点  构建图优化，先设定g2o
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>		//线性求解器类型
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(			//梯度下降方法，可以从GN, LM, DogLeg 中选
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));	
    g2o::SparseOptimizer optimizer;						// 图模型
    optimizer.setAlgorithm(solver);						// 设置求解器

    // pose 顶点，使用关键帧的id
    std::map<unsigned long, VertexPose *> vertices;				//顶点类指针的哈希表
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) 
    {
        auto kf = keyframe.second;   						//kf是指向关键帧的指针
        VertexPose *vertex_pose = new VertexPose();  				// camera vertex_pose   相机顶点位姿  往图中增加顶点
        vertex_pose->setId(kf->keyframe_id_);					//将帧的id设置为位姿优化顶点的id
        vertex_pose->setEstimate(kf->Pose());					//添加需要优化的节点变量 相当于一个变量声明  --这个需要优化的变量是帧的位姿
        optimizer.addVertex(vertex_pose);					//往图中添加优化顶点v
        if (kf->keyframe_id_ > max_kf_id) 
	{
            max_kf_id = kf->keyframe_id_;					//比较求得关键帧的最大值序号
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});			//插入键值对  关键帧的id号--需要优化的位姿定点
    }

    // 路标顶点，使用路标id索引
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    // K 和左右外参 在初始化传入第一帧的情况下是0相机和1相机的位姿和左目相机的K
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel 卡尔检验的一个阈值
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;		//哈希表 键值对--双边优化类--特征点
    
    //vertices是误差节点 edges是误差边
    for (auto &landmark : landmarks) 				//landmark是路标点的键值对  landmark.second是路标点
    {
        if (landmark.second->is_outlier_) continue;		//初始值是false
        int landmark_id = landmark.second->id_;
	// 如果landmark还没有被加入优化，则新加一个顶点
	if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) 
	    {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());		//需要优化的变量--路标点的世界坐标系下的位置
                v->setId(landmark_id + max_kf_id + 1);		//编号设置 +1从原位姿顶点的最大编号的下一号码开始计算
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }
        
        auto observations = landmark.second->GetObs();		//返回这个路标点维护的特征点weak_ptr的序列
        for (auto &obs : observations) 				//obs是weak_ptr<Feature>
	{
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();				//feat是指向特征点的shared_ptr
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

            auto frame = feat->frame_.lock();			//frame是该特征点的持有的指向帧的shared_ptr
            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image_) 			//如果特征点是左图提取的
	    {
                edge = new EdgeProjection(K, left_ext);		//构造时传入相机内参 其中外参是左相机的位姿
            } else {						//					     利用特征点的性质来决定外参是左相机还是右相机
                edge = new EdgeProjection(K, right_ext);	//构造时传入相机内参 其中外参是右相机的位姿
            }

	    //vertices和vertices_landmarks存储帧以及路标点的键值对 供之后的feat查询并连接 换句话说 怎么找到edge误差边对应的帧位姿和路标点 这两个是必须要靠特征点连接起来的
            edge->setId(index);					     //	feat(特征点)--edge(frame(特征点对应的帧),landmark(特征点对应的路标点)) 
            edge->setVertex(0, vertices.at(frame->keyframe_id_));    // 位姿顶点        	!将路标点以及对应的特征点所属的帧位姿一起进行优化!
            edge->setVertex(1, vertices_landmarks.at(landmark_id));  // 路标顶点
            edge->setMeasurement(toVec2(feat->position_.pt));	     //观测数值误差是特征点的像素误差  分为左图特征点像素误差以及右图特征点像素误差
            edge->setInformation(Mat22::Identity());		     //尽管分为左图与右图 即左相机和右相机的位姿不同 但是它们的相对变换是相同的(T) 
            auto rk = new g2o::RobustKernelHuber();                  //这也是单一的把世界坐标系->相机坐标系拆分为世界坐标系->帧坐标系->相机坐标系的原因
            rk->setDelta(chi2_th);				     //(因为经过时间变换单一的世界坐标系->相机坐标系在左右目下是不同的）
            edge->setRobustKernel(rk);				     //在图优化结构中 利用特征点误差(不管左右图)共同作为参量优化一个共同的T 这就是分开写的用意
            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);					//往图中增加误差边

            index++;
        }
    }

    // 进行优化并消除异常值  执行优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5) 
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // 确定我们是否要调整离群值阈值
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th)  //卡尔检验阈值函数
	    {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;	//修改标志位 大于一定阈值 就是一个外围的点       从这里设置特征点是外围的还是内线的
            // 删除观察  在路标点中维护的特征点序列中删除该特征点
            ef.second->map_point_.lock()->RemoveObservation(ef.second);        			//外围的点直接就从路标点维护点序列中删除该特征点的指针
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
              << cnt_inlier;

    // 设置路标点以及帧的优化位姿
    for (auto &v : vertices) 
    {
        keyframes.at(v.first)->SetPose(v.second->estimate());		//优化并设置关键帧的位姿
    }
    for (auto &v : vertices_landmarks) 
    {
        landmarks.at(v.first)->SetPos(v.second->estimate());		//优化并设置路标点的位置
    }
}

}  // namespace myslam