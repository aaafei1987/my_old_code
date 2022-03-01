/*
 * @Author: your name
 * @Date: 2021-12-23 13:51:02
 * @LastEditTime: 2022-01-05 13:45:25
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /mag_recognition/src/SCManager.cpp
 */
#include "SCManager.h"

SCManager::SCManager()
{
    sc_state.sc_matrix.clear();
    sc_state.ring_key.clear();
    sc_state.index_pose.clear();
    sc_state.kd_tree.reset();
}

SCManager::~SCManager()
{
    sc_state.sc_matrix.clear();
    sc_state.ring_key.clear();
    sc_state.index_pose.clear();
    sc_state.kd_tree.reset();
}

bool SCManager::InitConfig(const YAML::Node node)
{
    // a. ROI definition:
    MAX_RADIUS_ = node["max_radius"].as<float>();
    MAX_THETA_ = node["max_theta"].as<float>();
    // b. resolution:
    NUM_RINGS_ = node["num_rings"].as<int>();
    NUM_SECTORS_ = node["num_sectors"].as<int>();
    DEG_PER_SECTOR_ = MAX_THETA_ / NUM_SECTORS_;
    // c. ring key indexing interval:
    INDEXING_INTERVAL_ = node["indexing_interval"].as<int>();
    // d. min. key frame sequence distance:
    MIN_KEY_FRAME_SEQ_DISTANCE_ = node["min_key_frame_seq_distance"].as<int>();
    // e. num. of nearest-neighbor candidates to check:
    NUM_CANDIDATES_ = node["num_candidates"].as<int>();
    // f. sector key fast alignment search ratio:
    FAST_ALIGNMENT_SEARCH_RATIO_ = node["fast_alignment_search_ratio"].as<float>();
    // g. scan context distance threshold:
    SCAN_CONTEXT_DISTANCE_THRESH_ = node["scan_context_distance_thresh"].as<float>();

    // prompt:
    LOG(INFO) << "Scan Context params:" << std::endl
              << "\tmax. radius: " << MAX_RADIUS_ << std::endl
              << "\tmax. theta: " << MAX_THETA_ << std::endl
              << "\tnum. rings: " << NUM_RINGS_ << std::endl
              << "\tnum. sectors: " << NUM_SECTORS_  << std::endl
              << "\tre-indexing interval: " << INDEXING_INTERVAL_ << std::endl
              << "\tmin. key frame sequence distance: " << MIN_KEY_FRAME_SEQ_DISTANCE_ << std::endl
              << "\tnearest-neighbor candidates to check: " << NUM_CANDIDATES_ << std::endl
              << "\tfast alignment search ratio: " << FAST_ALIGNMENT_SEARCH_RATIO_ << std::endl
              << "\tloop-closure scan context distance thresh: " << SCAN_CONTEXT_DISTANCE_THRESH_ << std::endl
              << std::endl;

    return true;
}

bool SCManager::MakeHistoryDir()
{
    int frame_nums = record_history_laser_.size();
    for(int i=0;i<frame_nums;i++){
        pcl::PointCloud<pcl::PointXYZ> temp_frame = record_history_laser_[i];
        geometry_msgs::Pose temp_pose = record_mag_pose_[i];
        //计算scan context与其对应ring环
    
        Eigen::MatrixXf sc_temp = GetScanContext(temp_frame);
      
        std::vector<float> ring_king_temp = GetRingKing(sc_temp);
        //保存状态
        sc_state.sc_matrix.push_back(sc_temp);
        sc_state.ring_key.push_back(ring_king_temp);
        sc_state.index_pose.push_back(temp_pose);
    }

    sc_state.kd_tree.reset();
    sc_state.kd_tree = std::make_shared<RingKeyIndex>(NUM_RINGS_,sc_state.ring_key,10);

    return true;
}

//记录当前点云 
void SCManager::SetCurrentLaser(pcl::PointCloud<pcl::PointXYZ> cur_laser)
{
    current_matrix_ = GetScanContext(cur_laser);
    current_ring_key_ = GetRingKing(current_matrix_);
}
        
//查找 成功就返回true 失败false
bool SCManager::RecognLaser()
{
    int match_id = -1;
    if(
        sc_state.ring_key.size() <= (static_cast<size_t>(MIN_KEY_FRAME_SEQ_DISTANCE_))
    ) {
        match_result_.first = match_id;
        match_result_.second = 0.0;
        return false;
    }

    std::vector<size_t> candidate_indices(NUM_CANDIDATES_);
	std::vector<float> candidate_distances(NUM_CANDIDATES_);
    sc_state.kd_tree->query(&current_ring_key_.at(0) , NUM_CANDIDATES_ , &candidate_indices.at(0) , &candidate_distances.at(0));

    int optimal_index = 0;
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < NUM_CANDIDATES_; ++i)
    {   
        const Eigen::MatrixXf &candidate_scan_context = sc_state.sc_matrix.at(candidate_indices.at(i));

        std::pair<int, float> match_result = GetScanContextMatch(candidate_scan_context, current_matrix_); 
        
        int candidate_shift = match_result.first;
        float candidate_dist = match_result.second;
        
        if(candidate_dist < optimal_dist)
        {
            optimal_dist = candidate_dist;
            optimal_shift = candidate_shift;
            optimal_index = candidate_indices.at(i);
        }
    }

    float yaw_change_in_deg = optimal_shift * DEG_PER_SECTOR_;
    float yaw_change_in_rad = yaw_change_in_deg / 180.0f * M_PI;
    if(optimal_dist < SCAN_CONTEXT_DISTANCE_THRESH_ && abs(yaw_change_in_deg) <= 30)
    {
        match_id = optimal_index; 

        LOG(INFO) << std::endl
                  << "[Scan Context] Loop-Closure Detected " 
                  << "index : " << optimal_index << std::endl 
                  << "\tDistance " << optimal_dist << std::endl 
                  << "\tHeading Change " << yaw_change_in_deg << " deg." << std::endl
                  << std::endl;
    }else{
        match_result_.first = -1;
        match_result_.second = 0.0;
        //LOG(INFO)<<"no detected ";
        return false;
    }

    match_result_.first = match_id;
    match_result_.second = yaw_change_in_rad;
    return true;
}
        
//反馈结果 index号与对应位姿（该位姿是未调整 , 原有mag附带的）
void SCManager::GetOriginIndexAndPose(int & index , geometry_msgs::Pose & pose)
{
    index = match_result_.first;
    pose = record_mag_pose_[index];
    LOG(INFO)<<"find index is "<<index;
}
        
//反馈结果 index号与对应位姿（该位姿是调整过的 , 原有mag基础上调整）
void SCManager::GetCorrectIndexAndPose(int & index , geometry_msgs::Pose & pose)
{
    double yaw = match_result_.second;
    //@todo 需要pose旋转的
    index = match_result_.first;
    pose = record_mag_pose_[index];
    LOG(INFO)<<"find index is "<<index;
}

Eigen::MatrixXf SCManager::GetScanContext(pcl::PointCloud<pcl::PointXYZ> input_scan)
{
    //大致原理请自己看论文
   // num. of point measurements in current scan:
    const size_t N = input_scan.points.size();
            
    // init scan context:
    const float UNKNOWN_HEIGHT = -1000.0f;
    Eigen::MatrixXf scan_context = UNKNOWN_HEIGHT * Eigen::MatrixXf::Ones(NUM_RINGS_, NUM_SECTORS_);
        ;
    // iterate through point measurements and create scan context:
    float x, y, z;
    float radius, theta;
    //LOG(INFO)<<"MATRAD: "<<MAX_RADIUS_ <<" NUM_RINGS_ "<<NUM_RINGS_ << " MAX_THETA_ "<<MAX_THETA_<<" NUM_SECTORS_"<<NUM_SECTORS_;
    for (size_t i = 0; i < N; ++i) {
        // parse point measurement:
        x = input_scan.points.at(i).x;
        y = input_scan.points.at(i).y;
        z = input_scan.points.at(i).z + 2.0f;

        radius = hypot(x, y);
        theta = GetOrientation(x, y);

        // ROI check:
        if (radius > MAX_RADIUS_) {
            continue;
        }
 
        // get ring-sector index:
        int rid = GetIndex(radius, MAX_RADIUS_, NUM_RINGS_); 
        int sid = GetIndex(theta, MAX_THETA_, NUM_SECTORS_); 

        // update bin height:
        if (scan_context(rid, sid) < z) {
          
            scan_context(rid, sid) = z;
        }
            
    }
    // reset unknown height to 0.0 for later cosine distance calculation:
    for (int rid = 0; rid < scan_context.rows(); ++rid) {
        for (int sid = 0; sid < scan_context.cols(); ++sid) {
            if (UNKNOWN_HEIGHT == scan_context(rid, sid)) {
                scan_context(rid, sid) = 0.0;
            }
        }
    }

    return scan_context;
}

std::vector<float> SCManager::GetRingKing(Eigen::MatrixXf scan_context)
{
    std::vector<float> ring_key(scan_context.rows());

    for (int rid = 0; rid < scan_context.rows(); ++rid) {
        ring_key.at(rid) = scan_context.row(rid).mean();
    }

    return ring_key;    
}

Eigen::MatrixXf SCManager::GetSectorKey(const Eigen::MatrixXf &scan_context)
{
    Eigen::MatrixXf sector_key(1, scan_context.cols());

    for (int sid = 0; sid < scan_context.cols(); ++sid)
    {
        sector_key(0, sid) = scan_context.col(sid).mean();
    }
    return sector_key;
}

float  SCManager::GetOrientation(
    const float &x, 
    const float &y
) {
    float theta = 180.0f / M_PI * atan2(y, x);

    // make sure the orientation is consistent with scan context convension:
    if (theta < 0.0f) {
        theta += 360.0f;
    }

    return theta;
}

int SCManager::GetIndex(
    const float &value, 
    const float &MAX_VALUE, 
    const int RESOLUTION
) {
    int index = std::floor(static_cast<int>(RESOLUTION*value/MAX_VALUE));

    // this ensures value at MAX_VALUE will be cast into last bin:
    index = std::min(index, RESOLUTION - 1);

    return index;
} 


std::pair<int, float> SCManager::GetScanContextMatch(const Eigen::MatrixXf &target_scan_context, const Eigen::MatrixXf &source_scan_context) {
    // first perform fast alignment using sector key:
    Eigen::MatrixXf target_sector_key = GetSectorKey(target_scan_context);
    Eigen::MatrixXf source_sector_key = GetSectorKey(source_scan_context);
    int sector_key_shift = GetOptimalShiftUsingSectorKey(
        target_sector_key, 
        source_sector_key 
    );

    // generate precise alignment proposals:
    const int N = target_scan_context.cols();
    const int SEARCH_RADIUS = round(
        0.5 * FAST_ALIGNMENT_SEARCH_RATIO_ * N
    );
    std::vector<int> candidate_shifts{ sector_key_shift };
    for (int r = 1; r < SEARCH_RADIUS + 1; ++r)
    {
        candidate_shifts.push_back(
            (sector_key_shift + r + N) % N 
        );
        candidate_shifts.push_back( 
            (sector_key_shift - r + N) % N 
        );
    }
    std::sort(candidate_shifts.begin(), candidate_shifts.end());

    // then continue to precise alignment using scan context cosine distance:
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();
    for (int curr_shift: candidate_shifts)
    {
        Eigen::MatrixXf target_scan_context_shifted = CircularShift(
            target_scan_context, 
            curr_shift
        );

        float curr_dist = GetCosineDistance(
            target_scan_context_shifted, 
            source_scan_context 
        );

        if(curr_dist < optimal_dist)
        {   
            optimal_shift = curr_shift;
            optimal_dist = curr_dist;
        }
    }

    return std::make_pair(optimal_shift, optimal_dist);
}

int SCManager::GetOptimalShiftUsingSectorKey(
    const Eigen::MatrixXf &target, 
    const Eigen::MatrixXf &source
) {
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();

    for (int curr_shift = 0; curr_shift < target.cols(); ++curr_shift) {
        Eigen::MatrixXf curr_target = CircularShift(
            target, curr_shift
        );

        float curr_dist = (curr_target - source).norm();

        if(curr_dist < optimal_dist)
        {
            optimal_dist = curr_dist;
            optimal_shift = curr_shift;
        }
    }

    return optimal_shift;
}

Eigen::MatrixXf SCManager::CircularShift(
    const Eigen::MatrixXf &mat, 
    int shift 
) {
    if(0 == shift)
    {
        Eigen::MatrixXf shifted_mat(mat);
        return shifted_mat; // Early return 
    }

    Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( 
        mat.rows(), 
        mat.cols() 
    );
    for (int i = 0; i < mat.cols(); ++i) {
        int shifted_i = (i + shift) % mat.cols();
        shifted_mat.col(shifted_i) = mat.col(i);
    }

    return shifted_mat;
}

float SCManager::GetCosineDistance(
    const Eigen::MatrixXf &target_scan_context, 
    const Eigen::MatrixXf &source_scan_context 
) {
    const int N = target_scan_context.cols();

    int num_effective_cols = 0;
    float sum_sector_similarity = 0.0f;
    for (int sid = 0; sid < N; ++sid)
    {
        const Eigen::VectorXf target_sector = target_scan_context.col(sid);
        const Eigen::VectorXf source_sector = source_scan_context.col(sid);
        
        float target_sector_norm = target_sector.norm();
        float source_sector_norm = source_sector.norm();

        if( 0.0f == target_sector_norm || 0.0f == source_sector_norm ) {
            continue;
        }
             
        float sector_similarity = target_sector.dot(source_sector) / (target_sector_norm * source_sector_norm);

        sum_sector_similarity += sector_similarity;
        ++num_effective_cols;
    }
    
    return (0 == num_effective_cols ? 1.0f : (1.0f - sum_sector_similarity / num_effective_cols));
}
