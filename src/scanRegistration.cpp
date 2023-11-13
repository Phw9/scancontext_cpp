#include "scancontext/Scancontext.h"

#include "fmt/fmt.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(seq_dir, "/data/new_dataset_project/hangpa01",
    "sequence directory.");
DEFINE_string(data_txt_dir, "/root/dev/scancontext_cpp/data/hangpa01.txt",
    "data_txt_name.");    
DEFINE_int32(return_idx, 10,
    "loop return idx.");
using SCPointType = pcl::PointXYZI;

std::vector<std::string> scan_fns;
std::vector<double> timestamps;
std::vector<Eigen::Quaterniond> quaternions;
std::vector<Eigen::Matrix3d> rotations;
std::vector<Eigen::Vector3d> translations;
std::vector<pcl::PointCloud<SCPointType>> pcls;
std::vector<std::vector<double>> datas;
std::vector<double> dists;
int N_SCAN;


std::queue<std::pair<int, int> > scLoopICPBuf;
std::queue<double> scLoopDistBuf;
std::mutex mBuf;

SCManager scManager;
double scDistThres, scMaximumRadius;

Eigen::Vector3d point2vec(pcl::PointXYZI& point)
{
  double x = static_cast<double>(point.x);
  double y = static_cast<double>(point.y);
  double z = static_cast<double>(point.z);

  return Eigen::Vector3d(x,y,z);
}

pcl::PointXYZI vec2point(const Eigen::Vector3d& vec)
{
    double x = vec.x();
    double y = vec.y();
    double z = vec.z();

    pcl::PointXYZI point;
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(z);

    return point;
}


void readLidarData(const std::string & seq_dir)
{
  auto data_fn = fmt::format("{}/lidar_data.txt", seq_dir);
  std::fstream f_data(data_fn.c_str(), std::ios::in);
  std::string line;
  while (std::getline(f_data, line))
  {
    if (line.empty()) { break; }
    std::stringstream ss;
    ss << line;
    double timestamp;
    std::string scan_fn;

    ss >> timestamp >> scan_fn;
    scan_fns.push_back(fmt::format("{}/{}", seq_dir, scan_fn));
    if (scan_fns.size() == timestamps.size()) { break; }
  }
  f_data.close();

  // ROS_ASSERT(scan_fns.size() == timestamps.size());
  fmt::print("[readLidarData] 1st Scan '{}'\n", scan_fns.front());
  fmt::print("[readLidarData] Fin Scan '{}'\n", scan_fns.back());
}

void readPoses(const std::string & seq_dir)
{
  auto data_fn = fmt::format("{}/pose_fast_lio2.txt", seq_dir);
  std::fstream f_data(data_fn.c_str(), std::ios::in);
  // ROS_ASSERT(f_data.is_open());
  std::string line;
  while (std::getline(f_data, line))
  {
    if (line.empty()) { break; }
    std::istringstream iss(line);
    double timestamp;
    Eigen::Quaterniond quat;
    Eigen::Vector3d tran;
    iss >> timestamp;
    iss >> tran(0) >> tran(1) >> tran(2);
    iss >> quat.x() >> quat.y() >> quat.z() >> quat.w();
    timestamps.push_back(timestamp);
    quaternions.push_back(quat);
    rotations.push_back(quat.toRotationMatrix());
    translations.push_back(tran);
  }
  f_data.close();

  // ROS_ASSERT(timestamps.size() != 0UL);
  // ROS_ASSERT(timestamps.size() == translations.size());
  // ROS_ASSERT(timestamps.size() == quaternions.size());
  // ROS_ASSERT(timestamps.size() == rotations.size());
  N_SCAN = timestamps.size();
  fmt::print("[readPoses] N_SCAN (updated): {}\n", N_SCAN);
}


void addScan(
  const std::string & scan_fn,
  pcl::PointCloud<pcl::PointXYZI> & merge_cloud)
{
  std::ifstream f_bin;
  f_bin.open(scan_fn, std::ifstream::in | std::ifstream::binary);
  if (!f_bin.is_open())
  {
    std::cerr << "Fail to open " << scan_fn << std::endl;
  }

  f_bin.seekg(0, std::ios::end);
  const size_t num_elements = f_bin.tellg() / sizeof(float);
  std::vector<float> buf(num_elements);
  f_bin.seekg(0, std::ios::beg);
  f_bin.read(
    reinterpret_cast<char *>(
      &buf[0]),
      num_elements * sizeof(float));
  fmt::print("Add {} pts\n", num_elements/4);
  for (std::size_t i = 0; i < buf.size(); i += 4)
  {
    pcl::PointXYZI point;
    point.x = buf[i];
    point.y = buf[i + 1];
    point.z = buf[i + 2];
    point.intensity = buf[i + 3];

    Eigen::Vector3d pv = point2vec(point);
    // pv = R_velo_wrt_cam * pv + t_velo_wrt_cam;
    // pv = R0.transpose() * pv;
    // pv = rotation * pv + translation;
    point = vec2point(pv);
    merge_cloud.push_back(point);
  }
}

void performSCLoopClosure(int& iter)
{
    static int cur = 0;
    double candidate_dist_ = -1;
    // scManager.makeAndSaveScancontextAndKeys( _scan_down );
    // std::cout << "polar size: " << scManager.polarcontexts_.size() << " "
    //           << scManager.polarcontext_invkeys_.size() << " " << scManager.polarcontext_vkeys_.size();
    auto detectResult = scManager.detectLoopClosureID( candidate_dist_ , datas, FLAGS_return_idx, iter); // first: nn index, second: yaw diff 
    // std::cout << "datas & datas.back() size: " << datas.size() << ", " << datas.back().size() << std::endl;
    datas.back().push_back(translations[datas.back()[0]].x());
    datas.back().push_back(translations[datas.back()[0]].y());
    datas.back().push_back(translations[datas.back()[0]].z());
    datas.back().push_back(quaternions[datas.back()[0]].x());
    datas.back().push_back(quaternions[datas.back()[0]].y());
    datas.back().push_back(quaternions[datas.back()[0]].z());
    datas.back().push_back(quaternions[datas.back()[0]].w());
    datas.back().push_back(translations[datas.back()[1]].x());
    datas.back().push_back(translations[datas.back()[1]].y());
    datas.back().push_back(translations[datas.back()[1]].z());
    datas.back().push_back(quaternions[datas.back()[1]].x());
    datas.back().push_back(quaternions[datas.back()[1]].y());
    datas.back().push_back(quaternions[datas.back()[1]].z());
    datas.back().push_back(quaternions[datas.back()[1]].w());    
    datas.back().push_back(candidate_dist_);
    // std::ofstream loutputFile;
    // loutputFile.open(lfilename, std::ios_base::out);
    // if (!loutputFile.is_open()) {
    //     std::cerr << "can't open file: " << lfilename << std::endl;
    //     return;
    // } else {
    //     std::cout << "loop_histories.size : " << loop_histories.size() << std::endl;
    //     for(auto data : loop_histories) {
    //         // _loop_kf_idx, _curr_kf_idx, _loop_kf_idx.pose.x, _loop_kf_idx.pose.y, _loop_kf_idx.pose.z, _curr_kf_idx.pose.x, _curr_kf_idx.pose.y, _curr_kf_idx.pose.z, candidate distance
    //         if(data.size() < 9) continue;
    //         loutputFile << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << " " << data[7] << " " << data[8] << " " << std::endl;
    //     }
    // }

    // loutputFile.close();

    int SCclosestHistoryFrameID = detectResult.first;
    const int prev_node_idx = SCclosestHistoryFrameID;
    const int curr_node_idx = cur; // because cpp starts 0 and ends n-1
    // cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;
    cur++;
    
    scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    // addding actual 6D constraints in the other thread, icp_calculation.
} // performSCLoopClosure


int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  readLidarData(FLAGS_seq_dir);
  readPoses(FLAGS_seq_dir);

  for(int i=0; i < scan_fns.size(); i++) {
    pcl::PointCloud<SCPointType> pcl; 
    addScan(scan_fns.at(i), pcl);
    std::cout << scan_fns.at(i) << " file read and start SC loop detect" << std::endl;
    scManager.makeAndSaveScancontextAndKeys( pcl );

  }
  for(int i=0; i< scan_fns.size(); i++) {
    performSCLoopClosure(i);
  }

  std::ofstream outputFile;
  std::string lfilename = FLAGS_data_txt_dir;
  outputFile.open(lfilename, std::ios_base::out);
  if (!outputFile.is_open()) {
      std::cerr << "can't open file: " << lfilename << std::endl;
      return -1;
  } else {
      std::cout << "datas & scan_fns size: " << datas.size()  << " and " << scan_fns.size() << std::endl;
      for(auto data : datas) {
          // _loop_kf_idx, _curr_kf_idx, _loop_kf_idx.pose.x, _loop_kf_idx.pose.y, _loop_kf_idx.pose.z, _curr_kf_idx.pose.x, _curr_kf_idx.pose.y, _curr_kf_idx.pose.z, candidate distance
          if(data.size() < 17) continue;
          outputFile << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << " " 
          << data[7] << " " << data[8] << " " << data[9] << " " << data[10] << " " 
          << data[11] << " " << data[12] << " " << data[13] << " " << data[14] << " " 
          << data[15] << " " << data[16] << std::endl;
      }
  }

  outputFile.close();

  return 0;
}