#include "include/scancontext/Scancontext.h"

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

  ROS_ASSERT(scan_fns.size() == timestamps.size());
  fmt::print("[readLidarData] 1st Scan '{}'\n", scan_fns.front());
  fmt::print("[readLidarData] Fin Scan '{}'\n", scan_fns.back());
}

void readPoses(const std::string & seq_dir)
{
  auto data_fn = fmt::format("{}/pose_fast_lio2.txt", seq_dir);
  std::fstream f_data(data_fn.c_str(), std::ios::in);
  ROS_ASSERT(f_data.is_open());
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

  ROS_ASSERT(timestamps.size() != 0UL);
  ROS_ASSERT(timestamps.size() == translations.size());
  ROS_ASSERT(timestamps.size() == quaternions.size());
  ROS_ASSERT(timestamps.size() == rotations.size());
  N_SCAN = timestamps.size();
  fmt::print("[readPoses] N_SCAN (updated): {}\n", N_SCAN);
}


void addScan(
  const std::string & scan_fn,
  pcl::PointCloud<pcl::PointXYZI> & merge_cloud,
  const Eigen::Vector3d & translation,
  const Eigen::Matrix3d & rotation)
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
    pv = R_velo_wrt_cam * pv + t_velo_wrt_cam;
    pv = R0.transpose() * pv;
    pv = rotation * pv + translation;
    point = vec2point(pv);
    merge_cloud.push_back(point);
  }
}


int main(void) {

    return 0;
}