#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <gpd/grasp_detector.h>

namespace gpd {
namespace apps {
namespace detect_grasps_json {

bool checkFileExists(const std::string &file_name) {
  std::ifstream file(file_name.c_str());
  return static_cast<bool>(file);
}

void printGraspJson(const candidate::Hand &hand, int index) {
  const Eigen::Vector3d &t = hand.getPosition();
  const Eigen::Matrix3d &R = hand.getOrientation();
  const double score = hand.getScore();
  const double width = hand.getGraspWidth();

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "{";
  std::cout << "\"index\":" << index << ",";
  std::cout << "\"score\":" << score << ",";
  std::cout << "\"width\":" << width << ",";
  std::cout << "\"depth\":0.02,";
  std::cout << "\"translation\":[" << t.x() << "," << t.y() << "," << t.z() << "],";
  std::cout << "\"rotation_matrix\":[";
  for (int r = 0; r < 3; ++r) {
    std::cout << "[";
    for (int c = 0; c < 3; ++c) {
      std::cout << R(r, c);
      if (c < 2) {
        std::cout << ",";
      }
    }
    std::cout << "]";
    if (r < 2) {
      std::cout << ",";
    }
  }
  std::cout << "]}";
  std::cout << "\n";
}

int DoMain(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: detect_grasps_json CONFIG_FILE PCD_FILE [NORMALS_FILE]\n";
    return 1;
  }

  const std::string config_filename = argv[1];
  const std::string pcd_filename = argv[2];
  if (!checkFileExists(config_filename) || !checkFileExists(pcd_filename)) {
    std::cerr << "Error: config or PCD file not found\n";
    return 1;
  }

  util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();

  const std::vector<double> camera_position =
      config_file.getValueOfKeyAsStdVectorDouble("camera_position", "0.0 0.0 0.0");
  Eigen::Matrix3Xd view_points(3, 1);
  view_points << camera_position[0], camera_position[1], camera_position[2];

  util::Cloud cloud(pcd_filename, view_points);
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cerr << "Error: empty point cloud\n";
    return 1;
  }

  if (argc > 3) {
    cloud.setNormalsFromFile(argv[3]);
  }

  GraspDetector detector(config_filename);
  detector.preprocessPointCloud(cloud);

  const bool centered_at_origin =
      config_file.getValueOfKey<bool>("centered_at_origin", false);
  if (centered_at_origin) {
    cloud.setNormals(cloud.getNormals() * (-1.0));
  }

  std::vector<std::unique_ptr<candidate::Hand>> grasps = detector.detectGrasps(cloud);
  std::cout << std::boolalpha;
  std::cout << "{\"type\":\"gpd_summary\",\"count\":" << grasps.size() << "}\n";
  for (size_t i = 0; i < grasps.size(); ++i) {
    printGraspJson(*grasps[i], static_cast<int>(i));
  }
  return 0;
}

}  // namespace detect_grasps_json
}  // namespace apps
}  // namespace gpd

int main(int argc, char *argv[]) {
  return gpd::apps::detect_grasps_json::DoMain(argc, argv);
}
