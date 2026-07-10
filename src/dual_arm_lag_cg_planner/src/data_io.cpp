// =====================================================================
// data_io.cpp — CSV 寫入工具實作
// =====================================================================
//   最底層、最單純的檔案：只有兩個對外函式 write_csv / write_csv_labeled，
//   只被 avoidance_system.cpp 的 export_unified()（【呼叫順序 3】）呼叫，
//   本身不呼叫任何其他 .cpp 的函式，是整條呼叫鏈的末端
// =====================================================================
#include "dual_arm_lag_cg_planner/data_io.hpp"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>   // C++17: 自動建立父資料夾

namespace dual_arm_lag_cg_planner
{

// 寫檔前確保父資料夾存在 (path 含資料夾時自動 mkdir -p)
static void ensure_parent_dir(const std::string& path)
{
  try {
    std::filesystem::path p(path);
    if (p.has_parent_path() && !p.parent_path().empty())
      std::filesystem::create_directories(p.parent_path());
  } catch (const std::exception& e) {
    std::cerr << "ensure_parent_dir: 建立資料夾失敗 " << path << ": " << e.what() << "\n";
  }
}

static void write_header(std::ofstream& f, const std::vector<std::string>& header)
{
  for (size_t j = 0; j < header.size(); ++j) {
    if (j) f << ",";
    f << header[j];
  }
  f << "\n";
}

// 【呼叫順序 3-a】write_csv — export_unified() 大部分 CSV (meta/summary/
//   inner/danger_final/danger_rounds 等) 都呼叫這個；每列自動加逗號分隔
void write_csv(const std::string& path,
               const std::vector<std::string>& header,
               const Eigen::MatrixXd& mat)
{
  ensure_parent_dir(path);
  std::ofstream f(path);
  if (!f.is_open()) {
    std::cerr << "write_csv: 無法開啟 " << path << "\n";
    return;
  }
  f << std::setprecision(10);
  if (!header.empty()) write_header(f, header);
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      if (j) f << ",";
      f << mat(i, j);
    }
    f << "\n";
  }
}

// 【呼叫順序 3-b】write_csv_labeled — export_unified() 的 targets.csv 等
//   「每列要帶文字標籤」(如 Head/q1/peak/q3/Tail) 的 CSV 呼叫這個版本
void write_csv_labeled(const std::string& path,
                       const std::vector<std::string>& header,
                       const std::vector<std::string>& row_labels,
                       const Eigen::MatrixXd& mat)
{
  ensure_parent_dir(path);
  std::ofstream f(path);
  if (!f.is_open()) {
    std::cerr << "write_csv_labeled: 無法開啟 " << path << "\n";
    return;
  }
  f << std::setprecision(10);
  if (!header.empty()) write_header(f, header);
  for (int i = 0; i < mat.rows(); ++i) {
    f << (i < static_cast<int>(row_labels.size()) ? row_labels[i] : "");
    for (int j = 0; j < mat.cols(); ++j) f << "," << mat(i, j);
    f << "\n";
  }
}

}  // namespace dual_arm_lag_cg_planner
