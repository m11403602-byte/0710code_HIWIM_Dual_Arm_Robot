// =====================================================================
// data_io.hpp — CSV 寫入工具 (純 std, 零依賴)
// =====================================================================
//   多 sheet 用多檔對應 (MATLAB writecell 'Sheet' -> 各別 .csv)
//   只寫入, 不讀取 (waypoints 由呼叫端直接給)
// =====================================================================
#ifndef DUAL_ARM_LAG_CG_PLANNER_DATA_IO_HPP
#define DUAL_ARM_LAG_CG_PLANNER_DATA_IO_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dual_arm_lag_cg_planner
{

// 寫一個矩陣到 CSV (含表頭)
//   header: 欄名 (長度需 = mat.cols(); 空則不寫表頭)
//   mat:    資料 (每列一筆)
void write_csv(const std::string& path,
               const std::vector<std::string>& header,
               const Eigen::MatrixXd& mat);

// 寫「混合表頭 + 字串首欄 + 數值」的 CSV (給 Targets 表這種首欄是 Pt 名稱用)
//   row_labels: 每列的首欄字串 (長度 = mat.rows())
//   header:     欄名 (含首欄, 長度 = mat.cols()+1)
void write_csv_labeled(const std::string& path,
                       const std::vector<std::string>& header,
                       const std::vector<std::string>& row_labels,
                       const Eigen::MatrixXd& mat);

}  // namespace dual_arm_lag_cg_planner

#endif
