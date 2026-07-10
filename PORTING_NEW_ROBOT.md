# 移植到不同機械手臂模型 — 改造手冊

本手冊說明：這個 workspace 目前的 6 個雙臂避障規劃器（`dual_arm_alm_*_planner` /
`dual_arm_lag_*_planner`）是**針對 HIWIN RA610（A 臂）+ RA605（B 臂）寫死的**，如果要
換成別的機型（不同臂長、不同球包覆模型、不同基座間距，甚至不同軸數），需要改哪些檔案、
怎麼改。基本操作方式（建置、啟動、切換規劃器、RViz 使用）請見主手冊
[`README.md`](README.md)，本文件不重複、也不更動主手冊內容。

---

## 1. 為什麼不能只改 yaml 就套用到新機型

主手冊 §4 提到「改 `config/*.yaml` 不用重 build」，但那些 yaml
（`dual_arm_*_planning.yaml`）只調的是**演算法收斂參數**（危險因子閾值、ALM 罰參數…）。

真正決定「這是哪一支手臂」的資訊——**正向運動學（FK）連桿長度**與**球包覆碰撞模型**——
目前是**寫死在 6 個規劃器套件各自的 C++ 原始碼裡**，不是從 URDF 或 yaml 讀進來的。也就是說：

- 現在的 URDF/`kinematics.yaml`（給 RViz 顯示、給 OMPL/CHOMP/Pilz 用）跟
  6 個 `dual_arm_*` 規劃器內部用的 FK 常數，是**兩份各自獨立、需要手動保持一致**的資料。
- 只換 URDF 不改 C++，RViz 畫面會顯示新機型，但 `dual_arm_*` 規劃器算出來的軌跡仍然是照
  RA610/RA605 的臂長跟球在算，形狀對不上，可能誤判/漏判碰撞。

換機型必須同時改「MoveIt 標準設定」與「規劃器 C++ 常數」兩邊，以下分別說明。

---

## 2. 第一部分：MoveIt 標準設定（URDF / SRDF / kinematics / limits）

這部分是一般 MoveIt2 專案換機型都要做的事，可以直接用 **MoveIt Setup Assistant**
（`ros2 launch hiwin_dual_arm setup_assistant.launch.py`）輔助產生，不需要手改。

| 要改的東西 | 位置 | 說明 |
|---|---|---|
| Mesh | `hiwin_description/meshes/<新機型>/` | 放新機型的 visual/collision mesh（STL/DAE）|
| URDF/xacro 巨集 | `hiwin_description/urdf/ra_macro.xacro`（或另建新巨集） | 定義新機型的 link/joint 結構、mesh 路徑 |
| 每軸連桿長度、關節限制 | `hiwin_description/config/<新機型>/default_kinematics.yaml`、`joint_limits.yaml`、`visual_parameters.yaml`（可參考現有 `ra610_1476/`、`ra605_710/`、`ra610_1355/`、`ra610_1869/`、`rs405_*` 幾個目錄的格式） | **這裡的數字之後要手動抄一份到 C++（第 3 部分）**，是本次移植最容易漏改的地方 |
| 雙臂組裝方式 | `hiwin_description/urdf/dual_hiwin_arms.urdf.xacro` | 兩臂的 `hiwin_arm` 巨集呼叫、`origin xyz="0 ±0.70 0" rpy="..."`（目前是 1400mm 對裝、面對面轉 180°）、底座掛載高度 `z=0.117` / `z=0.802` |
| Group / tip_link / 關節命名狀態 | `hiwin_dual_arm/config/dual_hiwin.srdf` | `big`/`small`/`Dual_arm` 三個 group、`tip_link`、`group_state`（`Dual_ori`/`big_ori`/`small_ori`）、`disable_collisions` |
| IK 外掛設定 | `hiwin_dual_arm/config/kinematics.yaml` | group 名稱要跟 SRDF 一致，其餘沿用 KDL 插件即可 |
| 關節限制（規劃用） | `hiwin_dual_arm/config/joint_limits.yaml` | 每軸 `max_velocity`/`max_acceleration`，換機型要照新機型規格表填 |
| 控制器/初始姿態 | `hiwin_dual_arm/config/initial_positions.yaml`、`moveit_controllers.yaml`、`ros2_controllers.yaml` | 若軸數不同（非 6 軸）要對應增減關節列表 |

若新機型一樣是 6 軸機械手、只是尺寸不同，上面這幾份檔案可以用 Setup Assistant
重新產生大部分內容，改動量不大。

---

## 3. 第二部分：6 個規劃器套件內部寫死的 FK / 球包覆模型（重點、容易漏改）

以下內容在 **6 個規劃器套件都各自複製了一份**（`dual_arm_alm_newton_planner`、
`dual_arm_alm_cg_planner`、`dual_arm_alm_gd_planner`、`dual_arm_lag_newton_planner`、
`dual_arm_lag_cg_planner`、`dual_arm_lag_gd_planner`），檔名依套件而異
（`newton_solver.cpp` / `cg_solver.cpp` / `gd_solver.cpp`），但**內容結構完全相同**，
換機型時**6 份都要同步改**，改一份忘記改另外五份是最容易出錯的地方。

以 `dual_arm_alm_newton_planner/src/newton_solver.cpp` 為例：

### 3.1 正向運動學連桿長度（`robot_arm_bubble_RA610` / `robot_arm_bubble_RA605`）

`newton_solver.cpp:150-158`（A 臂 / RA610）與 `newton_solver.cpp:200-208`（B 臂 / RA605），
把每一軸的平移量寫死成 4x4 齊次矩陣連乘：

```cpp
// A 臂範例（newton_solver.cpp:150-158）
T[0] = T_base * make_translation('z', 117.0);
T[1] = make_translation('z', 448.5) * make_rotation('z', J[0]);
T[2] = make_translation('y', 140.0) * make_rotation('x', J[1]);
T[3] = make_translation('z', 640.0) * make_rotation('x', J[2]);
T[4] = make_translation('z', 160.0) * make_rotation('y', J[3]);
T[5] = make_translation('y', 678.0) * make_rotation('x', J[4]);
T[6] = make_translation('y', 101.0) * make_rotation('y', J[5]);
```

這些數字（117 / 448.5 / 140 / 640 / 160 / 678 / 101，單位 mm）跟
`hiwin_description/config/ra610_1476/default_kinematics.yaml` 裡的關節偏移量（換算成 mm）
是同一份資料，只是手動抄了一次進 C++。**換新機型時，要把新機型的
`default_kinematics.yaml` 對應數值，依同樣的旋轉軸順序（z→x→x→y→x→y，範例中的轉軸別忘了
也要對應新機型的關節朝向）重新填進這兩個函式。**

> 如果新機型軸數不是 6，或旋轉軸順序不同（例如某軸是 roll 而不是 pitch），這兩個函式要
> 整段重寫，不是改幾個數字就好。

### 3.2 球包覆碰撞模型（`PEDESTAL_A/B`、`BUBBLES_A/B`）

`newton_solver.cpp:19-66`，四張表：

| 表 | 對應 | 顆數（RA610+RA605 範例） |
|---|---|---|
| `PEDESTAL_A` | A 臂底座固定球 | 4 顆，半徑 320mm |
| `BUBBLES_A` | A 臂手臂隨動球（掛在各連桿座標系） | 12 顆 |
| `PEDESTAL_B` | B 臂底座固定球 | 8 顆，半徑 350mm |
| `BUBBLES_B` | B 臂手臂隨動球 | 10 顆 |

每一列是 `BubbleDef{ link_id, radius, cx, cy, cz }`（`newton_solver.hpp:23-27`），
`link_id` 對應第幾個連桿座標系（0-indexed，`-1` 代表固定在底座 `T_base`），
`radius` 是球半徑，`cx/cy/cz` 是球心在該連桿座標系下的偏移量（mm）。

換新機型時要：
1. 用新機型的實際外型（CAD/mesh 尺寸）**重新設計一組球覆蓋整支手臂**，球數量、位置、半徑
   都可以跟原本不同，只要能大致包住新機型的外殼即可（球太少會漏判碰撞、球太多會拖慢速度）。
2. `robot_arm_bubble_RA610`/`RA605`（3.1 節那兩個函式）裡的 `NUM_TOTAL`、`NUM_PED`
   常數要同步改成新的球數。
3. 若新機型軸數不是 6，`BubbleDef.link_id` 的合法範圍（0~6）也要跟著調整。

### 3.3 跨臂碰撞遮罩（`get_collision_masks`）

`newton_solver.cpp:126-140`，`Eigen::Array<bool,16,18>` 這個矩陣尺寸是寫死的
`16 x 18`（= A 臂球數 x B 臂球數），內容是「A 臂第幾顆球對到第幾號連桿、B 臂同理，
底座附近的球彼此不檢查（`cAB` 回傳 false），手臂球才檢查」。換球數之後：

- `Eigen::Array<bool,16,18>` 要改成 `<bool, 新A球數, 新B球數>`。
- `sA[16]`/`sB[18]` 這兩個「球→連桿」映射陣列要跟著 3.2 節新設計的球表重新填。
- `cAB` 的判斷邏輯（哪些連桿之間才需要檢查碰撞）要依新機型手臂形狀重新評估，不一定是
  「第 4 節連桿以後才檢查」。

### 3.4 雙臂基座間距與朝向（`avoidance_system.cpp`）

`avoidance_system.cpp:123-124`：

```cpp
robotA_base_ = NewtonSolver::make_translation('y', 700) * NewtonSolver::make_rotation('z', 180);
robotB_base_ = NewtonSolver::make_translation('y', -700) * NewtonSolver::make_rotation('z', 0);
```

寫死了兩臂各自離世界原點 700mm（合計 1400mm 間距）、A 臂繞 z 轉 180° 面對 B 臂。若新的
雙臂佈局間距或朝向不同，要同步改這兩行，並且要跟第 2 部分 `dual_hiwin_arms.urdf.xacro`
裡的 `origin xyz="0 ±0.70 0" rpy="..."` 保持一致（否則 RViz 顯示的位置跟規劃器內部算的
位置對不上）。

---

## 4. 6 個套件要同步修改的檔案清單

每個 `dual_arm_*_planner` 套件都要改以下 3 個檔案（檔名依套件命名規則略有不同）：

| 通用內容 | ALM 系列檔名 | Lag 系列檔名 |
|---|---|---|
| FK 常數（3.1）+ 球表（3.2）+ 碰撞遮罩（3.3） | `src/{newton,cg,gd}_solver.cpp`（依套件） | 同左 |
| 基座間距/朝向（3.4） | `src/avoidance_system.cpp` | 同左 |
| 上述常數對應的宣告（陣列大小、函式簽名） | `include/<package>/{newton,cg,gd}_solver.hpp` | 同左 |

**建議流程**：先在 1 個套件（例如 `dual_arm_alm_newton_planner`）改完、用 RViz 手動 Plan
測試新機型跑起來合理（軌跡形狀正確、不會誤判/漏判碰撞）之後，再把同樣的 3.1~3.4 改動
複製套用到其餘 5 個套件，並各自 `colcon build --symlink-install --packages-select
<套件名>` 重編（C++ 改動一定要重編，不像 yaml 可以 symlink 生效）。

---

## 5. 換機型檢查清單（Checklist）

- [ ] `hiwin_description`：新機型 mesh、xacro 巨集、`default_kinematics.yaml` /
      `joint_limits.yaml` / `visual_parameters.yaml`
- [ ] `dual_hiwin_arms.urdf.xacro`：雙臂 `origin`（間距/朝向）、掛載高度
- [ ] `hiwin_dual_arm/config/dual_hiwin.srdf`：group、tip_link、group_state、
      disable_collisions
- [ ] `hiwin_dual_arm/config/kinematics.yaml`、`joint_limits.yaml`、
      `initial_positions.yaml`、`moveit_controllers.yaml`、`ros2_controllers.yaml`
- [ ] 6 個 `dual_arm_*_planner` 套件，每個都要改：
  - [ ] `robot_arm_bubble_*`：FK 連桿長度、轉軸順序、軸數
  - [ ] `PEDESTAL_A/B`、`BUBBLES_A/B`：重新設計的球包覆模型
  - [ ] `get_collision_masks`：陣列尺寸、`sA/sB` 映射、`cAB` 判斷邏輯
  - [ ] `avoidance_system.cpp` 裡的 `robotA_base_`/`robotB_base_`（基座間距/朝向，需與
        URDF 一致）
  - [ ] 對應 `.hpp` 裡的陣列大小/函式簽名
  - [ ] `config/dual_arm_*_planning.yaml` 的 `joint_prefix_A`/`joint_prefix_B`
        （若改了 SRDF 裡的關節命名前綴才需要）
  - [ ] `colcon build --symlink-install --packages-select <套件名>` 重新編譯
- [ ] 在 RViz 用新機型跑幾組 Start/Goal，確認軌跡形狀合理、無誤判/漏判碰撞

---

## 6. 已知限制

- 目前的架構把「這是哪支手臂」的資訊複製了 **兩份**（URDF/yaml 一份、6 個套件各自的
  C++ 一份，等於總共 7 份），沒有單一事實來源（single source of truth）。長期若要支援
  多機型切換，建議的重構方向是把 3.1~3.3 的常數改成從 yaml/URDF 在執行期讀入，而不是
  編譯期寫死；但這需要改動 6 個套件的核心演算法介面，屬於較大工程，本手冊只說明目前
  架構下「如何手動移植」，不涉及重構。
- 若新機型軸數不是 6（例如 7 軸），除了 3.1~3.4，`X` 決策變數維度（每個組態 12 維 =
  2 臂 × 6 軸）、Hessian/Jacobian 相關矩陣維度也要跟著全面調整，改動範圍會遠大於本手冊
  列出的項目，建議另外評估工作量。
