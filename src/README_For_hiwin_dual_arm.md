# For_hiwin_dual_arm — HIWIN 雙臂避障系統

RA610-1476-GC2（A 臂）＋ RA605-710-GC2（B 臂）雙臂自我避障路徑規劃系統。
主機端負責 MoveIt 規劃與避障最佳化；手臂端跑原廠驅動、各自關進獨立 ROS_DOMAIN，
由橋層跨域串接（Plan B：domain 隔離，原廠驅動零修改）。

## 系統組成

| Package | 角色 |
|---|---|
| `hiwin_dual_arm` | MoveIt 設定 + `brain.launch.py`（規劃端主入口） |
| `hiwin_dual_arm_description` | 雙臂 URDF / meshes / 平台 |
| `dual_arm_lag_{gd,cg,newton}_planner` | 標準拉格朗日模型 × 三種求解法（規劃器插件） |
| `dual_arm_alm_{gd,cg,newton}_planner` | 增廣拉格朗日模型 × 三種求解法（規劃器插件） |
| `dual_arm_domain_bridge` | 跨域橋層：上行合併 joint_states、下行透明轉送軌跡（細節見該包 README） |

## Domain 配置（現行方案）

| 端 | ROS_DOMAIN_ID |
|---|---|
| 主機（brain / RViz / 橋層 host 端） | **0** |
| A 臂 RA610-1476 | **10** |
| B 臂 RA605-710 | **20** |

> ⚠ 橋層 launch 的**預設值是 10 / 20 / 30**，與現行方案不同——啟動橋層時**務必帶
> `host_domain:=0 arm_a_domain:=10 arm_b_domain:=20`**，不可省略。
> 另外：橋層內部以 `set_domain_id` 明碼指定 domain、**不吃 `ROS_DOMAIN_ID` 環境變數**；
> 但 brain 與手臂驅動是一般 ROS 節點，**吃**環境變數——所以主機端啟動 brain 前要
> `export ROS_DOMAIN_ID=0`，手臂端啟動驅動前要 export 各自的 10 / 20。

## 啟動總順序

```
① 手臂端 A（D10）→ ② 手臂端 B（D20）→ ③ 主機橋層 → ④ 主機 brain
```

橋層啟動時手臂驅動必須已在線；④ 不需另開 joint_states merger（上行橋已完成合併）。

---

## 主機端指令

每個終端先確認環境再啟動（`echo` 兩行用來確認目前 domain 與 RMW，避免上一個
終端殘留的設定造成跨域錯亂）：

```bash
# 終端 1：brain（MoveIt 規劃端）
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
source ~/hiwin_ws/install/setup.bash
ros2 launch hiwin_dual_arm brain.launch.py            # RViz 預設開啟；關閉用 use_rviz:=false
```

```bash
# 終端 2：橋層（二選一）
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
source ~/hiwin_ws/install/setup.bash

# 方式 1：用 action 發軌跡（端點透明轉送，RViz 原生 Execute/Stop）
ros2 launch dual_arm_domain_bridge bridge_relay.launch.py host_domain:=0 arm_a_domain:=10 arm_b_domain:=20

# 方式 2：用 topic 發軌跡
ros2 launch dual_arm_domain_bridge bridge_topic.launch.py host_domain:=0 arm_a_domain:=10 arm_b_domain:=20
```

> 註：`bridge_topic.launch.py` 未包含於本壓縮包之 bridge 目錄（僅含 bridge_relay），
> 使用前請確認工作空間內已有該 launch（待補）。

---

## 手臂端指令（三種情境）

共同前置（**實體控制器上**每次都要做；電腦模擬只需 source）：

```bash
sudo /etc/init.d/ethercat start        # 僅實體控制器需要
source /opt/ros/humble/setup.bash      # 僅實體控制器需要
cd ~/ws_ros2 && source install/setup.bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID                    # 確認目前 domain，再 export 覆蓋
```

### 情境一：電腦上模擬（fake hardware，開發機）

```bash
# RA610（A 臂）
source ~/hiwin_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra610_1476 cabinet:=gc2 use_fake_hardware:=true launch_rviz:=false

# RA605（B 臂）
source ~/hiwin_ws/install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra605_710 cabinet:=gc2 use_fake_hardware:=true launch_rviz:=false
```

### 情境二：機器上模擬（實體控制器 + fake hardware）

在各臂控制器上做完共同前置後：

```bash
# RA610（A 臂控制器）
export ROS_DOMAIN_ID=10
taskset -c 2 ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra610_1476 cabinet:=gc2 use_fake_hardware:=true launch_rviz:=false

# RA605（B 臂控制器）
export ROS_DOMAIN_ID=20
taskset -c 2 ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra605_710 cabinet:=gc2 use_fake_hardware:=true launch_rviz:=false
```

（`taskset -c 2` 將驅動綁定至 CPU 核心 2，確保 EtherCAT 週期即時性。）

### 情境三：機器實跑（實際驅動手臂）

與情境二差異：`cabinet:=ecat`、去掉 `use_fake_hardware`，且啟動後**須另開終端
重置安全繼電器**手臂才會上電。

```bash
# RA610（A 臂控制器）— 終端 1：驅動
export ROS_DOMAIN_ID=10
taskset -c 2 ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra610_1476 cabinet:=ecat launch_rviz:=false

# RA610 — 終端 2：重置安全繼電器（同樣先做共同前置 + export ROS_DOMAIN_ID=10）
ros2 service call /gpio_controller/set_io hiwin_msgs/srv/SetIO "{io_group: system, interface_name: reset_safety_rly, value: 1.0}"
```

```bash
# RA605（B 臂控制器）— 終端 1：驅動
export ROS_DOMAIN_ID=20
taskset -c 2 ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra605_710 cabinet:=ecat launch_rviz:=false

# RA605 — 終端 2：重置安全繼電器（先做共同前置 + export ROS_DOMAIN_ID=20）
ros2 service call /gpio_controller/set_io hiwin_msgs/srv/SetIO "{io_group: system, interface_name: reset_safety_rly, value: 1.0}"
```

---

## 啟動後驗證

主機（D0）上確認上行打通：

```bash
ros2 topic echo /joint_states --once
```

應看到 **12 個關節**（兩臂各 6 軸、名稱已加前綴）。

## 疑難排解

| 症狀 | 原因 | 處理 |
|---|---|---|
| 主機看不到 `/joint_states` | 橋層未啟動，或橋層 domain 參數與手臂實際 domain 不符 | 確認 ③ 已啟動；橋層必帶 `host_domain:=0 arm_a_domain:=10 arm_b_domain:=20` |
| 只看到單臂 6 個關節 | 另一臂驅動未啟動或 domain 錯 | 回到 ①② 檢查該臂的 `export ROS_DOMAIN_ID` |
| 改了 `ROS_DOMAIN_ID` 橋層沒反應 | 橋層不讀環境變數（設計如此） | 一律用 launch 參數指定 |
| 實跑時手臂不動、驅動無錯誤 | 安全繼電器未重置 | 執行該臂的 `set_io reset_safety_rly` 服務呼叫 |
