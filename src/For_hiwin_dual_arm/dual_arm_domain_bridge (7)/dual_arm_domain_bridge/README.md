# dual_arm_domain_bridge — 跨域橋層（Plan B：domain 隔離）

## 這個 package 在做什麼

兩支 HIWIN 手臂的原廠驅動皆以「單臂」為前提：關節名一律 `joint_1`～`joint_6`（無前綴）、狀態一律發佈於 `/joint_states`、控制器一律為 `/joint_trajectory_controller`。若兩臂直接放進同一個 ROS 2 網路，關節名與 topic 必然衝突。

本 package 採 **domain 隔離** 解法：**手臂端跑原封不動的原廠 launch**，各自關進獨立的 ROS_DOMAIN（A 臂 D20、B 臂 D30），彼此互不可見；主機（D10）透過本橋層跨域搬運資料，並在上行時為關節名加上前綴。原廠驅動與介面契約完全不需修改。

## 組成

兩支執行檔 + 一支 launch：

| 程式 | 方向 | 角色 | 備註 |
|------|------|------|------|
| `joint_state_uplink_bridge` | 上行（眼睛） | 訂閱 D20 / D30 的 `/joint_states`，各自加前綴後合併，發佈到 D10 | **必跑**，否則主機看不到手臂 |
| `trajectory_downlink_endpoint_relay` | 下行（手） | 將 RViz / MoveIt 的 Execute、Stop 透明轉送至各臂 | 失敗語意與直連相同 |

兩支執行檔皆可用位置參數 `[host_d armA_d armB_d]` 指定 domain。

> **重要**：domain 是程式內部以 `set_domain_id` 明碼指定的，**不吃終端機的 `ROS_DOMAIN_ID` 環境變數**。`export ROS_DOMAIN_ID=...` 對本橋層無效，要改 domain 請一律走 launch 參數或位置參數。

## 使用前提（介面契約）

手臂端驅動須符合以下介面（HIWIN 原廠驅動即符合，無須改動）：

- 關節名：`joint_1`～`joint_6`，無前綴，單位 radian
- 軌跡介面：`/joint_trajectory_controller/follow_joint_trajectory`
- 狀態發佈：`/joint_states`

## 快速啟動

### 橋層（一鍵）

```bash
# 使用預設 domain（host=10 / armA=20 / armB=30）
ros2 launch dual_arm_domain_bridge bridge_relay.launch.py

# 手臂 domain 與預設不同時，以參數覆蓋（三個參數都可個別指定）
ros2 launch dual_arm_domain_bridge bridge_relay.launch.py \
    host_domain:=10 arm_a_domain:=20 arm_b_domain:=30
```

| launch 參數 | 意義 | 預設 |
|---|---|---|
| `host_domain` | 主機（規劃端）domain | 10 |
| `arm_a_domain` | A 臂（RA610-1476）domain | 20 |
| `arm_b_domain` | B 臂（RA605-710）domain | 30 |

### 全系統啟動順序

依 ①→④ 順序啟動，**順序不可顛倒**（橋層啟動時手臂端必須已在線）：

```
① A 臂（RA610-1476）：於 Domain 20 依 HIWIN 提供之驅動方式啟動
② B 臂（RA605-710） ：於 Domain 30 依 HIWIN 提供之驅動方式啟動
③ 橋層：
   ros2 launch dual_arm_domain_bridge bridge_relay.launch.py
   （若手臂實際 domain 非 20 / 30，以 arm_a_domain / arm_b_domain 參數對應修改）
④ 主機（D10）：
   ros2 launch hiwin_dual_arm brain.launch.py launch_rviz:=true
   （不要再開 merger — joint_states 合併已由上行橋完成，重複開會造成雙重合併）
```

### 啟動後驗證

在主機（D10）確認上行是否打通：

```bash
ros2 topic echo /joint_states --once
```

應看到 **12 個關節**（兩臂各 6 軸、名稱已加前綴）。若只有 6 個或沒有輸出，見下方疑難排解。

## 疑難排解

| 症狀 | 原因 | 處理 |
|---|---|---|
| 主機看不到 `/joint_states` | 橋層未啟動，或 domain 參數與手臂實際 domain 不符 | 確認 ③ 已啟動；核對 `arm_a_domain` / `arm_b_domain` 與手臂端一致 |
| 改了 `ROS_DOMAIN_ID` 沒效果 | 本橋層不讀環境變數（設計如此） | 改用 launch 參數或位置參數 |
| 只看到單臂 6 個關節 | 另一臂驅動未啟動或 domain 錯 | 回到 ①② 檢查該臂 |
