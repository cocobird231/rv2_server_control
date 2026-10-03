# rv2_server_control R1

接收最新 `rv2_control_signal_transport` 的 R1 Joy／Twist sources，仲裁 active channel，將變化轉為 `/api/sport/request` 的 Unitree sport requests。控制描述與 service 完全使用 `r1_interfaces`，不依賴 legacy `rv2_interfaces` package。

## 啟動與實機觀察

建置並 source workspace 後，開三個終端：

```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
ros2 launch rv2_csm_topic_bridge topic_bridge.launch.py topic_name:=/joy
ros2 launch rv2_server_control control_server.launch.py
```

Server launch 預設同時啟動一個 `csm_master_node`，讓來源與伺服器重啟後能重新對帳註冊。已有 master 時加 `start_master:=false`；若改 `master_name`，bridge 與 server 必須使用相同名稱。直接 `ros2 run rv2_server_control control_server` 不會啟動 master，需另行執行 `ros2 run rv2_control_signal_transport csm_master_node`。

`joy_node` 的 `autorepeat_rate` 必須啟用：未持續收到新 Joy 訊息時，bridge 會停止轉發，server 會依來源的 timeout 急停。保持相同搖桿位置也必須有新樣本；bridge 不會重送舊樣本維持假存活。

```bash
# 觀察實際 robot API 指令與 R1 endpoints：
ros2 topic echo /api/sport/request
ros2 topic echo /control_server/status
ros2 service call /control_server/control_signal_info_req r1_interfaces/srv/ControlSignalInfoReq '{}'
```

預設 console 顯示 active channel 變更、`[OUTPUT/joy]`／`[OUTPUT/twist]`、`[STOP/...]`，`log_output:=false` 可關閉一般輸出紀錄。`ros2 launch rv2_server_control test_control_server.launch.py` 會另外啟動 Unitree request logger，可在無機器人時觀察 API ID 與 payload。

手動斷線驗收：先推動搖桿，再停止 joy node、拔除裝置、停止 bridge；確認 `/api/sport/request` 出現 StopMove（API 1003），保持失聯時不反覆發送。恢復後保持與斷線前相同的非零操控值，也應再次出現 Move（API 1008）。另可重啟 server，確認 master 對帳後恢復。不同 joy 驅動可能在拔除裝置時繼續發 neutral samples；此時 server 反映實際收到的資料，裝置是否斷線應另從 joy driver log 確认。

## 訊號與仲裁

- Joy 使用既有實機 mapping：axes 0／1／3 → Move x／y／yaw；axes 5（R2）小於 0.5 → 急停，放開時為 +1。A／B／X／Y（buttons 0–3）按下邊緣 → StandUp／StandDown／StopMove／RecoveryStand。button 11 → request-active；裝置沒有該索引時視為未按下，不越界。
- Twist 使用 linear.x／linear.y／angular.z；linear.z、angular.x、angular.y 同時 -99 → 急停，同時 +99 → request-active。sentinel 訊息不送入一般運動輸出。
- 每種 message type 獨立保有一個 active channel。第一個有效來源取得控制權；健康的 active source 不會因較高 priority 的普通訊息而被搶走。request-active 需嚴格較高 priority；手動 `setActiveSink<T>()` 只接受仍有新鮮資料的 channel。
- R1 priority 1–100 全部是一般優先級，100 沒有保留急停語意。R1 liveness 僅 INITIAL／ACTIVE／TIMEOUT／DISCONNECTED，不再使用 LOW_FREQ 或 UNKNOWN。
- active source 超時會選擇最高 priority 的可用 fallback，同 priority 以 channel 字典順序決定。**交出控制權前先 StopMove**，避免 neutral fallback 或持續按住 request-active 時殘留上一來源的非零運動。無 fallback 時清空 active，失聯只觸發一次急停。
- 普通訊號由收到的新訊息驅動；相同 Move 值與持續按住同一按鍵不重複輸出。切換／急停／失聯／重新註冊時清除轉換狀態，使同值重連能恢復。初次啟動的 idle zero 不產生多餘 Move。
- `timeout_ns` 與 `disconnect_timeout_ns` 來自 source descriptor；兩者都從最後活動時間計算，後者不是進入 TIMEOUT 後再等待的時間。0 分別代表停用；非零 disconnect 必須大於非零 timeout。沒有資料的新 registration 不會被自動選為輸出來源。

預設 server parameters 位於 `config/control_server.yaml`：watchdog 50ms、manager status 100ms、CSM timeout 600ms、CSM disconnect 6000ms。Manager status 必須小於 CSM timeout 的一半；參數無效會在啟動時拒絕。

## 建置依賴

標準 ROS dependencies 由 rosdep 安裝。將下列 source packages 放在 server 的 sibling 位置；測試框架由 `test_depends.repos` 唯讀掛載：

| 路徑 | 來源／本次驗證基線 |
|---|---|
| `../r1_interfaces` | R1 v0.1.2，已合併主線 `a0530d7` |
| `../rv2_control_signal_transport` | R1 v0.2.0，已合併主線 `5a91d31` |
| `../rv2_csm_topic_bridge` | 本次 R1 migration 配對版本 |
| `../joy_interpreter` | `test` 分支 `944306b61746dcdaa932a404994b8f829a2a9611` |
| `../unitree_api` | 官方 `unitree_ros2` commit `5204e6e098ee53f4bd929bd77eb1d387cd0fa842` 的 `cyclonedds_ws/src/unitree/unitree_api` |

`../unitree_api` 可用 symlink 指向該官方 source 子目錄，例如：

```bash
ln -s /path/to/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api ../unitree_api
```

只取 `unitree_api`，不需 `unitree_go` 或整個 Unitree workspace。上游 `unitree_api` CMake 使用但 manifest 未宣告的 `rosidl_generator_dds_idl`，本 package 的 test dependency 補足乾淨 Docker 建置環境。JSON CMake package 是 `nlohmann_json`，對應 rosdep key 為 `nlohmann-json-dev`。

## 測試

每個 package 使用自己的固定版 framework submodule；本包固定已合併 framework v0.5.1。所有 ROS 測試全域串行執行，不將不同 Docker 名稱視為 DDS 隔離。

```bash
git submodule update --init --recursive
./r1_test_framework/test_build.sh
./r1_test_framework/test_deps.sh
./r1_test_framework/test_run.sh
./r1_test_framework/test_clean.sh
./r1_test_framework/test_lint.sh
```

完整 test_run 包含 unit、integration 與 ament 檢查。新 owner 尚未列入 framework sanitizer matrix，因此 ASan／UBSan／TSan 都顯示 N/A，不冒稱通過 sanitizer；框架的顯式 TSan 開關仍預設 off。lint 是獨立唯讀 Docker gate。可在 build/deps 後以 `test_run.sh -s unit` 或 `-s integration` 單獨執行分類；結果保存於 `test_env/jazzy/`。

- `test/unit/test_control_server.cpp`：Joy 短陣列、Twist sentinel、converter per-channel／per-instance 去重、reset 同值恢復、button press edge。
- `test/integration/test_control_server.cpp`：R1 topic/service Joy/Twist、priority 100、request-active 優先權、超時 fallback 與單次急停、單封訊號、重新註冊、disabled timeout、manual selection、TypeConfig 更新、callback 執行中安全結束。
- `test/integration/test_joy_bridge.py`：真正 master／bridge／server processes，best-effort Joy→Unitree x/y/yaw 與按鍵、保持值去重、input silence 無重播、同值恢復、bridge crash/restart、server crash/restart 與 master 對帳。readiness 根據實際輸出與 R1 incarnation 判定。

新版基於已提交 develop `cf99d823`，原 checkout 的 staged／unstaged wireless intervention／ControlServerReq 實驗仍保留在原處；該未提交功能依賴 legacy interface，不屬本次 R1 發布。Package 版本、tag、PR 依 project `r1_todo.md` §1 在功能與驗證完成後獨立處理。
