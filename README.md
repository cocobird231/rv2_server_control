# rv2_server_control R1

接收最新 `rv2_control_signal_transport` 的 R1 Joy／Twist sources，在 Joy／Twist 間選擇唯一 active controller，將變化轉為 `/api/sport/request` 的 Unitree sport requests。控制描述與 service 完全使用 `r1_interfaces`，不依賴 legacy `rv2_interfaces` package。

可用 PyQt5／rclpy 桌面程式 `control_server_gui` 查看狀態與最後實際輸出，並透過既有三個 services 手動切換來源或解除 wireless interruption。現行 services、控制器切換與優先權規則，見 [Control Server 架構文件](docs/control_server_architecture.md)。

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
ros2 service call /control_server/control_server_status r1_interfaces/srv/ControlServerStatusReq '{}'
# 指定已註冊且 ACTIVE 的 controller_name；不是 channel_name
ros2 service call /control_server/switch_controller r1_interfaces/srv/SwitchControllerReq '{controller_name: local_xbox}'
# 手動解除 wireless interruption；不重播舊指令，等待新控制資料
ros2 service call /control_server/reset_interrupt r1_interfaces/srv/ResetInterruptReq '{}'
```

預設 console 顯示 active channel 變更、`[OUTPUT/joy]`／`[OUTPUT/twist]`、`[STOP/...]`，`log_output:=false` 可關閉一般輸出紀錄。`ros2 launch rv2_server_control test_control_server.launch.py` 會另外啟動 Unitree request logger，可在無機器人時觀察 API ID 與 payload。

手動斷線驗收：先推動搖桿，再停止 joy node、拔除裝置、停止 bridge；確認 `/api/sport/request` 出現 StopMove（API 1003），保持失聯時不反覆發送。恢復後保持與斷線前相同的非零操控值，也應再次出現 Move（API 1008）。另可重啟 server，確認 master 對帳後恢復。不同 joy 驅動可能在拔除裝置時繼續發 neutral samples；此時 server 反映實際收到的資料，裝置是否斷線應另從 joy driver log 確认。

`/wirelesscontroller`（`unitree_go/msg/WirelessController`）每收到一封訊息就鎖存 interruption，包含全零與重複值。第一次送出一次 StopMove，之後所有控制器輸出及 watchdog 停止指令都被阻擋；只有上面的 reset service 可解除。若 wireless 持續發送，復歸後下一封會再次中斷。可用 `wireless_topic` 參數或 ROS remapping 選擇其他 topic，subscription 接受 best-effort／reliable publisher。

狀態的 `active_controller_name` 是目前選定來源；`output_*` 是最後實際由此 server 發布的 Unitree request。`has_output` 表示曾經發布，不表示機器人仍移動，也不是執行 ACK。Wireless safety stop 的 `output_controller_name` 為空；去重的輸入不會更新這筆紀錄。

無機器人時可用以下單封資料驗證 interruption，再呼叫 status／reset。重複發送應保持 interrupted，且不重複 StopMove；reset 本身沒有輸出，下一封新 Joy／Twist 才能恢復。

```bash
ros2 topic pub --once /wirelesscontroller unitree_go/msg/WirelessController '{}'
```

## 桌面 GUI

建置、source workspace 後，在有 `DISPLAY` 或 `WAYLAND_DISPLAY` 的本機桌面執行。預設只開 GUI，連到既有 server：

```bash
ros2 launch rv2_server_control control_server_gui.launch.py
ros2 launch rv2_server_control control_server_gui.launch.py --show-args

# 也可直接啟動 GUI node，指定含 namespace 的 service prefix
ros2 run rv2_server_control control_server_gui --ros-args \
  -p server_name:=/robot/control_server \
  -p poll_interval_ms:=500 -p service_timeout_ms:=2000
```

尚未啟動 server 時，可以同時啟動 GUI、server 與預設 master；joy node、bridge 仍需另外啟動：

```bash
ros2 launch rv2_server_control control_server_gui.launch.py start_server:=true

# 已有 master，server 讀取指定 YAML；GUI 與 server 使用相同 service prefix
ros2 launch rv2_server_control control_server_gui.launch.py \
  start_server:=true start_master:=false \
  config_file:=/absolute/path/control_server.yaml \
  server_name:=/robot/control_server
```

| GUI node 參數／同名 launch argument | 預設 | 用途 |
|---|---|---|
| `server_name` | `control_server` | 三個操作服務的 prefix；相對名稱沿用 node namespace，`/robot/control_server` 為完整名稱 |
| `poll_interval_ms` | `500` | 非阻塞 status 輪詢間隔，必須為正整數 |
| `service_timeout_ms` | `2000` | Service request 等待期限，必須為正整數 |

GUI launch 另有 `start_server=false`、`start_master=true`、`config_file` 與 `master_name`。`start_master`、server YAML 與 `master_name` 只在 `start_server=true` 時生效。GUI launch 會將 `server_name` 明確傳給 server，因此即使使用預設 `control_server`，也會覆蓋 YAML 中的同名設定；要連其他名稱必須明確指定。`master_name` 預設留空，沿用 YAML。Watchdog、`log_output` 等 server 設定放在 YAML，GUI launch 不另設同名參數。

畫面顯示 Connected／stale、interrupted、目前 owner、已註冊 controller 表，以及最後實際 request 的 API ID、parameter、binary、來源與時間。`has_output=false` 表示尚無實際輸出；最後輸出不是最後輸入，也不是機器人 ACK。已斷線或過期的畫面資料不可當成目前狀態。

在表中選一列後按 **Switch to selected controller**，送出該列的 `controller_name`。Registered 不等於 transport ACTIVE；GUI 不自行推算來源是否可切換，最終由 server 檢查並回覆結果及 `reason`。Wireless interrupted 時停用切換；**Reset wireless interrupt** 只呼叫 reset service，成功後仍要等新的控制資料，下一封 wireless 訊息仍會再次鎖定。

失聯或操作尚未完成時停用操作按鈕。Service 逾時表示結果未知，server 可能已執行；需由後續新 status 確認，GUI 不自動重送 switch／reset，也不自動解除 interruption。Status 輪詢只查詢狀態，不影響 event-triggered 輸出；GUI 不直接發布 robot requests。

關閉視窗或按 Ctrl+C 會清理 GUI 的 ROS node 並結束程式。GUI-only 模式不停止既有 server；`start_server=true` 的組合 launch 在 GUI 結束時結束整個 launch session，包含該次啟動的 server／master。若將組合模式 include 到其他 launch，該 session 的其他節點也會一起結束；只要觀察既有系統時使用預設 GUI-only 模式。無桌面的自動化測試使用 `QT_QPA_PLATFORM=offscreen`，不代表已完成實體桌面驗收。

## 訊號與仲裁

- Joy 使用既有實機 mapping：axes 0／1／3 → Move x／y／yaw；axes 5（R2）小於 0.5 → 急停，放開時為 +1。A／B／X／Y（buttons 0–3）按下邊緣 → StandUp／StandDown／StopMove／RecoveryStand。button 11 → request-active；裝置沒有該索引時視為未按下，不越界。
- Twist 使用 linear.x／linear.y／angular.z；linear.z、angular.x、angular.y 同時 -99 → 急停，同時 +99 → request-active。sentinel 訊息不送入一般運動輸出。
- Joy／Twist 共用一個 active owner。第一個有效來源取得控制權；健康的 active source 不會因較高 priority 的普通訊息而被搶走。request-active 需嚴格較高 priority。手動 switch service 不比較 priority，但必須已註冊、支援該 type、目前狀態 ACTIVE，且有新鮮未 blocked 資料；interrupted 時拒絕切換。
- R1 priority 1–100 全部是一般優先級，100 沒有保留急停語意。R1 liveness 僅 INITIAL／ACTIVE／TIMEOUT／DISCONNECTED，不再使用 LOW_FREQ 或 UNKNOWN。
- active source 超時會選擇最高 priority 的可用 fallback，同 priority 先以 channel 字典順序決定，相同 channel 再比 type。**交出控制權前先 StopMove**，避免 neutral fallback 或持續按住 request-active 時殘留上一來源的非零運動。無 fallback 時清空 active，失聯只觸發一次急停。
- 普通訊號由收到的新訊息驅動；相同 Move 值與持續按住同一按鍵不重複輸出。切換／急停／失聯／重新註冊時清除轉換狀態，使同值重連能恢復。初次啟動的 idle zero 不產生多餘 Move。
- `timeout_ns` 與 `disconnect_timeout_ns` 來自 source descriptor；兩者都從最後活動時間計算，後者不是進入 TIMEOUT 後再等待的時間。0 分別代表停用；非零 disconnect 必須大於非零 timeout。沒有資料的新 registration 不會被自動選為輸出來源。
- Wireless interruption 與 R2／Twist sentinel 急停不同：前者全域鎖存、必須手動 reset；後者維持原先非鎖存行為。Reset 清掉復歸前候選與 converter，不移除 Manager 註冊，也不主動重播。

普通運動指令維持 **event-triggered**：只有新 Joy／Twist 進入 callback，且速度值改變或 Joy 按鍵出現 PRESS，才會發布；持續同值資料只保活，不按固定頻率重發 Move。Watchdog 定時檢查存活，只在失聯／換手時停止前任一次；R1 status／heartbeat 的定時流量不是機器人運動輸出。R2／Twist sentinel 仍每收到一封急停就執行 StopMove，wireless 則只在進入鎖存狀態時停止。

輸出狀態由本包 `sendCommand()` 記錄：command 使用原版 SportClient，成功發布後回傳同一份 Request，上層保存其實際 payload。`SportClientCmd` 的 C++ 回傳型別因此改為 `unitree_api::msg::Request`；自訂 command 需回傳已發布的 Request，現有 command 都只發布一次。這次 PR 不修改 `thirdparty`，SDK 不承擔 server 狀態紀錄。

預設 server parameters 位於 `config/control_server.yaml`：watchdog 50ms、manager status 100ms、CSM timeout 600ms、CSM disconnect 6000ms。Manager status 必須小於 CSM timeout 的一半；參數無效會在啟動時拒絕。

## 建置依賴

標準 ROS dependencies（包含 GUI 的 PyQt5、rclpy）由 rosdep 安裝。Server 仍宣告並以 `find_package(unitree_api REQUIRED)`／`find_package(unitree_go REQUIRED)` 使用獨立的 Unitree ROS package；colcon 在辨識到 server 的 `package.xml` 後不會繼續搜尋其內部套件，因此使用以下入口補足 discovery。

先 source ROS 及所需 underlay，從 workspace 根目錄執行：

```bash
# 預覽實際來源選擇與 colcon 命令，不編譯
python3 src/rv2_server_control/scripts/build_with_unitree.py --dry-run

# 自動選擇 Unitree API／Go sources，先建依賴再建 server
python3 src/rv2_server_control/scripts/build_with_unitree.py -- --symlink-install
```

入口對 `unitree_api`、`unitree_go` 各自先搜尋 `src`，其次使用已 source underlay；都沒有時才將對應的 `thirdparty/unitree/<package>` 加入 `--base-paths`。兩包可來自不同來源。預設 `--packages-up-to rv2_server_control` 保證來源依賴先完成，不自動建 `unitree_hg`。重複套件或 discovery 失敗會回報錯誤，不以 fallback 掩蓋。

來源位置不同時可重複傳入 `--source-root PATH`；例如在 project 根目錄，使用 `python3 ros2_ws/src/rv2_server_control/scripts/build_with_unitree.py --source-root ros2_ws/src`。`--` 之後可傳 colcon 的一般建置選項（如 `--cmake-args -DCMAKE_BUILD_TYPE=Release`），來源與 package selection 選項由入口管理。一般 `colcon build` 本身沒有新增 fallback 行為；要自行選任意套件時，明確指定其來源並直接使用 colcon。

其餘來源依賴須放在 server 的 sibling 位置，測試框架由 `test_depends.repos` 唯讀掛載：

| 路徑 | 來源／驗證基線 |
|---|---|
| `../r1_interfaces` | v0.1.3 已合併主線 `dc3e199`，與原 tag `34e5d1e` tree 相同；舊 v0.1.2 不包含新介面 |
| `../rv2_control_signal_transport` | R1 v0.2.0，已合併主線 `5a91d31` |
| `../rv2_csm_topic_bridge` | R1 v0.1.0，已合併主線 `78ba1a9` |
| `../joy_interpreter` | `test` 分支 `944306b61746dcdaa932a404994b8f829a2a9611` |
| `thirdparty/unitree/unitree_api`、`thirdparty/unitree/unitree_go` | 內附官方 `unitree_ros2` commit `5204e6e098ee53f4bd929bd77eb1d387cd0fa842` 的 API／Go sources |

不再需要外部 `../unitree_api` checkout 或 symlink。三份內附 Unitree manifest 都已補上其 CMake 所需的 `rosidl_generator_dds_idl` build dependency；來源、BSD 授權與本地修改見 [thirdparty/unitree/README.md](thirdparty/unitree/README.md)。`thirdparty` 內 README 保留匯入基線的說明，現行 server 所需 api/go 及建置選擇以本文件為準。JSON CMake package 是 `nlohmann_json`，對應 rosdep key 為 `nlohmann-json-dev`。

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

測試固定使用內附 API／Go，即使主機另有 Unitree underlay 亦不依賴它。完整 test_run 包含 unit、integration 與 ament 檢查。新 owner 尚未列入 framework sanitizer matrix，因此 ASan／UBSan／TSan 都顯示 N/A，不冒稱通過 sanitizer；框架的顯式 TSan 開關仍預設 off。lint 是獨立唯讀 Docker gate。可在 build/deps 後以 `test_run.sh -s unit` 或 `-s integration` 單獨執行分類；結果保存於 `test_env/jazzy/`。

- `test/unit/test_unitree_build.py`／`test/integration/test_unitree_discovery.py`：來源優先序、錯誤傳遞與真實 colcon nested discovery。
- `test/unit/test_control_server.cpp`：Joy 短陣列、Twist sentinel、converter per-channel／per-instance 去重、reset 同值恢復、button press edge。
- `test/integration/test_control_server.cpp`：R1 topic/service Joy/Twist、priority 100、request-active 優先權、超時 fallback 與單次急停、單封訊號、重新註冊、disabled timeout、manual selection、TypeConfig 更新、callback 執行中安全結束、跨型別唯一 owner、嚴格 switch、wireless latch／reset 與併發。
- `test/integration/test_joy_bridge.py`：真正 master／bridge／server processes，best-effort Joy→Unitree x/y/yaw 與按鍵、保持值去重、input silence 無重播、同值恢復、bridge crash/restart、server crash/restart 與 master 對帳。readiness 根據實際輸出與 R1 incarnation 判定。

新增 `test/integration/test_control_services.py` 驗證 production node 三服務、實際 wireless／Unitree topics、last-output metadata、手動切換與 interruption 復歸；另外跨多個 timer ticks 驗證 Joy／Twist 同值保活不重送、值改變單次輸出、輸入停止單次 StopMove 及同值新輸入恢復。Wireless 行為參考 develop `750938f7`，使用新的 R1 介面；不引入 legacy `ControlServerReq` 或 requester priority YAML。Package 版本、tag、PR 依 project `r1_todo.md` §1 在功能與驗證完成後獨立處理。

目前 framework v0.5.1 的 `test_packages.sh` 對內部 source path 與 sibling bind mount 有嚴格路徑比對限制；本次驗證涵蓋四步測試、lint 與 source build，未提供 Debian 打包支援。
