# nodding_lidar

実機とシミュレーションで共通化する走行中の点群補償・蓄積・自己位置推定の計画は、
[走行中の点群生成・自己位置推定設計](docs/MOTION_LOCALIZATION_DESIGN.md) を参照してください。
共通の点群蓄積と診断は実装済みです。機器時刻の同期・サーボ実測角の取得と、実機の精度評価は別途必要です。

首振り2D LiDARのスキャンを、時刻ごとのTFで変換して `/points` に出すROS 2 Jazzyパッケージです。
MIRS本体と首振り2D LiDARをまとめて起動する場合は、このパッケージの
`bringup_launch.py` を使います。RTAB-Mapを使う場合は
`enable_rtabmap:=true` を指定してください。この起動構成ではMIRSのEKFを有効にし、
`/odometry/local` をRTAB-Mapへ渡します。SLAM Toolboxとは同時起動しません。

## ノードとインターフェース

| ノード | 入力 | 出力 |
|---|---|---|
| `serial_angle_bridge` | UnoのUSBシリアル `timestamp_ms,angle_deg,angular_velocity_deg_s` | `/servo_state` (`sensor_msgs/msg/JointState`、positionはrad、velocityはrad/s) |
| `servo_tf_node` | `/servo_state` | `servo_link → laser` の動的TF |
| `scan_to_cloud` | `/scan`、角度・角速度を含む`/servo_state`、`odom → servo_link`等のTF | `/points` (`sensor_msgs/msg/PointCloud2`, スキャンごとの運動補償済み点群) |
| `motion_cloud_assembler` | `/points`、`/odometry/local`、計測時刻のTF | `/points_matching` (`PointCloud2`)、`/nodding_lidar/motion_cloud/diagnostics` (`DiagnosticArray`) |

`serial_angle_bridge` は `/dev/ttyACM0`、115200 baudを既定値とします。Arduino時刻とPCのROS時刻は同期していないので、JointState/TFのstampにはPCでの受信時刻を使います。Arduinoは正弦波の指令角度と解析的な角速度を送ります。角度はサーボの指令位置からの推定値であり、エンコーダによる実測値ではありません。`center_angle_deg` と `angle_sign` で中央角度と回転方向を調整します。

## TFと取り付け位置

MIRSの既存フレーム名を使います。

```text
base_footprint → base_link → servo_link → laser
                              static        dynamic
```

`bringup_launch.py` は、首振りを有効にするとMIRSの `base_link → laser` 静的TFを停止します。代わりに `base_link → servo_link` を静的配信し、Uno角度から `servo_link → laser` を配信します。これにより同じTFを複数ノードから配信しません。首振りを無効にした場合はMIRS既定の静的TFを使います。

既定の `base_to_servo_z=0.142 m` は、MIRS既存設定のLiDAR高さ `0.1745 m` から、回転軸より32.5 mm上にあるセンサ原点を差し引いた暫定値です。LiDAR高さ0.1745 mが実際のセンサ原点高さと一致するかは別途確認してください。回転軸の位置、サーボ中央角度、正方向、LiDARの軸からのオフセットは、実機に合わせて調整してください。起動引数 `base_to_servo_{x,y,z,roll,pitch,yaw}` とパラメータ `lidar_height`、`center_angle_deg`、`angle_sign` で設定できます。

`servo_node` は従来のPCA9685指令方式として残しています。通常のMIRS起動ではシリアル角度入力を使い、同じ角度やTFを二重配信しないようにします。PCA9685方式で単独起動する場合は `use_pca9685:=true` を指定してください。

## ビルドと起動

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-select nodding_lidar mirs
source install/setup.bash

ros2 launch nodding_lidar bringup_launch.py \
  enable_nodding_lidar:=true \
  enable_rtabmap:=true \
  servo_angle_port:=/dev/ttyACM0
```

`slam_launch.py` / `bringup_launch.py` / `nav_launch.py` は実機専用です。
MIRSのmicro-ROS AgentとEKFを起動し、実機の `/odom` とセンサーを使用します。
ファームウェアのROS Domain IDは `90` なので、実機用launchは次の環境で起動します。

```bash
export ROS_DOMAIN_ID=90
ros2 launch nodding_lidar slam_launch.py
```

Gazeboではこのlaunchを使わず、`simulation` パッケージのlaunchを使います。
Gazebo側はROS Domain ID `91` を使うため、実機用AgentやEKFとTF・センサーtopicが混線しません。

## ESP32なしで起動する

LiDARとサーボ角度送信用Unoだけを接続する構成です。
`standalone_slam_launch.py` はMIRSのlaunchを読み込まず、micro-ROS Agent、
車輪オドメトリ用EKF、Foxglove、Nav2を起動しません。

ROS 2環境のワークスペース直下で依存を導入してビルドします。

```bash
rosdep install --from-paths src/original/nodding_lidar --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-select nodding_lidar
source install/setup.bash
ros2 launch nodding_lidar standalone_slam_launch.py \
  lidar_port:=/dev/ttyUSB0 \
  servo_angle_port:=/dev/ttyACM0
```

ROS Domain IDの指定は不要ですが、確認用ターミナルとは同じ値にしてください。
MIRS側のSLAMやEKFと同じDomainで同時に起動しないでください。
既定でRTAB-Map VizとRViz2の両方を表示します。
個別に停止する場合は `enable_rtabmap_viz:=false` または
`enable_rviz:=false` を指定します。画面なしの場合は両方をfalseにします。
DBの既定保存先は `~/.ros/nodding_standalone.db` です。
`database_path` で変更でき、既存DBは自動削除しません。

```text
/scan + /servo_state → /points → icp_odometry → /odometry/lidar
                         └────────────────────────┬─────────┘
                                               RTAB-Map
map → odom → base_link → servo_link → laser
```

`odom → base_link` はICPオドメトリ、`map → odom` はRTAB-Mapが配信します。
点群生成の `fixed_frame` は `base_link` に固定しています。
これにより最初の点群を生成する前にオドメトリが必要になる循環を避けます。
サーボ角度は補正しますが、スキャン中の車体移動による歪みは補正しません。
最初は静止状態で点群とオドメトリの出力を確認し、その後ゆっくり動かしてください。
1スキャンは薄い点群であり、首振りによる重なり不足や平坦な環境では
ICPが追跡を失う可能性があります。実機での精度・安定性は未検証です。

```bash
ros2 topic hz /points
ros2 topic hz /odometry/lidar
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

SLAMなしで点群だけ表示する場合は既存の単体launchを使えます。
この構成もESP32を必要としません。

```bash
ros2 launch nodding_lidar lidar_launch.py \
  servo_serial_port:=/dev/ttyACM0 fixed_frame:=base_link
```

ICPオドメトリの接続構成は
[RTAB-Map公式LiDAR例](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/lidar3d.launch.py)
を参照しています。

## 3D 地図の作成と保存

`slam_launch.py` は固定の `base_link → laser` TFを無効にし、
`base_link → servo_link → laser` のみを使用します。EKF用の
`base_link → imu_link` TFも有効にします。TFの全体は次の構成です。

```text
map → odom → base_footprint → base_link → servo_link → laser
 RTAB-Map   EKF                 └→ imu_link
```

`config/rtabmap.yaml` は `slam_launch.py` と
`bringup_launch.py enable_rtabmap:=true` の共通設定です。
`/points_matching` と `/odometry/local` を同期して読み込み、ICPと空間的に近い
過去のスキャンによるループ閉じ込みを使います。カメラは不要です。
`motion_cloud_assembler` は各スキャンを計測時刻のTFでodomへ変換し、
時間・旋回量・点数の上限内で重ねます。出力は最新スキャンの基準時刻のbase_link座標です。
直進時は最大2 s、旋回時は絶対角速度の積分が0.2 rad以内になるように古い点を除きます。
左右切返しの角度を相殺せず、オドメトリ履歴が欠ける場合は単発点群だけを残します。
首振りの全範囲を含むとは限らないため、形状不足による照合失敗は別途評価が必要です。


`slam_launch.py` はRTAB-Mapの3Dモードで起動し、RVizに `/cloud_map` と
現在の `/points` を表示します。RTAB-MapのセッションDBは既定で
`~/.ros/mirs_3d.db` に保存されます。別の地図を新規に作るときは、異なる
`database_path` を指定してください。

```bash
ros2 launch nodding_lidar slam_launch.py \
  servo_angle_port:=/dev/ttyACM0 \
  database_path:=$HOME/.ros/mirs_3d.db
```

地図作成を止めてDBが閉じられた後、点群地図をPCDとして書き出せます。
`<workspace>` はこのリポジトリのワークスペースのパスに置き換えます。

```bash
mkdir -p <workspace>/maps/real/3d
rtabmap-export --scan --voxel 0.05 \
  --output gazebo_3d.pcd \
  --output_dir <workspace>/maps/real/3d \
  $HOME/.ros/mirs_3d.db
```

`gazebo.yaml` と `gazebo.pgm` は既存の2D占有地図として残し、3D点群は
`maps/3d/gazebo_3d.pcd` に分けて保存します。RTAB-Map DBは姿勢グラフと
センサーデータを含む編集・再処理用の元データ、PCDは統合済み3D点群です。

`bringup_launch.py` はMIRSの `mirs_launch.py` とこのパッケージの
`lidar_launch.py` をincludeします。MIRS側のLiDAR起動引数 `lidar_frame` と
サーボTFの `child_frame` は既定で `laser` です。
`enable_nodding_lidar:=false` にすると首振り関連ノードを起動せず、MIRS既定の静的LiDAR TFを使います。
`enable_lidar:=false` にするとLiDARドライバーも起動しません。

MIRSを起動せず首振り機能だけを起動する場合は、LiDARノードと親フレームTFを別途用意してください。

```bash
ros2 launch nodding_lidar lidar_launch.py \
  enable_nodding_lidar:=true \
  servo_serial_port:=/dev/ttyACM0
```

シリアルブリッジには、リポジトリ内の `micon/servo_angle` のUnoファームウェアが出力するCSV形式を使います。ファームウェアは次のコマンドでビルド・書き込みできます。

```bash
cd micon/servo_angle
pio run -e uno
pio run -e uno --target upload
```

ファームウェアのサーボ最小・最大角度とピンは、実機に取り付ける前に安全を確認してください。

## 点群生成と時刻の制約

- `/servo_state` の角度と角速度から隣接サンプル間を3次Hermite補間し、各LaserScanビームの時刻でLiDAR姿勢を計算します。各点はその時刻の `fixed_frame`（既定 `odom`）へ変換し、蓄積開始時刻の `base_link` に戻します。`odom -> servo_link` など親フレームまでのTFと、サーボ状態サンプルの両方が測定時刻をカバーする必要があります。
- 範囲外、NaNなどの無効な距離値は点群から除外されます。
- `time_increment` が0の場合は `scan_time`（なければ `default_scan_time`）から点間時間を推定します。この推定値は実測点時刻とは限りません。
- S1ドライバの `LaserScan` は点ごとの実測時刻を含まず、`time_increment` をスキャン時間と点数から作ります。角度補償で点が並べ替えられるため、各点に割り当てる時刻とサーボ角度は推定値です。実機で壁などの形状を確認してください。
- `/scan` のstampが終了時刻なら `stamp_is_scan_end` を有効にし、配列方向と時間方向が逆なら `reverse_time_order` を有効にしてください。使用するLiDARドライバがどちらを使うか、実機またはドライバ仕様で確認してください。
- 必要な時刻のTFが `tf_timeout` 内に得られないスキャンは警告を出して破棄します。最新TFで置き換えません。
- 1回の `/scan` から1個の `/points` をpublishします。各ビームの測定時刻に対応するサーボ角度と車体TFを使い、スキャン開始時刻の `base_link` 座標に揃えます。S1が10 Hzでスキャンを出せば、欠測やTF待ちがない場合の点群出力も10 Hzです。1個の点群が首振りの全範囲を含むわけではありません。RTAB-Mapで `scan_cloud` として使う場合、`frame_id` を `base_link` に合わせます。
- `/points` はRTAB-MapのICP用PointCloud2です。FAST-LIO2固有の `ring` や点ごとのtimestamp形式を出力するものではありません。
- 現在のRTAB-Map統合はMIRSのEKFオドメトリをICPの初期推定として使う構成です。RTAB-MapとEKFのTF競合を避けるため、RTAB-Map側はロボット姿勢TFを直接配信せず、SLAM補正の `map → odom` を配信します。

## 確認

SLAM起動後、別ターミナルでもワークスペースをsourceし、
`export ROS_DOMAIN_ID=90` を設定してから確認します。

```bash
ros2 topic hz /points
ros2 topic hz /odometry/local
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /info --once
```

RVizのFixed Frameは `map` にし、`/cloud_map` が走行に合わせて広がることを
確認します。点群が出ない場合は角度入力と `odom → base_link` のTF、
地図が出ない場合は `/odometry/local` とRTAB-Mapの同期・ICP警告を確認します。
同じ場所に戻ったときの地図の重なりとループ閉じ込みは実機で確認してください。
`/cloud_map` の表示だけでは、ループ閉じ込みの成功を確認したことにはなりません。


```bash
ros2 topic echo /servo_state
ros2 topic echo /scan --once
ros2 topic echo /points --once
ros2 run tf2_ros tf2_echo servo_link laser
```

FoxgloveまたはRVizではFixed Frameを `base_link` に設定し、`/points` を `PointCloud2` 表示として追加します。実機でのシリアル接続、回転方向、取り付け寸法、点群の形状は別途検証が必要です。

## 旋回量に応じた照合用点群

`slam_launch.py` と `bringup_launch.py enable_rtabmap:=true` は共通の
`matching_cloud_launch.py` を起動します。LiDARだけで移動を推定する
`standalone_slam_launch.py` と2D地図で走行する `nav_launch.py` は変更対象外です。

`motion_cloud_config:=/absolute/path/config.yaml` で設定を差し替えられます。
既定は `config/motion_cloud.yaml`。実機とシミュレーションで同じ実行ファイル・設定を使い、
topicとシミュレーション時刻の有無だけを起動側で指定します。
実機の取付位置・点時刻とGazeboの同時刻計測の違いは、入力アダプタで扱います。

| 設定 | 既定値 | 意味 |
|---|---|---|
| `max_window_s` | 2.0 s | 点群の最初と最後の基準時刻の差の上限 |
| `max_turn_span_rad` | 0.2 rad | 局所オドメトリの絶対yaw角速度の積分上限 |
| `max_motion_gap_s` | 0.15 s | 角速度を補間できるサンプル間隔の上限 |
| `max_input_age_s` | 1.0 s | 到着した点群・オドメトリの許容経過時間 |
| `tf_wait_s` | 0.3 s | 計測時刻のTF・角速度履歴の到着待ち上限（実時間） |
| `max_output_rate_hz` | 2.0 Hz | 計測時刻で制限する点群出力頻度の上限 |
| `voxel_size_m` | 0.1 m | odom座標で点をまとめる格子幅。0で無効 |
| `max_scans` | 32 | 保持スキャン数上限。1なら単発点群の比較構成 |
| `max_points` | 200000 | 保持点数と単発入力点数の上限 |
| `min_points` | 100 | 格子でまとめた後の出力点数の下限 |
| `max_pending_scans` | 8 | TF待ちの入力数上限。超過時は古い入力を破棄 |

これらは調整開始用の値です。角度・時間の制限はスキャン間の蓄積に適用し、
単発スキャン内の角度変化・点時刻の推定誤差を制限するものではありません。
角速度はEKFのtwistを車体座標に変換して使います。別のIMU積分は行いません。
設定値は起動時に読み、実行中の変更は反映しません。

入力はlittle-endianのFLOAT32 x・y・zを持つ、行末パディングのないPointCloud2です。
NaN・Infの点を除き、出力にはx・y・z [m]だけを含めます。intensityなどの追加fieldは引き継ぎません。
pointの時刻ごとの補償は上流の `scan_to_cloud` が担当し、蓄積ノードは各点群のstampで変換します。
TFが得られない場合は古いTFや最新TFで代用せず、待ち期限を超えると破棄します。
角速度履歴が欠ける場合は蓄積を単発点群へ縮め、最低点数がなければ照合用点群を出しません。
時計が戻ると、点群・角速度履歴・出力周期の状態を初期化します。

診断は1 Hzで、蓄積幅、旋回量、保持スキャン数・点数、入力棄却数、オドメトリ無効数、
出力の経過時間・遅延 [s]と直近の処理時間（実時間）[s]を配信します。旋回量が-1 radの場合は角速度履歴不足です。
出力停止はERROR、単発への退避はWARNです。診断だけで車体の停止を指令しません。
TFが出続けることを自己位置推定の成功と判断せず、実機の自律運用には別途停止・復帰条件が必要です。

```bash
ros2 topic echo /nodding_lidar/motion_cloud/diagnostics --once
ros2 topic hz /points_matching
```

車輪・IMU・TFと単発点群を記録したbagから、同じ起動定義で再生評価できます。
実機のドライバ・EKFを同時起動せず、bagの時刻と静的TFが再生されることを確認してください。

```bash
ros2 launch nodding_lidar matching_cloud_launch.py use_sim_time:=true
```


### 実機launchの担当と同時起動

実機用の入口launchはROS_DOMAIN_IDを90に設定します。Gazeboのsimulationは91を使います。
同じ実機に対する`bringup_launch.py`、`nav_launch.py`、`slam_launch.py`、`standalone_slam_launch.py`は、用途に応じて一つだけ起動してください。
`standalone_slam_launch.py`はLiDAR照合がodomのTFを配信するため、車輪用EKFを別に追加しないでください。

`nav_launch.py`では、mirsがEKF・IMU取付TF・Nav2・安全ゲートを担当し、lidar_launchがLiDARドライバと首振りTFを担当します。
固定のbase_link→laserは配信せず、base_link→servo_link→laserを使います。
各includeの引数はGroupActionの範囲に閉じ、ドライバ停止指定やRViz設定を隣のincludeに引き継ぎません。
`bringup_launch.py`で`enable_rtabmap`と`enable_nav2`を同時にtrueにすると、AMCLとRTAB-Mapがmap→odomを二重配信するため起動を拒否します。
