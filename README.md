# nodding_lidar

首振り式2D LiDARのスキャンを、時刻ごとのTFで変換して `/points` に出すROS 2 Jazzyパッケージです。
MIRS本体と首振りLiDARをまとめて起動する場合は、このパッケージの
`bringup_launch.py` を使います。RTAB-Mapを使う場合は
`enable_rtabmap:=true` を指定してください。この起動構成ではMIRSのEKFを有効にし、
`/odometry/local` をRTAB-Mapへ渡します。SLAM Toolboxとは同時起動しません。

## ノードとインターフェース

| ノード | 入力 | 出力 |
|---|---|---|
| `serial_angle_bridge` | UnoのUSBシリアル `timestamp_ms,angle_deg,angular_velocity_deg_s` | `/servo_state` (`sensor_msgs/msg/JointState`、positionはrad、velocityはrad/s) |
| `servo_tf_node` | `/servo_state` | `servo_link → laser` の動的TF |
| `scan_to_cloud` | `/scan`、角度・角速度を含む`/servo_state`、`odom → servo_link`等のTF | `/points` (`sensor_msgs/msg/PointCloud2`, RTAB-Map ICP入力向け) |

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
cd ~/ws
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

## 3D 地図の作成と保存

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
mkdir -p <workspace>/src/original/mirs/maps/3d
rtabmap-export --scan --voxel 0.05 \
  --output gazebo_3d.pcd \
  --output_dir <workspace>/src/original/mirs/maps/3d \
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

```bash
ros2 topic echo /servo_state
ros2 topic echo /scan --once
ros2 topic echo /points --once
ros2 run tf2_ros tf2_echo servo_link laser
```

FoxgloveまたはRVizではFixed Frameを `base_link` に設定し、`/points` を `PointCloud2` 表示として追加します。実機でのシリアル接続、回転方向、取り付け寸法、点群の形状は別途検証が必要です。
