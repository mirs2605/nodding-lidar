# nodding_lidar

首振り式2D LiDARのスキャンを、時刻ごとのTFで変換して `/points` に出すROS 2 Jazzyパッケージです。
MIRS本体と首振りLiDARをまとめて起動する場合は、このパッケージの
`nodding_bringup.launch.py` を使います。RTAB-Mapを使う場合は
`enable_rtabmap:=true` を指定してください。この起動構成ではMIRSのEKFを有効にし、
`/odometry/local` をRTAB-Mapへ渡します。SLAM Toolboxとは同時起動しません。

## ノードとインターフェース

| ノード | 入力 | 出力 |
|---|---|---|
| `serial_angle_bridge` | UnoのUSBシリアル `timestamp_ms,angle_deg` | `/servo_state` (`sensor_msgs/msg/JointState`、positionはrad) |
| `servo_tf_node` | `/servo_state` | `servo_link → laser` の動的TF |
| `scan_to_cloud` | `/scan` (`sensor_msgs/msg/LaserScan`) とTF | `/points` (`sensor_msgs/msg/PointCloud2`, RTAB-Map ICP入力向け) |

`serial_angle_bridge` は `/dev/ttyACM0`、115200 baudを既定値とします。Arduino時刻とPCのROS時刻は同期していないので、JointState/TFのstampにはPCでの受信時刻を使います。角度はサーボの指令位置からの推定値であり、エンコーダによる実測値ではありません。`center_angle_deg` と `angle_sign` で中央角度と回転方向を調整します。

## TFと取り付け位置

MIRSの既存フレーム名を使います。

```text
base_footprint → base_link → servo_link → laser
                              static        dynamic
```

`nodding_bringup.launch.py` は、首振りを有効にするとMIRSの `base_link → laser` 静的TFを停止します。代わりに `base_link → servo_link` を静的配信し、Uno角度から `servo_link → laser` を配信します。これにより同じTFを複数ノードから配信しません。首振りを無効にした場合はMIRS既定の静的TFを使います。

既定の `base_to_servo_z=0.12945 m` は、MIRS既存設定のLiDAR高さ `0.1745 m` と、以前のLiDARオフセット `lidar_height=0.04505 m` から計算した暫定値です。機械図面や実測で確認した値ではありません。回転軸の位置、サーボ中央角度、正方向、LiDARの軸からのオフセットは、実機に合わせて調整してください。起動引数 `base_to_servo_{x,y,z,roll,pitch,yaw}` とパラメータ `lidar_height`、`center_angle_deg`、`angle_sign` で設定できます。

`servo_node` は従来のPCA9685指令方式として残しています。通常のMIRS起動ではシリアル角度入力を使い、同じ角度やTFを二重配信しないようにします。PCA9685方式で単独起動する場合は `use_pca9685:=true` を指定してください。

## ビルドと起動

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws
colcon build --symlink-install --packages-select nodding_lidar mirs
source install/setup.bash

ros2 launch nodding_lidar nodding_bringup.launch.py \
  enable_nodding_lidar:=true \
  enable_rtabmap:=true \
  servo_angle_port:=/dev/ttyACM0
```

`nodding_bringup.launch.py` はMIRSの `mirs_launch.py` とこのパッケージの
`nodding_lidar.launch.py` をincludeします。MIRS側のLiDAR起動引数 `lidar_frame` と
サーボTFの `child_frame` は既定で `laser` です。
`enable_nodding_lidar:=false` にすると首振り関連ノードを起動せず、MIRS既定の静的LiDAR TFを使います。
`enable_lidar:=false` にするとLiDARドライバーも起動しません。

MIRSを起動せず首振り機能だけを起動する場合は、LiDARノードと親フレームTFを別途用意してください。

```bash
ros2 launch nodding_lidar nodding_lidar.launch.py \
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

- `laser_geometry::LaserProjection::transformLaserScanToPointCloud` を使い、各スキャンの点を固定フレーム (`fixed_frame`, 既定 `odom`) に時刻補正して投影します。その後、蓄積開始時刻の `base_link` に戻すため、複数スキャンの点群は同じロボット姿勢を基準に揃います。`odom -> base_link` とサーボTFの両方が測定時刻をカバーする必要があります。
- 範囲外、NaNなどの無効な距離値は `laser_geometry` により点群から除外されます。
- `time_increment` が0の場合は `scan_time`（なければ `default_scan_time`）から点間時間を推定します。この推定値は実測点時刻とは限りません。
- `/scan` のstampが終了時刻なら `stamp_is_scan_end` を有効にし、配列方向と時間方向が逆なら `reverse_time_order` を有効にしてください。S1ドライバがどちらを使うか、実機またはドライバ仕様で確認してください。
- 必要な時刻のTFが `tf_timeout` 内に得られないスキャンは警告を出して破棄します。最新TFで置き換えません。
- `frames_per_cloud` スキャンを蓄積してからpublishします。出力点群は蓄積開始時刻の `base_link` 座標です。RTAB-Mapの `icp_odometry` / `rtabmap` で `scan_cloud` として使う場合、両ノードの `frame_id` を `base_link` に合わせます。蓄積時間の分だけ点群出力が遅れるため、Nav2の即時衝突回避には現在の `/scan` を引き続き使ってください。
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
