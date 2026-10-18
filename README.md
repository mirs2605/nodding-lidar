# nodding_lidar

サーボで首振りさせる2D LiDAR (RPLIDAR S1) から、FAST-LIO2に渡せる3D点群を作るパッケージ。

## 構成

| ノード | 役割 |
|---|---|
| `servo_node` | ±15度の首振りを生成して、PCA9685経由でサーボに指令する。`/servo_state` (JointState) と、`servo_link -> lidar_link` の動的TFを出す |
| `scan_to_cloud` | `/scan` とTFから、`time` と `ring` 付きの点群 `/points` を作る (数回転分をまとめて1フレーム) |

```
servo_node --TF(servo_link->lidar_link)--> scan_to_cloud --/points--> FAST-LIO2
sllidar_ros2 --/scan--------------------->
```

## ビルドと起動

```bash
cd ~/ros2_ws
colcon build --packages-select nodding_lidar
colcon test --packages-select nodding_lidar   # 計算部分の単体テスト (ROS不要)
source install/setup.bash

# サーボなしで確認 (角度とTFだけ出す)
ros2 launch nodding_lidar nodding_lidar.launch.py use_hardware:=false
# 実機
ros2 launch nodding_lidar nodding_lidar.launch.py
```

LiDAR (`sllidar_ros2`) は、既存の起動ファイルで起動する。その `frame_id` は `lidar_link` に合わせること。

## 座標系と角度の約束

- `servo_link`: X=前方, Y=左 (回転軸), Z=上
- 角度は、Y軸まわりの右手系 [rad]。**正の角度で、前方が下がる**
- LiDARのセンサ原点は、回転軸から `lidar_height` (0.04505 m) 上。首を振ると、位置も回転軸のまわりを円弧を描いて動くため、TFの平行移動も角度で変わる
  - 回転: `R_y(φ)`、平行移動: `R_y(φ)·(0, 0, lidar_height)`
- LiDARの0度方向は前方 (+X) と仮定している **【要確認:CADで確認する】**

## 制約 (必ず読むこと)

1. **角度は、エンコーダのない指令値ベースの推定。** 実際の角度ではない。サーボの遅れや不感帯の分だけ、誤差が出る。
2. **点ごとの時刻は、`time_increment` から求める。** 0の場合は `scan_time ÷ 点数` で代用する。実際の各点の測定時刻と一致するとは限らない。
3. **TFが届かないスキャンは、捨てる。** `tf_timeout` (0.1秒) 待ち、超えたら警告を出して捨てる。最新のTFでの代用はしない。角度ノードが止まった場合も、この動作になる。
4. **`/scan` の `header.stamp` の意味 (開始時刻か終了時刻か) と、配列の向きと時刻の進む向きの関係は、未確認。** 実機で確認し、`stamp_is_scan_end` と `reverse_time_order` で合わせる。
5. **FAST-LIO2移植版が要求するフィールド名・単位 (`time` が秒かミリ秒かなど) は、未確認。** 使う移植版の設定に合わせること。

## 実機での調整手順

1. `use_hardware:=false` で起動し、rviz か foxglove で、`lidar_link` が首を振る様子を見る
2. サーボをつなぎ、`servo_center_us` で、角度0が水平になるように合わせる
3. 正の角度で前方が下がるように、`servo_sign` を合わせる
4. 平らな壁に向けて、ロボットを止めたまま首を振り、`/points` を見る
   - 往路と復路の点が重なるように、`time_offset` (サーボの遅れ τ [秒]、正の値) を調整する
   - 壁がまっすぐになるように、`angle_scale` を調整する
5. `i2c_bus` と `i2c_address` を、`i2cdetect` の結果に合わせる
