# field_world

Gazebo Classic + ROS 2 Humble 用のテストフィールドと、orne-box を **自己位置推定なし(オドメトリのみ)** で
Nav2 走行させるための launch 一式です。

- フィールド: a〜d の4エリアに青Box(エリア名ラベル付き・エリア内ランダム配置)、中央に緑Box(ラベルは毎回ランダム)
- ロボット: [orne-box](https://github.com/open-rdc/orne-box)(`humble/white_line_check` ブランチ)
  + MID360(低い位置の障害物検出用)+ Webカメラ
- ナビゲーション: AMCL / emcl2 / map_server を使わず、`map → odom` を固定 TF にした Nav2

> 動作確認について: ファイルの構文チェックまでは行っていますが、Gazebo / ROS 上での起動は
> 利用者側で確認した範囲(コストに囲まれる問題の修正まで)です。カメラ画像の取得などは未確認の部分があります。

## 1. フィールド

上から見た配置(左から d, c, 緑Box, b, a)。`tools/render_topview.py` で図を作れます。
寸法と間隔は、つくばチャレンジ2026 の配置図(delivery_area_2026.jpg)の 10m スケールバーから読み取った概算です。

```
 Area d   Area c                [緑]                Area b   Area a
┌───────┐┌───────┐             (0,0)              ┌───────┐ ┌───────┐
│  ■d   ││  ■c   │ <---- 11m ---> | <---- 11m ---> │   ■b  │ │   ■a  │
└───────┘└───────┘                                 └───────┘ └───────┘
 13.2m × 8.5m       d-c 間 1.5m          b-a 間 2.1m      x: 約 -39 〜 +40 m
```

| 項目 | 内容 |
|---|---|
| エリア | a〜d、各 13.2m × 8.5m。中心の y = 2.4m(緑Boxはエリア下端から約1.9m内側) |
| 緑Boxとの間隔 | 緑Boxの端から内側のエリア(c, b)の端まで **11m**(`GREEN_GAP`) |
| エリア間の間隔 | d-c 間 1.5m(`AREA_GAP_CD`)、b-a 間 2.1m(`AREA_GAP_BA`) |
| 宅配ボックス(青) | 各エリアに1個。エリア内ランダムな位置・向き。**上面開放の折りたたみコンテナ**(下記)。短辺の側面(-x 側の端面)に白ラベル「宅配ボックス + エリア名」 |
| 集荷ボックス(緑) | 中央 (0, 0)。**蓋付きの折りたたみコンテナ**。上面と長辺の側面(+y 側 = ロボットの出発側)に「配達先 + a〜d(起動ごとにランダム)」、短辺の側面(-x 側)に「集荷ボックス」のラベル |
| 床 | 見た目のみ(コリジョンなし)。LiDAR には映らない |
| 正解データ | `/tmp/field_random.answer.yaml`(緑Boxのラベルと各宅配ボックスの座標) |

**コンテナの寸法**は 50L 相当の想定値(長辺 0.53m × 短辺 0.37m × 高さ 0.31m)です。写真の縦横比から推定したもので、
実物の寸法が分かれば `generate_world.py` の `BOX_L/BOX_W/BOX_H` を直してください。
青コンテナは底と4枚の壁(厚み 15mm)でできており、上面は開いています。

寸法・配置は `field_world/generate_world.py` 上部の定数(`AREA_SIZE`, `GREEN_GAP`, `AREA_GAP` など)で変更できます。

## 2. 依存関係

### orne-box のセットアップ

```bash
cd ~/ros2_ws/src
git clone https://github.com/open-rdc/orne-box.git -b humble/white_line_check
wstool init
wstool merge orne-box/orne_box3_simulation_pkgs.install
wstool up
```

> `velodyne_simulator`(VLP-16 のモデル)などはこの wstool 手順で入ります。

### apt パッケージ

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization ros-humble-pointcloud-to-laserscan \
  ros-humble-laser-filters ros-humble-xacro \
  ros-humble-joint-state-publisher ros-humble-rviz2 \
  ros-humble-image-transport-plugins ros-humble-rqt-image-view \
  python3-pil python3-opencv ros-humble-cv-bridge
```

## 3. ビルド

```bash
cd ~/ros2_ws/src
tar xzf field_world.tar.gz            # または field_world を src に配置
cd ~/ros2_ws
colcon build --packages-select field_world
source install/setup.bash
```

ラベルのテクスチャ(PNG)とマテリアルはパッケージに同梱済みです。作り直す場合のみ
`python3 tools/make_textures.py`(Pillow と日本語フォント Noto Sans CJK が必要)を実行してください。

## 4. 起動

### 4-1. フィールドだけ(Gazebo のみ)

```bash
ros2 launch field_world field.launch.py                  # 毎回ランダム
ros2 launch field_world field.launch.py seed:=42         # 再現可能
ros2 launch field_world field.launch.py green_label:=c   # 緑Boxのラベルを固定
```

### 4-2. フィールド + orne-box + Nav2(自己位置推定なし)

```bash
ros2 launch field_world sim_nav.launch.py
```

| 引数 | 既定値 | 説明 |
|---|---|---|
| `seed` | (空) | 乱数シード。空なら毎回ランダム |
| `green_label` | (空) | 緑Boxのラベルを a/b/c/d で固定 |
| `world_out` | `/tmp/field_random.world` | 生成するワールドの出力先 |
| `x`, `y`, `yaw` | `0.0`, `3.0`, `-1.5708` | ロボットの初期位置(Gazebo のワールド座標)。緑Boxの上側(+y)3m から、緑Boxの方(-y 向き)を向いて出発 |
| `rviz` | `true` | RViz を起動するか |
| `camera_x/y/z` | `0.40`, `0.0`, `0.29` | カメラ位置(`base_link` 基準 [m]) |
| `camera_pitch` | `0.0` | カメラの上下角 [rad] |

起動される主なもの:

| 役割 | ノード / 出力 |
|---|---|
| Gazebo + ワールド | `gazebo_ros` の `gazebo.launch.py` |
| ロボットモデル | `urdf/orne_box_sim.urdf.xacro`(orne-box のモデル + MID360 + カメラ) |
| odom → base_link | `robot_localization` の EKF(orne-box の設定をそのまま使用) |
| map → odom | static TF(出発位置で固定 = 自己位置補正なし。map = ワールド座標) |
| 障害物センサ | MID360 点群 → `pointcloud_to_laserscan` → `laser_filters`(自機除去)→ `/low_scan` |
| Nav2 | 標準の `navigation_launch.py`(`config/nav2_params_odom_only.yaml`) |

### 4-3. ゴールを送る

RViz の「Nav2 Goal」か、コマンドで送ります。

```bash
# 緑Box (0,0) の向こう側 (0, -3) へ。エリアは x=±11m より外側にある
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: -3.0}, orientation: {w: 1.0}}}}"
```

`map → odom` は出発位置(`x`, `y`, `yaw`)で固定しているので、**`map` 座標は Gazebo のワールド座標と一致**します
(`/tmp/field_random.answer.yaml` の座標をそのままゴールに使えます)。出発位置は既知として固定するだけで、自己位置の補正はしません。

## 5. 自己位置推定なしでの走行について

- Nav2 に必要なのは地図や AMCL ではなく、`map → odom → base_link` の TF が繋がっていることです。
  `navigation_launch.py`(map_server / AMCL を含まない)を使い、`map → odom` を出発位置で固定した static TF にしています。
- global_costmap は静的地図を使わず、`odom` 基準の rolling window(100m × 50m)です。フィールドが約80m幅あるため大きく取っています。
- シミュレーションのオドメトリはほぼ理想値なので、補正なしでもゴールに到達できます。
  実機ではホイールのスリップ等でドリフトするため、長距離ではずれます。
  補正なしのベースラインとして使い、ノイズやスリップを加えて比較する使い方を想定しています。

### orne-box 側から使うもの / 使わないもの

| 使う | 使わない |
|---|---|
| `orne_box_description`(xacro)<br>`orne_box_bringup/.../robot_localization_ekf.launch.py`(+ `ekf.yaml`)<br>`nav2_params.yaml`(`tools/make_nav2_params.py` で加工して使用) | emcl2、map_server、keepout マスク、collision_monitor、joy/teleop、waypoint_manager、実機用 bringup |

`config/nav2_params_odom_only.yaml` は orne-box の `nav2_params.yaml` から自動生成したものです。
orne-box 側のパラメータを更新した場合は作り直せます。

```bash
python3 tools/make_nav2_params.py \
  ~/ros2_ws/src/orne-box/orne_box_navigation_executor/config/params/nav2_params.yaml \
  config/nav2_params_odom_only.yaml
```

主な変更点: global_costmap を odom 基準の rolling window にして static_layer を削除 /
emcl2・keepout・collision_monitor の設定を削除 / BT ファイルの絶対パス(`/home/takumi/...`)をパッケージ参照に置換。

## 6. センサ

### MID360(低い位置の障害物検出)

- 取り付け位置は orne-box の URDF 既定(`livox_frame`): `base_link` から (0.424, 0, 0.203)、下向き。地面から約0.2m。
- 水平360°、垂直 -7°〜+52°、10Hz、最大40m。点群は `/livox/lidar`。
- 点群は `pointcloud_to_laserscan` で LaserScan(前方180°、高さ 0.05〜0.60m)にし、
  `config/low_scan_filter.yaml` で自機の点(`base_link` 基準 x:-0.35〜0.65, y:±0.35)を除いて `/low_scan` に出します。
  Nav2 の costmap はこの `/low_scan` を使います(実機と同じトピック名)。
- 高さの下限を 0.05m にしているのは、`base_link` が地面の高さにあり、0 以下だと床を障害物として拾うためです。

### VLP-16

orne-box の既定どおり `base_link` から (-0.12, 0, 0.60) に残してあり、点群は `/surestar_points` に出ます(研究用データ取得用。Nav2 には使っていません)。

### Webカメラ

- 1280×720、30fps、水平画角 約70°、軽い画像ノイズあり。
- 既定位置は前面の棚板の上(MID360 の真上)、`base_link` から (0.40, 0, 0.29)、地面から約0.29m。
- 後方マスト上に付ける場合: `ros2 launch field_world sim_nav.launch.py camera_x:=-0.12 camera_z:=0.90`
- トピック: `/camera/image_raw`、`/camera/camera_info`。光学フレームは `camera_optical_frame`。

## 7. カメラで写真・動画を撮る

```bash
# 写真
python3 tools/camera_capture.py photo -o shot.jpg
python3 tools/camera_capture.py photo -o shots/ --count 10 --interval 2     # 2秒ごとに10枚

# 動画
python3 tools/camera_capture.py video -o run.mp4 --duration 30              # 30秒録画
python3 tools/camera_capture.py video -o run.mp4                            # Ctrl+C で終了

# 画像を見るだけ(右上の保存ボタンで1枚保存も可)
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

- Gazebo が実時間より遅いと、動画が速く再生されます。`ros2 topic hz /camera/image_raw` で実際の fps を確認し、`--fps` に指定してください。
- 走行ごと記録する場合: `ros2 bag record /camera/image_raw/compressed /low_scan /odometry/filtered /tf /tf_static`
  (生画像は容量が大きいので `compressed` を使用)

## 8. 付属ツール

| ファイル | 用途 |
|---|---|
| `tools/render_topview.py` | 上から見たフィールド図(PNG)を生成。`--seed`, `--green` に対応 |
| `tools/make_textures.py` | コンテナの色・ラベル(宅配ボックス/配達先/集荷ボックス)のテクスチャと OGRE マテリアルを再生成 |
| `tools/make_nav2_params.py` | orne-box の `nav2_params.yaml` から自己位置推定なし用の設定を生成 |
| `tools/camera_capture.py` | カメラ画像から写真・動画を保存 |
| `tools/preview_render.py` | Gazebo なしでコンテナ・ラベル・出発位置のカメラ視点を簡易レンダリング(numpy + Pillow)。`objects` / `robot` の2モード。Gazebo(OGRE)の描画ではないので、見た目の目安として使う |
| `field_world/generate_world.py` | ワールド生成(単体実行も可: `python3 -m field_world.generate_world -o out.world --seed 1`) |

## 9. トラブルシューティング

| 症状 | 確認すること |
|---|---|
| ロボットの周りがコストで囲まれて動けない | RViz で `/low_scan` を表示し、自機の周りや床に点が出ていないか確認。出ている場合は `config/low_scan_filter.yaml` の範囲を広げる / `pointcloud_to_laserscan` の `min_height` を上げる |
| 箱が costmap に出ない | `/livox/lidar` が出ているか(`ros2 topic hz /livox/lidar`)。MID360 はコンテナ(0.31m)の上端より低い位置にあるので水平方向の光線は当たるはず。TF(`base_link` ↔ `livox_frame`)も確認 |
| ラベルが白い / 表示されない / 文字が左右反転している | `GAZEBO_MODEL_PATH` に `field_world/share/.../models` が入っているか。launch は自動で設定する。文字の向きは Gazebo の box の面ごとの UV に依存するため、反転する面があれば知らせてほしい(未確認) |
| `box_filter_box3.yaml` が見つからない | このブランチには入っていない。`field_world` は `config/low_scan_filter.yaml` を使うので、orne-box 側の `pointcloud_to_laserscan.launch.py` は使わない |
| Nav2 が起動しない(BT の読み込みエラー) | `default_nav_to_pose_bt_xml` のパス。`config/nav2_params_odom_only.yaml` を `tools/make_nav2_params.py` で再生成する |
| 箱が近づくと消える / 衝突する | コンテナの高さは0.31mと低い。LiDAR の取り付け高さ・`min_height`/`max_height` を見直す(VLP-16 の `/surestar_points` は箱の上を通る) |
| ゴールに着いても向きがずれる | オドメトリのドリフト。実機相当の評価をするならノイズを加えて比較する |

## 10. ディレクトリ構成

```
field_world/
├── config/
│   ├── nav2_params_odom_only.yaml   # Nav2 設定(自己位置推定なし)
│   └── low_scan_filter.yaml         # 自機の点を除く laser_filters 設定
├── field_world/generate_world.py    # ワールド生成(青Box配置・緑Boxラベルのランダム化)
├── launch/
│   ├── field.launch.py              # フィールドだけ
│   └── sim_nav.launch.py            # フィールド + ロボット + Nav2
├── models/label_materials/          # ラベルのテクスチャ・マテリアル
├── tools/                           # 付属ツール(第8章)
└── urdf/orne_box_sim.urdf.xacro     # orne-box + MID360 センサ + Webカメラ
```
