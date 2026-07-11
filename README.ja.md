# map2sdf

[![CI](https://github.com/atinfinity/map2sdf/actions/workflows/ci.yml/badge.svg)](https://github.com/atinfinity/map2sdf/actions/workflows/ci.yml)

[English](README.md)

ROS 2 の占有格子地図（nav2 `map_server` 形式: YAML + PGM/PNG）から、
Gazebo (gz sim) 用の SDF world ファイルを生成する ROS 2 パッケージです。

- 動作確認環境: ROS 2 Jazzy / Gazebo Harmonic (gz sim 8)
- 追加の Python 依存なし（numpy / OpenCV / PyYAML のみ）

| 入力: 占有格子地図 | 出力: Gazebo world |
| :---: | :---: |
| ![サンプル占有格子地図](doc/sample_map.png) | ![生成された Gazebo world](doc/sample_world.png) |

## 仕組み

1. 地図 YAML と画像を読み込み、nav2 と同じしきい値処理で占有セルを二値化します。
2. 占有領域の輪郭を穴付きポリゴンとして抽出し（`cv2.findContours`）、
   必要に応じて Douglas-Peucker 法で簡略化（`--simplify`）した上で、
   セル境界にオフセットします（1 セル幅の壁も厚みを維持）。
3. ポリゴンを watertight な角柱（側壁＋三角形分割した上下面）として押し出し、
   1 つのバイナリ STL メッシュに書き出して、それを参照する SDF world を生成します。
   collision / visual が 1 組で済むため、大規模地図でも高速にロードできます。

地図 YAML の `origin: [x, y, yaw]` は壁モデルの pose に反映されるため、
生成された world の座標系は元の map フレームと一致します。

## ビルド

```bash
cd ~/dev_ws
colcon build --packages-select map2sdf
source install/setup.bash
```

## 使い方

```bash
ros2 run map2sdf map2sdf --map <map.yaml> -o <出力ディレクトリ> [オプション]
```

| オプション | 既定値 | 説明 |
| --- | --- | --- |
| `--map` | (必須) | 地図 YAML ファイルのパス |
| `-o, --out` | `.` | 出力ディレクトリ |
| `--wall-height` | `2.0` | 壁の高さ [m] |
| `--world-name` | `map_world` | world / モデル名（出力ファイル名にも使用） |
| `--format {world,model}` | `world` | 完全な world を出力するか、`<include>` 用の Gazebo モデルディレクトリを出力するか |
| `--no-ground` | - | 地面（ground plane）を追加しない |
| `--shadows` | - | 影を有効化（既定はレンダリング軽量化のため無効） |
| `--unknown-as {free,occupied}` | `free` | 未知セルの扱い |
| `--occupied-thresh` | YAML の値 | 占有しきい値の上書き |
| `--simplify TOL` | `0`（無効） | メッシュ化の前に壁の輪郭を TOL メートル以内で近似。ギザギザの SLAM 地図で三角形数を大幅に削減できます（TOL より細かい形状は消えることがあります） |

### 例: サンプル地図から生成して Gazebo で表示

```bash
ros2 run map2sdf map2sdf \
  --map $(ros2 pkg prefix map2sdf)/share/map2sdf/maps/sample.yaml \
  -o /tmp/map_world
gz sim -r /tmp/map_world/map_world.sdf
```

launch ファイル経由（ros_gz_sim を使用）:

```bash
ros2 launch map2sdf map2sdf_demo.launch.py world:=/tmp/map_world/map_world.sdf
```

### `/map` トピックから直接生成

`map2sdf_node` は `nav_msgs/OccupancyGrid` トピック（`map_server` や
SLAM ノードが publish するラッチドトピック）を購読し、CLI と同じ
ファイルを出力します。SLAM 実行中の地図からそのまま world を生成できます:

```bash
ros2 run map2sdf map2sdf_node --ros-args -p out:=/tmp/map_world \
  -p simplify:=0.05 -r map:=/your_map_topic
```

パラメータは CLI オプションに対応します（`out`, `world_name`,
`wall_height`, `simplify`, `ground`, `shadows`, `unknown_as`,
`occupied_thresh`）。`one_shot`（既定 `true`）の場合は最初の変換後に
終了します。`false` にすると地図が更新されるたびに再生成します。

### 既存の world に壁を追加する

`--format model` を指定すると、world の代わりに Gazebo モデル
ディレクトリ（`<出力先>/<名前>/model.config`, `model.sdf`, STL）を
出力します。既存の world に壁だけを差し込めます:

```xml
<include>
  <uri>file:///path/to/out/map_walls</uri>
</include>
```

地図の origin の pose はモデル側に埋め込まれているため、pose を
指定せずに include すれば元の map フレームと位置が一致します。

### 出力ファイル

出力ディレクトリに `map_world.sdf` と `map_world_walls.stl` が生成されます。
SDF からは world ファイル基準の相対パスでメッシュを参照するため、
2 ファイルを同じディレクトリに置いたまま移動すればそのまま動きます
（`GZ_SIM_RESOURCE_PATH` などの環境変数設定は不要です）。

## 対応している地図 YAML フィールド

`image`（YAML からの相対パス可）, `resolution`, `origin`, `negate`,
`occupied_thresh`, `free_thresh`, `mode`（`trinary` / `scale` / `raw`）

注意: nav2 のしきい値の仕様では、`free_thresh` が 0.25 の場合、慣例的な
未知領域のグレー（205）は「自由」に分類されます。グレーのピクセルを
未知として扱いたい場合は、従来の 0.196 を使用してください
（`--unknown-as` も参照）。

## テスト

```bash
colcon test --packages-select map2sdf && colcon test-result --verbose
```

## ライセンス

Apache License 2.0 — [LICENSE](LICENSE) を参照してください。
