# xarm_utils_cpp

xArm6 + MoveIt2 (ROS2 Humble)  を扱うためのC++ユーティリティ
MoveGroupInterfaceを簡単に扱うためのラッパー


## 出来ること

- C++からxArm6のMoveIt2操作
- パイプライン動的切替（OMPL/STOMPなど）
- 現在の関節値・エンドエフェクタ姿勢取得（fakeでは未動作）
- 関節目標値の設定・計画・実行（基本的な動作）
- サンプル(exampleファイル)で動作確認可能

---

## 動作環境・依存

- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **MoveIt2**: `ros-humble-moveit` 

## インストール方法
```bash
cd ~/<your workspace>/src
git clone https://github.com/your-username/xarm_utils_cpp.git
cd ~/<your workspace>
colcon build --packages-select xarm_utils_cpp
source install/setup.bash
```

## 関数リスト・API
| 関数名                                              | 概要                                   | 引数例・備考                                  |
| :----------------------------------------------- | :----------------------------------- | :-------------------------------------- |
| `static void setup_xarm_moveit(node)`            | `/move_group`からURDF/SRDFを取得してノードにセット | `node`: `std::shared_ptr<rclcpp::Node>` |
| `void set_planning_pipeline(pipeline_name)`      | プランナーパイプラインを動的に切り替える                 | `"stomp"`, `"ompl"` など                  |
| `bool set_joint_value_target(joint_values)`      | 関節角度リストを目標としてセット                     | `std::vector<double>`                   |
| `bool plan()`                                    | 計画実行                                 | 戻り値: 成功で`true`                          |
| `bool execute()`                                 | 実際にロボットを動かす（fake環境でもOK）              | 戻り値: 成功で`true`                          |
| `std::vector<double> get_current_joint_values()` | 現在の関節角度を取得                           |                                         |
| `geometry_msgs::msg::Pose get_current_pose()`    | 現在のエンドエフェクタの姿勢を取得                    |                                         |


## 使い方サンプル
src/example_xarm_utils.cpp より抜粋：
```cpp
#include <rclcpp/rclcpp.hpp>
#include "xarm_utils_cpp/xarm_utils.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("example_xarm_utils");

    // MoveItの初期セットアップ（URDF/SRDF取得など）
    XArmUtils::setup_xarm_moveit(node); // 必須

    // xarm6グループでMoveGroupインタフェース作成
    XArmUtils xarm(node, "xarm6");
    xarm.set_planning_pipeline("stomp"); // stomp or omplの指定が可能
    xarm.set_goal_joint_tolerance(0.01); // toleranceの設定が可能

    std::vector<double> target = {0.916, 0.724, -1.700, 0.001, 0.977, -0.67}; // 任意のジョイント角を指定
    if (xarm.set_joint_value_target(target) && xarm.plan())
    {
        xarm.execute();
    }

    rclcpp::shutdown();
    return 0;
}
```
## 注意・既知の課題
MoveIt2とxArm6 MoveGroup構成（SRDF等）は事前にセットアップしておくこと

実機で使う場合は、ロボット側のコントローラ/トピック設定に注意

今後Pythonバインディング対応も検討中


