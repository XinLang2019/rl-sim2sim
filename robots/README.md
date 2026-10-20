# 机器人仿真资产

按 `wheel/`（轮足）和 `legged/`（点足）分类。每个描述包统一为：

```text
my_robot_description/
├── package.xml          # ROS 包名、来源信息与依赖
├── CMakeLists.txt       # 资源安装规则
├── urdf/                # 机器人描述文件（.urdf、.xacro）
│   ├── robot.urdf       # 几何、惯量、关节、碰撞体、effort transmission
│   └── gazebo.xacro     # 加载几何、通用插件，以及本机接触/材质参数
├── config/
│   └── controllers.yaml # 关节名称与控制器的绑定
└── meshes/              # URDF 实际引用的网格及关联纹理
```

保留 `_description` 包名是为了让 `package://.../meshes/...` 和 ROS 资源查找继续有效。几何文件只保留当前使用的模型；模型修订可通过 Git 管理，配置中的 `variant` 描述当前版本。

Gazebo Classic 的控制与 IMU 插件由 [`robot_simulation.xacro`](../rlsim/interfaces/gazebo/rlsim_gazebo/models/robot_simulation.xacro)统一提供。各机器人保留自己的接触参数。原独立 launch、RViz、ROS2 控制配置及未使用的传感器入口已移除；仿真启动文件由任务系统生成，网页相机独立于机器人描述。

`controllers.yaml` 只管理接口绑定。策略 PD、参考关节位置、限幅、输入输出和控制周期在 `configs/{wheel,legged}/<robot>.yaml` 中设置。

新增机器人时参考 [A1](legged/a1_description/)或 [Go2W](wheel/go2w_description/)，填写对应配置，再执行：

```bash
/usr/bin/python3 tools/rlsim.py assets --check --xacro /opt/ros/noetic/bin/xacro
```

检查会覆盖几何树、effort transmission、控制器、网格、Collada 纹理、xacro 展开和 Gazebo 引用。资产校验通过表示具备描述和接口条件；策略部署仍需完整契约及实际模型验证。包内保留原资产作者与许可证信息，尚未填写的来源字段应由维护者补齐。
