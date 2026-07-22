# 四轴机床轨迹详细设计

## 职责

四轴轨迹生产链将有效 `Rotary4Axis ProcessPlan` 转换为按加工顺序排列的 XYZ/A 机床运动。
该链同时支持无方管截面的普通四轴和带有效截面的截面增强四轴。
轨迹层负责安全移动、连续连接、A 轴展开、加工面 Z 修正和闭合过切。

## 生产入口

- 四轴导出进入 `NcProgramService::buildRotaryProgram()`。
- 该服务调用 `MachineTrajectoryService::buildRotaryTrajectory()` 构建轨迹。
- 轨迹服务捕获当前文档几何，按计划分配重新编译每条生产路径。
- `RotaryTrajectoryBuilder` 调用 `RotaryKinematics` 生成机床姿态并组织安全与切削运动。
- 当前轨迹仅在导出过程中按需生成，主窗口不长期保存轨迹对象。

## 输入与输出

输入包括当前文档、加工状态、有效四轴计划、可选方管截面、四轴配置和可选显式截面中心。
计划提供顺序、方向、起点和连续组；几何快照提供不可变源几何。
输出 `MachineTrajectory`，包含旋转上下文和按实体组织的接近、切削、连接及过切运动。
输出继续传给 `NcProgramBuilder`，轨迹层不生成 G-code 文本。

## 主要数据类型

- `GeometrySourceSnapshot`：在文档线程捕获的精确源几何和稳定实体属性。
- `TrajectoryEntityInput`：按计划方向和起点编译后的单图元 `Path3D` 及连续组信息。
- `RotaryTrajectoryInput`：实体路径、计划分组、revision 和可选截面。
- `RotaryMachinePolicy`：旋转轴、截面中心、安全距离、Z 修正、过切和角度策略。
- `RotarySurfaceRegion`：轨迹构建期间使用的顶、右、底、左、圆角、径向或未知表面分类。
- `MachinePose4D`：单个 XYZ/A 机床姿态。
- `MachineMove`：快速、切削、连续切削连接或过切运动。
- `MachineTrajectory`：轨迹实体、revision 和旋转安全上下文。

## 当前生产数据流

普通四轴：

```text
Rotary4Axis ProcessPlan
+ CadDocument
+ 旋转轴和运动配置
→ GeometrySourceSnapshot
→ 按 assignment 重新编译 Path3D
→ RotaryKinematics
→ RotaryTrajectoryBuilder
→ MachineTrajectory
```

截面增强四轴：

```text
普通四轴输入
+ TubeSectionModel
+ 截面中心
→ 圆角匹配与表面方向
→ 截面边界上下文
→ 增强四轴 MachineTrajectory
```

中心选择：

```text
有效截面几何中心
→ 否则显式截面中心
→ 否则全部加工路径的 YZ 包围盒中心
```

## 状态所有权

`ProcessPlan` 由主窗口持有，`MachineTrajectory` 是导出调用中的临时值对象。
轨迹保存计划的文档 revision 和加工状态 revision，供 NC 阶段继续校验。
四轴配置由当前 `GProfile` 提供，构建时复制到 `RotaryMachinePolicy`。
方管截面由主窗口提供；业务状态按用户中心、自动中心、`(0,0)` 解析，并将有效中心同步到核心模型。
轨迹服务只读取可选核心模型和显式中心，不直接解析中心来源。

## 失效条件

- 计划模式不是 `Rotary4Axis` 时拒绝构建。
- 文档快照、计划、加工状态或截面 revision 不一致时拒绝构建。
- 计划未覆盖全部可加工且未排除的图元时拒绝构建。
- assignment 顺序不连续、源实体缺失或几何编译失败时拒绝构建。
- 旋转轴中心、截面中心或运动参数出现非有限值时拒绝构建。
- 旋转轴、安全距离、初始位置、Z 修正、过切或 A 轴策略变化需要重新生成轨迹及下游结果。

## 关键业务规则

- 每条路径使用计划确定的 `reverse` 和 `startParameter` 重新编译，轨迹不重新排序。
- 单图元闭合圆和完整椭圆的自动入口由规划器联合确定；轨迹服务使用 Assignment 中的最终起点参数和方向重新编译同一源几何，不再次选择入口。
- 多图元闭合加工单元由规划阶段提供经过简单环验证的成员顺序和逐成员方向；轨迹层按该顺序连续编译，不重新执行最近端点选择。
- 多图元闭环的计划结束位置是经过物理首尾连接验证的真实入口，异常闭环在计划发布前被拒绝，不会把错误结束位置传入后续轨迹。
- `centerY`、`centerZ` 始终作为旋转轴中心参与坐标变换。
- 方管截面中心与旋转轴中心保持独立；用户中心不会被后续自动识别静默覆盖。
- 无截面时普通四轴继续运行；非固定平面路径按旋转轴进行径向展开。
- 有效截面存在时，固定表面分类以 `TubeSectionModel` 的真实 YZ 边界为权威；整条路径全部点必须共同满足同一平面候选，并按最大距离、平均距离消除多候选歧义。
- 固定平面不再依赖 Y/Z 跨度判断的先后顺序。顶、右、底、左面分别使用 0、90、180、-90 度原始 A 角，整条路径在应用反向、偏移和连续展开前保持同一原始角度。
- 圆角识别继续使用截面圆角中心，但仅在整条路径属于同一圆角时生效；平面路径端点落在圆角交界容差内不会改变整段平面分类。
- 无截面时继续使用跨度兼容规则；Y/Z 同时恒定的 X 向路径改为按路径平均位置相对管中心的径向主方向分类，径向分量接近时返回未知并停止错误轨迹输出。
- 表面分类容差独立于连接容差，取数值误差、`1e-6 mm` 和截面最大尺寸 `1e-8` 倍中的最大值。
- A 轴偏移、反向和连续角度展开在运动学转换中统一应用。
- 加工面 Z 修正加入变换后的机床 Z 坐标。
- 首刀可从配置的初始机床位置进入；组间快速移动可先到安全 Z。
- 安全机床 Z 当前由截面中心、加工路径最大碰撞半径和离轴额外距离组成。
- 同一连续组内使用切削连接，组间使用快速移动。相邻源路径满足连接容差后，还要校验变换后的机床 XYZ 端点距离和展开后的 A 轴连续性；失败时返回前后实体及两种距离，不继续生成 NC。
- 闭合组先精确回到组起点，再按原方向执行可选过切；过切不超过一圈总长。
- 圆和完整椭圆采用新入口后，闭合回起点和过切继续沿重新编译后的同一方向执行。

## 相关源码

- `src/application/machine/MachineTrajectoryService.cpp`：校验计划并把文档几何和配置组装为轨迹输入。
- `src/core/machine/RotaryKinematics.cpp`：将 `Path3D` 转换为连续 XYZ/A 姿态。
- `src/core/machine/RotaryTrajectoryBuilder.cpp`：组织安全移动、连续切削、闭合和过切。
- `include/core/machine/MachineTrajectory.h`：定义四轴轨迹、运动和旋转上下文。
- `include/infrastructure/config/GProfile.h`：定义当前四轴运动配置来源。
- `src/application/nc/NcProgramService.cpp`：在四轴 NC 生产链中调用轨迹服务。

## 当前实现差异

| 事项 | 当前生产实现 | 需求或概要设计要求 | 影响 |
|---|---|---|---|
| 无截面默认中心 | 状态模型解析为 `(0,0)`；轨迹服务无截面和显式中心时仍使用路径 YZ 包围盒中心 | 截面中心缺失时默认 `(0,0)` | 本阶段保留的轨迹回退仍可能改变表面方向和安全位置 |
| 手动中心输入 | 业务状态和接口已支持用户中心，UI 未提供独立中心输入 | 用户可单独设置 `(Ycenter, Zcenter)` | 当前用户仍无法通过界面直接校正中心 |
| 碰撞半径 | 最大碰撞半径按加工路径点相对截面中心计算 | 截面增强安全计算应依赖真实工件外形 | 未加工到的外形极值可能未计入安全半径 |
| 配置版本 | 轨迹保存文档和加工状态 revision，没有独立配置 revision | 运动配置变化应使轨迹及下游失效 | 当前依靠同步导出和上层清计划保证，不具备独立版本证明 |
| 轨迹持久状态 | 轨迹仅在导出调用内临时生成 | 概要设计允许轨迹作为派生结果管理 | 当前没有可供 UI 单独检查或复用的轨迹生命周期 |

## 对应需求与概要设计

需求：[MACH-031～MACH-040](../requirements/machining.md)、[PROCESS-008～PROCESS-010、PROCESS-013～PROCESS-017](../requirements/machining-process.md)、[GCODE-015～GCODE-020](../requirements/gcode-and-export.md)。

概要设计：[数据流](../architecture/data-flow.md)、[领域模型](../architecture/domain-model.md)、[状态与生命周期](../architecture/state-and-lifecycle.md)。
