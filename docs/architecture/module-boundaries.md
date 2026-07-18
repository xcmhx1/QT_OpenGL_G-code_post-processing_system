# 模块边界

## 1. 职责划分

| 区域 | 负责 | 不负责 |
| --- | --- | --- |
| `desktop` / `ui` | 用户命令、窗口、面板、对话框、状态展示和消息 | 几何、拓扑、加工规划和 NC 决策 |
| `cad` | 文档、图元、编辑、图层、选择、Viewer 和稳定图元身份 | 加工计划、机床轨迹和 NC 语义 |
| `application` | 业务编排、文档数据捕获、用户加工状态、状态一致性和结果分发 | 数学算法和外部文本方言 |
| `core/geometry` | 精确源几何、统一加工路径和几何不变量 | UI、文件选择和外部格式读写 |
| `core/topology` | 路径连通、严格闭环和拓扑关系 | 方管工艺、排序和 G-code |
| `core/machining` | 方管截面、内部路径和加工断面等工艺几何 | 加工顺序和文本输出 |
| `core/planning` | 加工顺序、方向、起点、连续组、排除项和工艺屏障 | 机床运动文本 |
| `core/machine` | XYZ/A 运动、安全移动、连续连接和过切 | G-code 方言和文件写入 |
| `core/nc` | 与文本格式分离的 NC 语义 | 配置管理和文件读写 |
| `infrastructure` | DXF/DWG、配置、图像、G-code 文本和外部文件边界 | 加工顺序、工艺判断和轨迹决策 |
| `compatibility` | 旧对象与当前业务边界之间的转换 | 新业务规则和核心事实所有权 |

## 2. 依赖方向

主要业务依赖方向为：

```text
UI / Desktop
      ↓
CAD 与 Application
      ↓
Core
```

Infrastructure 为 Application 和 Core 的外部边界提供适配。文件解析、配置读取和文本输出通过 Infrastructure 进入或离开生产数据流。

Compatibility 仅位于遗留对象转换边界。新业务事实进入 Core 后，不再由 Compatibility 决定其含义。

## 3. Core 边界

Core 的输入和输出使用独立值对象。Core 的依赖边界不包含：

- UI 或桌面窗口；
- CAD Viewer 和 CadItem；
- Compatibility；
- Qt Widgets；
- DRW 实体和外部文件对象。

CAD 与外部格式数据应在进入 Core 前完成捕获和适配。Core 计算结果由 Application 层校验版本后交给 CAD 或 UI 展示。

## 4. 显示与业务边界

Viewer 使用 CAD 文档和当前加工展示快照完成渲染、拾取、标签和方向箭头显示。稳定加工身份、用户加工状态和派生计划保持在业务链中，GPU 缓存键和临时选择状态仅在 Viewer 生命周期内使用。

## 5. 外部格式边界

DXF/DWG 输入、配置文件和 G-code 文本属于外部表示。Infrastructure 负责在外部表示与内部业务对象之间转换：

```text
外部格式
↔ Infrastructure
↔ Application / Core 业务对象
```

格式规则不进入几何、拓扑、规划或机床运动模块。加工决策也不由文件读写代码推断。

## 相关需求

SCOPE-001、SCOPE-002、SCOPE-003、FILE-001、FILE-002、FILE-005、FILE-006、PROCESS-017、GCODE-001、GCODE-005、GCODE-024、GCODE-025。
