# 模块边界

## 1. 职责划分

| 区域 | 负责 | 不负责 |
| --- | --- | --- |
| `desktop` / `ui` | 用户命令、窗口、面板、对话框、状态展示和消息 | 加工单元身份、顺序事实、几何和 NC 决策 |
| `cad` | 文档、图元、编辑、图层、选择、Viewer 和稳定图元身份 | 加工单元序列、加工计划、机床轨迹和 NC 语义 |
| `application` | 业务编排、文档数据捕获、用户加工状态、加工单元序列编辑、状态一致性和结果分发 | 连续关系算法和外部文本方言 |
| `core/geometry` | 精确源几何、统一加工路径和几何不变量 | UI、文件选择和外部格式读写 |
| `core/topology` | 路径连通、严格闭环和拓扑关系 | 加工单元序列所有权、方管工艺和 G-code |
| `core/machining` | 方管截面、内部路径和加工断面等工艺几何 | 加工顺序和文本输出 |
| `core/planning` | 根据连续关系形成加工单元，并生成顺序、方向、起点、排除项和工艺屏障 | 当前加工单元序列持久状态和机床运动文本 |
| `core/machine` | XYZ/A 运动、安全移动、连续连接和过切 | G-code 方言和文件写入 |
| `core/nc` | 与文本格式分离的 NC 语义 | 配置管理和文件读写 |
| `infrastructure` | DXF/DWG、配置、图像、G-code 文本和外部文件边界 | 加工顺序、工艺判断和轨迹决策 |
| `compatibility` | 旧对象与当前业务边界之间的转换 | 新业务规则和核心事实所有权 |

## 2. 依赖方向

主要业务依赖方向为：

```text
UI / CAD ──→ Application ──→ Core
                 ↕
          Infrastructure
                 ↕
              外部格式
```

Application 组织 Infrastructure 的边界适配。文件解析、配置读取和文本输出通过 Infrastructure 进入或离开生产数据流，Core 不依赖 Infrastructure。

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

Viewer 使用 CAD 文档和当前加工展示快照完成渲染、拾取、标签和方向箭头显示。图元选择通过稳定图元身份映射到加工单元选择，一个加工单元只显示一个顺序编号。加工单元身份和序列由 Core 与 Application 提供，Viewer 不自行生成或保存这些业务事实。GPU 缓存键和临时选择状态仅在 Viewer 生命周期内使用。

## 5. 外部格式边界

DXF/DWG 输入、配置文件和 G-code 文本属于外部表示。Infrastructure 负责在外部表示与内部业务对象之间转换：

```text
外部格式 ↔ Infrastructure ↔ Application → Core 业务对象
```

格式规则不进入几何、拓扑、规划或机床运动模块。加工决策也不由文件读写代码推断。

## 相关需求

MACH-042、MACH-043、PROCESS-018、PROCESS-020、PROCESS-022、EDIT-029、EDIT-033。
