// 声明 DxfViewer 兼容头，继续为 UI 自定义控件提供旧类名入口。
// 兼容适配模块，把历史界面文件中的 DxfViewer 映射到当前 CadViewer 实现。
#pragma once

#include "CadViewer.h"

using DxfViewer = CadViewer;
