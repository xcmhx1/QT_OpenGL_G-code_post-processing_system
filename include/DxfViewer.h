#pragma once

#include "CadViewer.h"

class DxfViewer : public CadViewer
{
public:
    explicit DxfViewer(QWidget* parent = nullptr)
        : CadViewer(parent)
    {
    }
};
