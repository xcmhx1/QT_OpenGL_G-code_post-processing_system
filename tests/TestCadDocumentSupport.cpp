#include "CadDocument.h"

#include "CadItem.h"
#include "dx_data.h"

CadDocument::CadDocument(QObject* parent)
    : QObject(parent)
    , m_data(std::make_unique<dx_data>())
{
}

CadDocument::~CadDocument() = default;

