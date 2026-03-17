/******************************************************************************
**  dwg2dxf - Program to convert dwg/dxf to dxf(ascii & binary)              **
**                                                                           **
**  Copyright (C) 2015 José F. Soriano, rallazz@gmail.com                    **
**                                                                           **
**  This library is free software, licensed under the terms of the GNU       **
**  General Public License as published by the Free Software Foundation,     **
**  either version 2 of the License, or (at your option) any later version.  **
**  You should have received a copy of the GNU General Public License        **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.    **
******************************************************************************/

#ifndef DX_DATA_H
#define DX_DATA_H
#include "libdxfrw.h"

#include <list>

//存储图片数据还有图片路径
//class to store image data and path from DRW_ImageDef
class dx_ifaceImg : public DRW_Image {
public:
    dx_ifaceImg(){}
    dx_ifaceImg(const DRW_Image& p):DRW_Image(p){}
    ~dx_ifaceImg(){}
    std::string path; //stores the image path
};

//存储所有的实体
//container class to store entites.
class dx_ifaceBlock : public DRW_Block {
public:
    dx_ifaceBlock(){}
    dx_ifaceBlock(const DRW_Block& p):DRW_Block(p){}
    // 析构函数->遍历链表，并删除所有实体
    ~dx_ifaceBlock(){
        for (std::list<DRW_Entity*>::const_iterator it=ent.begin(); it!=ent.end(); ++it)
            delete *it;
    }
    // 使用链表存储所有的实体
    std::list<DRW_Entity*>ent; //stores the entities list
};

/**
 * @class dx_data
 * @brief 存储DXF/DWG文件的完整数据结构
 *
 * 该类聚合了DXF文件的所有核心数据模块，包括：
 * - 文件头元数据
 * - 线型、图层、标注样式等基础配置
 * - 视口与文本样式定义
 * - 块定义及内部实体集合
 * - 临时图像数据引用
 *
 * 主要用于解析后的数据存储与访问，提供完整的图形数据结构接口。
 */
//container class to store full dwg/dxf data.
//存储文件所有的数据
class dx_data {
public:
    dx_data(){
        
        mBlock = new dx_ifaceBlock();
    }
    ~dx_data(){
        //cleanup,
        for (std::list<dx_ifaceBlock*>::const_iterator it=blocks.begin(); it!=blocks.end(); ++it)
            delete *it;
        delete mBlock;
    }
    // 头部信息存储
    //< 文件头数据副本（版本、单位、编码等）
    DRW_Header headerC;                 //stores a copy of the header vars
    //< 所有线型定义集合
    std::list<DRW_LType>lineTypes;      //stores a copy of all line types
    //< 全局图层配置列表
    std::list<DRW_Layer>layers;         //stores a copy of all layers
    //< 标注样式定义列表
    std::list<DRW_Dimstyle>dimStyles;   //stores a copy of all dimension styles
    //< 视口配置集合
    std::list<DRW_Vport>VPorts;         //stores a copy of all vports
    //< 文本样式定义列表
    std::list<DRW_Textstyle>textStyles; //stores a copy of all text styles
    //< 应用标识符列表
    std::list<DRW_AppId>appIds;         //stores a copy of all line types

    // 实体存储系统
    //< 块定义及内部实体容器
    std::list<dx_ifaceBlock*>blocks;    //stores a copy of all blocks and the entities in it
    //< 临时图像数据（需关联DRW_ImageDef）
    std::list<dx_ifaceImg*>images;      //temporary list to find images for link with DRW_ImageDef. Do not delete it!!

    // 模型空间容器
    //< 主模型空间实体存储指针
    dx_ifaceBlock* mBlock;              //container to store model entities


};

#endif // DX_DATA_H
