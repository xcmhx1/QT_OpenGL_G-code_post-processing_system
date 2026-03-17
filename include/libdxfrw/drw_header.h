/******************************************************************************
**  libDXFrw - Library to read/write DXF files (ascii & binary)              **
**                                                                           **
**  Copyright (C) 2016-2022 A. Stebich (librecad@mail.lordofbikes.de)        **
**  Copyright (C) 2011-2015 José F. Soriano, rallazz@gmail.com               **
**                                                                           **
**  This library is free software, licensed under the terms of the GNU       **
**  General Public License as published by the Free Software Foundation,     **
**  either version 2 of the License, or (at your option) any later version.  **
**  You should have received a copy of the GNU General Public License        **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.    **
******************************************************************************/

#ifndef DRW_HEADER_H
#define DRW_HEADER_H

#include <memory>
#include <unordered_map>
#include "drw_base.h"

class dxfReader;
class dxfWriter;
class dwgBuffer;

#define SETHDRFRIENDS  friend class dxfRW; \
                       friend class dwgReader;

//! 用于处理 DXF 文件头（Header）条目的类
/*!
 *  该类用于管理 DXF 文件头变量。
 *  读取时：遍历 "std::unordered_map vars" 成员。
 *  写入时：向 "std::unordered_map vars" 中添加 DRW_Variant* 指针。
 *  注意：不需要手动删除指针，析构函数会自动清理 vars 映射表。
 *  也可以使用 add* 系列辅助函数进行添加。
 *  @作者 Rallaz
 */
class DRW_Header {
    SETHDRFRIENDS
public:
    DRW_Header();
    ~DRW_Header() {
        clearVars(); // 释放所有变体对象内存
    }

    // 单位枚举定义
    enum Units {
        /** $INSUNITS 变量，自 ACAD2000/AC1015 版本起引入，定义插入时的缩放单位 */
        None = 0,               ///< 无单位（继承父级单位）
        Inch = 1,               ///< 英寸 (25.4 mm)
        Foot = 2,               ///< 英尺 (12 英寸 = 0.3048 m)
        Mile = 3,               ///< 英里 (1760 码 = 1609 m)
        Millimeter = 4,         ///< 毫米 (0.001 m)
        Centimeter = 5,         ///< 厘米 (0.01 m)
        Meter = 6,              ///< 米
        Kilometer = 7,          ///< 公里 (1000 m)
        Microinch = 8,          ///< 微英寸 (0.000001 英寸 = 25.4 纳米)
        Mil = 9,                ///< 密耳 (0.001 英寸 = 0.0254 mm)
        Yard = 10,              ///< 码 (3 英尺 = 0.9144 m)
        Angstrom = 11,          ///< 埃 (10^-10 m)
        Nanometer = 12,         ///< 纳米 (10^-9 m)
        Micron = 13,            ///< 微米 (10^-6 m)
        Decimeter = 14,         ///< 分米 (0.1 m)
        Decameter = 15,         ///< 十米 (10 m)
        Hectometer = 16,        ///< 百米 (100 m)
        Gigameter = 17,         ///< 十亿米 (10^9 m)
        Astro = 18,             ///< 天文单位 (~149.6 x 10^9 m)
        Lightyear = 19,         ///< 光年 (~9.46 x 10^15 m)
        Parsec = 20,            ///< 秒差距 (~3.0857 x 10^16 m)
        UnitCount = 21,         ///< 用于遍历单位的计数标识

        /** $MEASUREMENT 变量，自 R14/AC1014 版本起引入，定义测量系统 */
        English = 0,            ///< 英制 (Imperial)
        Metric = 1,             ///< 公制 (Metric)
    };

    // 拷贝构造函数：实现 vars 映射表的深拷贝
    DRW_Header(const DRW_Header& h) {
        this->version = h.version;
        this->comments = h.comments;
        for (auto it = h.vars.begin(); it != h.vars.end(); ++it) {
            // 为每个变量创建新的副本
            this->vars[it->first] = new DRW_Variant(*(it->second));
        }
        this->curr = NULL;
    }

    // 赋值运算符重载：安全释放旧内存并深拷贝新数据
    DRW_Header& operator=(const DRW_Header& h) {
        if (this != &h) {
            clearVars(); // 清理当前对象的变量
            this->version = h.version;
            this->comments = h.comments;
            for (auto it = h.vars.begin(); it != h.vars.end(); ++it) {
                this->vars[it->first] = new DRW_Variant(*(it->second));
            }
        }
        return *this;
    }

    // 添加变量的辅助函数
    void addDouble(std::string key, double value, int code); // 添加双精度浮点数
    void addInt(std::string key, int value, int code);       // 添加整数
    void addStr(std::string key, std::string value, int code); // 添加字符串
    void addCoord(std::string key, DRW_Coord value, int code); // 添加坐标点

    std::string getComments() const { return comments; } // 获取文件注释
    void write(const std::unique_ptr<dxfWriter>& writer, DRW::Version ver); // 写入 DXF
    void addComment(std::string c); // 添加注释

protected:
    // 解析代码：处理来自 DXF 读取器的组码
    bool parseCode(int code, const std::unique_ptr<dxfReader>& reader);
    // 解析 DWG：处理来自 DWG 缓冲区的二进制数据
    bool parseDwg(DRW::Version version, dwgBuffer* buf, dwgBuffer* hBbuf, duint8 mv = 0);

private:
    // 内部获取变量值的辅助函数
    bool getDouble(std::string key, double* varDouble);
    bool getInt(std::string key, int* varInt);
    bool getStr(std::string key, std::string* varStr);
    bool getCoord(std::string key, DRW_Coord* varStr);

    // 清空变量映射表并释放动态内存
    void clearVars() {
        for (auto it = vars.begin(); it != vars.end(); ++it)
            delete it->second;
        vars.clear();
    }

public:
    // 核心存储结构：变量名到变体对象的映射表
    std::unordered_map<std::string, DRW_Variant*> vars;

private:
    std::string comments;      // 文件头注释
    std::string name;          // 头部名称
    DRW_Variant* curr{ nullptr }; // 当前正在处理的变量指针
    int version;               // 读取时记录的 DXF 版本

    // 以下为各类 DXF 表的控制柄（Handles/Control IDs），用于维护对象之间的引用关系
    duint32 linetypeCtrl;      // 线型表控制柄
    duint32 layerCtrl;         // 图层表控制柄
    duint32 styleCtrl;         // 文字样式表控制柄
    duint32 dimstyleCtrl;      // 标注样式表控制柄
    duint32 appidCtrl;         // 应用程序 ID 表控制柄
    duint32 blockCtrl;         // 块记录表控制柄
    duint32 viewCtrl;          // 视图表控制柄
    duint32 ucsCtrl;           // 用户坐标系表控制柄
    duint32 vportCtrl;         // 视口表控制柄
    duint32 vpEntHeaderCtrl;   // 视口实体头控制柄

    // 根据单位计算测量系统（英制或公制）
    int measurement(const int unit);
};

#endif

// EOF

