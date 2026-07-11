# 装甲板检测教程 —— 从像素到目标

> 📘 **运行上下文**：装甲板检测在 Armor 线程中作为视觉流水线第一步执行，帧数据来自 Camera 线程通过 SPSCQueue 传递，详见 [[MAS_Vision_多线程架构学习指南]]

## 目录

1. [概述](#1-概述)
2. [数据结构](#2-数据结构)
3. [检测流水线全景](#3-检测流水线全景)
4. [图像预处理](#4-图像预处理)
5. [灯条提取](#5-灯条提取)
6. [颜色验证](#6-颜色验证)
7. [装甲板配对](#7-装甲板配对)
8. [角点亚像素优化：梯度搜索算法](#8-角点亚像素优化梯度搜索算法)
9. [数字识别：ONNX 推理](#9-数字识别onnx-推理)
10. [配置参数完整速查](#10-配置参数完整速查)
11. [调试与可视化](#11-调试与可视化)

---

## 1. 概述

![哨兵自瞄流水线](https://raw.githubusercontent.com/tianyuecao/SJTU-RM-CV-2019/master/%E8%87%AA%E7%9E%84%E6%B5%81%E7%A8%8B%E5%9B%BE.png)
*▲ RoboMaster 自瞄处理流程图 — 从图像采集到云台控制 (图片来源: SJTU-RM-CV)*

### 1.1 装甲板检测的目标

给定一帧 1440×1080 的 BGR 图像，在 **16ms 内**（60fps）找到所有敌方装甲板，输出：
- 4 个角点的精确亚像素坐标
- 装甲板类型（大/小）
- 识别出的数字（1-5, sentry, outpost, base）
- 分类置信度

### 1.2 为什么传统 CV 而不是深度学习检测

| 方案 | 优点 | 缺点 |
|------|------|------|
| YOLO/深度学习检测器 | 鲁棒、泛化好 | Jetson 上 > 10ms，可能丢帧 |
| 传统 CV（本项目） | **< 3ms**，确定性强 | 需要调参，光照敏感 |

在哨兵的 Jetson Orin 上，每帧总预算约 16ms（60fps），检测占 3ms，留下 13ms 给位姿解算、跟踪、弹道和通信。传统 CV 流水线在这个约束下是最优选择。

### 1.3 核心思路

装甲板由**两个 LED 灯条**组成。LED 灯条在图像中呈现为**细长的亮矩形**。因此检测策略是：
1. 找所有亮的细长条（灯条候选）
2. 配对灯条（它们之间的几何关系对应真实装甲板）
3. 识别中间的数字确认目标身份
4. 优化角点坐标到亚像素精度

---

## 2. 数据结构

### 2.1 LightBar（灯条）

```cpp
struct LightBar {
    cv::RotatedRect rotated_rect;  // 旋转矩形（minAreaRect 拟合）
    cv::Point2f center, top, bottom;  // 中心、上端点、下端点
    double angle;       // 灯条倾斜角度 (rad)
    double angle_error; // 与垂直方向的偏差 (rad)
    double length;      // 长轴长度 (像素)
    double width;       // 短轴长度 (像素)
    double ratio;       // 长宽比 = length / width
    EnemyColor color;   // RED / BLUE
    std::vector<cv::Point2f> points; // [top, bottom]
};
```

**灯条端点计算**（构造函数）：
```cpp
// 1. 获取旋转矩形的 4 个顶点，按 y 坐标排序
// 2. top    = 上方两顶点的中点
// 3. bottom = 下方两顶点的中点
// 4. top2bottom = bottom - top（主轴方向向量）
// 5. angle = atan2(top2bottom.y, top2bottom.x)
// 6. angle_error = |angle - π/2|（偏离垂直的程度）
```

### 2.2 Armor（装甲板）

```cpp
struct Armor {
    LightBar left, right;             // 左右灯条
    std::vector<cv::Point2f> points;  // 4个角点: 左上→右上→右下→左下
    cv::Point2f center;               // 装甲板中心
    ArmorType type;      // BIG (ratio>3.0) / SMALL
    std::string number;  // "1"~"5", "sentry", "outpost", "base", "negative"
    float confidence;    // 分类置信度
    double ratio;        // 宽高比 = 宽度/灯条长度
    double side_ratio;   // 左右灯条长度比
    double rectangular_error; // 灯条与水平线的垂直度误差
    
    // 位姿解算填充（后续步骤）
    Eigen::Vector3d xyz_in_gimbal, xyz_in_world;
    Eigen::Vector3d ypr_in_gimbal, ypr_in_world, ypd_in_world;
};
```

**装甲板构造逻辑**：
```cpp
// 按 x 坐标排序确定左右灯条
// center = (left.center + right.center) / 2
// points = [left.top, right.top, right.bottom, left.bottom]
//   ↑ 这个顺序与 solvePnP 的 3D 模型点顺序严格对应！
// ratio = ||right.center - left.center|| / max(left.length, right.length)
//   ratio > 3.0 → BIG (大装甲板，宽度远大于灯条长度)
//   ratio ≤ 3.0 → SMALL (小装甲板)
```

### 2.3 3D 模型点

```cpp
// 装甲板坐标系 3D 点（单位：米）
// 原点 = 装甲板几何中心，X 轴垂直板面向外
BIG_ARMOR_POINTS = {
    {0,  0.115,  0.028},   // 左上
    {0, -0.115,  0.028},   // 右上
    {0, -0.115, -0.028},   // 右下
    {0,  0.115, -0.028}    // 左下
};

SMALL_ARMOR_POINTS = {
    {0,  0.0675,  0.028},  // 左上
    {0, -0.0675,  0.028},  // 右上
    {0, -0.0675, -0.028},  // 右下
    {0,  0.0675, -0.028}   // 左下
};

// 物理尺寸常量
LIGHTBAR_LENGTH   = 0.056 m  (56 mm — 灯条 LED 长度)
BIG_ARMOR_WIDTH   = 0.230 m  (230 mm — 大装甲板宽度)
SMALL_ARMOR_WIDTH = 0.135 m  (135 mm — 小装甲板宽度)
```

---

## 3. 检测流水线全景

```
输入: BGR 图像 (1440×1080)
  │
  ├─ Step 1: 灰度化  BGR → Gray
  │
  ├─ Step 2: 二值化  cv::threshold(gray, binary, 90, 255, THRESH_BINARY)
  │     └─ 阈值 = 90: 提取亮区域（LED 灯条在相机中是高亮的）
  │
  ├─ Step 3: findLights()
  │     ├─ cv::findContours(RETR_EXTERNAL) → 所有外轮廓
  │     ├─ 面积过滤 → 丢弃太大/太小的轮廓
  │     ├─ cv::minAreaRect → 旋转矩形拟合
  │     ├─ 几何过滤 (长宽比、长度、角度偏差)
  │     ├─ 颜色验证 (沿灯条轴采样 R-B 或 B-R)
  │     └─ 按 center.x 升序排序
  │
  ├─ Step 4: findArmors()
  │     ├─ 嵌套循环遍历所有灯条对 (left, right)
  │     ├─ 颜色一致性检查
  │     ├─ 共享灯条检测 (containLight)
  │     ├─ 几何过滤 (宽高比、灯条长度比、垂直度)
  │     ├─ 类型分类 (BIG / SMALL)
  │     ├─ ONNX 数字识别 → 过滤无效分类
  │     └─ LightCornerCorrector → 亚像素角点优化
  │
  └─ 输出: std::vector<Armor>
```

---

## 4. 图像预处理

### 4.1 灰度化

```cpp
cv::cvtColor(frame, gray_, cv::COLOR_BGR2GRAY);
```

直接使用 BGR → Gray 的标准加权转换。LED 灯条在任意颜色通道中都是亮的，单通道灰度图足够。

### 4.2 固定阈值二值化

```cpp
cv::threshold(gray_, binary_, binary_thres_, 255, cv::THRESH_BINARY);
// binary_thres_ = 90（默认值，可在 YAML 中调整）
```

**为什么用固定阈值而不是自适应阈值？**
- LED 灯条相对于背景非常亮（通常灰度值 > 200）
- 固定阈值 90 足够低，能捕获所有灯条，同时过滤掉大部分暗区域
- 自适应阈值（如大津法 OTSU）在场景明暗变化时阈值会漂移，导致检测不稳定

**阈值调参指南**：
- 阈值太低（< 60）：大量噪声 → 检测变慢，误检增多
- 阈值太高（> 140）：灯条断裂 → 漏检
- 推荐范围：80–120

---

## 5. 灯条提取

### 5.1 轮廓检测

```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(binary_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
```

**参数解析**：
- `RETR_EXTERNAL`：只检测最外层轮廓（灯条内部可能有孔洞，但孔洞轮廓不需要）
- `CHAIN_APPROX_SIMPLE`：压缩水平/垂直/对角线方向上的冗余点，节省内存

### 5.2 初步过滤

```cpp
for (auto &contour : contours) {
    if (contour.size() < 4) continue;  // 至少 4 个点才能拟合最小外接矩形
    
    double area = cv::contourArea(contour);
    if (area > max_lightbar_area_) continue;  // 面积 > 30000 → 可能是大光斑/反射
    
    cv::RotatedRect rrect = cv::minAreaRect(contour);
    LightBar light(rrect);
    // ... 后续几何过滤
}
```

### 5.3 几何过滤

```cpp
// 1. 角度过滤：灯条应该近似竖直
if (light.angle_error > max_angle_error_)  // max_angle_error_ = 45° → 57.3 → 约 0.785 rad
    continue;  // 偏斜超过 45° 的亮条不是灯条

// 2. 长宽比过滤：灯条应该是细长的
if (light.ratio < min_lightbar_ratio_)     // < 1.5 → 太"方"，不是细长条
    continue;
if (light.ratio > max_lightbar_ratio_)     // > 20 → 太细，可能是噪点线
    continue;

// 3. 长度过滤：灯条不能太短
if (light.length < min_lightbar_length_)   // < 8 像素 → 太远/太小
    continue;
```

**每个过滤条件的设计原理**：

| 条件 | 物理含义 | 反例 |
|------|---------|------|
| `angle_error < 45°` | 灯条竖直安装 | 地面反光、横向线缆 |
| `ratio ∈ [1.5, 20]` | 细长矩形 | 方形光斑、圆形灯具 |
| `length > 8px` | 足够近/足够大 | 远处噪点 |

---

## 6. 颜色验证

### 6.1 算法

沿灯条轴线从 `top` 到 `bottom` 均匀采样 10 个点，采样点通过双线性插值获取像素值：

```cpp
// 红色目标：累加 (R - B) 差值
if (detect_color == RED) {
    diff_sum += (pixel[2] - pixel[0]);  // BGR: [B=0, G=1, R=2]
}
// 蓝色目标：累加 (B - R) 差值
else if (detect_color == BLUE) {
    diff_sum += (pixel[0] - pixel[2]);
}

if (diff_sum <= 0) reject;  // 颜色特征不符
```

### 6.2 设计原理

这是一种**轻量级的颜色确认**：
- 不对全图做颜色分割（太慢）
- 只对已通过几何筛选的候选灯条做颜色验证（< 10 个候选）
- 纯白灯条（R≈B）的 diff_sum ≈ 0，不会通过颜色过滤
- 这种方法比 HSV 颜色空间变换 + 阈值分割更快（无需全图转换）

### 6.3 调参

颜色验证没有单独的阈值参数。如果发现特定光照下灯条被错误拒绝：
- 检查曝光时间是否合适（灯条应该过曝——这是故意的，保证灯条在二值化后是完整的大白块）
- 考虑调节 `binary_thres` 而不是改变颜色验证逻辑

---

## 7. 装甲板配对

### 7.1 配对循环

```cpp
for (size_t i = 0; i < lights.size(); i++) {
    for (size_t j = i + 1; j < lights.size(); j++) {
        LightBar *left  = &lights[i];   // 按 x 排序后，i < j → left.x < right.x
        LightBar *right = &lights[j];
        
        // 1. 颜色必须匹配
        if (left->color != right->color) continue;
        
        // 2. 共享灯条检测
        if (containLight(left, right, lights)) continue;
        
        // 3. 构建 Armor
        Armor armor(*left, *right);
        
        // 4. 几何过滤
        // ...
    }
}
```

### 7.2 共享灯条检测（containLight）

这个函数防止同一个灯条被用于多个装甲板：

```cpp
bool containLight(LightBar *left, LightBar *right, vector<LightBar*> &lights) {
    // 构造包含左右灯条所有顶点的边界框
    // 检查是否有第三个灯条落在这个边界框内
    for (auto *light : lights) {
        if (light == left || light == right) continue;
        
        // 如果中间灯条像"数字" → 跳过（不应阻止配对）
        if (light->width > 2 * avg_width) continue;     // 太宽 → 可能是数字
        if (light->length < 0.5 * avg_length) continue;  // 太短 → 可能是数字
        
        if (pointInRect(light->center, bounding_rect))
            return true;  // 有共享灯条 → 拒绝配对
    }
    return false;
}
```

**为什么需要这个检查？**

在复杂场景中，可能有多个 LED 灯条。如果三个灯条共线，可能产生两个重叠的装甲板候选。containLight 检测保证了每个灯条至多属于一个装甲板候选。

### 7.3 几何过滤

```cpp
// 1. 宽高比过滤
if (armor.ratio < min_armor_ratio_ || armor.ratio > max_armor_ratio_)
    continue;  // min=1, max=5: 装甲板宽度与灯条长度的比值
    
// 2. 灯条长度比过滤
if (armor.side_ratio > max_side_ratio_)  // > 1.5
    continue;  // 左右灯条长度差异太大 → 可能不是一对
    
// 3. 垂直度过滤
if (armor.rectangular_error > max_rectangular_error_)  // > 25°
    continue;  // 灯条没有与装甲板水平边缘对齐
```

**rectangular_error 的计算**：
```cpp
double roll = atan2(left2right.y, left2right.x);  // 两个灯条中心连线与水平方向的夹角
rectangular_error = max(|cos(left.angle - roll)|, |cos(right.angle - roll)|);
// 理想情况：灯条 ⊥ 连线 → angle - roll ≈ ±π/2 → cos ≈ 0 → error ≈ 0
// 偏差情况：灯条倾斜 → |cos| > 0 → error > 0
```

### 7.4 类型分类

```cpp
armor.type = (armor.ratio > 3.0) ? ArmorType::BIG : ArmorType::SMALL;
```

- **BIG**：大装甲板（步兵机器人前后面的主装甲），宽度/灯条长度 > 3.0
- **SMALL**：小装甲板（哨兵、英雄侧面装甲），宽度/灯条长度 ≤ 3.0

这个简单的比值规则源于实际机械设计：大装甲板的物理宽度（230mm）远大于灯条长度（56mm），而小装甲板（135mm）的比值更接近。

---

## 8. 角点亚像素优化：梯度搜索算法

### 8.1 为什么需要亚像素优化

`solvePnP` 对角点精度**极度敏感**：
- 1 像素的角点误差 → 5 米处约 5–10 cm 的位置误差
- 10 cm 的位置误差 → yaw 偏差约 1.15° → 5 米处约 10 cm 的瞄准偏差

原始角点来自 `cv::minAreaRect`，精度受限于像素网格。`LightCornerCorrector` 通过**局部梯度搜索**将精度提升到亚像素级别。

### 8.2 完整算法

```
对每个灯条分别调用 lightbar_points_corrector()：

Step 1: 局部 ROI 提取
  ├── 获取灯条旋转矩形的 boundingRect
  ├── 四周各扩展 ROI_SCALE (10%)
  └── 裁剪到图像边界

Step 2: 局部 Otsu 二值化
  ├── cv::threshold(roi, binary, 0, 255, THRESH_OTSU)
  ├── cv::findContours → 找最大轮廓
  └── cv::minAreaRect → 重新拟合（更精确的灯条矩形）

Step 3: 轴线提取
  ├── 从新的旋转矩形提取主轴方向 axis
  ├── axis 归一化，确保指向下方 (axis.y > 0)
  └── 确定灯条宽度 half_width

Step 4: 梯度搜索 — 找灯条端点（核心）
  参数：
    SEARCH_START = 0.4   # 从灯条长度的 40% 处开始搜索
    SEARCH_END   = 0.6   # 搜索到灯条长度的 60% 处
    SEARCH_STEP  = 0.5   # 步长 0.5 像素（亚像素！）

  对每个方向 direction = -1 (top) 和 +1 (bottom)：
    ├── 在 [-half_width, +half_width] 内生成多条搜索线
    │   └── 搜索线方向 = 垂直于灯条主轴
    ├── 每条线沿主轴从 SEARCH_START 搜索到 SEARCH_END
    │   每步计算：
    │     prev_val = 双线性插值(prev_point)
    │     cur_val  = 双线性插值(cur_point)
    │     diff = prev_val - cur_val
    │     追踪 max_diff，要求 prev_val > ROI 均值
    │     → "灯条内部亮 → 边界外暗" 的跳变点
    ├── 所有搜索线的候选点取平均
    └── 降级方案：没找到梯度跳变 → 回退到几何端点
          center ± axis * length * 0.5
```

### 8.3 关键设计决策

**为什么搜索范围是 40%–60% 而不是整个灯条长度？**
- 灯条几何中心已由 `minAreaRect` 给出，相对准确
- 从中心附近开始向外搜索，追踪灯条的边界
- 如果从端点开始搜索，端点本身就是要修正的目标——会引入偏差

**为什么用多条搜索线取平均？**
- 灯条的边界可能不平整（制造公差、透视畸变）
- 多条线的平均可以提高鲁棒性
- 倾斜的灯条在不同位置的边界可能有微小差异

**双线性插值的精度**：
- `SEARCH_STEP = 0.5` 意味着在亚像素位置采样
- `cv::getRectSubPix` 或手动双线性插值 → 精度约 0.1 像素

### 8.4 优化后的角点重组

```cpp
// 用优化后的灯条端点重建装甲板角点
armor.points = {
    left.top,      // 左上（优化后）
    right.top,     // 右上（优化后）
    right.bottom,  // 右下（优化后）
    left.bottom    // 左下（优化后）
};
armor.center = (left.center + right.center) / 2;  // 用优化后的灯条中心
```

---

## 9. 数字识别：ONNX 推理

### 9.1 模型选择

使用预训练的 **LeNet / MLP** 网络，通过 OpenCV DNN 模块进行 ONNX 推理。

**为什么是 LeNet 而不是更深的网络？**
1. **输入极小**：28×28 单通道图像 — 更深网络会过拟合
2. **速度**：LeNet < 1ms vs ResNet > 10ms — 60fps 下每帧仅 16ms 预算
3. **兼容性**：OpenCV DNN 对简单 ONNX 模型支持稳定

### 9.2 类别标签

```
1, 2, 3, 4, 5, outpost, sentry, base, negative
```

共 9 类。"negative" 表示非装甲板（背景误检）。

### 9.3 数字区域提取

```cpp
// Step 1: 推算装甲板的四个角点（利用几何先验）
// k_extend_ratio = 1.125 = (装甲板高度/2) / 灯条长度
//   = (126mm/2) / 56mm = 63/56 = 1.125
tl = left.center - left.top2bottom * 1.125;   // 左上角
bl = left.center + left.top2bottom * 1.125;   // 左下角
tr = right.center - right.top2bottom * 1.125;  // 右上角
br = right.center + right.top2bottom * 1.125;  // 右下角

// Step 2: 透视变换 — 将装甲板区域"展平"
cv::getPerspectiveTransform({bl, tl, tr, br}, dst_rect);
cv::warpPerspective(frame, warped, M, dst_size);
// 小装甲板 → 32×28, 大装甲板 → 54×28

// Step 3: 裁剪数字 ROI
int roi_x = (warped_width - 20) / 2;
cv::Rect roi(roi_x, 0, 20, 28);  // 中间 20×28 区域
cv::Mat number_roi = warped(roi);

// Step 4: 预处理
cv::cvtColor(number_roi, gray, COLOR_RGB2GRAY);
cv::threshold(gray, binary, 0, 255, THRESH_OTSU);  // 自适应二值化
cv::resize(binary, input, Size(28, 28));           // 模型输入尺寸
```

### 9.4 ONNX 推理

```cpp
// 转为 float 并归一化
input.convertTo(input, CV_32F, 1.0 / 255.0);

// 创建 blob (batch=1, channel=1, height=28, width=28)
cv::Mat blob = cv::dnn::blobFromImage(input);

// 前向传播
net_.setInput(blob);
cv::Mat output = net_.forward();

// 找最高置信度的类别
cv::Point max_loc;
double confidence;
cv::minMaxLoc(output, nullptr, &confidence, nullptr, &max_loc);
int class_id = max_loc.x;
```

### 9.5 过滤规则

```cpp
// 1. 置信度过滤
if (confidence < classifier_threshold_) return "negative";  // < 0.7

// 2. 类别一致性检查
string number = labels_[class_id];

// 大装甲板上的数字只能是 "1"（比赛规则：1号机器人使用大装甲板）
if (armor.type == BIG && number != "1") return "negative";

// 小装甲板上的数字不能是 "1"
if (armor.type == SMALL && number == "1") return "negative";

// 3. ignore_classes 过滤
if (number == "negative" || number == "sentry")  // sentry 不攻击哨兵
    return "negative";
```

**为什么需要类型一致性检查？**

比赛规则规定了机器人编号与装甲板类型的对应关系：
- 1 号（步兵/英雄）：大装甲板
- 2–5 号（步兵）：小装甲板
- sentry（哨兵）：小装甲板
- outpost（前哨站）：特殊

如果分类器输出与装甲板类型矛盾，说明分类错误，应该拒绝。

---

## 10. 配置参数完整速查

### 10.1 灯条参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `min_lightbar_ratio` | 1.5 | 最小长宽比 |
| `max_lightbar_ratio` | 20 | 最大长宽比 |
| `min_lightbar_length` | 8 | 最小像素长度 |
| `max_angle_error` | 45° | 与垂直方向最大偏差 |
| `max_lightbar_area` | 30000 | 最大轮廓面积 (px²) |

### 10.2 装甲板参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `min_armor_ratio` | 1 | 最小宽高比 |
| `max_armor_ratio` | 5 | 最大宽高比 |
| `max_side_ratio` | 1.5 | 左右灯条最大长度比 |
| `max_rectangular_error` | 25° | 灯条与水平线最大垂直度误差 |

### 10.3 数字识别参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `classifier_threshold` | 0.7 | 最低置信度 |
| `ignore_classes` | ["negative"] | 忽略的类别 |
| `model_path` | `lenet.onnx` | ONNX 模型路径 |

### 10.4 通用参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `binary_thres` | 90 | 二值化固定阈值 |
| `detect_color` | "RED" | 目标颜色 (RED/BLUE) |
| `debug` | false | 启用调试可视化 |

---

## 11. 调试与可视化

### 11.1 启用调试模式

```yaml
# config/auto_aim.yaml
auto_aim:
  armor_detector:
    debug: true
```

### 11.2 可视化内容

调试模式下会在图像上叠加：
- **绿色框**：灯条旋转矩形
- **红色框**：装甲板 4 个角点
- **蓝色标签**：识别出的数字 + 置信度
- **中心十字**：装甲板中心

### 11.3 性能监控

```cpp
auto t1 = chrono::steady_clock::now();
auto armors = detector.ArmorDetect(frame);
auto t2 = chrono::steady_clock::now();

double ms = chrono::duration<double, milli>(t2 - t1).count();
// 期望：< 5ms（整个检测流水线）
// findLights: ~1-2ms
// findArmors: ~0.5-1ms
// ONNX 推理: ~0.5-1ms per armor
// CornerCorrector: ~0.2-0.5ms per armor
```

### 11.4 常见问题排查

| 问题 | 诊断方法 | 解决方案 |
|------|---------|---------|
| 灯条检测不到 | 查看 binary_ 输出 | 降低 `binary_thres` |
| 灯条太多（包括噪声） | 查看 findLights 中间输出 | 提高 `binary_thres`，增加面积上限 |
| 装甲板配对错误 | 查看配对日志 | 调节 `max_side_ratio` 和 `max_rectangular_error` |
| 数字识别率低 | 查看 number_roi 图像 | 确认透视变换参数，检查模型文件 |
| 检测太慢 (>10ms) | 分析各步骤耗时 | 减小 `max_lightbar_area`，减少候选数量 |
