# SP_Vision 装甲板识别与解算详解

> 📘 **母文档**：[[SP_Vision_学习指南]]（先读母文档建立全局框架，本篇深化其 §6.1 / §6.2）
> 📘 **下游**：识别与解算的输出（`Armor` 世界坐标）进入 [[SP_Vision_整车估计EKF详解]]
> 📘 **相关**：[[MAS_Vision_装甲板检测教程]]、[[MAS_Vision_位姿解算与目标跟踪教程]]（本战队同类模块的不同实现，可对照）

本篇聚焦自瞄流水线的**感知前两级**：识别器（图像 → 装甲板四点 + 类别）与解算器（四点 → 世界坐标 + 朝向）。核心问题是"**两条识别路径如何归一到同一出口**"与"**共面 PnP 的 yaw 为什么不可信、怎么修**"。

---

## 目录

1. [识别的统一出口：为什么一切归到四点](#1-识别的统一出口为什么一切归到四点)
2. [传统灯条识别全流程](#2-传统灯条识别全流程)
3. [神经网络识别：YOLO11 / v8 / v5 三条路](#3-神经网络识别yolo11--v8--v5-三条路)
4. [数字分类器](#4-数字分类器)
5. [位姿解算：3D 物点模型与 IPPE](#5-位姿解算3d-物点模型与-ippe)
6. [yaw 优化：全套解算精度的命门](#6-yaw-优化全套解算精度的命门)
7. [四点顺序一致性：贯穿全链的隐形契约](#7-四点顺序一致性贯穿全链的隐形契约)
8. [概念关系图谱](#8-概念关系图谱)

---

## 1. 识别的统一出口：为什么一切归到四点

自瞄识别有两条技术路线：**传统灯条配对**（二值化 + 几何过滤，可解释、无需 GPU）与**神经网络关键点检测**（端到端、高召回）。sp_vision_25 的设计哲学是——无论走哪条路，最终都产出同一个 `Armor` 结构体，其中最关键的字段是：

```cpp
std::vector<cv::Point2f> points;   // 四角点，固定顺序 [左上, 右上, 右下, 左下]
ArmorName name;                    // one~five / sentry / outpost / base / not_armor
ArmorType type;                    // big / small
```

**为什么这样设计**：下游的 PnP 解算（`solver`）只吃 `points` 四点，完全不关心这四点是灯条端点算出来的还是神经网络回归出来的。这就把"识别方式"与"解算方式"彻底解耦——换模型、加路径都不影响下游。这是整个视觉框架"可替换、可离线测"的地基。

一句话：**识别器的职责被压缩为"无论如何都要吐出四个像素点 + 一个类别"**，其余全部下沉到统一的 `Armor`。

---

## 2. 传统灯条识别全流程

传统路径在 `detector.cpp:33-120` 的 `detect()`，是一条固定管线。

### 2.1 从图像到灯条

```cpp
cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);              // detector.cpp:36
cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);  // :40
cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // :45
```

三个关键决策：

- **全局固定阈值二值化**：`threshold_` 从 YAML 读入（`detector.cpp:18`），对灰度整图一刀切。这里**不做颜色通道相减**——判色留到后面用轮廓像素做，二值化只负责"找到亮块"。
- **`RETR_EXTERNAL`**：灯条是实心亮块，只需最外层轮廓。
- **`CHAIN_APPROX_NONE`**：保留轮廓上**全部**像素点。这不是浪费——后面 `get_color` 要沿轮廓所有点累加 RGB 来判红蓝。

### 2.2 Lightbar 几何量：把旋转矩形抽象成一根灯条

每个轮廓 `cv::minAreaRect` 得最小外接旋转矩形，构造 `Lightbar`（`armor.cpp:9-31`）：

```cpp
rotated_rect.points(&corners[0]);
std::sort(corners..., [](a,b){ return a.y < b.y; });  // 按 y 升序：前两个"上边"，后两个"下边"
top    = (corners[0] + corners[1]) / 2;               // 上边中点
bottom = (corners[2] + corners[3]) / 2;               // 下边中点
top2bottom = bottom - top;                            // 主轴向量（从上指向下）
width  = cv::norm(corners[0] - corners[1]);           // 短边 = 灯条宽
angle  = std::atan2(top2bottom.y, top2bottom.x);
angle_error = std::abs(angle - CV_PI/2);              // 与竖直(90°)偏差
length = cv::norm(top2bottom);
ratio  = length / width;                              // 长宽比
```

**几何直觉**：图像坐标系 y 轴向下，一根理想竖直灯条的主轴指向正下方，即 `angle = π/2`。`angle_error` 就是灯条偏离竖直的程度，是最有力的几何过滤量。

### 2.3 灯条几何过滤（`check_geometry`，detector.cpp:233-239）

三条与门（AND）：

| 条件 | 含义 | 滤除对象 |
|:----|:----|:----|
| `angle_error < max_angle_error_` | 必须足够竖直（YAML 值 `/57.3` 转弧度，`:19`） | 倾斜的杂光 |
| `min_lightbar_ratio_ < ratio < max_lightbar_ratio_` | 长宽比范围 | 近方形反光、过细噪点 |
| `length > min_lightbar_length_` | 最小长度 | 远处噪点 |

### 2.4 判色：沿轮廓累加 R/B 通道（`get_color`，detector.cpp:279-289）

```cpp
for (auto & point : contour) {
  red_sum  += bgr_img.at<cv::Vec3b>(point)[2];   // R 通道
  blue_sum += bgr_img.at<cv::Vec3b>(point)[0];   // B 通道
}
return blue_sum > red_sum ? Color::blue : Color::red;
```

**为什么用轮廓像素而非填充区域**：`CHAIN_APPROX_NONE` 已保留全部轮廓点，且灯条边缘的颜色信息最稳定（中心常因过曝发白），计算量还小。传统路径只区分 red/blue 两类——`extinguish`（熄灭）/`purple`（紫，基地）是留给神经网络路径的枚举。

### 2.5 配对成 Armor（detector.cpp:63-84）

先按 `center.x` 升序排列所有灯条（`:63`），保证 `left` 一定在 `right` 左侧。然后双重循环、`right = std::next(left)` 只向右配（避免重复对），**同色才配**（`:69`）。

配对后计算装甲板几何量（`armor.cpp:34-56`）：

```cpp
points = {left.top, right.top, right.bottom, left.bottom};  // [左上,右上,右下,左下] 顺时针
left2right = right.center - left.center;
width  = norm(left2right);                          // 两灯条中心间距
ratio  = width / max_lightbar_length;               // ★ 判大小装甲的关键量：中心距 / 长灯条长
side_ratio = max_lightbar_length / min_lightbar_length;  // 两灯条长度比（应接近1）
roll = atan2(left2right.y, left2right.x);            // 装甲板横向倾角
rectangular_error = max(|left.angle-roll-π/2|, |right.angle-roll-π/2|);  // 偏离直角的量
```

> ⚠️ `center = (left.center + right.center)/2` 并**不是对角线交点**，不能当作装甲板真实中心（`armor.hpp:84` 有注释警告）。真实中心要靠 PnP 解算。

装甲板过滤（`check_geometry(Armor)`，detector.cpp:241-247）：`ratio` 在范围内、`side_ratio < max_side_ratio_`（两灯条不能长度悬殊，否则是错配）、`rectangular_error < max_rectangular_error_`（保证近似矩形）。

### 2.6 pattern ROI 提取与 1.125 常数的由来（detector.cpp:291-309）

配对成功后，要裁出装甲板中央的数字贴片送分类器。灯条只覆盖装甲板高度的一部分，需沿主轴外推：

```cpp
// detector.cpp:293-294 原注释：
// 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
tl = armor.left.center  - armor.left.top2bottom  * 1.125;
bl = armor.left.center  + armor.left.top2bottom  * 1.125;
tr = armor.right.center - armor.right.top2bottom * 1.125;
br = armor.right.center + armor.right.top2bottom * 1.125;
```

**常数推导**：装甲板全高 126mm，灯条长 56mm。要从灯条中心沿主轴向上/下各延伸 `armor_height/2 = 63mm`，而 `top2bottom` 的模长恰等于灯条长 56mm，所以延伸系数 = `63 / 56 = 1.125`。这样把灯条端点外推到装甲板真实上下边缘，取到完整数字区。最后用四点极值 + 图像边界钳位得正矩形 `cv::Rect` 裁出。

> 记住：**1.125 只用于 pattern 提取（数字识别）**。PnP 解算的物点纵向只到灯条端点（用半灯条长 28mm，见 §5）。这是两个不同的"高度"概念，切勿混淆。

### 2.7 大小装甲判定（`get_type`，detector.cpp:311-338）

**几何优先，图案兜底**：

```
ratio > 3.0            → big
ratio < 2.5            → small
2.5 ≤ ratio ≤ 3.0      → 看名字：one(英雄)/base(基地) → big，其余 → small
```

阈值由来：小装甲板宽 135mm、大装甲板宽 230mm，除以灯条长 56mm 得中心距/灯条长的理论比值约 2.4 与 4.1，取 2.5/3.0 作保守分界。

### 2.8 共用灯条去重（detector.cpp:87-115）

同一根灯条可能被两块候选装甲板共用（错配），需去重。`Lightbar.id`（`armor.hpp:69`）就是为此设计的唯一标识，在 detect 循环里按加入顺序自增。双重循环遍历所有 armor 对，两种重叠：

| 情形 | 判据 | 保留策略 | 逻辑 |
|:----|:----|:----|:----|
| **同侧共用（重叠）** | 两块的 left.id 相同 或 right.id 相同 | 保留 pattern **面积小**者 | 错配框往往更大，紧凑 ROI 更可能是真装甲 |
| **首尾相连（相邻）** | 一块的 right 是另一块的 left | 保留**置信度大**者 | 分类器更有把握的更可信 |

被淘汰者标 `duplicated=true`，最后 `armors.remove_if(duplicated)`（`:115`）。

> **一句话速记**：传统识别 = 灰度二值化 → 找轮廓 → minAreaRect 成灯条 → 三条几何门过滤 → 同色左右配对成装甲板 → 1.125 外推取数字贴片 → ratio 判大小 → 共用灯条去重。全程可解释、无需 GPU。

---

## 3. 神经网络识别：YOLO11 / v8 / v5 三条路

三种模型都产出统一的 `Armor`，但内部结构差异很大。核心区别在于**颜色和数字的识别分工**。

### 3.1 三者一览对比

| | 输出类别 | 输出转置 | objectness | 数字识别 | 输入尺寸 |
|:----|:----|:----|:----|:----|:----|
| **YOLO11** | 38 类合一（`class_num_=38`） | 需转置 | 否，分数直接用 | 模型端到端 | 640² |
| **YOLOV8** | 2 类（仅颜色/框） | 需转置 | 否 | 传统 Classifier | 416² |
| **YOLOV5** | 颜色 4 + 数字 9 独立头 | 不转置 | 需 sigmoid | 模型（可选传统矫正四点） | 640² |

### 3.2 YOLO11：端到端 38 类（yolo11.cpp）

**OpenVINO 图内预处理**（`yolo11.cpp:33-53`）是工程亮点。用 `ov::preprocess::PrePostProcessor` 把预处理编进模型计算图：

```cpp
// 输入声明为 u8 / NHWC / BGR —— 直接喂 OpenCV 的 BGR u8 图
ppp.input().tensor().set_element_type(u8).set_layout("NHWC").set_color_format(BGR);
ppp.input().model().set_layout("NCHW");
ppp.input().preprocess()
   .convert_element_type(f32)   // u8 → f32
   .convert_color(RGB)          // BGR → RGB
   .scale(255.0);               // 归一化到 [0,1]
```

**好处**：颜色转换、类型转换、归一化全在推理设备（CPU/GPU）上做，主机 CPU 只需 letterbox。`compile_model` 用 `LATENCY` 模式优化单帧延迟（`:52`）。

**letterbox**（`yolo11.cpp:77-87`）等比缩放贴**左上角**（非居中），右/下补黑，`scale` 存下用于后处理坐标还原：

```cpp
scale = min(640.0/rows, 640.0/cols);
resize(bgr_img, input(cv::Rect(0,0, cols*scale, rows*scale)), ...);  // 左上角贴图，其余补黑
```

**转置输出解码**（`yolo11.cpp:102-176`）是理解 YOLO11 的关键：

```cpp
cv::transpose(output, output);   // [42, 8400] → [8400, 42]，每行变成一个候选框
```

YOLO11 原始输出每**列**是一个候选框，转置后每**行**一框，便于遍历。每行 42 维布局：

```
[0,4)   = xywh（中心 + 宽高）
[4,42)  = 38 类分数
[42,50) = 4 个关键点 (x,y) ×4 = 8 个数
```

解码：`minMaxLoc` 取最大类分（即 class_id），`score < 0.7` 跳过（`:122`）；xywh 与四点都除以 `scale` 还原到原图。然后 `cv::dnn::NMSBoxes` 去重（`:146`），`sort_keypoints` 重排四点（见 §7），`class_id` 查 `armor_properties` 表得颜色/名称/类型。

### 3.3 armor_properties：38 类映射表（armor.hpp:51-64）

`class_id`（下标）→ `(Color, ArmorName, ArmorType)` 元组表：

| id | Color | Name | Type |
|:--|:--|:--|:--|
| 0-2 | blue/red/extinguish | sentry | small |
| 3-5 | b/r/e | one | small |
| 6-8 | b/r/e | two | small |
| 9-11 | b/r/e | three | small |
| 12-14 | b/r/e | four | small |
| 15-17 | b/r/e | five | small |
| 18-20 | b/r/e | outpost | small |
| 21-24 | b/r/e/**purple** | base | big |
| 25-28 | b/r/e/**purple** | base | small |
| 29-31 | b/r/e | three | big |
| 32-34 | b/r/e | four | big |
| 35-37 | b/r/e | five | big |

规律：主色三个一组循环；base 有第四色 purple 且 big/small 两种（25 赛季基地顶装甲是小板）；three/four/five 各有 small/big 两套（平衡步兵用大装甲）。查表越界则默认 `blue/not_armor/small`（`armor.cpp:87-96`）。

### 3.4 YOLOV8：NN 定位 + 传统分类（yolov8.cpp）

`class_num_=2`，**只输出颜色框（蓝/红），不识别数字**。每行 14 维：`xywh(0-4) + 2类颜色(4-6) + 4关键点(6-14)`。

关键：parse 里仍调用传统分类器补齐数字——`get_pattern` + `classifier_.classify`（`yolov8.cpp:164-165`）。所以 YOLOV8 持有 `Classifier` 和 `Detector` 两个成员（`yolov8.hpp:29-30`），`get_pattern` 同样用 **1.125** 系数，但基于神经网络四点而非灯条（`yolov8.cpp:240-243`）。`get_type` 纯按名字判（不看 ratio）。

**分工**：YOLOV8 负责"定位 + 判色"，传统 Classifier 负责"判数字/大小"。

### 3.5 YOLOV5：颜色/数字独立双头（yolov5.cpp）

`class_num_=13`，与 v8/v11 结构差异最大：

- **输出不转置**（`:110`）——v5 输出本就每行一框。
- **objectness 过 sigmoid**（`:111-112`）：`score = sigmoid(output.at<float>(r, 8))`。第 8 列是 objectness，需 sigmoid 激活（v8/v11 的分类分数已激活，直接用）。
- **颜色/数字独立头**（`:119-127`）：`colRange(9,13)` = 4 类颜色分，`colRange(13,22)` = 9 类数字分，各自 argmax。这是 v5 的核心特征——把颜色与数字**解耦**成两个分类头。
- 四点在 0-7 列，但模型点序与 [左上,右上,右下,左下] 不同，手工重映射（`:129-136`，顺序 `(0,1),(6,7),(4,5),(2,3)`）。
- **可选传统二次矫正**（`:183-184`）：`if (use_traditional_) detector_.detect(*it, bgr_img)`——用传统灯条重新精修四点，提高 PnP 精度（见下）。

sigmoid 用数值稳定版（`yolov5.cpp:254-260`）：`x>0` 用 `1/(1+e^-x)`，否则 `e^x/(1+e^x)`，避免 `exp` 上溢。

### 3.6 神经网络四点的传统二次矫正（detector.cpp:122-231）

给 YOLOV5 用。逻辑：拿神经网络四点 → 沿灯条方向外扩构造更大 ROI → 在 ROI 内重新二值化找灯条 → 用四点到灯条端点的距离找最近的左右灯条 → 若 `min_distance_br_tr + min_distance_tl_bl < 15`（像素阈值，`:221`）则用传统灯条端点覆盖神经网络四点。

**为什么**：神经网络四点定位有 1~2 像素抖动，而 PnP 对像素误差敏感（尤其 yaw）。传统灯条端点亚像素更稳，二次矫正能提升解算精度。距离阈值 15px 是"矫正只在 NN 四点与传统灯条足够接近时才可信"的保险。

> **一句话速记**：YOLO11 端到端出颜色+编号+四点（转置解码 + 38 类查表）；YOLOV8 是"NN 定位 + 传统分类"混合；YOLOV5 颜色/数字双头、objectness 过 sigmoid，还可用传统灯条二次矫正四点。三者殊途同归到 `Armor.points`。

---

## 4. 数字分类器

`classifier.cpp` 有两套实现：`classify`（cv::dnn，`:17-59`，实际被 detector/yolov8 调用）与 `ovclassify`（OpenVINO，备用）。

**32×32 灰度 letterbox**（`classifier.cpp:24-39`）：

```cpp
cvtColor(pattern, gray, COLOR_BGR2GRAY);
scale = min(32.0/cols, 32.0/rows);       // 等比
resize(gray, input(cv::Rect(0,0, cols*scale, rows*scale)), ...);  // 贴左上角，其余补 0
```

pattern 为空或退化（h/w==0）直接判 `not_armor`。

**前向 + 数值稳定 softmax**（`classifier.cpp:41-51`）：

```cpp
blobFromImage(input, 1.0/255.0, ...);   // 归一化
net_.forward(outputs);
float max = *std::max_element(...);      // ★ 减最大值防溢出（log-sum-exp 稳定化）
cv::exp(outputs - max, outputs);
outputs /= cv::sum(outputs)[0];
```

减最大值再 exp 是标准做法——大 logit 时 `exp` 会上溢到 inf，减最大值后最大项变 `e^0=1`，安全。最后 `minMaxLoc` 取 9 类 argmax（对应 `ArmorName` 的 9 个枚举，含 `not_armor`）。

---

## 5. 位姿解算：3D 物点模型与 IPPE

解算器 `solver.cpp` 把每块装甲板的四点从像素坐标解算到世界坐标。

### 5.1 3D 物点模型与常数（solver.cpp:12-25）

```cpp
constexpr double LIGHTBAR_LENGTH   = 56e-3;    // m  灯条长
constexpr double BIG_ARMOR_WIDTH   = 230e-3;   // m  大装甲板宽
constexpr double SMALL_ARMOR_WIDTH = 135e-3;   // m  小装甲板宽
```

物点定义在**装甲板自身坐标系**（x 朝法向外、y 朝左、z 朝上），四点顺序与图像 `points` 一致：

```cpp
BIG_ARMOR_POINTS = {
  {0, +W/2, +L/2},   // 左上：y 正=左，z 正=上
  {0, -W/2, +L/2},   // 右上
  {0, -W/2, -L/2},   // 右下
  {0, +W/2, -L/2}};  // 左下     其中 W=装甲板宽, L=灯条长
```

> ★ 关键：z 方向用的是**半灯条长 56/2=28mm**，而非装甲板全高 126mm。因为四点是**灯条的上下端点**，纵向跨度就是一根灯条。这与 §2.6 pattern 提取用全高 126mm 是两码事——pattern 要取整块贴片，PnP 物点只到灯条端点。

### 5.2 IPPE 求解 PnP（solver.cpp:55-63）

```cpp
object_points = (type==big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_,
             rvec, tvec, false, cv::SOLVEPNP_IPPE);
```

`SOLVEPNP_IPPE`（Infinitesimal Plane-based Pose Estimation）专为**共面四点**设计，比迭代法快且稳。装甲板四点共面（x=0）正好适用。**但 IPPE 对绕竖直轴的 yaw 存在法向翻转歧义**——这正是下一节 yaw 优化的动机。

### 5.3 坐标变换链：相机 → 云台 → 世界（solver.cpp:65-79）

```cpp
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;  // 相机→云台（含平移）
armor.xyz_in_world  = R_gimbal2world_  * armor.xyz_in_gimbal;               // 云台→世界（仅旋转）
```

姿态同理逐级左乘，最后用 `tools::eulers(R, 2,1,0)`（ZYX 内旋）提取 `[yaw,pitch,roll]`，`tools::xyz2ypd` 转球坐标 `(yaw, pitch, distance)`。

`R_gimbal2world_` 的构造（`solver.cpp:48-52`）是**相似变换**：

```cpp
void set_R_gimbal2world(const Eigen::Quaterniond & q) {
  R_imubody2imuabs = q.toRotationMatrix();   // 电控发来的 IMU 姿态四元数
  R_gimbal2world_  = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}
```

`R_gimbal2imubody_` 是云台到 IMU 机体的固定安装外参（YAML）。用它把 IMU 测得的 body→abs 旋转共轭变换到 gimbal→world。这里的四元数 `q` 就是由 [[SP_Vision_IO与多线程架构详解]] 中 `imu_at(t)` 按图像时间戳 slerp 插值出来的那一帧姿态。

---

## 6. yaw 优化：全套解算精度的命门

### 6.1 问题：共面 PnP 的 yaw 为什么不可信

平面装甲板绕竖直轴旋转时，四点在图像上的变化量很小（尤其正对相机时），导致 IPPE 解出的 yaw 噪声极大、甚至法向翻转。但同一次 PnP 的**距离和 pitch 是可靠的**——因为它们对应四点的尺度和纵向分布，观测充分。

**思路**：固定可靠的量（xyz），只对不可靠的 yaw 做**重投影误差暴力搜索**。

### 6.2 暴力搜索实现（solver.cpp:196-218）

```cpp
gimbal_ypr = eulers(R_gimbal2world_, 2,1,0);
constexpr double SEARCH_RANGE = 140;                        // degree，总跨度 ±70°
yaw0 = limit_rad(gimbal_ypr[0] - SEARCH_RANGE/2 * CV_PI/180);  // 以云台朝向为中心
for (int i = 0; i < 140; i++) {                             // 步长 1°，共 140 次
  double yaw = limit_rad(yaw0 + i * CV_PI/180);
  double error = armor_reprojection_error(armor, yaw, (i-70)*CV_PI/180);
  if (error < min_error) { min_error = error; best_yaw = yaw; }
}
armor.yaw_raw = armor.ypr_in_world[0];   // 保留 PnP 原始 yaw 供对比
armor.ypr_in_world[0] = best_yaw;        // 用搜索结果覆盖
```

两个关键假定：

1. **固定 xyz**：搜索时平移量用 PnP 结果不动，只变 yaw。
2. **pitch = 15°**：装甲板都是后倾 15° 安装的标准件（`reproject_armor` 里 `pitch = (name==outpost) ? -15° : +15°`，`solver.cpp:96`——前哨站装甲朝上倾故取负）。

### 6.3 重投影误差（solver.cpp:90-127, 254-263）

`reproject_armor(xyz_world, yaw, type, name)` 用给定 yaw + 固定 pitch=15° 显式构造 `R_armor2world`（roll=0）：

```
R_armor2world = [cy·cp, -sy, cy·sp;
                 sy·cp,  cy, sy·sp;
                   -sp,   0,    cp]
```

再逆变换回相机系、`cv::projectPoints` 投影出 4 个像素点，误差取四点像素 L2 距离和：

```cpp
error = Σ_{i=0..3} cv::norm(armor.points[i] - image_points[i]);   // solver.cpp:254-263
```

（另有更复杂的 `SJTU_cost`——线段长度差 + 角度差加权，参考上交，当前注释关闭。）

### 6.4 平衡步兵不做 yaw 优化（solver.cpp:81-87）

```cpp
bool is_balance = (type==big) && (name==three || name==four || name==five);
if (is_balance) return;   // 平衡步兵跳过优化，保留原始 PnP yaw
optimize_yaw(armor);
```

**为什么**：平衡步兵装甲板不是标准 15° 后倾安装，pitch=15° 假设不成立，暴力搜索反而会得到错误 yaw。其余所有兵种都优化。

> **一句话速记**：共面装甲板用 IPPE 解 PnP，距离/pitch 可靠但 yaw 有法向翻转歧义；于是固定 xyz + 假定 pitch=15°，在 ±70° 内以 1° 步暴力搜索让重投影误差最小的 yaw——这是全套解算精度的命门。平衡步兵因安装角特殊而例外。

---

## 7. 四点顺序一致性：贯穿全链的隐形契约

**[左上, 右上, 右下, 左下]**（顺时针）这个顺序必须在三处严格一致，否则 PnP 解算错位：

| 出处 | 位置 | 如何保证 |
|:----|:----|:----|
| 传统配对 | `armor.cpp:40-43` | `{left.top, right.top, right.bottom, left.bottom}` 直接构造 |
| YOLO 检测 | `yolo11.cpp:209-235` `sort_keypoints` | 先按 y 分上下，再按 x 分左右，重排为固定序 |
| PnP 物点 | `solver.cpp:16-20` | 物点数组按同一顺序排列 |

`sort_keypoints` 的重排逻辑（`yolo11.cpp:209-235`）：

```cpp
sort(kp, by y 升序);                  // 前2=上边，后2=下边
sort(top_points, by x 升序);          // top-left, top-right
sort(bottom_points, by x 升序);
keypoints = {top[0], top[1], bottom[1], bottom[0]};  // [左上,右上,右下,左下]
```

**为什么关键**：神经网络回归出的四点顺序是任意的（取决于训练标注），必须归一化。这个"隐形契约"是识别与解算解耦的前提——只要四点按约定排好，solver 就不用管来源。

---

## 8. 概念关系图谱

### 8.1 识别到解算的数据流

```
                        ┌─── 传统路径 ───┐
   BGR 图像 ──▶ 二值化 ─▶ 找轮廓 ─▶ Lightbar ─▶ 同色配对 ─┐
                                    (几何过滤)            │
                        ┌─── NN 路径 ───┐                 ▼
   BGR 图像 ──▶ letterbox ─▶ OpenVINO ─▶ 解码四点 ─▶ NMS ─▶ 【Armor.points 四点 + name/type】
                                        sort_keypoints    │ (统一出口)
                                                          ▼
                                          solvePnP(IPPE) ── xyz(可靠) + yaw(不可靠)
                                                          │
                                          固定xyz+pitch=15° 暴力搜索 yaw ── best_yaw
                                                          │
                                          相机→云台→世界变换 ── xyz_in_world / ypr_in_world / ypd
                                                          │
                                                          ▼  进入 EKF 整车估计
```

### 8.2 易混概念对照

| 易混概念 | 区别 |
|:----|:----|
| **1.125 系数 vs 半灯条长 28mm** | 1.125 用于 pattern 提取（取整块贴片 126mm）；PnP 物点纵向只到灯条端点（56/2=28mm）。两个不同高度概念 |
| **传统判色 vs NN 判色** | 传统沿轮廓累加 R/B（只出 red/blue）；NN 用颜色头（可出 extinguish/purple） |
| **传统 check_type vs NN check_type** | 传统：big 必须是 one/base；NN：big 只排除 two/sentry/outpost（允许平衡步兵 3/4/5） |
| **ratio（传统）vs ratio（NN）** | 传统 = 两灯条中心距/长灯条长；NN = 上下边最大长/左右边最大宽。定义不同 |
| **PnP yaw vs 优化后 yaw** | IPPE 平面 PnP 的 yaw 有翻转歧义、噪声大 → 固定 xyz + pitch=15° 暴力搜索修正；原始值存 `yaw_raw` |
| **YOLO11 vs v8 vs v5** | 11=端到端38类；v8=NN定位+传统分类；v5=颜色/数字双头+可选传统矫正 |
| **Armor.center vs 真实中心** | `(left+right)/2` 不是对角线交点，非真实中心；真实中心靠 PnP |

### 8.3 一句话速记

> 识别 = "传统灯条配对" 或 "YOLO 四点检测" 都归一到 `Armor.points`（[左上,右上,右下,左下]）+ 类别；解算 = IPPE 解 PnP 得可靠的 xyz、不可靠的 yaw，再固定 xyz + 假定 pitch=15° 暴力搜索修正 yaw，最后经相机→云台→世界变换输出世界坐标，喂给 EKF。

---

## 相关笔记

- [[SP_Vision_学习指南]] — 母文档，全局框架
- [[SP_Vision_整车估计EKF详解]] — 本篇输出的世界坐标是 EKF 的观测输入
- [[SP_Vision_IO与多线程架构详解]] — `set_R_gimbal2world` 用的四元数由 `imu_at` slerp 插值而来
- [[MAS_Vision_装甲板检测教程]]、[[MAS_Vision_位姿解算与目标跟踪教程]] — 本战队同类模块的不同实现
- [[RM_C程序员_C++速通指南]] — 多态、模板、`std::function` 等本项目大量使用的技法
