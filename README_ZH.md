# ufactory_vision (中文版)

## 项目概述

`ufactory_vision` 是一个基于 UFACTORY 850、Lite 6 和 xArm 系列机械臂，结合 Intel Realsense D435 摄像头或 Luxonis OAK-D-Pro-PoE 深度摄像头的视觉抓取演示项目。用户可以通过本项目快速实现基于视觉的物体检测和抓取。

[![观看视频](https://img.youtube.com/vi/ijnuqsNcfUY/0.jpg)](https://www.youtube.com/watch?v=ijnuqsNcfUY)

## 硬件要求

**UFACTORY 机械臂与 Intel Realsense D435 摄像头**

视频中我们使用了 UFACTORY xArm 6 机械臂，UFACTORY 850、xArm 5、xArm 7 和 Lite 6 同样适用。

- **机械臂**: [UFACTORY xArm 6](https://www.ufactory.cc/products/xarm)
- **夹爪**: [UFACTORY xArm 夹爪](https://www.ufactory.cc/product-page/ufactory-xarm-gripper/)
- **摄像头**: [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/)
- **摄像头支架**: UFACTORY 提供 (可购买或 3D 打印)
  - 购买链接: [UFACTORY 摄像头支架](https://www.ufactory.cc/product-page/ufactory-xarm-camera-stand/)
  - 3D 文件下载: [Realsense_Camera_Stand.STEP](https://www.ufactory.cc/wp-content/uploads/2024/05/CameraStand_1300.zip)

**UFACTORY 机械臂与 Luxonis OAK-D-Pro-PoE 摄像头**

视频中我们使用了一台内置千兆网线的定制版 850 进行拍摄，如果您需要定制版内置千兆网线的 UFACTORY 850 机械臂，请[联系 UFACTORY 销售](https://www.ufactory.cc/contact-us/)。UFACTORY xArm 5、xArm 6 和 xArm 7 同样适用，Lite 6 暂时不支持。

- **机械臂**: [UFACTORY 850](https://www.ufactory.cc/ufactory-850/)[机械臂外置网线]
- **夹爪**: [UFACTORY xArm 夹爪](https://www.ufactory.cc/product-page/ufactory-xarm-gripper/)
- **摄像头**: [OAK-D Pro PoE](https://shop.luxonis.com/products/oak-d-pro-poe?variant=42469208883423)
- **PoE 注入器**: Tenda POE30G-AT, 1000M, 30W (其他品牌也适用)
- **摄像头支架**: UFACTORY 提供 (使用 3D 文件进行 3D 打印或 CNC 加工)
  - 3D 文件下载: [OAK_Camera_Stand.STEP](https://www.ufactory.cc/wp-content/uploads/2025/05/oak_camera_stand.zip)

## 软件

### 支持的 Python 版本

支持的 Python 版本：3.8-3.11 (推荐：3.11)。

## 安装

### 克隆仓库

```bash
git clone https://github.com/xArm-Developer/ufactory_vision.git
```

### 创建 Python 虚拟环境

建议使用虚拟环境运行此项目。

#### **Windows (使用 Anaconda)**

```bash
conda create --name ufactory_vision python=3.11
conda activate ufactory_vision
```

#### **Linux (使用 venv)**

```bash
python3.11 -m venv ufactory_vision
source ufactory_vision/bin/activate
```

### 安装依赖

**Intel Realsense D435 摄像头**
```bash
cd ggcnn_grasping_demo
pip install -r requirements_rs.txt
```
**Luxonis OAK-D-Pro-PoE 摄像头**
```bash
cd ggcnn_grasping_demo
pip install -r requirements_depthai.txt
```

## 运行示例

**UFACTORY 850 或 xArm 5/6/7 与 Intel Realsense D435 摄像头**

```bash
python run_rs_grasp.py 192.168.1.xxx
```

**UFACTORY Lite 6 与 Intel Realsense D435 摄像头**

```bash
python run_rs_grasp_lite6.py 192.168.1.xxx
```

**Luxonis OAK-D-Pro-PoE 摄像头**

```bash
python run_depthai_grasp.py 192.168.1.xxx
```

将 `192.168.1.xxx` 替换为控制器的 IP 地址。

## 重要提示
* 不要设置 TCP 偏移或坐标系偏移，否则您需要微调代码。
* 设置 TCP 负载以避免错误的碰撞检测。
* 运行示例前，请确保已启用碰撞检测。建议将碰撞灵敏度设置为 3 或更高。

## 许可证

本项目采用 **BSD 3-Clause 许可证**。详情请查看 [LICENSE](LICENSE) 文件。

## 参考

我们的演示项目基于以下开源项目构建：

[GGCNN](https://github.com/dougsm/ggcnn)
[ggcnn_kinova_grasping](https://github.com/dougsm/ggcnn_kinova_grasping)