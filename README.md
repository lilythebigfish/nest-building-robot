# ufactory_vision

## Project Overview
`ufactory_vision` is a vision-based grasping demo based on UFACTORY xArm 6 and Intel Realsense D435 camera. Users can quickly implement vision-based object detection and grasping with this project.

## Hardware Requirements
This project requires the following hardware:

- **Robot Arm**: [UFACTORY xArm 6](https://www.ufactory.cc/products/xarm)
- **Camera**: [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/)
- **Camera Mount**: Provided by UFACTORY (available for purchase or 3D printing)
  - Purchase Link: [UFACTORY Camera Stand](https://www.ufactory.cc/product-page/ufactory-xarm-camera-stand/)
  - 3D File Download: [Download Link](#) (Replace with actual link)

## Supported Python Versions
Python versions 3.8 to 3.11 are supported.

## Installation
### Clone the repository
```bash
git clone https://github.com/xArm-Developer/ufactory_vision.git
```

### Create a Python Virtual Environment
It is recommended to use a virtual environment for running this project.

#### **Windows (Using Anaconda)**
```bash
conda create --name ufactory_vision python=3.8
conda activate ufactory_vision
```

#### **Linux (Using venv)**
```bash
python3 -m venv ufactory_vision
source ufactory_vision/bin/activate
```

### Install dependencies
```bash
pip install -r requirements_rs.txt
```

## Run Example
```bash
python run_rs_grasp.py 192.168.1.xxx
```
Replace `192.168.1.xxx` with the IP address of the controller.

## Important Notes
Before running the example, ensure that collision detection is enabled. It is recommended to set the collision sensitivity to 3 or higher.

## License
This project is licensed under the **BSD 3-Clause License**. For details, please check the [LICENSE](LICENSE) file.

