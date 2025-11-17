# JLU_HUAWEI-Experiment

<div align="center">

[![JLU 华为实验](https://img.shields.io/badge/JLU-%E5%8D%8E%E4%B8%BA%E5%AE%9E%E9%AA%8C-blue)](https://github.com/JLU-Huawei/JLU_HUAWEI-Experiment) [![STM32](https://img.shields.io/badge/MCU-STM32-orange)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)

**吉林大学电机拖动课程华为实验10组 - 智能小车项目**

基于华为 Atlas 200i DK A2 的目标识别与导航智能小车


</div>

---

## 🚗 项目概述

本仓库包含智能小车系统的完整代码库。项目结合了硬件控制、嵌入式系统编程和AI驱动的目标检测技术，创建了一个自动驾驶车辆，能够在避开障碍物的同时导航到目标物体。

## 📁 项目结构

```
JLU_HUAWEI-Experiment/
├── Hardware/                   # STM32微控制器嵌入式代码
│   ├── CORE/                   # 核心STM32文件
│   ├── FWLIB/                  # 固件库
│   ├── HARDWARE/               # 硬件特定驱动程序
│   ├── SYSTEM/                 # 系统函数
│   ├── USER/                   # 主应用程序代码
│   ├── BALANCE/                # 平衡算法
│   ├── FreeRTOS/               # 实时操作系统组件
│   └── OBJ/                    # 构建输出目录
├── Software/                   # 目标检测和PC端软件
│   ├── data_collection/        # PC端数据标注工具
│   └── yolo_detect/            # 目标检测与通信代码
├── Model/                      # 机械模型设计文件
│   └── atlas_card-Body001.stl  # 板子3D模型
└── Public/                     # 媒体文件和文档
    ├── images/
    └── videos/
```

## 🧱 组件说明

### 🖥️ 硬件组件 (`Hardware/`)
- **微控制器**: 基于STM32的嵌入式系统
- **功能**: 电机控制、传感器集成、导航算法
- **实时操作系统**: FreeRTOS用于实时任务管理
- **能力**: 障碍物检测、平衡控制

### 🤖 软件组件 (`Software/`)
- **计算机视觉**: 目标检测算法
- **人工智能集成**: 用于目标识别的机器学习模型
- **导航**: 智能距离计算和运动控制

### 🧠 模型设计 (`Model/`)
- **3D模型**: 连接板的物理设计 (`atlas_card-Body001.stl`)

## 🛠️ 技术栈

| 组件 | 技术 | 描述 |
|------|------|------|
| **微控制器** | STM32 | ARM Cortex-M系列嵌入式控制 |
| **操作系统** | FreeRTOS / Ubuntu22 | 实时操作系统 |
| **开发** | Keil MDK / VsCode | IDE |
| **视觉** | OpenCV | 计算机视觉处理 |
| **人工智能** | Yolov5s 视觉模型 | 目标检测和识别 |
| **通信** | UART / WiFi | 设备通信协议 |

## 🚀 快速开始

### 环境要求
- STM32开发环境 (Keil MDK)
- Python 3.7+ (用于软件组件)
- 所需库 (OpenCV, NumPy等)

### 安装步骤

1. **克隆仓库**
   ```bash
   git clone git@github.com:KingcobraROC/JLU_HUAWEI-Experiment.git
   cd JLU_HUAWEI-Experiment
   ```

2. **硬件设置**
   - 在Keil MDK中打开 `Hardware/` 目录
   - 为特定STM32板配置项目设置
   - 构建并烧录到微控制器

3. **软件设置**
   - 进入 `Software/` 目录
   - 安装所需依赖项
   - 配置摄像头输入

## 📸 项目展示

**pink cube forward:**

<div align='center'>
    
https://github.com/user-attachments/assets/390d4d4b-3fe4-4955-a18c-3e62185675b2

https://github.com/user-attachments/assets/350021f1-3ed4-45e9-9a86-e853747d65fd

</div>

**pink cube backeard:**

<div align='center'>
    
https://github.com/user-attachments/assets/0dd70806-1985-40c0-a31c-0de6ae516084

https://github.com/user-attachments/assets/36b4159f-e1ae-4ec7-9b51-9603651e5ff1

</div>

**pink cube forward right:**

<div align='center'>
    
https://github.com/user-attachments/assets/6c9370f6-1200-44f6-ac24-84420d1c7364

https://github.com/user-attachments/assets/bde503a5-5b78-4632-b105-0fc0c74531f7

</div>

**pink cube tracker:**

<div align='center'>
    
https://github.com/user-attachments/assets/04cee0c6-dfa7-4be5-b723-5524ee6d8ff7

https://github.com/user-attachments/assets/d5d1bf33-516a-409d-a876-fd6d2ea77163

</div>

**yellow cube tracker:**

<div align='center'>
    
https://github.com/user-attachments/assets/814f30e3-345f-4355-ae93-0925c00add43

https://github.com/user-attachments/assets/99078d1e-29b3-49ad-b53b-35ddc5722a28

</div>

## 🤝 贡献指南

欢迎为本项目贡献代码！请遵循以下步骤：

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/新功能`)
3. 提交更改 (`git commit -m '添加新功能'`)
4. 推送到分支 (`git push origin feature/新功能`)
5. 提交 Pull Request

## 📄 许可证

- 详见 [LICENSE](LICENSE) 文件。
