# CherryFC_F103 🍒🚀

### 掌控天空，释放无限可能！

---

[![GitHub license](https://img.shields.io/github/license/your_username/CherryFC_F103?style=for-the-badge)](https://github.com/your_username/CherryFC_F103/blob/main/LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/your_username/CherryFC_F103?style=for-the-badge)](https://github.com/your_username/CherryFC_F103/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/your_username/CherryFC_F103?style=for-the-badge)](https://github.com/your_username/CherryFC_F103/network/members)
[![GitHub issues](https://img.shields.io/github/issues/your_username/CherryFC_F103?style=for-the-badge)](https://github.com/your_username/CherryFC_F103/issues)

---

## ✨ 项目简介

欢迎来到 **CherryFC** 的天空实验室。这是一个基于最便宜且随处可见的 **STM32F103** 微控制器所打造的返祖廉价飞行控制器。我致力于提供一个稳定且高度可定制的平台，让你的低成本无人机梦想翱翔天际，无论是竞速无人机、航拍平台还是创新型飞行器，都能拥有卓越（能飞就行）的表现。

问题仍然存在，ELRS接收这部分有严重问题，我马上通过外接接收机解决。

Forget the ordinary. Embrace the extraordinary. 💫

## 🌟 核心特性

*   **macOS兼容**：兼容性，可以用STMcubeIDE开发。
*   **模块化设计**：清晰的代码结构,能删的都删了。
*   **丰富接口**：支持多种传感器（陀螺仪、加速度计、磁力计等）、GPS、遥控器协议（SBUS, PPM, DSMX）以及更多外部模块。
*   **智能飞行模式**：正在开发。
*   **实时调试**：便捷的串口输出与数据可视化支持，助你轻松调优。
*   **开源共享**：坚信开源的力量，欢迎社区的每一次贡献和创新！

## 🛠️ 快速开始

### 🚀 前置条件

在开始你的 CherryFC 之旅前，请确保你已具备以下环境：

*   **STM32CubeIDE** 或其他兼容的 ARM GCC 开发环境
*   **ST-Link/J-Link** 调试器
*   **Python 3.x** (用于地面站或数据分析工具)
*   **也是我的开源飞控硬件（立创开源广场见）** 

### ⚙️ 环境配置

1.  **克隆仓库**：

    ```bash
    git clone https://github.com/your_username/CherryFC_F103.git
    cd CherryFC
    ```

2.  **打开项目**：
    使用 STM32CubeIDE 导入项目文件。

3.  **编译与烧录**：
    -   根据你的硬件配置调整 `config.h` 。
    -   编译项目。
    -   通过调试器将固件烧录到你的 PCB 。

### 📡 连接地面站 (可选)

我们正在开发一个功能强大的地面站，用于实时监控、参数配置和固件升级。敬请期待！✨

## 📚 使用指南

一旦固件烧录成功，你可以开始连接你的传感器、遥控器和动力系统。详细的接线图与参数配置手册将在 [`docs/`](./docs/) 文件夹中提供（即将发布）。

## 🤝 贡献

我欢迎所有形式的贡献！无论是代码改进、Bug 修复、新功能实现、文档优化还是提供宝贵建议，你的每一次参与都将让 CherryFC 变得更强大。

请查阅我们的 [CONTRIBUTING.md](CONTRIBUTING.md) 文件了解更多细节。

## 📜 许可证

本项目采用 **MIT 许可证**。详情请参阅 [LICENSE](LICENSE) 文件。

## 📞 联系我们

有任何问题、建议或只是想打个招呼？欢迎通过以下方式联系我们：

*   **GitHub Issues**：[在此提交你的问题](https://github.com/ludwekin/CherryFC/issues)
*   **电子邮件**：[welfarewangyifei@gmail.com](mailto:welfarewangyifei@gmail.com)

感谢你对 CherryFC 的关注与支持！

---

Made with ❤️ by Benjamin.