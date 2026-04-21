# GM6020 PID 仿真器

一个面向入门学习者的 GM6020 电机 PID 网页仿真工具。  
项目重点是帮助 STM32 电机控制初学者理解「位置环 + 速度环」的调参过程与响应变化。

## 功能概览

- 支持两种模式：
  - `静态模式`：一次性计算并绘制完整响应
  - `实时模式`：曲线随时间滚动更新，支持在线修改目标值
- 支持两种控制结构：
  - `仅速度环`
  - `角度环 + 速度环（串级）`
- 支持图表交互：
  - 鼠标悬停显示当前点的时间与数值
  - 实时模式固定时间窗口滚动显示
- 支持教学常用参数调节：
  - PID 参数、输出限幅、积分限幅
  - 采样周期、机械时间常数、负载扰动、测量噪声等

## 快速开始

本项目是纯前端页面，无需构建工具：

1. 克隆或下载仓库  
2. 直接用浏览器打开 `index.html`

示例路径：  
[index.html](/C:/BBBBBBBHAPPYBBBBBBB/VScode/MotorPIDSimulation/simulator/index.html)

## 主要模型参数（默认）

基于 GM6020 手册关键参数建立教学化简模型：

- 额定电压：`24V`
- 转矩常数：`0.741 N·m/A`
- 电流映射：`±16384 -> ±3A`
- 空载最高转速：`320 rpm`
- 机械时间常数：`3 ms`
- 转速转矩梯度：`156 rpm/(N·m)`

## 工程结构

- [index.html](/C:/BBBBBBBHAPPYBBBBBBB/VScode/MotorPIDSimulation/simulator/index.html)：页面结构与控件
- [styles.css](/C:/BBBBBBBHAPPYBBBBBBB/VScode/MotorPIDSimulation/simulator/styles.css)：界面样式
- [app.js](/C:/BBBBBBBHAPPYBBBBBBB/VScode/MotorPIDSimulation/simulator/app.js)：仿真模型、PID 计算、图表绘制与实时逻辑

## 模型说明（简化）

该项目是教学导向的简化仿真，不追求完整电机电磁细节：

- 电机与负载被建模为一阶惯性响应
- 输出电流受限幅影响并映射为电磁转矩
- 支持恒定负载与阶跃扰动负载
- 角度由转速积分得到

## 已知限制

- 当前模型未显式引入电枢电感、电阻、电压反电动势完整方程
- 参数默认值为教学起点，不等于所有实机工况
- 仿真结果应作为调参趋势参考，不能直接替代实机闭环验证

## 适用人群

- 正在学习 STM32 电机控制与 CAN 电机闭环控制的同学
- 希望先在网页里快速理解 PID 参数影响，再上板调试的开发者

## License
