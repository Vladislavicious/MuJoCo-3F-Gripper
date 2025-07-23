# MuJoCo UR5e with Robotiq 2F85 and 3F Grippers

## 项目介绍
本项目基于MuJoCo仿真环境，实现了UR5e机械臂与Robotiq 2F85（两指夹爪）、Robotiq 3F（三指夹爪）的集成。  
- 针对2F85夹爪的示例代码：`main2f.py`  
- 针对3F夹爪的示例代码：`main3f.py`  


## 环境安装教程

### 推荐配置（已验证）
- 操作系统：Ubuntu 20.04  
- Python 版本：3.8  


### 依赖安装
通过`requirements.txt`安装所需依赖包：  
```bash
pip install -r requirements.txt
```

依赖清单如下：
```
matplotlib==3.7.5              # 数据可视化库
modern_robotics==1.1.1         # 现代机器人学算法库
mujoco==3.2.3                  # MuJoCo物理引擎核心库
numpy==1.23.4                  # 数值计算基础库
pandas==2.0.3                  # 数据处理库
roboticstoolbox_python==1.1.1  # 机器人工具箱
scipy==1.16.0                  # 科学计算库
spatialmath_python==1.1.14     # 空间数学运算库
```


## 使用说明

1. **运行2F85夹爪示例**：  
   ```bash
   python main2f.py
   ```

2. **运行3F夹爪示例**：  
   ```bash
   python main3f.py
   ```


## 致谢
本项目开发利用了以下开源资源：
- `manipulator_grasp`（Gitee）：https://gitee.com/chaomingsanhua/manipulator_grasp  
- `MuJoCo_RL_UR5`（GitHub）：https://github.com/PaulDanielML/MuJoCo_RL_UR5