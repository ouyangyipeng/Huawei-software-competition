# 华为软件精英挑战赛 2026 - 初赛解决方案

## 赛题概述

**题目名称**：简单多边形重叠消除 - 最小平移向量计算

**核心问题**：给定两个简单多边形A和B，计算最小平移向量(MTV, Minimum Translation Vector)，使多边形B沿该向量平移后与多边形A完全无重叠。

## 解决方案

### 算法设计

本方案采用混合算法策略：

1. **凸多边形检测**：预处理阶段检测多边形类型
2. **SAT算法**：对凸多边形使用分离轴定理计算MTV
3. **凹多边形分解**：使用耳切法将凹多边形分解为凸多边形
4. **MTV合并**：对分解后的子多边形计算MTV并选择最优解

### 核心技术

- **分离轴定理(SAT)**：高效计算凸多边形碰撞检测和MTV
- **耳切法分解**：将凹多边形分解为三角形
- **预处理优化**：预计算边法向量，减少运行时计算

### 性能表现

| 测试数据 | 顶点数(n1, n2) | 测试样本数 | 运行时间 |
|---------|---------------|-----------|---------|
| practice_1.in | 4, 4 | 10000 | ~18ms |
| practice_2.in | 6, 4 | 10000 | ~28ms |
| practice_3.in | 6, 172 | 10000 | ~1.4s |
| practice_4.in | 17, 12 | 10000 | ~35ms |
| practice_5.in | 24, 33 | 10000 | ~74ms |
| practice_6.in | 47, 50 | 10000 | ~200ms |
| practice_7.in | 17, 15 | 10000 | ~119ms |

## 项目结构

```
huawei-sft/
├── README.md              # 项目说明
├── PROGRESS.md            # 进度记录
├── Solution.cpp           # 最终提交的解决方案
├── Solution.zip           # 提交的压缩包
├── docs/                  # 文档目录
│   ├── algorithm-analysis.md
│   └── implementation-plan.md
├── src/                   # 源代码目录
│   └── Solution.cpp
└── Linux/                 # 官方提供的资源
    ├── Interactor
    ├── Runner
    ├── data/
    └── demos/
```

## 编译运行

### 编译命令
```bash
g++ -std=c++11 Solution.cpp -o Solution -O2 -lpthread
```

### 运行测试
```bash
cd Linux
bash run_and_test.sh ../Solution.cpp ./data/practice_1.in ./data/practice_1.out
```

## 技术要点

1. **精度控制**：使用EPS=1e-9确保计算精度
2. **预处理时间**：在1秒内完成多边形类型检测和法向量预计算
3. **内存优化**：避免不必要的拷贝，使用引用传递
4. **IO优化**：使用sync_with_stdio(false)加速输入输出

## 作者

ouyangyipeng

## 许可证

本项目仅用于2026华为软件精英挑战赛参赛使用。
