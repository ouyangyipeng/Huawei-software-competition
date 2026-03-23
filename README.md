# 华为软件精英挑战赛 2026 - 初赛解决方案

## 赛题概述

**题目名称**：简单多边形重叠消除 - 最小平移向量计算

**核心问题**：给定两个简单多边形A和B，计算最小平移向量(MTV, Minimum Translation Vector)，使多边形B沿该向量平移后与多边形A完全无重叠。

## 解决方案

### 算法设计

本方案采用**NFP（临界多边形）+ 闵可夫斯基差**算法策略：

1. **预处理阶段（1秒内）**：
   - 检测多边形类型（凸/凹）
   - 凹多边形分解为凸多边形
   - 计算闵可夫斯基差构建NFP

2. **查询阶段**：
   - 判断测试点是否在NFP内部（射线法）
   - 在内部：计算点到NFP边界的最短向量作为MTV
   - 在外部：无重叠，返回(0,0)

### 核心技术

- **闵可夫斯基差**：A - B = {a - b | a∈A, b∈B}，用于构建NFP
- **射线法**：高效判断点是否在多边形内部
- **点到多边形最短距离**：遍历所有边计算投影距离

### 性能对比

| 测试数据 | 顶点数(n1, n2) | V1运行时间 | V2运行时间 | 提升 |
|---------|---------------|-----------|-----------|------|
| practice_1.in | 4, 4 | ~18ms | ~7ms | 2.5x |
| practice_2.in | 6, 4 | ~28ms | ~7ms | 4x |
| practice_3.in | 6, 172 | ~1400ms | ~7ms | **200x** |
| practice_4.in | 17, 12 | ~35ms | ~10ms | 3.5x |
| practice_5.in | 24, 33 | ~74ms | ~20ms | 3.7x |
| practice_6.in | 47, 50 | ~200ms | ~11ms | 18x |
| practice_7.in | 17, 15 | ~119ms | ~7ms | 17x |

**关键优化**：practice_3从1.4秒降到7毫秒，性能提升200倍！

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
│   ├── Solution.cpp       # V1版本
│   └── Solution_v2.cpp     # V2优化版本
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

1. **NFP预处理**：利用1秒预处理时间计算闵可夫斯基差
2. **O(1)查询**：点在多边形内判断和距离计算
3. **精度控制**：EPS=1e-6确保计算精度
4. **IO优化**：使用'\n'替代endl，显式flush

## 作者

ouyangyipeng

## 许可证

本项目仅用于2026华为软件精英挑战赛参赛使用。
