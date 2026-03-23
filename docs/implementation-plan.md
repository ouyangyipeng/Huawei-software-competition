# 实现计划

## 项目结构

```
huawei-sft/
├── PROGRESS.md           # 进度记录
├── README.md             # 项目说明
├── docs/                 # 文档目录
│   ├── algorithm-analysis.md
│   └── implementation-plan.md
├── Linux/                # 官方提供的资源
│   ├── Interactor
│   ├── Runner
│   ├── data/
│   └── demos/
└── src/                  # 源代码目录
    └── Solution.cpp      # 最终提交的解决方案
```

## 技术选型

### 语言选择：C++

**理由**：
1. 性能最优，适合计算密集型任务
2. SAT算法在C++中实现效率最高
3. 内存控制更精确
4. 官方示例代码最完整

## 算法实现策略

### 阶段一：基础实现（凸多边形）

1. **向量运算库**
   - Vector2D结构体
   - 点积、叉积、长度、归一化
   - 向量加减乘除

2. **多边形表示**
   - Polygon结构体
   - 顶点存储
   - 中心点计算
   - 平移操作

3. **SAT算法实现**
   - 投影计算
   - 分离轴检测
   - MTV计算

### 阶段二：凹多边形支持

1. **凹多边形检测**
   - 计算内角
   - 判断凸凹性

2. **凹多边形分解**（如果需要）
   - Hertel-Mehlhorn算法
   - 将凹多边形分解为凸多边形

3. **多MTV合并**
   - 计算所有凸多边形对的MTV
   - 选择最优解

### 阶段三：性能优化

1. **预处理优化**
   - 预计算边法向量
   - 预计算包围盒

2. **查询优化**
   - 提前终止条件
   - 缓存友好访问

3. **精度控制**
   - EPS常量设置
   - 浮点数比较

## 详细实现步骤

### Step 1: 基础框架

```cpp
// 常量定义
const double EPS = 1e-6;
const double INF = 1e18;

// 2D向量
struct Vector2D {
    double x, y;
    // 运算符重载
    // 基本运算方法
};

// 多边形
struct Polygon {
    vector<Vector2D> vertices;
    // 辅助方法
};

// 投影
struct Projection {
    double min, max;
};
```

### Step 2: SAT核心算法

```cpp
// 计算多边形在轴上的投影
Projection ProjectPolygon(const Polygon& poly, const Vector2D& axis);

// 检测重叠并计算MTV
Vector2D ComputeMTV(const Polygon& A, const Polygon& B);
```

### Step 3: 凹多边形处理

```cpp
// 检测多边形是否为凸
bool IsConvex(const Polygon& poly);

// 分解凹多边形
vector<Polygon> Decompose(const Polygon& poly);

// 计算凹多边形的MTV
Vector2D ComputeMTVConcave(const Polygon& A, const Polygon& B);
```

### Step 4: 主程序流程

```cpp
int main() {
    // 1. 读取多边形数据
    // 2. 预处理
    // 3. 输出OK
    // 4. 读取测试样本
    // 5. 计算并输出MTV
    // 6. 输出OK
}
```

## 测试计划

### 单元测试

1. 向量运算测试
2. 投影计算测试
3. SAT算法测试（凸多边形）
4. 凹多边形检测测试
5. 凹多边形MTV测试

### 集成测试

使用所有practice数据进行测试：
- practice_1.in ~ practice_7.in
- 验证输出格式
- 检查精度要求

### 性能测试

- 预处理时间 < 1秒
- 单个查询时间 < 60ms（10000个查询/10分钟）

## 提交清单

1. Solution.cpp - 源代码文件
2. 打包为zip格式
3. 确保文件名为Solution.cpp

## 风险与应对

| 风险 | 应对措施 |
|-----|---------|
| 凹多边形分解复杂 | 先实现基础SAT，再逐步优化 |
| 预处理超时 | 简化预处理，延迟计算 |
| 精度问题 | 使用double，设置合适EPS |
| 内存超限 | 避免不必要的拷贝 |

## 时间规划

1. 基础框架搭建
2. SAT算法实现
3. 凹多边形支持
4. 测试验证
5. 性能优化
6. 最终提交