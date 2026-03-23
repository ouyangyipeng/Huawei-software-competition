# V3版本优化计划

## 问题分析

### V2版本的三个致命Bug

#### Bug 1: 抛弃了99%的NFP数据
```cpp
// 错误代码
if (!minkowskiDiffs.empty()) {
    nfpVertices = minkowskiDiffs[0]; // 只保留第一个！
}
```
**问题**：计算了所有子多边形对的闵可夫斯基差，但只用了第一个，导致大部分碰撞检测失效。

#### Bug 2: 错误的闵可夫斯基差计算
```cpp
// 错误方法：极角排序
std::sort(edges.begin(), edges.end(), ...);
```
**问题**：极角排序法只在起始点正确时才成立，否则会产生自交和位移。

#### Bug 3: 残缺的耳切法分解
```cpp
// 错误代码
if (cross > EPS) { // 只判断凸角
    // 没有检查三角形内是否包含其他顶点
}
```
**问题**：凸角三角形内部可能包含其他顶点，导致分解错误。

## 解决方案：Union-Free NFP

### 核心思想

1. **不需求并集**：避免复杂的布尔运算
2. **凸包方法**：用凸包计算闵可夫斯基差，保证正确性
3. **逃逸向量**：遍历所有候选点找最优解

### 算法流程

```
预处理阶段：
1. 分解凹多边形为凸多边形（修复耳切法）
2. 对每对凸多边形计算闵可夫斯基差（用凸包方法）
3. 存储所有M_ij

查询阶段：
1. 判断点t是否在任意M_ij内部
2. 如果在内部（有重叠）：
   a. 收集所有M_ij边上的投影点
   b. 收集所有M_ij边缘的交点
   c. 找到不在任何M_ij内部且距离t最近的点
   d. 该点-t就是MTV
3. 如果在外部（无重叠）：返回(0,0)
```

## 详细实现计划

### 1. 修复耳切法分解

```cpp
// 检查点是否在三角形内
bool PointInTriangle(const Vector2D& p, const Vector2D& a, 
                     const Vector2D& b, const Vector2D& c);

// 修复后的耳切法
vector<Polygon> DecomposePolygon(const Polygon& poly) {
    // 1. 找凸角
    // 2. 检查三角形内无其他顶点
    // 3. 切割并继续
}
```

### 2. 正确的闵可夫斯基差（凸包方法）

```cpp
// 计算凸包（Andrew算法）
vector<Vector2D> ConvexHull(vector<Vector2D>& points);

// 闵可夫斯基差：A - B
vector<Vector2D> MinkowskiDifference(const Polygon& A, const Polygon& B) {
    // 1. 对B取反得到-B
    // 2. 计算所有顶点对的差：{a - b | a∈A, b∈B}
    // 3. 求这些点的凸包
    return ConvexHull(differencePoints);
}
```

### 3. Union-Free NFP查询

```cpp
// 判断点是否在任意M_ij内部
bool IsInAnyMinkowski(const Vector2D& p, 
                       const vector<vector<Vector2D>>& minkowskis);

// 计算逃逸向量
Vector2D ComputeEscapeVector(const Vector2D& t,
                              const vector<vector<Vector2D>>& minkowskis) {
    vector<Vector2D> candidates;
    
    // 1. 收集所有边上的投影点
    for (const auto& m : minkowskis) {
        for (int i = 0; i < m.size(); ++i) {
            Vector2D proj = ProjectToSegment(t, m[i], m[(i+1)%m.size()]);
            candidates.push_back(proj);
        }
    }
    
    // 2. 收集边的交点（可选，用于处理凹陷情况）
    // ...
    
    // 3. 找最近的有效逃逸点
    Vector2D best;
    double minDist = INF;
    for (const auto& c : candidates) {
        if (!IsInAnyMinkowski(c, minkowskis)) {
            double d = (c - t).Length();
            if (d < minDist) {
                minDist = d;
                best = c;
            }
        }
    }
    
    return best - t;
}
```

### 4. SAT作为回退方案

对于简单凸多边形情况，SAT仍然是最优解：
- 保持V1的SAT实现
- 只对凹多边形使用NFP方法

## 其他优化

### 1. 空间索引加速
- 对M_ij构建空间索引（如R树）
- 加速点在多边形内的判断

### 2. 缓存优化
- 预计算所有M_ij的包围盒
- 先用包围盒快速排除

### 3. 数值稳定性
- 使用更精确的浮点比较
- 处理边界情况（点在边上）

## 实现优先级

1. **高优先级**：修复闵可夫斯基差计算（凸包方法）
2. **高优先级**：保留所有M_ij，不丢弃数据
3. **中优先级**：修复耳切法分解
4. **中优先级**：实现逃逸向量计算
5. **低优先级**：空间索引优化

## 预期效果

- 正确率：从45665分恢复到18万+
- 性能：保持V2的查询速度
- 最终目标：30万+分