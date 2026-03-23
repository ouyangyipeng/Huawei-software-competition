#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// 常量定义
const double EPS = 1e-9;
const double PI = 3.14159265358979323846;

// 2D向量结构体
struct Vector2D {
    double x, y;

    Vector2D(double x = 0, double y = 0) : x(x), y(y) {}

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator*(double scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    // 点积
    double Dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }

    // 叉积 (2D叉积返回标量)
    double Cross(const Vector2D& other) const {
        return x * other.y - y * other.x;
    }

    // 向量长度
    double Length() const {
        return std::sqrt(x * x + y * y);
    }

    // 归一化
    Vector2D Normalize() const {
        double len = Length();
        if (len < EPS) {
            return *this;
        }
        return Vector2D(x / len, y / len);
    }

    // 垂直向量 (逆时针旋转90度)
    Vector2D Perp() const {
        return Vector2D(-y, x);
    }
};

// 多边形结构体
struct Polygon {
    std::vector<Vector2D> vertices;
    bool isConvex;
    std::vector<Vector2D> edgeNormals;  // 预计算的边法向量

    Polygon() : isConvex(true) {}

    // 获取多边形中心
    Vector2D GetCenter() const {
        Vector2D center(0, 0);
        if (vertices.empty()) {
            return center;
        }
        for (const auto& v : vertices) {
            center = center + v;
        }
        return center * (1.0 / vertices.size());
    }

    // 按向量平移
    void MoveByVec(const Vector2D& vec) {
        for (auto& v : vertices) {
            v = v + vec;
        }
    }

    // 检测是否为凸多边形
    bool CheckConvex() {
        if (vertices.size() < 3) {
            return true;
        }

        int n = vertices.size();
        int sign = 0;

        for (int i = 0; i < n; ++i) {
            Vector2D v1 = vertices[(i + 1) % n] - vertices[i];
            Vector2D v2 = vertices[(i + 2) % n] - vertices[(i + 1) % n];
            double cross = v1.Cross(v2);

            if (std::abs(cross) > EPS) {
                if (sign == 0) {
                    sign = cross > 0 ? 1 : -1;
                } else if ((cross > 0 ? 1 : -1) != sign) {
                    isConvex = false;
                    return false;
                }
            }
        }

        isConvex = true;
        return true;
    }

    // 预计算边法向量
    void PrecomputeNormals() {
        edgeNormals.clear();
        int n = vertices.size();
        for (int i = 0; i < n; ++i) {
            Vector2D edge = vertices[(i + 1) % n] - vertices[i];
            edgeNormals.push_back(edge.Perp().Normalize());
        }
    }
};

// 投影结构体
struct Projection {
    double min, max;

    bool Overlaps(const Projection& other) const {
        return !(max < other.min - EPS || other.max < min - EPS);
    }

    double GetOverlap(const Projection& other) const {
        return std::min(max, other.max) - std::max(min, other.min);
    }
};

// 全局变量
int n1 = 0, n2 = 0, m = 0;
Polygon polygon1, polygon2;
std::vector<Vector2D> testCases;

// 计算多边形在轴上的投影
Projection ProjectPolygon(const Polygon& poly, const Vector2D& axis) {
    double minProj = poly.vertices[0].Dot(axis);
    double maxProj = minProj;

    for (size_t i = 1; i < poly.vertices.size(); ++i) {
        double proj = poly.vertices[i].Dot(axis);
        if (proj < minProj) {
            minProj = proj;
        }
        if (proj > maxProj) {
            maxProj = proj;
        }
    }
    return {minProj, maxProj};
}

// 使用SAT算法计算两个凸多边形的MTV
Vector2D ComputeMTVConvex(const Polygon& A, const Polygon& B) {
    double minOverlap = std::numeric_limits<double>::infinity();
    Vector2D smallestAxis;

    // 检查A的所有边
    int nA = A.vertices.size();
    for (int i = 0; i < nA; ++i) {
        Vector2D axis = A.edgeNormals[i];

        Projection projA = ProjectPolygon(A, axis);
        Projection projB = ProjectPolygon(B, axis);

        // 如果存在分离轴，返回零向量
        if (projA.max <= projB.min + EPS || projB.max <= projA.min + EPS) {
            return {0.0, 0.0};
        }

        double overlap = projA.GetOverlap(projB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    // 检查B的所有边
    int nB = B.vertices.size();
    for (int i = 0; i < nB; ++i) {
        Vector2D axis = B.edgeNormals[i];

        Projection projA = ProjectPolygon(A, axis);
        Projection projB = ProjectPolygon(B, axis);

        if (projA.max <= projB.min + EPS || projB.max <= projA.min + EPS) {
            return {0.0, 0.0};
        }

        double overlap = projA.GetOverlap(projB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    // 确保MTV方向正确（从A指向B）
    Vector2D centerA = A.GetCenter();
    Vector2D centerB = B.GetCenter();
    Vector2D dir = centerB - centerA;

    if (smallestAxis.Dot(dir) < 0.0) {
        smallestAxis = smallestAxis * -1.0;
    }

    return smallestAxis * minOverlap;
}

// 凹多边形分解 - 使用对角线分割
std::vector<Polygon> DecomposePolygon(const Polygon& poly) {
    std::vector<Polygon> result;
    
    if (poly.vertices.size() <= 3) {
        result.push_back(poly);
        return result;
    }

    // 简单的耳切法分解
    std::vector<Vector2D> remaining = poly.vertices;
    std::vector<bool> removed(remaining.size(), false);
    
    int maxIterations = remaining.size() * 2;
    int iterations = 0;
    
    while (remaining.size() > 3 && iterations < maxIterations) {
        iterations++;
        bool found = false;
        
        for (int i = 0; i < (int)remaining.size(); ++i) {
            if (removed[i]) continue;
            
            int prev = -1, next = -1;
            for (int j = 1; j < (int)remaining.size(); ++j) {
                if (!removed[(i - j + remaining.size()) % remaining.size()]) {
                    prev = (i - j + remaining.size()) % remaining.size();
                    break;
                }
            }
            for (int j = 1; j < (int)remaining.size(); ++j) {
                if (!removed[(i + j) % remaining.size()]) {
                    next = (i + j) % remaining.size();
                    break;
                }
            }
            
            if (prev == -1 || next == -1) continue;
            
            Vector2D v1 = remaining[prev];
            Vector2D v2 = remaining[i];
            Vector2D v3 = remaining[next];
            
            Vector2D edge1 = v2 - v1;
            Vector2D edge2 = v3 - v2;
            double cross = edge1.Cross(edge2);
            
            // 检查是否是凸角
            if (cross > EPS) {
                Polygon triangle;
                triangle.vertices.push_back(v1);
                triangle.vertices.push_back(v2);
                triangle.vertices.push_back(v3);
                triangle.isConvex = true;
                triangle.PrecomputeNormals();
                result.push_back(triangle);
                
                removed[i] = true;
                found = true;
                break;
            }
        }
        
        if (!found) break;
    }
    
    // 添加剩余的顶点形成的多边形
    Polygon last;
    for (int i = 0; i < (int)remaining.size(); ++i) {
        if (!removed[i]) {
            last.vertices.push_back(remaining[i]);
        }
    }
    if (last.vertices.size() >= 3) {
        last.CheckConvex();
        last.PrecomputeNormals();
        result.push_back(last);
    }
    
    return result;
}

// 计算两个多边形的MTV（支持凹多边形）
Vector2D ComputeMTV(const Polygon& A, const Polygon& B) {
    // 如果两个都是凸多边形，直接使用SAT
    if (A.isConvex && B.isConvex) {
        return ComputeMTVConvex(A, B);
    }

    // 分解凹多边形
    std::vector<Polygon> polysA, polysB;
    
    if (A.isConvex) {
        polysA.push_back(A);
    } else {
        polysA = DecomposePolygon(A);
    }
    
    if (B.isConvex) {
        polysB.push_back(B);
    } else {
        polysB = DecomposePolygon(B);
    }

    // 计算所有子多边形对的MTV
    Vector2D bestMTV(0.0, 0.0);
    double minLen = std::numeric_limits<double>::infinity();
    bool hasOverlap = false;

    for (const auto& subA : polysA) {
        for (const auto& subB : polysB) {
            Vector2D mtv = ComputeMTVConvex(subA, subB);
            double len = mtv.Length();
            
            if (len > EPS) {
                hasOverlap = true;
                if (len < minLen) {
                    minLen = len;
                    bestMTV = mtv;
                }
            }
        }
    }

    return hasOverlap ? bestMTV : Vector2D(0.0, 0.0);
}

// 生成解决方案
Vector2D GenSolution(const Vector2D& vec) {
    Polygon polyB = polygon2;
    polyB.MoveByVec(vec);
    polyB.PrecomputeNormals();
    
    return ComputeMTV(polygon1, polyB);
}

// 预处理函数
void PreProcess() {
    // 检测多边形类型
    polygon1.CheckConvex();
    polygon2.CheckConvex();
    
    // 预计算边法向量
    polygon1.PrecomputeNormals();
    polygon2.PrecomputeNormals();
}

int main() {
    // 优化IO
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    // =============== 1. 读取多边形数据 ===================
    std::cin >> n1 >> n2;

    if (n1 <= 0 || n2 <= 0) {
        std::cerr << "Input data error: the number of vertices should be positive" << std::endl;
        return 1;
    }

    polygon1.vertices.resize(n1);
    for (int i = 0; i < n1; ++i) {
        std::cin >> polygon1.vertices[i].x >> polygon1.vertices[i].y;
    }

    polygon2.vertices.resize(n2);
    for (int i = 0; i < n2; ++i) {
        std::cin >> polygon2.vertices[i].x >> polygon2.vertices[i].y;
    }

    // 等待OK确认多边形数据已接收
    std::string okResp;
    std::cin >> okResp;
    if (okResp != "OK") {
        std::cerr << "Input data error: waiting for OK after polygons but got " << okResp << std::endl;
        return 1;
    }

    // ============== 2. 预处理 ===================
    PreProcess();
    std::cout << "OK" << std::endl;
    std::cout.flush();

    // ============== 3. 读取测试数据 ===================
    std::cin >> m;
    testCases.resize(m);

    for (int i = 0; i < m; ++i) {
        std::cin >> testCases[i].x >> testCases[i].y;
    }

    // 等待OK确认所有测试数据已接收
    std::cin >> okResp;
    if (okResp != "OK") {
        std::cerr << "Input data error: waiting for OK after test points but got " << okResp << std::endl;
        return 1;
    }

    // ================ 4. 求解并输出结果 ===================
    for (int i = 0; i < m; ++i) {
        Vector2D res = GenSolution(testCases[i]);
        std::cout << std::fixed << std::setprecision(5) << res.x << " " << res.y << std::endl;
        std::cout.flush();
    }

    // 输出OK表示所有答案已输出
    std::cout << "OK" << std::endl;
    std::cout.flush();

    return 0;
}