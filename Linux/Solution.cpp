#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// 常量定义
const double EPS = 1e-9;
const double MTV_EPSILON = 1e-5;  // 适中的epsilon，平衡分离率和向量长度

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

    double Dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }

    double Cross(const Vector2D& other) const {
        return x * other.y - y * other.x;
    }

    double Length() const {
        return std::sqrt(x * x + y * y);
    }

    Vector2D Normalize() const {
        double len = Length();
        if (len < EPS) return *this;
        return Vector2D(x / len, y / len);
    }

    Vector2D Perp() const {
        return Vector2D(-y, x);
    }
};

// 多边形结构体
struct Polygon {
    std::vector<Vector2D> vertices;

    Vector2D GetCenter() const {
        Vector2D center(0, 0);
        for (const auto& v : vertices) {
            center = center + v;
        }
        return center * (1.0 / vertices.size());
    }

    void MoveByVec(const Vector2D& vec) {
        for (auto& v : vertices) {
            v = v + vec;
        }
    }
};

// 全局变量
int n1, n2, m;
Polygon polygon1, polygon2;
std::vector<Vector2D> testCases;

// 检查点是否在线段上
bool PointOnSegment(const Vector2D& p, const Vector2D& a, const Vector2D& b) {
    double cross = (p - a).Cross(b - a);
    if (std::abs(cross) > EPS) return false;
    
    double minX = std::min(a.x, b.x);
    double maxX = std::max(a.x, b.x);
    double minY = std::min(a.y, b.y);
    double maxY = std::max(a.y, b.y);
    
    return p.x >= minX - EPS && p.x <= maxX + EPS && 
           p.y >= minY - EPS && p.y <= maxY + EPS;
}

// 检查点是否在多边形内（包括边界）
bool PointInPolygon(const Vector2D& point, const Polygon& poly) {
    int n = poly.vertices.size();
    
    // 首先检查是否在边界上
    for (int i = 0; i < n; ++i) {
        if (PointOnSegment(point, poly.vertices[i], poly.vertices[(i + 1) % n])) {
            return true;
        }
    }
    
    // 射线法
    int count = 0;
    for (int i = 0; i < n; ++i) {
        Vector2D p1 = poly.vertices[i];
        Vector2D p2 = poly.vertices[(i + 1) % n];
        
        if (p1.y > p2.y) {
            Vector2D tmp = p1;
            p1 = p2;
            p2 = tmp;
        }
        
        if (p1.y - EPS <= point.y && point.y < p2.y - EPS) {
            double x = p1.x + (point.y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x);
            if (x > point.x + EPS) {
                count++;
            }
        }
    }
    
    return count % 2 == 1;
}

// 检查两条线段是否相交（包括端点接触）
bool SegmentsIntersect(const Vector2D& p1, const Vector2D& p2,
                       const Vector2D& p3, const Vector2D& p4) {
    auto ccw = [](const Vector2D& A, const Vector2D& B, const Vector2D& C) {
        double cross = (C.y - A.y) * (B.x - A.x) - (B.y - A.y) * (C.x - A.x);
        return cross > EPS;
    };
    
    // 检查端点是否在另一条线段上
    if (PointOnSegment(p1, p3, p4) || PointOnSegment(p2, p3, p4) ||
        PointOnSegment(p3, p1, p2) || PointOnSegment(p4, p1, p2)) {
        return true;
    }
    
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) && ccw(p1, p2, p3) != ccw(p1, p2, p4);
}

// 检查两个多边形是否重叠
bool PolygonsOverlap(const Polygon& A, const Polygon& B) {
    // 检查A的顶点是否在B内（包括边界）
    for (const auto& v : A.vertices) {
        if (PointInPolygon(v, B)) return true;
    }
    
    // 检查B的顶点是否在A内（包括边界）
    for (const auto& v : B.vertices) {
        if (PointInPolygon(v, A)) return true;
    }
    
    // 检查边是否相交
    for (size_t i = 0; i < A.vertices.size(); ++i) {
        for (size_t j = 0; j < B.vertices.size(); ++j) {
            if (SegmentsIntersect(
                A.vertices[i], A.vertices[(i + 1) % A.vertices.size()],
                B.vertices[j], B.vertices[(j + 1) % B.vertices.size()])) {
                return true;
            }
        }
    }
    
    return false;
}

// 投影结构体
struct Projection {
    double min, max;
};

// 将多边形投影到轴上
Projection ProjectPolygon(const Polygon& poly, const Vector2D& axis) {
    double minProj = poly.vertices[0].Dot(axis);
    double maxProj = minProj;

    for (size_t i = 1; i < poly.vertices.size(); ++i) {
        double proj = poly.vertices[i].Dot(axis);
        if (proj < minProj) minProj = proj;
        if (proj > maxProj) maxProj = proj;
    }
    return {minProj, maxProj};
}

// 使用SAT计算MTV
Vector2D ComputeMTV_SAT(const Polygon& A, const Polygon& B) {
    double minOverlap = std::numeric_limits<double>::infinity();
    Vector2D smallestAxis;

    const Polygon* polygons[2] = {&A, &B};

    for (int i = 0; i < 2; ++i) {
        const Polygon& currentPoly = *polygons[i];
        for (size_t j = 0; j < currentPoly.vertices.size(); ++j) {
            Vector2D p1 = currentPoly.vertices[j];
            Vector2D p2 = currentPoly.vertices[(j + 1) % currentPoly.vertices.size()];
            Vector2D edge = p2 - p1;
            Vector2D axis = edge.Perp().Normalize();

            Projection projA = ProjectPolygon(A, axis);
            Projection projB = ProjectPolygon(B, axis);

            // 如果存在分离轴，返回零向量
            if (projA.max <= projB.min + EPS || projB.max <= projA.min + EPS) {
                return {0.0, 0.0};
            }

            double overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);

            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }
    }

    // 确保MTV方向正确（从A指向B，即B需要移动的方向）
    Vector2D centerA = A.GetCenter();
    Vector2D centerB = B.GetCenter();
    Vector2D dir = centerB - centerA;

    if (smallestAxis.Dot(dir) < 0.0) {
        smallestAxis = smallestAxis * -1.0;
    }

    // 添加epsilon确保分离
    return smallestAxis * (minOverlap + MTV_EPSILON);
}

// 使用迭代方法计算精确的MTV
Vector2D ComputeMTV_Iterative(const Polygon& A, const Polygon& B) {
    if (!PolygonsOverlap(A, B)) {
        return {0.0, 0.0};
    }
    
    // 收集所有可能的逃逸方向
    std::vector<Vector2D> candidates;
    
    // 1. 从B的中心指向A的中心的反方向
    Vector2D centerA = A.GetCenter();
    Vector2D centerB = B.GetCenter();
    Vector2D escapeDir = centerB - centerA;
    if (escapeDir.Length() > EPS) {
        candidates.push_back(escapeDir.Normalize());
    }
    
    // 2. 每条边的法向量
    for (size_t i = 0; i < A.vertices.size(); ++i) {
        Vector2D p1 = A.vertices[i];
        Vector2D p2 = A.vertices[(i + 1) % A.vertices.size()];
        Vector2D edge = p2 - p1;
        Vector2D normal = edge.Perp().Normalize();
        candidates.push_back(normal);
        candidates.push_back(normal * -1.0);
    }
    
    for (size_t i = 0; i < B.vertices.size(); ++i) {
        Vector2D p1 = B.vertices[i];
        Vector2D p2 = B.vertices[(i + 1) % B.vertices.size()];
        Vector2D edge = p2 - p1;
        Vector2D normal = edge.Perp().Normalize();
        candidates.push_back(normal);
        candidates.push_back(normal * -1.0);
    }
    
    // 3. 顶点到顶点的方向
    for (const auto& va : A.vertices) {
        for (const auto& vb : B.vertices) {
            Vector2D dir = vb - va;
            if (dir.Length() > EPS) {
                candidates.push_back(dir.Normalize());
            }
        }
    }
    
    // 尝试每个候选方向，找到最短的逃逸向量
    double bestLen = std::numeric_limits<double>::infinity();
    Vector2D bestMTV(0.0, 0.0);
    
    for (const auto& dir : candidates) {
        // 二分查找找到最小的逃逸距离
        double low = 0.0;
        double high = 10.0;
        
        // 先检查是否能逃逸
        Polygon testB = B;
        testB.MoveByVec(dir * high);
        if (PolygonsOverlap(A, testB)) {
            continue;
        }
        
        // 二分查找最小距离
        for (int iter = 0; iter < 50; ++iter) {
            double mid = (low + high) / 2;
            Polygon testB = B;
            testB.MoveByVec(dir * mid);
            if (PolygonsOverlap(A, testB)) {
                low = mid;
            } else {
                high = mid;
            }
        }
        
        // 添加epsilon确保分离
        double escapeLen = high + MTV_EPSILON;
        if (escapeLen < bestLen) {
            bestLen = escapeLen;
            bestMTV = dir * escapeLen;
        }
    }
    
    return bestMTV;
}

// 计算MTV
Vector2D ComputeMTV(const Polygon& A, const Polygon& B) {
    // 首先检查是否重叠
    if (!PolygonsOverlap(A, B)) {
        return {0.0, 0.0};
    }
    
    // 首先尝试SAT
    Vector2D mtv = ComputeMTV_SAT(A, B);
    
    // 验证SAT结果
    Polygon testB = B;
    testB.MoveByVec(mtv);
    if (!PolygonsOverlap(A, testB)) {
        return mtv;
    }
    
    // SAT失败，使用迭代方法
    mtv = ComputeMTV_Iterative(A, B);
    
    return mtv;
}

// 生成解决方案
Vector2D GenSolution(const Vector2D& vec) {
    // 将poly2移动vec
    Polygon movedB = polygon2;
    movedB.MoveByVec(vec);
    
    // 计算MTV
    return ComputeMTV(polygon1, movedB);
}

// 预处理
void PreProcess() {
}

int main() {
    // =============== 1. 读取多边形 ===================
    std::cin >> n1 >> n2;

    if (n1 <= 0 || n2 <= 0) {
        std::cerr << "Input data error: the number of vertices of both polygons should be greater than 2" << std::endl;
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

    // 等待OK确保所有多边形数据已接收
    std::string okResp;
    std::cin >> okResp;
    if (okResp != "OK") {
        std::cerr << "Input data error: waiting for OK after obtaining polygons but I get " << okResp << std::endl;
        return 0;
    }

    // ============== 2. 预处理 ===================
    PreProcess();
    // 预处理完成后发送OK
    std::cout << "OK" << std::endl;
    std::cout.flush();

    // ============== 3. 读取测试数据 ===================
    std::cin >> m;
    testCases.resize(m);

    for (int i = 0; i < m; ++i) {
        std::cin >> testCases[i].x >> testCases[i].y;
    }

    // 等待OK确保所有测试用例已接收
    std::cin >> okResp;
    if (okResp != "OK") {
        std::cerr << "Input data error: waiting for OK after that I have received all test points but I get " << okResp
                  << std::endl;
        return 0;
    }

    // ================ 4. 求解并输出结果 ===================
    for (int i = 0; i < m; ++i) {
        const Vector2D& res = GenSolution(testCases[i]);
        std::cout << std::fixed << std::setprecision(5) << res.x << " " << res.y << std::endl;
        std::cout.flush();
    }

    // 输出所有答案后发送OK
    std::cout << "OK" << std::endl;
    std::cout.flush();

    return 0;
}