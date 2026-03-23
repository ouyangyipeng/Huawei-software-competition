#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <set>

// 常量定义
const double EPS = 1e-6;
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

    Vector2D operator-() const {
        return Vector2D(-x, -y);
    }

    bool operator==(const Vector2D& other) const {
        return std::abs(x - other.x) < EPS && std::abs(y - other.y) < EPS;
    }

    bool operator<(const Vector2D& other) const {
        if (std::abs(x - other.x) > EPS) return x < other.x;
        return y < other.y - EPS;
    }

    // 点积
    double Dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }

    // 叉积
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

    // 极角
    double Angle() const {
        return std::atan2(y, x);
    }
};

// 线段结构体
struct Segment {
    Vector2D p1, p2;
    
    Segment(const Vector2D& a, const Vector2D& b) : p1(a), p2(b) {}
    
    // 点到线段的最短距离和最近点
    double DistanceToPoint(const Vector2D& p, Vector2D& closest) const {
        Vector2D v = p2 - p1;
        Vector2D w = p - p1;
        double c1 = w.Dot(v);
        if (c1 <= 0) {
            closest = p1;
            return (p - p1).Length();
        }
        double c2 = v.Dot(v);
        if (c2 <= c1) {
            closest = p2;
            return (p - p2).Length();
        }
        double b = c1 / c2;
        closest = p1 + v * b;
        return (p - closest).Length();
    }
};

// 多边形结构体
struct Polygon {
    std::vector<Vector2D> vertices;
    bool isConvex;

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

    // 计算有向面积（正数表示逆时针）
    double SignedArea() const {
        double area = 0;
        int n = vertices.size();
        for (int i = 0; i < n; ++i) {
            Vector2D v1 = vertices[i];
            Vector2D v2 = vertices[(i + 1) % n];
            area += v1.Cross(v2);
        }
        return area / 2.0;
    }

    // 确保逆时针方向
    void EnsureCCW() {
        if (SignedArea() < 0) {
            std::reverse(vertices.begin(), vertices.end());
        }
    }

    // 按向量平移
    void MoveByVec(const Vector2D& vec) {
        for (auto& v : vertices) {
            v = v + vec;
        }
    }
};

// 全局变量
int n1 = 0, n2 = 0, m = 0;
Polygon polygon1, polygon2;
std::vector<Vector2D> testCases;

// NFP相关全局变量
std::vector<Vector2D> nfpVertices;
std::vector<Segment> nfpEdges;
bool nfpComputed = false;

// 判断点是否在多边形内部（射线法）
bool PointInPolygon(const Vector2D& point, const std::vector<Vector2D>& poly) {
    int n = poly.size();
    if (n < 3) return false;

    int count = 0;
    for (int i = 0; i < n; ++i) {
        Vector2D p1 = poly[i];
        Vector2D p2 = poly[(i + 1) % n];

        // 检查水平射线是否与边相交
        if (p1.y > p2.y) std::swap(p1, p2);

        if (p1.y - EPS <= point.y && point.y < p2.y - EPS) {
            double x = p1.x + (point.y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x);
            if (x > point.x + EPS) {
                count++;
            }
        }
    }

    return count % 2 == 1;
}

// 计算点到多边形边界的最短距离和MTV
Vector2D PointToPolygonMTV(const Vector2D& point, const std::vector<Vector2D>& poly) {
    int n = poly.size();
    if (n < 3) return Vector2D(0, 0);

    double minDist = std::numeric_limits<double>::infinity();
    Vector2D closestPoint;

    for (int i = 0; i < n; ++i) {
        Segment seg(poly[i], poly[(i + 1) % n]);
        Vector2D cp;
        double dist = seg.DistanceToPoint(point, cp);
        if (dist < minDist) {
            minDist = dist;
            closestPoint = cp;
        }
    }

    return closestPoint - point;
}

// 计算两个凸多边形的闵可夫斯基差
// Minkowski Difference: A - B = {a - b | a in A, b in B}
std::vector<Vector2D> MinkowskiDifference(const Polygon& A, const Polygon& B) {
    std::vector<Vector2D> result;
    
    // 对B取反得到-B
    std::vector<Vector2D> negB;
    for (const auto& v : B.vertices) {
        negB.push_back(-v);
    }
    
    // 合并A和-B的所有边，按极角排序
    std::vector<std::pair<double, Vector2D>> edges;
    
    int nA = A.vertices.size();
    for (int i = 0; i < nA; ++i) {
        Vector2D edge = A.vertices[(i + 1) % nA] - A.vertices[i];
        edges.push_back({edge.Angle(), edge});
    }
    
    int nB = negB.size();
    for (int i = 0; i < nB; ++i) {
        Vector2D edge = negB[(i + 1) % nB] - negB[i];
        edges.push_back({edge.Angle(), edge});
    }
    
    // 按极角排序
    std::sort(edges.begin(), edges.end(), [](const std::pair<double, Vector2D>& a,
                                              const std::pair<double, Vector2D>& b) {
        return a.first < b.first;
    });
    
    // 构建闵可夫斯基差多边形
    Vector2D current = A.vertices[0] + negB[0];  // 起始点
    result.push_back(current);
    
    for (const auto& e : edges) {
        current = current + e.second;
        result.push_back(current);
    }
    
    // 移除重复点
    std::vector<Vector2D> cleaned;
    for (const auto& v : result) {
        bool duplicate = false;
        for (const auto& c : cleaned) {
            if ((v - c).Length() < EPS) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            cleaned.push_back(v);
        }
    }
    
    return cleaned;
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
            
            if (cross > EPS) {
                Polygon triangle;
                triangle.vertices.push_back(v1);
                triangle.vertices.push_back(v2);
                triangle.vertices.push_back(v3);
                triangle.isConvex = true;
                result.push_back(triangle);
                
                removed[i] = true;
                found = true;
                break;
            }
        }
        
        if (!found) break;
    }
    
    Polygon last;
    for (int i = 0; i < (int)remaining.size(); ++i) {
        if (!removed[i]) {
            last.vertices.push_back(remaining[i]);
        }
    }
    if (last.vertices.size() >= 3) {
        last.CheckConvex();
        result.push_back(last);
    }
    
    return result;
}

// 计算NFP（临界多边形）
void ComputeNFP() {
    // 分解多边形
    std::vector<Polygon> polysA, polysB;
    
    if (polygon1.isConvex) {
        polysA.push_back(polygon1);
    } else {
        polysA = DecomposePolygon(polygon1);
    }
    
    if (polygon2.isConvex) {
        polysB.push_back(polygon2);
    } else {
        polysB = DecomposePolygon(polygon2);
    }
    
    // 计算所有凸多边形对的闵可夫斯基差
    std::vector<std::vector<Vector2D>> minkowskiDiffs;
    
    for (const auto& subA : polysA) {
        for (const auto& subB : polysB) {
            auto md = MinkowskiDifference(subA, subB);
            if (!md.empty()) {
                minkowskiDiffs.push_back(md);
            }
        }
    }
    
    // 简化处理：使用第一个闵可夫斯基差作为NFP的近似
    // 对于更精确的结果，需要计算所有闵可夫斯基差的并集
    if (!minkowskiDiffs.empty()) {
        nfpVertices = minkowskiDiffs[0];
        
        // 构建NFP边
        nfpEdges.clear();
        int n = nfpVertices.size();
        for (int i = 0; i < n; ++i) {
            nfpEdges.push_back(Segment(nfpVertices[i], nfpVertices[(i + 1) % n]));
        }
        
        nfpComputed = true;
    }
}

// 使用SAT算法计算两个凸多边形的MTV
Vector2D ComputeMTVConvex(const Polygon& A, const Polygon& B) {
    double minOverlap = std::numeric_limits<double>::infinity();
    Vector2D smallestAxis;

    int nA = A.vertices.size();
    for (int i = 0; i < nA; ++i) {
        Vector2D edge = A.vertices[(i + 1) % nA] - A.vertices[i];
        Vector2D axis = edge.Perp().Normalize();

        double minA = A.vertices[0].Dot(axis), maxA = minA;
        for (int j = 1; j < nA; ++j) {
            double proj = A.vertices[j].Dot(axis);
            minA = std::min(minA, proj);
            maxA = std::max(maxA, proj);
        }

        double minB = B.vertices[0].Dot(axis), maxB = minB;
        for (int j = 1; j < (int)B.vertices.size(); ++j) {
            double proj = B.vertices[j].Dot(axis);
            minB = std::min(minB, proj);
            maxB = std::max(maxB, proj);
        }

        if (maxA <= minB + EPS || maxB <= minA + EPS) {
            return {0.0, 0.0};
        }

        double overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    int nB = B.vertices.size();
    for (int i = 0; i < nB; ++i) {
        Vector2D edge = B.vertices[(i + 1) % nB] - B.vertices[i];
        Vector2D axis = edge.Perp().Normalize();

        double minA = A.vertices[0].Dot(axis), maxA = minA;
        for (int j = 1; j < (int)A.vertices.size(); ++j) {
            double proj = A.vertices[j].Dot(axis);
            minA = std::min(minA, proj);
            maxA = std::max(maxA, proj);
        }

        double minB = B.vertices[0].Dot(axis), maxB = minB;
        for (int j = 1; j < nB; ++j) {
            double proj = B.vertices[j].Dot(axis);
            minB = std::min(minB, proj);
            maxB = std::max(maxB, proj);
        }

        if (maxA <= minB + EPS || maxB <= minA + EPS) {
            return {0.0, 0.0};
        }

        double overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    Vector2D centerA = A.GetCenter();
    Vector2D centerB = B.GetCenter();
    Vector2D dir = centerB - centerA;

    if (smallestAxis.Dot(dir) < 0.0) {
        smallestAxis = smallestAxis * -1.0;
    }

    return smallestAxis * minOverlap;
}

// 计算两个多边形的MTV（支持凹多边形）
Vector2D ComputeMTV(const Polygon& A, const Polygon& B) {
    if (A.isConvex && B.isConvex) {
        return ComputeMTVConvex(A, B);
    }

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
    // 使用NFP方法
    if (nfpComputed && !nfpVertices.empty()) {
        // 测试点就是vec
        // 判断点是否在NFP内部
        if (PointInPolygon(vec, nfpVertices)) {
            // 在内部，计算到边界的MTV
            return PointToPolygonMTV(vec, nfpVertices);
        } else {
            // 在外部，无重叠
            return Vector2D(0.0, 0.0);
        }
    }
    
    // 回退到SAT方法
    Polygon polyB = polygon2;
    polyB.MoveByVec(vec);
    return ComputeMTV(polygon1, polyB);
}

// 预处理函数
void PreProcess() {
    polygon1.CheckConvex();
    polygon2.CheckConvex();
    
    // 尝试计算NFP
    ComputeNFP();
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    // 读取多边形数据
    std::cin >> n1 >> n2;

    if (n1 <= 0 || n2 <= 0) {
        std::cerr << "Input data error" << std::endl;
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

    std::string okResp;
    std::cin >> okResp;
    if (okResp != "OK") {
        return 1;
    }

    // 预处理
    PreProcess();
    std::cout << "OK" << std::endl;
    std::cout.flush();

    // 读取测试数据
    std::cin >> m;
    testCases.resize(m);

    for (int i = 0; i < m; ++i) {
        std::cin >> testCases[i].x >> testCases[i].y;
    }

    std::cin >> okResp;
    if (okResp != "OK") {
        return 1;
    }

    // 求解并输出结果
    for (int i = 0; i < m; ++i) {
        Vector2D res = GenSolution(testCases[i]);
        std::cout << std::fixed << std::setprecision(5) << res.x << " " << res.y << '\n';
    }
    std::cout.flush();

    std::cout << "OK" << std::endl;
    std::cout.flush();

    return 0;
}