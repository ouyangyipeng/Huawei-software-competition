#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// 常量定义
const double EPS = 1e-6;
const double INF = 1e18;

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
    bool isConvex;

    Polygon() : isConvex(true) {}

    Vector2D GetCenter() const {
        Vector2D center(0, 0);
        if (vertices.empty()) return center;
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

    bool CheckConvex() {
        if (vertices.size() < 3) return true;

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

    double SignedArea() const {
        double area = 0;
        int n = vertices.size();
        for (int i = 0; i < n; ++i) {
            area += vertices[i].Cross(vertices[(i + 1) % n]);
        }
        return area / 2.0;
    }
};

// 全局变量
int n1 = 0, n2 = 0, m = 0;
Polygon polygon1, polygon2;
std::vector<Vector2D> testCases;

// 存储所有闵可夫斯基差
std::vector<std::vector<Vector2D>> allMinkowskis;

// 凸包算法 (Andrew's monotone chain)
std::vector<Vector2D> ConvexHull(std::vector<Vector2D>& points) {
    int n = points.size();
    if (n < 3) return points;

    std::sort(points.begin(), points.end());

    // 移除重复点
    std::vector<Vector2D> unique;
    for (int i = 0; i < n; ++i) {
        if (unique.empty() || !(unique.back() == points[i])) {
            unique.push_back(points[i]);
        }
    }
    points = unique;
    n = points.size();

    if (n < 3) return points;

    std::vector<Vector2D> hull;

    // 下凸包
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2) {
            Vector2D a = hull[hull.size() - 2];
            Vector2D b = hull[hull.size() - 1];
            Vector2D c = points[i];
            if ((b - a).Cross(c - b) < EPS) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(points[i]);
    }

    // 上凸包
    int lowerSize = hull.size();
    for (int i = n - 2; i >= 0; --i) {
        while (hull.size() > lowerSize) {
            Vector2D a = hull[hull.size() - 2];
            Vector2D b = hull[hull.size() - 1];
            Vector2D c = points[i];
            if ((b - a).Cross(c - b) < EPS) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(points[i]);
    }

    // 移除最后一个点（重复）
    if (hull.size() > 1) hull.pop_back();

    return hull;
}

// 判断点是否在三角形内
bool PointInTriangle(const Vector2D& p, const Vector2D& a, 
                     const Vector2D& b, const Vector2D& c) {
    Vector2D v0 = c - a;
    Vector2D v1 = b - a;
    Vector2D v2 = p - a;

    double dot00 = v0.Dot(v0);
    double dot01 = v0.Dot(v1);
    double dot02 = v0.Dot(v2);
    double dot11 = v1.Dot(v1);
    double dot12 = v1.Dot(v2);

    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return (u >= -EPS) && (v >= -EPS) && (u + v <= 1 + EPS);
}

// 判断点是否在多边形内部（射线法）
bool PointInPolygon(const Vector2D& point, const std::vector<Vector2D>& poly) {
    int n = poly.size();
    if (n < 3) return false;

    int count = 0;
    for (int i = 0; i < n; ++i) {
        Vector2D p1 = poly[i];
        Vector2D p2 = poly[(i + 1) % n];

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

// 判断点是否在任意闵可夫斯基差内部
bool IsInAnyMinkowski(const Vector2D& p) {
    for (const auto& m : allMinkowskis) {
        if (PointInPolygon(p, m)) {
            return true;
        }
    }
    return false;
}

// 点到线段的投影
Vector2D ProjectToSegment(const Vector2D& p, const Vector2D& a, const Vector2D& b) {
    Vector2D ab = b - a;
    Vector2D ap = p - a;
    double t = ap.Dot(ab) / ab.Dot(ab);
    t = std::max(0.0, std::min(1.0, t));
    return a + ab * t;
}

// 点到线段的距离
double DistanceToSegment(const Vector2D& p, const Vector2D& a, const Vector2D& b) {
    return (p - ProjectToSegment(p, a, b)).Length();
}

// 计算两个凸多边形的闵可夫斯基差（使用凸包方法）
std::vector<Vector2D> MinkowskiDifference(const Polygon& A, const Polygon& B) {
    std::vector<Vector2D> diffPoints;

    // 计算所有顶点对的差
    for (const auto& a : A.vertices) {
        for (const auto& b : B.vertices) {
            diffPoints.push_back(a - b);
        }
    }

    // 求凸包
    return ConvexHull(diffPoints);
}

// 修复后的耳切法分解
std::vector<Polygon> DecomposePolygon(const Polygon& poly) {
    std::vector<Polygon> result;

    if (poly.vertices.size() <= 3) {
        result.push_back(poly);
        return result;
    }

    std::vector<Vector2D> remaining = poly.vertices;
    std::vector<bool> removed(remaining.size(), false);

    int maxIterations = remaining.size() * 3;
    int iterations = 0;

    while (iterations < maxIterations) {
        iterations++;
        bool found = false;

        // 统计剩余顶点数
        int remainingCount = 0;
        for (int i = 0; i < (int)remaining.size(); ++i) {
            if (!removed[i]) remainingCount++;
        }

        if (remainingCount < 3) break;
        if (remainingCount == 3) {
            // 添加最后一个三角形
            Polygon last;
            for (int i = 0; i < (int)remaining.size(); ++i) {
                if (!removed[i]) {
                    last.vertices.push_back(remaining[i]);
                }
            }
            if (last.vertices.size() == 3) {
                last.isConvex = true;
                result.push_back(last);
            }
            break;
        }

        for (int i = 0; i < (int)remaining.size(); ++i) {
            if (removed[i]) continue;

            // 找前一个和后一个未删除的顶点
            int prev = -1, next = -1;
            for (int j = 1; j < (int)remaining.size(); ++j) {
                int idx = (i - j + remaining.size()) % remaining.size();
                if (!removed[idx]) {
                    prev = idx;
                    break;
                }
            }
            for (int j = 1; j < (int)remaining.size(); ++j) {
                int idx = (i + j) % remaining.size();
                if (!removed[idx]) {
                    next = idx;
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
                // 检查三角形内是否包含其他顶点
                bool hasPointInside = false;
                for (int j = 0; j < (int)remaining.size(); ++j) {
                    if (removed[j] || j == prev || j == i || j == next) continue;
                    if (PointInTriangle(remaining[j], v1, v2, v3)) {
                        hasPointInside = true;
                        break;
                    }
                }

                if (!hasPointInside) {
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
        }

        if (!found) break;
    }

    return result;
}

// SAT算法计算两个凸多边形的MTV
Vector2D ComputeMTVConvex(const Polygon& A, const Polygon& B) {
    double minOverlap = INF;
    Vector2D smallestAxis;

    auto checkAxes = [&](const Polygon& poly) {
        int n = poly.vertices.size();
        for (int i = 0; i < n; ++i) {
            Vector2D edge = poly.vertices[(i + 1) % n] - poly.vertices[i];
            Vector2D axis = edge.Perp().Normalize();

            double minA = INF, maxA = -INF;
            double minB = INF, maxB = -INF;

            for (const auto& v : A.vertices) {
                double proj = v.Dot(axis);
                minA = std::min(minA, proj);
                maxA = std::max(maxA, proj);
            }

            for (const auto& v : B.vertices) {
                double proj = v.Dot(axis);
                minB = std::min(minB, proj);
                maxB = std::max(maxB, proj);
            }

            if (maxA <= minB + EPS || maxB <= minA + EPS) {
                return Vector2D(0, 0);
            }

            double overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }
        return Vector2D();
    };

    Vector2D result = checkAxes(A);
    if (result.x == 0 && result.y == 0 && minOverlap == INF) {
        result = checkAxes(B);
        if (result.x == 0 && result.y == 0 && minOverlap == INF) {
            return Vector2D(0, 0);
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

// 计算逃逸向量
Vector2D ComputeEscapeVector(const Vector2D& t) {
    std::vector<std::pair<double, Vector2D>> candidates;

    // 收集所有边上的投影点
    for (const auto& m : allMinkowskis) {
        int n = m.size();
        for (int i = 0; i < n; ++i) {
            Vector2D proj = ProjectToSegment(t, m[i], m[(i + 1) % n]);
            double dist = (proj - t).Length();
            candidates.push_back({dist, proj});
        }
    }

    // 按距离排序
    std::sort(candidates.begin(), candidates.end());

    // 找最近的不在任何M内部点
    for (const auto& c : candidates) {
        if (!IsInAnyMinkowski(c.second)) {
            return c.second - t;
        }
    }

    // 如果所有投影点都在内部，返回最近的
    if (!candidates.empty()) {
        return candidates[0].second - t;
    }

    return Vector2D(0, 0);
}

// 预处理：计算所有闵可夫斯基差
void PreProcess() {
    polygon1.CheckConvex();
    polygon2.CheckConvex();

    allMinkowskis.clear();

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

    // 计算所有闵可夫斯基差
    for (const auto& subA : polysA) {
        for (const auto& subB : polysB) {
            std::vector<Vector2D> md = MinkowskiDifference(subA, subB);
            if (md.size() >= 3) {
                allMinkowskis.push_back(md);
            }
        }
    }
}

// 生成解决方案
Vector2D GenSolution(const Vector2D& vec) {
    // 使用Union-Free NFP方法
    if (!allMinkowskis.empty()) {
        // 判断点是否在任意闵可夫斯基差内部
        if (IsInAnyMinkowski(vec)) {
            // 有重叠，计算逃逸向量
            return ComputeEscapeVector(vec);
        } else {
            // 无重叠
            return Vector2D(0, 0);
        }
    }

    // 回退到SAT方法
    Polygon polyB = polygon2;
    polyB.MoveByVec(vec);

    if (polygon1.isConvex && polyB.isConvex) {
        return ComputeMTVConvex(polygon1, polyB);
    }

    // 分解并计算
    std::vector<Polygon> polysA, polysB;
    if (polygon1.isConvex) {
        polysA.push_back(polygon1);
    } else {
        polysA = DecomposePolygon(polygon1);
    }

    if (polyB.isConvex) {
        polysB.push_back(polyB);
    } else {
        polysB = DecomposePolygon(polyB);
    }

    Vector2D bestMTV(0, 0);
    double minLen = INF;
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

    return hasOverlap ? bestMTV : Vector2D(0, 0);
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