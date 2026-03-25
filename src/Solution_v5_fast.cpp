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

// AABB包围盒
struct AABB {
    double minX, maxX, minY, maxY;
    
    AABB() : minX(INF), maxX(-INF), minY(INF), maxY(-INF) {}
    
    void Update(const Vector2D& v) {
        minX = std::min(minX, v.x);
        maxX = std::max(maxX, v.x);
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
    }
    
    bool Contains(const Vector2D& p) const {
        return p.x >= minX - EPS && p.x <= maxX + EPS &&
               p.y >= minY - EPS && p.y <= maxY + EPS;
    }
    
    bool Intersects(const AABB& other) const {
        return !(maxX < other.minX - EPS || minX > other.maxX + EPS ||
                 maxY < other.minY - EPS || minY > other.maxY + EPS);
    }
};

// 多边形结构体
struct Polygon {
    std::vector<Vector2D> vertices;
    bool isConvex;
    AABB aabb;

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

    void ComputeAABB() {
        aabb = AABB();
        for (const auto& v : vertices) {
            aabb.Update(v);
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
};

// 全局变量
int n1 = 0, n2 = 0, m = 0;
Polygon polygon1, polygon2;
std::vector<Vector2D> testCases;

// 存储所有闵可夫斯基差及其AABB
std::vector<std::vector<Vector2D>> allMinkowskis;
std::vector<AABB> allAABBs;

// 凸包算法 (Andrew's monotone chain)
std::vector<Vector2D> ConvexHull(std::vector<Vector2D>& points) {
    int n = points.size();
    if (n < 3) return points;

    std::sort(points.begin(), points.end());

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

    if (hull.size() > 1) hull.pop_back();

    return hull;
}


struct Segment {
    Vector2D A, B;
};
std::vector<Segment> union_segments;

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

bool IsStrictlyInsideConvex(const Vector2D& p, const std::vector<Vector2D>& poly) {
    int n = poly.size();
    if (n < 3) return false;
    for (int i = 0; i < n; ++i) {
        Vector2D a = poly[i];
        Vector2D b = poly[(i + 1) % n];
        if ((b - a).Cross(p - a) <= 1e-9) {
            return false;
        }
    }
    return true;
}


// 判断点是否在任意闵可夫斯基差内部

bool IsInAnyMinkowski(const Vector2D& p) {
    for (const auto& m : allMinkowskis) {
        if (IsStrictlyInsideConvex(p, m)) return true;
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

// 计算两条线段的交点
bool SegmentIntersection(const Vector2D& p1, const Vector2D& p2,
                         const Vector2D& p3, const Vector2D& p4,
                         Vector2D& intersection) {
    Vector2D d1 = p2 - p1;
    Vector2D d2 = p4 - p3;
    
    double cross = d1.Cross(d2);
    if (std::abs(cross) < EPS) {
        return false; // 平行或共线
    }
    
    Vector2D d3 = p3 - p1;
    double t1 = d3.Cross(d2) / cross;
    double t2 = d3.Cross(d1) / cross;
    
    if (t1 >= -EPS && t1 <= 1 + EPS && t2 >= -EPS && t2 <= 1 + EPS) {
        intersection = p1 + d1 * t1;
        return true;
    }
    
    return false;
}

// 计算两个凸多边形的闵可夫斯基差（使用凸包方法）
std::vector<Vector2D> MinkowskiDifference(const Polygon& A, const Polygon& B) {
    std::vector<Vector2D> diffPoints;

    for (const auto& a : A.vertices) {
        for (const auto& b : B.vertices) {
            diffPoints.push_back(a - b);
        }
    }

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

        int remainingCount = 0;
        for (int i = 0; i < (int)remaining.size(); ++i) {
            if (!removed[i]) remainingCount++;
        }

        if (remainingCount < 3) break;
        if (remainingCount == 3) {
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

            if (cross > EPS) {
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

// 预计算所有NFP边缘的交点

void GenerateUnionSegments() {
    union_segments.clear();
    for (size_t i = 0; i < allMinkowskis.size(); ++i) {
        const auto& Mi = allMinkowskis[i];
        for (size_t a = 0; a < Mi.size(); ++a) {
            Vector2D A = Mi[a];
            Vector2D B = Mi[(a + 1) % Mi.size()];
            Vector2D D = B - A;
            
            std::vector<std::pair<double, double>> covered;
            
            for (size_t j = 0; j < allMinkowskis.size(); ++j) {
                if (i == j) continue;
                if (!allAABBs[i].Intersects(allAABBs[j])) continue;
                
                const auto& Mj = allMinkowskis[j];
                double t_min = 0.0;
                double t_max = 1.0;
                bool valid = true;
                
                for (size_t b = 0; b < Mj.size(); ++b) {
                    Vector2D Vm = Mj[b];
                    Vector2D Vm1 = Mj[(b + 1) % Mj.size()];
                    Vector2D Edir = Vm1 - Vm;
                    
                    double crossA = Edir.Cross(A - Vm);
                    double K = Edir.Cross(D);
                    
                    double EPS_IN = 1e-7;
                    if (std::abs(K) < 1e-9) {
                        if (crossA <= EPS_IN) {
                            valid = false;
                            break;
                        }
                    } else if (K > 0) {
                        double t = (EPS_IN - crossA) / K;
                        if (t > t_min) t_min = t;
                    } else {
                        double t = (EPS_IN - crossA) / K;
                        if (t < t_max) t_max = t;
                    }
                }
                if (valid && t_min < t_max) {
                    covered.push_back({t_min, t_max});
                }
            }
            
            std::vector<std::pair<double, double>> merged;
            std::sort(covered.begin(), covered.end());
            for (const auto& iv : covered) {
                if (merged.empty()) merged.push_back(iv);
                else {
                    if (iv.first <= merged.back().second + 1e-9) {
                        merged.back().second = std::max(merged.back().second, iv.second);
                    } else {
                        merged.push_back(iv);
                    }
                }
            }
            
            double cur = 0.0;
            for (const auto& iv : merged) {
                if (iv.first > cur + 1e-7) {
                    union_segments.push_back({A + D * cur, A + D * iv.first});
                }
                cur = std::max(cur, iv.second);
            }
            if (cur < 1.0 - 1e-7) {
                union_segments.push_back({A + D * cur, B});
            }
        }
    }
}


// 存储预计算的交点
// 计算逃逸向量
Vector2D ComputeEscapeVector(const Vector2D& t) {
    if (union_segments.empty()) return Vector2D(0, 0);
    
    double min_dist2 = std::numeric_limits<double>::infinity();
    Vector2D best_escape(0, 0);
    
    for (const auto& S : union_segments) {
        Vector2D proj = ProjectToSegment(t, S.A, S.B);
        Vector2D vec = proj - t;
        double dist2 = vec.x * vec.x + vec.y * vec.y;
        if (dist2 < min_dist2) {
            min_dist2 = dist2;
            best_escape = vec;
        }
    }
    
    double len = std::sqrt(min_dist2);
    if (len > 1e-9) {
        return best_escape + best_escape * (1e-5 / len);
    }
    return best_escape;
}

void PreProcess() {
    polygon1.CheckConvex();
    polygon2.CheckConvex();
    
    polygon1.ComputeAABB();
    polygon2.ComputeAABB();

    allMinkowskis.clear();
    allAABBs.clear();

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

    for (const auto& subA : polysA) {
        for (const auto& subB : polysB) {
            std::vector<Vector2D> md = MinkowskiDifference(subA, subB);
            if (md.size() >= 3) {
                allMinkowskis.push_back(md);
                
                // 计算AABB
                AABB aabb;
                for (const auto& v : md) {
                    aabb.Update(v);
                }
                allAABBs.push_back(aabb);
            }
        }
    }
    
    // 预计算所有交点
    GenerateUnionSegments();
}

// 生成解决方案（纯NFP方法，无SAT回退）
Vector2D GenSolution(const Vector2D& vec) {
    if (allMinkowskis.empty()) {
        return Vector2D(0, 0);
    }
    
    // 判断点是否在任意闵可夫斯基差内部
    if (IsInAnyMinkowski(vec)) {
        // 有重叠，计算逃逸向量
        return ComputeEscapeVector(vec);
    } else {
        // 无重叠
        return Vector2D(0, 0);
    }
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