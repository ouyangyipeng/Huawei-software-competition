#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

struct Vector2D;
struct Polygon;

using namespace std;
const double MTV_EPSILON = 1e-5;

struct Vector2D {
    double x, y;

    Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    Vector2D operator-(const Vector2D& other) const { return Vector2D(x - other.x, y - other.y); }
    Vector2D operator+(const Vector2D& other) const { return Vector2D(x + other.x, y + other.y); }
    Vector2D operator*(double scalar) const { return Vector2D(x * scalar, y * scalar); }
    double Dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    double Length() const { return std::sqrt(x * x + y * y); }
    Vector2D Normalize() const {
        double len = Length();
        if (len < 1e-10) return *this;
        return Vector2D(x / len, y / len);
    }
    Vector2D Perp() const { return Vector2D(-y, x); }
};

struct Polygon {
    std::vector<Vector2D> vertices;

    Polygon() = default;
    Vector2D GetCenter() const {
        Vector2D center(0, 0);
        if (vertices.empty()) return center;
        for (const auto& v : vertices) center = center + v;
        return center * (1.0 / vertices.size());
    }
    void MoveByVec(const Vector2D& vec) {
        for (auto& v : vertices) v = v + vec;
    }
};

int n1 = 0, n2 = 0, m = 0;
Polygon polygon1;
Polygon polygon2;
vector<Vector2D> testCases;

struct Projection {
    double min, max;
};

// 点是否在多边形内（射线法）
bool PointInPolygon(const Vector2D& point, const Polygon& poly) {
    int n = poly.vertices.size();
    if (n < 3) return false;
    
    int count = 0;
    for (int i = 0; i < n; ++i) {
        Vector2D p1 = poly.vertices[i];
        Vector2D p2 = poly.vertices[(i + 1) % n];
        
        if (p1.y > p2.y) {
            Vector2D tmp = p1;
            p1 = p2;
            p2 = tmp;
        }
        
        if (p1.y <= point.y && point.y < p2.y) {
            double x = p1.x + (point.y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x);
            if (x > point.x) {
                count++;
            }
        }
    }
    return count % 2 == 1;
}

// 两条线段是否相交
bool SegmentsIntersect(const Vector2D& p1, const Vector2D& p2,
                       const Vector2D& p3, const Vector2D& p4) {
    auto ccw = [](const Vector2D& A, const Vector2D& B, const Vector2D& C) {
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
    };
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) && ccw(p1, p2, p3) != ccw(p1, p2, p4);
}

// 两个多边形是否重叠
bool PolygonsOverlap(const Polygon& A, const Polygon& B) {
    // 检查A的顶点是否在B内
    for (const auto& v : A.vertices) {
        if (PointInPolygon(v, B)) return true;
    }
    
    // 检查B的顶点是否在A内
    for (const auto& v : B.vertices) {
        if (PointInPolygon(v, A)) return true;
    }
    
    // 检查边是否相交
    for (size_t i = 0; i < A.vertices.size(); ++i) {
        for (size_t j = 0; j < B.vertices.size(); ++j) {
            if (SegmentsIntersect(A.vertices[i], A.vertices[(i + 1) % A.vertices.size()],
                                  B.vertices[j], B.vertices[(j + 1) % B.vertices.size()])) {
                return true;
            }
        }
    }
    
    return false;
}

Projection ProjectPolygon(const Polygon& poly, const Vector2D& axis) {
    Projection proj;
    proj.min = proj.max = poly.vertices[0].Dot(axis);
    for (size_t i = 1; i < poly.vertices.size(); ++i) {
        double val = poly.vertices[i].Dot(axis);
        if (val < proj.min) proj.min = val;
        if (val > proj.max) proj.max = val;
    }
    return proj;
}

// 使用SAT计算MTV
Vector2D ComputeMTV_SAT(const Polygon& A, const Polygon& B) {
    double minOverlap = std::numeric_limits<double>::infinity();
    Vector2D minAxis(0, 0);
    
    // 检查A的所有边
    for (size_t i = 0; i < A.vertices.size(); ++i) {
        Vector2D p1 = A.vertices[i];
        Vector2D p2 = A.vertices[(i + 1) % A.vertices.size()];
        Vector2D edge = p2 - p1;
        Vector2D axis = edge.Perp().Normalize();
        
        Projection projA = ProjectPolygon(A, axis);
        Projection projB = ProjectPolygon(B, axis);
        
        double overlap = std::min(projA.max - projB.min, projB.max - projA.min);
        
        if (overlap <= 0) {
            return Vector2D(0, 0);
        }
        
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minAxis = axis;
        }
    }
    
    // 检查B的所有边
    for (size_t i = 0; i < B.vertices.size(); ++i) {
        Vector2D p1 = B.vertices[i];
        Vector2D p2 = B.vertices[(i + 1) % B.vertices.size()];
        Vector2D edge = p2 - p1;
        Vector2D axis = edge.Perp().Normalize();
        
        Projection projA = ProjectPolygon(A, axis);
        Projection projB = ProjectPolygon(B, axis);
        
        double overlap = std::min(projA.max - projB.min, projB.max - projA.min);
        
        if (overlap <= 0) {
            return Vector2D(0, 0);
        }
        
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minAxis = axis;
        }
    }
    
    // 确保方向正确
    Vector2D centerA = A.GetCenter();
    Vector2D centerB = B.GetCenter();
    Vector2D dir = centerB - centerA;
    
    if (dir.Dot(minAxis) < 0) {
        minAxis = minAxis * -1.0;
    }
    
    return minAxis * (minOverlap + MTV_EPSILON);
}

// 二分搜索找精确分离距离
double FindSeparationDistance(const Polygon& A, const Polygon& B, const Vector2D& dir, double maxDist = 2.0) {
    double lo = 0.0;
    double hi = maxDist;
    
    for (int iter = 0; iter < 20; ++iter) {
        double mid = (lo + hi) * 0.5;
        Polygon testB = B;
        testB.MoveByVec(dir * mid);
        
        if (PolygonsOverlap(A, testB)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    
    return hi;
}

// 计算MTV（优化版：尝试多个方向找最短）
Vector2D ComputeMTV(const Polygon& A, const Polygon& B) {
    // 首先检查是否重叠
    if (!PolygonsOverlap(A, B)) {
        return Vector2D(0, 0);
    }
    
    // 使用SAT计算MTV
    Vector2D mtv = ComputeMTV_SAT(A, B);
    
    // 验证MTV是否有效
    Polygon movedB = B;
    movedB.MoveByVec(mtv);
    
    if (!PolygonsOverlap(A, movedB)) {
        // SAT成功
        return mtv;
    }
    
    // SAT失败，尝试多个方向找最短MTV
    Vector2D centerA = A.GetCenter();
    Vector2D centerB = B.GetCenter();
    
    double bestLen = std::numeric_limits<double>::infinity();
    Vector2D bestMTV;
    
    // 尝试16个方向
    for (int i = 0; i < 16; ++i) {
        double angle = i * M_PI / 8.0;
        Vector2D dir(cos(angle), sin(angle));
        
        double dist = FindSeparationDistance(A, B, dir);
        if (dist < bestLen) {
            bestLen = dist;
            bestMTV = dir * (dist + MTV_EPSILON);
        }
    }
    
    // 尝试中心方向
    Vector2D centerDir = centerB - centerA;
    if (centerDir.Length() > 1e-10) {
        centerDir = centerDir.Normalize();
        double dist = FindSeparationDistance(A, B, centerDir);
        if (dist < bestLen) {
            bestLen = dist;
            bestMTV = centerDir * (dist + MTV_EPSILON);
        }
    }
    
    return bestMTV;
}

Vector2D GenSolution(const Vector2D& vec) {
    Polygon movedB = polygon2;
    movedB.MoveByVec(vec);
    return ComputeMTV(polygon1, movedB);
}

void PreProcess() {
    // 预处理
}

int main() {
    // =============== 1. read polygons ===================
    cin >> n1 >> n2;

    if (n1 <= 0 || n2 <= 0) {
        cerr << "Input data error: the number of vertices of both polygons should be greater than 2" << endl;
        return 1;
    }

    polygon1.vertices.resize(n1);
    for (int i = 0; i < n1; ++i) {
        cin >> polygon1.vertices[i].x >> polygon1.vertices[i].y;
    }

    polygon2.vertices.resize(n2);
    for (int i = 0; i < n2; ++i) {
        cin >> polygon2.vertices[i].x >> polygon2.vertices[i].y;
    }

    // wait for OK to ensure all polygon data is received
    string okResp;
    cin >> okResp;
    if (okResp != "OK") {
        cerr << "Input data error: waiting for OK after obtaining polygons but I get " << okResp << endl;
        return 0;
    }

    // ============== 2. preprocess ===================
    PreProcess();
    // send OK after finishing preprocessing
    cout << "OK" << endl;
    cout.flush();

    // ============== 3. read test data ===================
    cin >> m;
    testCases.resize(m);

    for (int i = 0; i < m; ++i) {
        cin >> testCases[i].x >> testCases[i].y;
    }

    // wait for OK to ensure all test cases are received
    cin >> okResp;
    if (okResp != "OK") {
        cerr << "Input data error: waiting for OK after that I have received all test points but I get " << okResp
             << endl;
        return 0;
    }

    // ================ 4. solve and output results ===================
    for (int i = 0; i < m; ++i) {
        const Vector2D& res = GenSolution(testCases[i]);
        cout << fixed << std::setprecision(5) << res.x << " " << res.y << endl;
        cout.flush();
    }

    // send OK after outputting all answer
    cout << "OK" << endl;
    cout.flush();

    return 0;
}