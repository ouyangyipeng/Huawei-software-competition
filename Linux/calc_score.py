#!/usr/bin/env python3
import sys
import math

def read_output(filename):
    """读取输出文件，返回测试点数量和结果向量列表"""
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    n = int(lines[0].strip())
    vectors = []
    for i in range(1, n + 1):
        parts = lines[i].strip().split()
        x, y = float(parts[0]), float(parts[1])
        vectors.append((x, y))
    
    return n, vectors

def read_input(filename):
    """读取输入文件，返回多边形和测试点"""
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    idx = 0
    # 第一行是 n1 n2
    parts = lines[idx].strip().split()
    n1, n2 = int(parts[0]), int(parts[1])
    idx += 1
    
    # 读取多边形1的顶点（每行可能有多个顶点）
    poly1 = []
    while len(poly1) < n1:
        parts = lines[idx].strip().split()
        for i in range(0, len(parts), 2):
            if len(poly1) < n1:
                x, y = float(parts[i]), float(parts[i + 1])
                poly1.append((x, y))
        idx += 1
    
    # 读取多边形2的顶点
    poly2 = []
    while len(poly2) < n2:
        parts = lines[idx].strip().split()
        for i in range(0, len(parts), 2):
            if len(poly2) < n2:
                x, y = float(parts[i]), float(parts[i + 1])
                poly2.append((x, y))
        idx += 1
    
    # 读取测试点数量
    m = int(lines[idx].strip())
    idx += 1
    
    test_points = []
    while len(test_points) < m:
        parts = lines[idx].strip().split()
        for i in range(0, len(parts), 2):
            if len(test_points) < m:
                x, y = float(parts[i]), float(parts[i + 1])
                test_points.append((x, y))
        idx += 1
    
    return poly1, poly2, test_points

def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def point_in_polygon(p, poly):
    """射线法判断点是否在多边形内"""
    n = len(poly)
    if n < 3:
        return False
    
    count = 0
    for i in range(n):
        p1 = poly[i]
        p2 = poly[(i + 1) % n]
        
        if p1[1] > p2[1]:
            p1, p2 = p2, p1
        
        if p1[1] <= p[1] and p[1] < p2[1]:
            x = p1[0] + (p[1] - p1[1]) / (p2[1] - p1[1]) * (p2[0] - p1[0])
            if x > p[0]:
                count += 1
    
    return count % 2 == 1

def segments_intersect(p1, p2, p3, p4):
    """检查两条线段是否相交"""
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
    
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

def polygons_overlap(poly1, poly2, test_point, mtv):
    """检查两个多边形是否重叠（poly2先移动test_point，再移动mtv后）"""
    # 移动poly2: 先移动test_point，再移动mtv
    moved_poly2 = [(p[0] + test_point[0] + mtv[0], p[1] + test_point[1] + mtv[1]) for p in poly2]
    
    # 检查poly1的顶点是否在moved_poly2内
    for p in poly1:
        if point_in_polygon(p, moved_poly2):
            return True
    
    # 检查moved_poly2的顶点是否在poly1内
    for p in moved_poly2:
        if point_in_polygon(p, poly1):
            return True
    
    # 检查边是否相交
    for i in range(len(poly1)):
        for j in range(len(moved_poly2)):
            a1, a2 = poly1[i], poly1[(i + 1) % len(poly1)]
            b1, b2 = moved_poly2[j], moved_poly2[(j + 1) % len(moved_poly2)]
            
            if segments_intersect(a1, a2, b1, b2):
                return True
    
    return False

def calc_score(input_file, output_file):
    """计算单个测试用例的分数"""
    poly1, poly2, test_points = read_input(input_file)
    n, vectors = read_output(output_file)
    
    if n != len(test_points):
        print(f"Warning: output count {n} != test count {len(test_points)}")
    
    total_score = 0.0
    for i, (tp, vec) in enumerate(zip(test_points, vectors)):
        # 计算移动向量长度
        vec_len = math.sqrt(vec[0]**2 + vec[1]**2)
        
        # 检查是否分离（poly2先移动tp，再移动vec）
        separated = not polygons_overlap(poly1, poly2, tp, vec)
        
        if separated:
            # 分离成功，分数 = 1 / (1 + 向量长度)
            score = 1.0 / (1.0 + vec_len)
        else:
            # 未分离，分数 = 0
            score = 0.0
        
        total_score += score
    
    return total_score

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python calc_score.py <input_file> <output_file>")
        sys.exit(1)
    
    score = calc_score(sys.argv[1], sys.argv[2])
    print(f"Score: {score:.2f}")
