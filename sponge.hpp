#pragma once
#ifndef SPONGE_HPP
#define SPONGE_HPP

#include <algorithm>
#include <random>

#include "vector.hpp"
#include "color.hpp"
#include "ray.hpp"
#include "stack.hpp"
#include "space_converter.hpp"

#define INF (INFINITY)

constexpr int W = 800;
constexpr int H = 600;
constexpr int SPONGE_L = 3; // 递归时每边细分为的小格数（门格海绵为 3）
constexpr V3d INIT_CAMERA_FRONT (-1.01, -1.02, -1.01);
// constexpr V3d INIT_CAMERA_FRONT (-0.322058, -0.89191, -0.31745);
constexpr V3d INIT_CAMERA_HEAD  (-1, 0, 1);
constexpr V3d INIT_CAMERA_POS   (SPONGE_L * 1.01, SPONGE_L * 1.02, SPONGE_L * 1.03);
// constexpr V3d INIT_CAMERA_POS = inv((V3d){ SPONGE_L * 2.01, SPONGE_L * 2.02, SPONGE_L * 2.03 });
constexpr double CAMERA_AXIS = 3;
constexpr double INIT_MOVE_VELOCITY = 0.9;
constexpr double ROTATE_VELOCITY = 0.3;
constexpr int RELATIVE_FRACTAL_ITER = 10;    // 分形从现在相机在的递归层数往里渲染几层
constexpr int SCALE = 300;
constexpr double REFLECT_PROBABILITY = 0.9;
constexpr Color SKY_COLOR(0.0, 0.0, 0.0);
constexpr double DIST_EPS = 1e-10;  // 小于这个数的算作距离的计算误差
constexpr double SPONGE_ROUGHNESS = 1.0;    // 海绵的粗糙程度（和正规定义不太一样，详见 deflectRay）

#ifndef DEBUG
constexpr int N = 100005;   // 一个很大的数，用于最大帧数与最大海绵递归层数 待优化
#else
constexpr int N = 50;
#endif

// 这些是另一个视角的相机设置
// constexpr V3d INIT_CAMERA_FRONT (-1, 0, -0);
// constexpr V3d INIT_CAMERA_HEAD  (0, 0, 1);
// constexpr V3d INIT_CAMERA_POS   (11.0 / 18.0 - 0.00001, 1.0 / 6.0 - 0.0001, 1.0 / 6.0 - 0.000001);

// random number generator
std::mt19937 rng;

double move_velocity = INIT_MOVE_VELOCITY;

bool sgn(double x) {
    if (x > 0)  return true;
    return false;
}

// 描述了门格海绵长啥样，pos 代表的单元是实心还是空心，pos 从 0 开始。
bool inSponge(Vector3D<int> pos) {
    //return false;   //DEBUG
    if (pos.x != 1)
        return (pos.y != 1 || pos.z != 1);
    //else
        return (pos.y != 1 && pos.z != 1);
}

// bool inSponge(Vector3D<int> pos) {
//     if (pos.x == 0)
//         return true;
//     return pos.y == 1 && pos.z == 1;
// }

V3d enterBlock(V3d v, Vector3D<int> pos) {
    return  {
        (v.x - pos.x) * SPONGE_L,
        (v.y - pos.y) * SPONGE_L,
        (v.z - pos.z) * SPONGE_L
    };
}

V3d exitBlock(V3d v, Vector3D<int> pos) {
    return {
        v.x / SPONGE_L + pos.x,
        v.y / SPONGE_L + pos.y,
        v.z / SPONGE_L + pos.z
    };
}

double side_len[N];

void initSideLen() {
    side_len[0] = SPONGE_L;
    for (int i = 1; i < N; i++) {
        side_len[i] = side_len[i - 1] / SPONGE_L;
        if (side_len[i] == 0)
            return;
    }
}

// 栈描述的立方体的绝对坐标的左下顶点
V3d blockVertex(Stack<Vector3D<int>, N> &st) {
    V3d v = { 0, 0, 0 };
    for (int i = 1; &(st.arr[i]) <= st.cursor; i++) {
        v.x += st.arr[i].x * side_len[i];
        v.y += st.arr[i].y * side_len[i];
        v.z += st.arr[i].z * side_len[i];
    }
    return v;
}

Stack<Vector3D<int>, N> init_blocks;

V3d enterInitBlock(V3d v) {
    for (Vector3D<int> *i = init_blocks.arr + 1; i <= init_blocks.cursor; i++)
        v = enterBlock(v, *i);
    return v;
}

void calcInitBlock(V3d cam_pos) {

    cam_pos = inv(cam_pos); // 先把坐标反演再计算
    cam_pos += { (double)SPONGE_L / 2, (double)SPONGE_L / 2, (double)SPONGE_L / 2 };    // BUG POSITION

    // move_velocity = INIT_MOVE_VELOCITY;
    init_blocks.clear();
    init_blocks.arr[0] = { 0, 0, 0 };
    if (cam_pos.x <= 0 || cam_pos.x >= SPONGE_L || cam_pos.y <= 0 || cam_pos.y >= SPONGE_L || cam_pos.z <= 0 || cam_pos.z >= SPONGE_L) {
        return;
    }
    Vector3D<int> pos;
    while (init_blocks.size() < N) {
        pos = { (int)cam_pos.x, (int)cam_pos.y, (int)cam_pos.z };
        init_blocks.push(pos);
        cam_pos = enterBlock(cam_pos, pos);
        // move_velocity /= SPONGE_L;
        if (!inSponge(pos))
            return;
    }
    if (init_blocks.size() == N) {
        std::cout << "init block overflowed.\n";
        getchar();
        exit(0);
    }
}

Ray res_ray;    // 待优化
V3d res_normal;
Stack<Vector3D<int>, N> blocks;
Stack<Vector3D<int>, N> temp_blocks;  //进行 calc 计算穿透一个方块进入下一个方块 之前 栈的状态。方便光追。

/// @brief 光线入射到面上之后，处理（栈中的）方块转换
/// @tparam CvtDouble space converter of double varibles
/// @tparam CvtInt space converter of int varibles
/// @param p 线面交点
/// @note 这么长其实是因为相似代码复制了一遍
template<typename CvtDouble, typename CvtInt>
CvtDouble calc(CvtDouble p, const bool up) {
    
    //每次在函数内判断 first_in 会损失一点性能，以后有时间管一下。

    if ((!up) ^ blocks.empty()) {    //如果是第一次入射，那么碰到上表面是从上方射入新立方体；如果不是第一次入射，那么碰到旧立方体的下表面是从上方射入新立方体。
        if (!blocks.empty()) {
            while (!blocks.empty() && ((CvtInt *)&(blocks.top()))->X() == 0) {   //出方块 【改】
                p = exitBlock(p, blocks.top());
                blocks.pop();
            }
            if (blocks.empty())
                return p;
            ((CvtInt *)&(blocks.top()))->X()--; //【改】
            p.X() = SPONGE_L;  //【改】
            if (!inSponge(blocks.top()))
                return p; //进入空方块：返回，进行下一次传播（propagate）。
        }
        while (blocks.size() < init_blocks.size() + RELATIVE_FRACTAL_ITER) {  //进方块
            Vector3D<int> pos;
            pos = (CvtInt){ SPONGE_L - 1, (int)p.Y(), (int)p.Z() }; //【改】
            p = enterBlock(p, pos);
            blocks.push(pos);
            if (!inSponge(pos))
                break;
        }
    } else {
        if (!blocks.empty()) {
            while (!blocks.empty() && ((CvtInt *)&(blocks.top()))->X() == SPONGE_L - 1) {   //出方块 【改】
                p = exitBlock(p, blocks.top());
                blocks.pop();
            }
            if (blocks.empty())
                return p;
            ((CvtInt *)&(blocks.top()))->X()++; //【改】
            p.X() = 0;  //【改】
            if (!inSponge(blocks.top()))
                return p; //进入空方块：返回，进行下一次传播（propagate）。
        }
        while (blocks.size() < init_blocks.size() + RELATIVE_FRACTAL_ITER) {  //进方块
            Vector3D<int> pos;
            pos = (CvtInt){ 0, (int)p.Y(), (int)p.Z() }; //【改】
            p = enterBlock(p, pos);
            blocks.push(pos);
            if (!inSponge(pos))
                break;
        }
    }
    return p; // 只是为了消除 WARNING
}

/// @brief 反演变换后的射线与和坐标轴平行或垂直的正方形求交
/// @tparam Cvt space converter, 标示正方形法向量与哪条轴平行（把它放第一个）
/// @param ray 反演变换前的光线
/// @param sqr 正方形左下（坐标值最小的顶点）坐标
/// @param a 正方形边长
/// @return 求出的两个根，用 Ray::at 可以求出坐标。
template<typename Cvt>
std::pair<double, double> invIntersect(Ray ray, V3d sqr, double a) {
    if (((Cvt)sqr).X() == 0) {
        const double res = (((Cvt)ray.origin).X() - ((Cvt)sqr).X() * dot(ray.origin, ray.origin)) / (-((Cvt)ray.direction).X() + 2 * ((Cvt)sqr).X() * dot(ray.origin, ray.direction));
        if (res >= DIST_EPS)
            return std::make_pair(res, INF);
        else
            return std::make_pair(INF, INF);
    }
    const double tmp = 2 * ((Cvt)sqr).X() * dot(ray.origin, ray.direction) - ((Cvt)ray.direction).X();
    const double delta = tmp * tmp + 4 * dot(ray.direction, ray.direction) * (((Cvt)ray.origin).X() - ((Cvt)sqr).X() * dot(ray.origin, ray.origin)) * ((Cvt)sqr).X();
    if (delta < 0)
        return std::make_pair(INF, INF);
    const double denom = 2 * ((Cvt)sqr).X() * dot(ray.direction, ray.direction);
    const double term2 = sqrt(delta) / denom;
    const double term1 = (((Cvt)ray.direction).X() - 2 * ((Cvt)sqr).X() * dot(ray.origin, ray.direction)) / denom;
    double x1 = term1 + term2;
    double x2 = term1 - term2;
    const V3d p1 = inv(ray.at(x1));
    const V3d p2 = inv(ray.at(x2));
    if (
        ((Cvt)p1).Y() < ((Cvt)sqr).Y() || ((Cvt)p1).Y() > ((Cvt)sqr).Y() + a ||
        ((Cvt)p1).Z() < ((Cvt)sqr).Z() || ((Cvt)p1).Z() > ((Cvt)sqr).Z() + a
    )   x1 = INF;   // 注意这里这样写的话直线不是封闭曲线，要是想让射线从无穷远处射回后脑勺应该写 x1 = -DIST_EPS;
    if (
        ((Cvt)p2).Y() < ((Cvt)sqr).Y() || ((Cvt)p2).Y() > ((Cvt)sqr).Y() + a ||
        ((Cvt)p2).Z() < ((Cvt)sqr).Z() || ((Cvt)p2).Z() > ((Cvt)sqr).Z() + a
    )   x2 = INF;   // 注意这里这样写的话直线不是封闭曲线，要是想让射线从无穷远处射回后脑勺应该写 x2 = -DIST_EPS;
    if (x1 < DIST_EPS)
        x1 = INF;
    if (x2 < DIST_EPS)
        x2 = INF;
    return std::make_pair(x1, x2);
}

double res_dist;
double min_dist;

//单次光线计算
//return: 若撞到面了就返回 true
bool propagate(Ray ray) {
    // ray.origin -= { SPONGE_L / 2.0, SPONGE_L / 2.0, SPONGE_L / 2.0 };  // 一开始就把 ray 偏移过了，后面不用处理了。
    
    res_dist = 0;

    while (true) {
        // std::cout << "f";
        V3d v = blockVertex(blocks);
        v -= { SPONGE_L / 2.0, SPONGE_L / 2.0, SPONGE_L / 2.0 };
        V3d dbg_ = inv(ray.origin); // DEBUG

        const double a = side_len[blocks.size()];   // 曾经的 BUG 出现地

        std::pair<double, double> r1 = invIntersect<dXYZ>(ray, v, a);
        std::pair<double, double> r2 = invIntersect<dXYZ>(ray, v + (V3d){ a, 0, 0 }, a);
        std::pair<double, double> r3 = invIntersect<dYXZ>(ray, v, a);
        std::pair<double, double> r4 = invIntersect<dYXZ>(ray, v + (V3d){ 0, a, 0 }, a);
        std::pair<double, double> r5 = invIntersect<dZXY>(ray, v, a);
        std::pair<double, double> r6 = invIntersect<dZXY>(ray, v + (V3d){ 0, 0, a }, a);
        
        const double r = std::min({
            r1.first, r1.second, r2.first, r2.second,
            r3.first, r3.second, r4.first, r4.second,
            r5.first, r5.second, r6.first, r6.second
        });

        res_dist += r;

        if (r == INF) {
            //std::cout << "Warning: didn't hit surface in non-first propagate.\n";
            return false;   // HERE
        }

        V3d p = inv(ray.at(r));

        V3d relative_p = (p - v) / a * (double)SPONGE_L;
        
        if (r1.first == r || r1.second == r) {
            p.x = v.x;
            relative_p.x = 0;
            calc<dXYZ, XYZ<int> >(relative_p, false);
            res_normal = { 1, 0, 0 };
        } else if (r2.first == r || r2.second == r) {
            p.x = v.x + a;
            relative_p.x = SPONGE_L;
            calc<dXYZ, XYZ<int> >(relative_p, true);
            res_normal = { 1, 0, 0 };
        } else if (r3.first == r || r3.second == r) {
            p.y = v.y;
            relative_p.y = 0;
            calc<dYXZ, YXZ<int> >(relative_p, false);
            res_normal = { 0, 1, 0 };
        } else if (r4.first == r || r4.second == r) {
            p.y = v.y + a;
            relative_p.y = SPONGE_L;
            calc<dYXZ, YXZ<int> >(relative_p, true);
            res_normal = { 0, 1, 0 };
        } else if (r5.first == r || r5.second == r) {
            p.z = v.z;
            relative_p.z = 0;
            calc<dZXY, ZXY<int> >(relative_p, false);
            res_normal = { 0, 0, 1 };
        } else if (r6.first == r || r6.second == r) {
            p.z = v.z + a;
            relative_p.z = SPONGE_L;
            calc<dZXY, ZXY<int> >(relative_p, true);
            res_normal = { 0, 0, 1 };
        } else {
            std::cout << "Error: propgrate min ..." << r << '\n';
        }

        // copied from old code.
        if (blocks.size() >= init_blocks.size() + RELATIVE_FRACTAL_ITER) {
            res_ray = ray;
            return true;
        }
        // else if (blocks.empty()) {
        //     res_ray.direction = ray.direction;
        //     return false;
        // }
        ray.origin = inv(p);    // 还有另一种写法: ray.origin = ray.at(r);
        
    }

    return true;
}

// //单次光线计算 从最外层块外射入情况
// //三倍体代码（绷）
// //return: 若撞到面了就返回 true
// bool firstPropagate(Ray ray) {

//     // 不偏移 ray 了。
//     // ray.origin -= { SPONGE_L / 2.0, SPONGE_L / 2.0, SPONGE_L / 2.0 };  // 一开始就把 ray 偏移过了，后面不用处理了。

//     V3d v = { -SPONGE_L / 2.0, -SPONGE_L / 2.0, -SPONGE_L / 2.0 };

//     const double a = SPONGE_L;

//     std::pair<double, double> r1 = invIntersect<dXYZ>(ray, v, a);
//     std::pair<double, double> r2 = invIntersect<dXYZ>(ray, v + (V3d){ a, 0, 0 }, a);
//     std::pair<double, double> r3 = invIntersect<dYXZ>(ray, v, a);
//     std::pair<double, double> r4 = invIntersect<dYXZ>(ray, v + (V3d){ 0, a, 0 }, a);
//     std::pair<double, double> r5 = invIntersect<dZXY>(ray, v, a);
//     std::pair<double, double> r6 = invIntersect<dZXY>(ray, v + (V3d){ 0, 0, a }, a);
    
//     const double r = std::min({
//         r1.first, r1.second, r2.first, r2.second,
//         r3.first, r3.second, r4.first, r4.second,
//         r5.first, r5.second, r6.first, r6.second
//     });

//     if (r == INF) {
//         return false;
//     }

//     V3d p = inv(ray.at(r));

//     V3d relative_p = p - v;
    
//     if (r1.first == r || r1.second == r) {
//         p.x = v.x;
//         relative_p.x = 0;
//         calc<dXYZ, XYZ<int> >(relative_p, false);
//         res_normal = { 1, 0, 0 };
//     } else if (r2.first == r || r2.second == r) {
//         p.x = v.x + a;
//         relative_p.x = SPONGE_L;
//         calc<dXYZ, XYZ<int> >(relative_p, true);
//         res_normal = { 1, 0, 0 };
//     } else if (r3.first == r || r3.second == r) {
//         p.y = v.y;
//         relative_p.y = 0;
//         calc<dYXZ, YXZ<int> >(relative_p, false);
//         res_normal = { 0, 1, 0 };
//     } else if (r4.first == r || r4.second == r) {
//         p.y = v.y + a;
//         relative_p.y = SPONGE_L;
//         calc<dYXZ, YXZ<int> >(relative_p, true);
//         res_normal = { 0, 1, 0 };
//     } else if (r5.first == r || r5.second == r) {
//         p.z = v.z;
//         relative_p.z = 0;
//         calc<dZXY, ZXY<int> >(relative_p, false);
//         res_normal = { 0, 0, 1 };
//     } else if (r6.first == r || r6.second == r) {
//         p.z = v.z + a;
//         relative_p.z = SPONGE_L;
//         calc<dZXY, ZXY<int> >(relative_p, true);
//         res_normal = { 0, 0, 1 };
//     } else {
//         std::cout << "Error: propgrate min ...";
//     }

//     // copied from old code.
//     if (blocks.size() >= init_blocks.size() + RELATIVE_FRACTAL_ITER) {
//         res_ray = ray;
//         return true;
//     }
//     // else if (blocks.empty()) {
//     //     res_ray.direction = ray.direction;
//     //     return false;
//     // }

//     ray.origin = inv(p);    // 还有另一种写法: ray.origin = ray.at(r);

//     return propagate(ray);
// }

void initDist() { min_dist = INF; }
void updateDist(const double dist) {
    if (dist < min_dist)
        min_dist = dist;
}
void calcVelocity() {
    if (min_dist >= INF) {
        move_velocity = INIT_MOVE_VELOCITY;
        return;
    }
    move_velocity = min_dist * INIT_MOVE_VELOCITY;
}

/// @brief 光线追踪着色器（多次光线反射计算）
/// 现在暂时还是不带反射的版本
Color rayShader(Ray ray) {

    blocks = init_blocks;
    //memcpy(blocks.arr, init_blocks.arr, (init_blocks.size() + 2) * sizeof(Cube));   //这里好像 +1 就可以了，但是还是避免一下意外的溢出。
    //blocks.cursor = blocks.arr + init_blocks.size();
    ray.direction.normalize();  // （至少是）计算 res_dist 需要这一行
    bool hit = /*blocks.empty() ? firstPropagate(ray) : */propagate(ray);
    //std::cout << hit << std::endl;
    if (!hit)   return SKY_COLOR;
    // else if (res_normal.x == 1)
    //     return { 1.0, 0.0, 0.0 };
    // else if (res_normal.y == 1)
    //     return { 0.0, 1.0, 0.0 };
    // //else if (res_normal.z == 1)
    //     return { 0.0, 0.0, 1.0 };
    return { res_normal.x, res_normal.y, res_normal.z };

}

// 计算距离趋向无穷时，此方向光线的亮度（假定光线在传播过程中不衰减）
Color calcLight(V3d direction) {
    return SKY_COLOR; //先不加方向光
}

// 生成 l~r 的随机数
double randLR(const double l, const double r) {
    return l + rng() / rng.max() * (r - l);
}

/// @brief 零到一之间的随机数
double rand01() {
    return rng() / (double)rng.max();
}

// 生成单位球体内的随机向量
V3d randBall() {
    const double r = rand01();
    const double theta = 2 * M_PI * rand01();   // 俯仰角
    const double phi = M_PI * rand01();         // 方位角
    const double sin_phi = sin(phi);
    return (V3d){ sin_phi * cos(theta), sin_phi * sin(theta), cos(phi) } * r;
}

// 适用于与坐标轴垂直的面，要求法向量标准化
V3d reflectVertical(V3d in, V3d normal) {
	if (abs(normal.x - 1) < DIST_EPS)
        return { -in.x, in.y, in.z };
    if (abs(normal.y - 1) < DIST_EPS)
        return { in.x, -in.y, in.z };
    // if (abs(normal.z) < DIST_EPS)
        return { in.x, in.y, -in.z };
}

// 光线偏转
void deflectRay(V3d &direction, V3d normal) {
    V3d res;
    direction.normalize();
    if (dot(direction, normal) < 0)
        normal = { -normal.x, -normal.y, -normal.z };   // 如果法向量是扎向面里的，那么把它反转
    while (true) {
        res = direction + randBall() * SPONGE_ROUGHNESS;
        if (dot(res, normal) > 0) {
            direction = res;
            return; //一直循环直到产生在面外的反射光线为止。
        }
    }
}

// 计算光线经过传播、反射等之后的颜色
Color RTShader(Ray ray) {
    blocks = init_blocks;
    res_ray = ray;
    Color color = { 2, 2, 2 };
    double reflect_param = 1;   // 俄罗斯轮盘赌的方法里 根据反射次数算出的这个光线的权值
    do {
        if (propagate(res_ray)) {   // 碰上了
            res_ray.direction = reflectVertical(res_ray.direction, res_normal); // 反射
            deflectRay(res_ray.direction, res_normal);
            blocks = temp_blocks;
            color *= (Color){ 0.8, 0.8, 0.8 };  // 这里定义海绵颜色
        } else {
            color *= calcLight(res_ray.direction);
            return color * reflect_param;
        }
        reflect_param *= 1 / REFLECT_PROBABILITY;
    } while (rand01() < REFLECT_PROBABILITY);
    return { 0, 0, 0 };
}

#endif