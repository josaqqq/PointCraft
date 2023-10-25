#include <glm/glm.hpp>
#include <vector>
#include <iostream>

#include "model.hpp"
#include "geometry.hpp"

extern float INF;
extern float EPS;

/*
    多角形の内外判定：
        ある点からx軸に平行な半直線を引き、この半直線と多角形の交点が
        奇数個なら内部、偶数個なら外部と判定する。
*/
bool inside_polygon(double x, double y, std::vector<Vertex> &vertices, int siz) {
    int cross_count = 0;
    for (int i = 0; i < siz; i++) {
        glm::dvec2 u = glm::dvec2(vertices[i].x, vertices[i].y);
        glm::dvec2 v = glm::dvec2(vertices[(i + 1)%siz].x, vertices[(i + 1)%siz].y);

        // 半直線に対して同じサイドなら交わらない
        if ((u.y - y)*(v.y - y) >= 0.0d) continue;

        // u-vと半直線が交わらない
        double cross_x = u.x + (v.x - u.x)*std::abs(y - u.y)/std::abs(v.y - u.y);
        if (cross_x < x) continue;

        cross_count++;
    }

    if (cross_count % 2 == 0) return false;
    else return true;
}