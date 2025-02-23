#pragma once
#ifndef RENDERER_HPP
#define RENDERER_HPP

#define SHOW_CONSOLE
#include <graphics.h>
#include <iostream>

#include "camera.hpp"
#include "scene.hpp"
#include "stack.hpp"

struct Renderer {
    Camera camera;
    Color graph[H][W];
    int count;

    void init() {
        initgraph(W, H, INIT_RENDERMANUAL);
        initSideLen();    // 你好像不该在这的。我对不起你。
        count = 0;
    }
    void render() {
        initDist();
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                graph[i][j] = rayShader({ camera.pos, camera.pixelToSpace({ i, j }) - camera.pos });
                putpixel(j, i, graph[i][j].toNum());
                updateDist(res_dist);   // 不该在这的。我对不起你。
            }
        }
        calcVelocity();
    };
    void renderRT() {
        count++;
        initDist();
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                if (count != 1)
                    graph[i][j] += RTShader({ camera.pos, camera.pixelToSpace(i + rand01(), j + rand01()) - camera.pos });
                else
                    graph[i][j] = RTShader({ camera.pos, camera.pixelToSpace(i + rand01(), j + rand01()) - camera.pos });  // 若是第一次累积，就清空之前的
                putpixel(j, i, (graph[i][j] / count).limited().toNum());
                updateDist(res_dist);
            }
        }
        calcVelocity();
        std::cout << "accumulated frame count: " << count << '\n';
    }
    void renderAA() {
        count++;
        initDist();
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                if (count != 1)
                    graph[i][j] += rayShader({ camera.pos, camera.pixelToSpace(i + rand01(), j + rand01()) - camera.pos });
                else
                    graph[i][j] = rayShader({ camera.pos, camera.pixelToSpace(i + rand01(), j + rand01()) - camera.pos });  // 若是第一次累积，就清空之前的
#ifdef REAL_TIME_SHOW
                putpixel(j, i, (graph[i][j] / count).limited().toNum());
                updateDist(res_dist);
#endif
            }
        }
        calcVelocity();
        std::cout << count << ' ';
    }
    void clearAccumulate() {
        count = 0;
    }
};

#endif