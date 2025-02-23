//光追功能暂未维护，可能无法使用。想要使用可以去找老代码。
// g++ ./main.cpp -O2 -o main.exe -lgraphics -luuid -lmsimg32 -lgdi32 -limm32 -lole32 -loleaut32 -lwinmm -lgdiplus

#include <iostream>

#define REAL_TIME_SHOW
// #define DEBUG

#include "main_loop.hpp"

Renderer renderer;

void recordPath() {
    mainLoopPR(renderer);
    path_recorder.smoothenPath();
    path_recorder.storgePath("PATH");
    // path_recorder.readPath("PATH");
    path_recorder.showPath(renderer);
}

void generateImages() {
    path_recorder.readPath("PATH");
    renderer.init();
    // path_recorder.showPath(renderer);
    path_recorder.generateImage(renderer);
}

int main() {
    // initSideLen();
    // blocks.push({ 1, 1, 1 });
    // blocks.push({ 1, 0, 1 });
    // blockVertex(blocks).print();

    //calcInitBlock({ 2, 2, 2 });
    //rayShader({ { 2, 2, 2 }, { -1.1, -1.2, -1.3 } });
    //rayShader({ { 2, 2, 2 }, { -1.1, -1.2, -1.3 } });
    //renderer.init();
    //renderer.render();

    mainLoopAA(renderer);

    // recordPath();
    // generateImages();
    
    //getchar();
    return 0;
}