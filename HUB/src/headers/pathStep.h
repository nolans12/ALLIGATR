#pragma once

struct pathStep {
    // 1x3 array of the commanded point at this step [ft]
    float path[3];
    pathStep* next;
};