#include "headers/minInd.h"

short int* minInd(float arr[2][2]) {
    short int i, j;
    float smallest = arr[0][0];
    static short int indices[2];
    for (i=0; i<2; i++) {
        for (j=0; j<2; j++) {
            if (smallest > arr[i][j]) {
                smallest = arr[i][j];
                indices[0] = i;
                indices[1] = j;
            }
        }
    }
    return indices;
}