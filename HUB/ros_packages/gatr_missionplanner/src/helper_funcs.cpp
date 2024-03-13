#include "headers/helper_funcs.h"

// Returns the indices of the smallest value in a 2x2 array
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

int sign(float num) {
    return (num >= 0) - (num < 0);
}

// bool isRGVAClosest(std::vector<double> droneState, std::vector<double> rgvAPosition, std::vector<double> rgvBPosition) {
//     /* Returns true if the closest RGV to the drone is RGV-A
//      * Input:  
//      *         droneState - 1x4 double vector of current drone position
//      *         rgvAPosition   - 1x4 vector of current RGV-A position
//      *         rgvBPosition   - 1x4 vector of current RGV-B position
//      * Output: 
//      *         isRGVAClosest - bool true or false
//      */

//     double distA, distB;
//     // get the x-y plane 2d distance from the drone to each RGV
//     distA = pow(pow(droneState[0]-rgvAPosition[0], 2) + pow(droneState[1]-rgvAPosition[1], 2), 0.5);
//     distB = pow(pow(droneState[0]-rgvBPosition[0], 2) + pow(droneState[1]-rgvBPosition[1], 2), 0.5);

//     // return true if the distance from the drone to RGV-A is smaller than the distance to RGV-B
//     return (distA <= distB);
// }

std::vector<double> cross(std::vector<double> const &a, std::vector<double> const &b) {
    /* Cross product of 2 1x3 double vectors; a x b
     * Input:
     *         a - 1x3 double vector, left vector in cross product
     *         b - 1x3 double vector, right vector in cross product
     * Output:
     *         cross - 1x3 double vector, result of a x b
     */

    std::vector<double> r (a.size());  
    r[0] = a[1]*b[2]-a[2]*b[1];
    r[1] = a[2]*b[0]-a[0]*b[2];
    r[2] = a[0]*b[1]-a[1]*b[0];
    return r;
}

double dot(std::vector<double> const &a, std::vector<double> const &b) {
    /* Dot product of 2 1x3 double vectors, a ⋅ b
     * Input:  
     *         a - 1x3 double vector to be dotted
     *         b - 1x3 double vector to be dotted
     * Output:
     *         dot - double, result of a ⋅ b
     */

    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

double norm(std::vector<double> const &a) {
    /* Get magnitude/norm of arbitrary length double vector
     * Input: 
     *         a - 1d double vector of arbitrary length
     * Output:
     *         norm - double, magnitude of vector a
     */

    double sum = 0;
    for (int i = 0; i < a.size(); i++) {
        sum += pow(a[i], 2.0);
    }
    return pow(sum, 0.5);
}
