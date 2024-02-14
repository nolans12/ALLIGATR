#include "SearchPath.h"
#include <math.h>
using namespace std;

float* search(uas drone, enviornment env, float* path) {
    /* 
     * Returns the next point for a creeping line search pattern
     * Input:
     *     drone  -  uas object
     *     env    -  enviornment object
     *     path   -  length 3 vector of the previously commanded point [x, y, z]
     *     Any units, so long as consistent between passed arguments
     * Output:
     *     path   - length 3 vector of the new commanded point [x, y, z]
     *     drone  - uas object with p property updated
     *     env    - enviornment object with boundery property updated
     * The function makes use of the uas p iterator property to track the
     * phases of the creeping line search pattern with the following:
     *     0:    has not begun the search pattern
     *     1:    is moving to the start point of the search pattern
     *     even: is moving left to right (along x)
     *     odd:  is moving up or down (along y)
     * The value of p is incremented once the drone has entered the search phase, 
     * and after when it reaches the commanded point for that phase of the search pattern.
     * The pattern is constructed such that the left/right phases always moves to and from W away from the boundary,
     * while the up/down phases only go either up or down during a search,
     * with the up/down length calculated using L and the value of p. 
     * The upper/lower bounds are calculated from how many Ls fit in the y boundary.
     */

    /*
     *   notes for further implementation:
     *   add drone, enviornment class file in HUB/src
     *   env.bounds in format [[x1, y1]; [x2, y2]]
     *   drone.state in format [x, y, z]
     *   assumes variables of drone and env are public
     *   need to define getFOVDims that takes in uas object and outputs float pointer to array [L, W]
     *   need to define reachedPoint that takes in uas object and point to float array of path and outputs true or false
     *   need to define minInd that takes in 2x2 float array and returns pointer to float array of [i, j] indices of the smallest array entry
     */


    // variables always used
    float* fovDims;
    float L, W, yEnd;
    short int xneg = 1, yneg = 1, pEnd;

    //variables used when arranging bounds
    float d[2][2] = {{0., 0.}, {0., 0.}};
    short int i, j;
    short int* indices;
    float x1_old, y1_old;

    // length and width of the camera projected box on ground
    fovDims = getFOVDims(drone);
    // remove some distance to allow for margin of error as well as RGV moving into last search line from new search line before drone reaches that point
    // 10% margin can be adjusted as needed
    L = fovDims[0] * 0.8;
    W = fovDims[1] * 0.8;
    // appropriately changes addition to subtraction and vice versa if the boundary has been changed to require it
    if (env.bounds[1][0] < env.bounds[0][0]) {
        xneg = -1;
    }
    if (env.bounds[1][1] < env.bounds[0][1]) {
        yneg = -1;
    }

    if ((drone.p != 0) && reachedPoint(drone, path)) {
        // if the drone has reached the commanded point from the previous phase of search, increase the iteration counter p by 1
        drone.p++;
    }

    if (drone.p == 0) {
        /* if the drone has not entered the search pattern yet,
         * first change the boundary variables such that x1,y1 is the closest boundary corner to the drone, 
         * then move to the start location for the creeping line search pattern
         */
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++) {
                // gets the distance from the current drone location to each corner
                d[i][j] = sqrt(pow(env.bounds[i][0]-drone.state[0], 2) + pow(env.bounds[j][1]-drone.state[1], 2));
            }
        }
        // gets the indices of the closest corner
        indices = minInd(d);
        i = indices[0];
        j = indices[1];
        // check if bounds are same first to save some computation time
        if ((i != 0) || (j != 0)){
            // change x1,y1 to the closest corner then change x2,y2 accordingly
            x1_old = env.bounds[0][0];
            y1_old = env.bounds[0][1];
            env.bounds[0][0] = env.bounds[i][0];
            env.bounds[0][1] = env.bounds[j][1];
            if (i == 1) {
                env.bounds[1][0] = x1_old;
            }
            if (j == 1) {
                env.bounds[1][1] = y1_old;
            }
            if (env.bounds[1][0] < env.bounds[0][0]) {
                xneg = -1;
            }
            if (env.bounds[1][1] < env.bounds[0][1]) {
                yneg = -1;
            }
            // set commanded point to the closest corner + camera buffer
            path[0] = env.bounds[0][0]+xneg*W;
            path[1] = env.bounds[0][1]+yneg*L;
            path[2] = drone.state[3];
            drone.p++;
        }
    }
    else if ((drone.p % 2) == 0) {
        // if p is even, the drone is on left-right portion of flight
        if (abs(drone.state[0]-(env.bounds[0][0]+xneg*W)) < 0.2) {
            // if drone is on the left, set commanded point to the right
            path[0] = env.bounds[0][1]-xneg*W;
            path[1] = drone.state[1];
            path[2] = drone.state[2];
        }
        else if (abs(drone.state[0]-(env.bounds[1][0]+xneg*W)) < 0.2) {
            // if drone is on the right, set commanded point to the left
            path[0] = env.bounds[0][0]+xneg*W;
            path[1] = drone.state[1];
            path[2] = drone.state[2];
        }
    }
    else if (((drone.p % 2) == 1) && (drone.p != 1)) {
        // if p is odd, the drone is on upwards portion of flight
        // ignore when p is 1 - moving to search start point
        path[0] = drone.state[0];
        path[1] = env.bounds[0][1]+yneg*L*drone.p;
        path[2] = drone.state[2];
    }

    // get the p and y of the last phase we want
    pEnd = floor(abs(env.bounds[1][1]-env.bounds[0][1])/L);
    yEnd = env.bounds[0][1] + yneg*L*pEnd;
    if ((pEnd%2) != 0) {
        pEnd++;
    }
    if ((drone.p > pEnd) || (((yneg == 1) && (drone.state[1] >= yEnd)) || ((yneg == -1) && (drone.state[1] <= yEnd)))) {
        // if we are outside the last p and y we want, set p = 0, restarting search
        drone.p = 0;
    }
}
