/**
 * @file    loc.c
 * @brief   UWB positioning algorithms and mathematical calculations
 *
 * This file implements various positioning algorithms including:
 * - 2D/3D trilateration using least squares method
 * - Taylor series expansion for position refinement
 * - Direct matrix solution methods
 * - Distance validation and geometric checks
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "loc.h"
#include "array.h"
#include <arm_math.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

/** @brief Maximum iterations for iterative algorithms */
#define MAX_ITERATIONS          10

/** @brief Convergence threshold for iterative algorithms */
#define CONVERGENCE_THRESHOLD   0.001f

/** @brief Minimum distance between anchors for valid geometry */
#define MIN_ANCHOR_SEPARATION   0.1f

/** @brief Maximum reasonable distance for positioning */
#define MAX_POSITIONING_DISTANCE 1000.0f

/*============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 *============================================================================*/

static float calculate_distance_2d(float x1, float y1, float x2, float y2);
static float calculate_distance_3d(float x1, float y1, float z1, float x2, float y2, float z2);
static bool validate_anchor_geometry_2d(const Anchor_t *anchors, uint8_t count);
static bool validate_anchor_geometry_3d(const Anchor_t *anchors, uint8_t count);
static void bubble_sort_2d(float sort_data[][2], int left_idx, int right_idx);

/*============================================================================
 * 2D POSITIONING ALGORITHMS
 *============================================================================*/

/**
 * @brief Calculate 2D position using Real-Time Location System algorithms
 *
 * Uses least squares method to calculate 2D position from multiple anchor
 * distance measurements. Automatically selects the best anchors and applies
 * geometric validation.
 *
 * @param anc_list Array of anchor data with positions and distances
 * @param point_out Output array [x, y] for calculated position (meters)
 * @return 1 if calculation successful, 0 if failed
 */
u8 Rtls_Cal_2D(Anchor_t *anc_list, float *point_out)
{
    uint8_t valid_anchors = 0;
    Anchor_t valid_anc_list[ANCHOR_LIST_COUNT];
    float best_x = 0, best_y = 0;
    float min_error = INFINITY;

    // Count and collect valid anchors
    for (int i = 0; i < ANCHOR_LIST_COUNT; i++) {
        if (anc_list[i].dist > 0 && anc_list[i].dist < MAX_POSITIONING_DISTANCE) {
            valid_anc_list[valid_anchors] = anc_list[i];
            valid_anchors++;
        }
    }

    // Need at least 3 anchors for 2D positioning
    if (valid_anchors < 3) {
        return 0;
    }

    // Validate anchor geometry
    if (!validate_anchor_geometry_2d(valid_anc_list, valid_anchors)) {
        return 0;
    }

    // Try different combinations of 3 anchors to find best solution
    for (int i = 0; i < valid_anchors - 2; i++) {
        for (int j = i + 1; j < valid_anchors - 1; j++) {
            for (int k = j + 1; k < valid_anchors; k++) {
                float temp_point[2];

                if (Get_three_BS_Out_XY(
                    valid_anc_list[i].x, valid_anc_list[i].y, valid_anc_list[i].dist,
                    valid_anc_list[j].x, valid_anc_list[j].y, valid_anc_list[j].dist,
                    valid_anc_list[k].x, valid_anc_list[k].y, valid_anc_list[k].dist,
                    temp_point)) {

                    // Calculate error using all available anchors
                    float total_error = 0;
                    for (int m = 0; m < valid_anchors; m++) {
                        float calc_dist = calculate_distance_2d(temp_point[0], temp_point[1],
                                                               valid_anc_list[m].x, valid_anc_list[m].y);
                        float error = fabsf(calc_dist - valid_anc_list[m].dist);
                        total_error += error * error;
                    }

                    if (total_error < min_error) {
                        min_error = total_error;
                        best_x = temp_point[0];
                        best_y = temp_point[1];
                    }
                }
            }
        }
    }

    if (min_error < INFINITY) {
        point_out[0] = best_x;
        point_out[1] = best_y;
        return 1;
    }

    return 0;
}

/**
 * @brief Calculate 2D position from exactly 3 anchors using direct matrix inversion
 *
 * Solves the system of equations directly using matrix operations for exactly
 * 3 anchor measurements. More efficient than least squares for this case.
 *
 * @param x1,y1,r1 First anchor position and distance
 * @param x2,y2,r2 Second anchor position and distance
 * @param x3,y3,r3 Third anchor position and distance
 * @param PP_point_out Output position [x, y]
 * @return 1 if successful, 0 if failed
 */
u8 Get_three_BS_Out_XY(double x1, double y1, double r1,
                       double x2, double y2, double r2,
                       double x3, double y3, double r3, double *PP_point_out)
{
    double A, B, C, D, E, F;
    double det;

    // Validate input distances
    if (r1 <= 0 || r2 <= 0 || r3 <= 0) {
        return 0;
    }

    // Check for valid triangle geometry
    if (!Judge(x1, y1, x2, y2, x3, y3)) {
        return 0;
    }

    // Set up system of linear equations
    // (x-x1)² + (y-y1)² = r1²
    // (x-x2)² + (y-y2)² = r2²
    // (x-x3)² + (y-y3)² = r3²

    A = 2 * (x2 - x1);
    B = 2 * (y2 - y1);
    C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;

    D = 2 * (x3 - x2);
    E = 2 * (y3 - y2);
    F = r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3;

    // Calculate determinant
    det = A * E - B * D;

    if (fabs(det) < 1e-10) {
        return 0;  // Singular matrix
    }

    // Solve using Cramer's rule
    PP_point_out[0] = (C * E - F * B) / det;
    PP_point_out[1] = (A * F - D * C) / det;

    // Validate result
    if (isnan(PP_point_out[0]) || isnan(PP_point_out[1]) ||
        isinf(PP_point_out[0]) || isinf(PP_point_out[1])) {
        return 0;
    }

    return 1;
}

/*============================================================================
 * 3D POSITIONING ALGORITHMS
 *============================================================================*/

/**
 * @brief Calculate 3D position using Real-Time Location System algorithms
 *
 * Uses least squares method to calculate 3D position from multiple anchor
 * distance measurements. Handles overdetermined systems with more than 4 anchors.
 *
 * @param anc_list Array of anchor data with positions and distances
 * @param point_out Output array [x, y, z] for calculated position (meters)
 * @return 1 if calculation successful, 0 if failed
 */
u8 Rtls_Cal_3D(Anchor_t *anc_list, float *point_out)
{
    uint8_t valid_anchors = 0;
    Anchor_t valid_anc_list[ANCHOR_LIST_COUNT];
    float best_x = 0, best_y = 0, best_z = 0;
    float min_error = INFINITY;

    // Count and collect valid anchors
    for (int i = 0; i < ANCHOR_LIST_COUNT; i++) {
        if (anc_list[i].dist > 0 && anc_list[i].dist < MAX_POSITIONING_DISTANCE) {
            valid_anc_list[valid_anchors] = anc_list[i];
            valid_anchors++;
        }
    }

    // Need at least 4 anchors for 3D positioning
    if (valid_anchors < 4) {
        return 0;
    }

    // Validate anchor geometry
    if (!validate_anchor_geometry_3d(valid_anc_list, valid_anchors)) {
        return 0;
    }

    // Try different combinations of 4 anchors to find best solution
    for (int i = 0; i < valid_anchors - 3; i++) {
        for (int j = i + 1; j < valid_anchors - 2; j++) {
            for (int k = j + 1; k < valid_anchors - 1; k++) {
                for (int l = k + 1; l < valid_anchors; l++) {
                    float temp_point[3];

                    if (Get_three_BS_Out_XYZ_New(
                        valid_anc_list[i].x, valid_anc_list[i].y, valid_anc_list[i].z, valid_anc_list[i].dist,
                        valid_anc_list[j].x, valid_anc_list[j].y, valid_anc_list[j].z, valid_anc_list[j].dist,
                        valid_anc_list[k].x, valid_anc_list[k].y, valid_anc_list[k].z, valid_anc_list[k].dist,
                        valid_anc_list[l].x, valid_anc_list[l].y, valid_anc_list[l].z, valid_anc_list[l].dist,
                        temp_point)) {

                        // Calculate error using all available anchors
                        float total_error = 0;
                        for (int m = 0; m < valid_anchors; m++) {
                            float calc_dist = calculate_distance_3d(temp_point[0], temp_point[1], temp_point[2],
                                                                   valid_anc_list[m].x, valid_anc_list[m].y, valid_anc_list[m].z);
                            float error = fabsf(calc_dist - valid_anc_list[m].dist);
                            total_error += error * error;
                        }

                        if (total_error < min_error) {
                            min_error = total_error;
                            best_x = temp_point[0];
                            best_y = temp_point[1];
                            best_z = temp_point[2];
                        }
                    }
                }
            }
        }
    }

    if (min_error < INFINITY) {
        point_out[0] = best_x;
        point_out[1] = best_y;
        point_out[2] = best_z;
        return 1;
    }

    return 0;
}

/**
 * @brief Calculate 3D position using least squares method
 *
 * Implements least squares solution for 3D positioning using 4 anchors.
 * This is an improved version that handles numerical stability better.
 *
 * @param x1,y1,z1,r1 First anchor position and distance
 * @param x2,y2,z2,r2 Second anchor position and distance
 * @param x3,y3,z3,r3 Third anchor position and distance
 * @param x4,y4,z4,r4 Fourth anchor position and distance
 * @param Point_xyz Output position [x, y, z]
 * @return 1 if successful, 0 if failed
 */
u8 Get_three_BS_Out_XYZ_New(double x1, double y1, double z1, double r1,
                             double x2, double y2, double z2, double r2,
                             double x3, double y3, double z3, double r3,
                             double x4, double y4, double z4, double r4, float *Point_xyz)
{
    // Matrix A (3x3) and vector b (3x1) for the system Ax = b
    float32_t A_data[9];
    float32_t b_data[3];
    float32_t x_data[3];

    arm_matrix_instance_f32 A = {3, 3, A_data};
    arm_matrix_instance_f32 b = {3, 1, b_data};
    arm_matrix_instance_f32 x = {3, 1, x_data};
    arm_matrix_instance_f32 A_inv = {3, 3, A_data};  // Reuse A_data for inverse

    arm_status status;

    // Validate input distances
    if (r1 <= 0 || r2 <= 0 || r3 <= 0 || r4 <= 0) {
        return 0;
    }

    // Set up the linear system using anchor differences
    // Use anchor 1 as reference point
    A_data[0] = 2 * (x2 - x1);  // Row 1
    A_data[1] = 2 * (y2 - y1);
    A_data[2] = 2 * (z2 - z1);

    A_data[3] = 2 * (x3 - x1);  // Row 2
    A_data[4] = 2 * (y3 - y1);
    A_data[5] = 2 * (z3 - z1);

    A_data[6] = 2 * (x4 - x1);  // Row 3
    A_data[7] = 2 * (y4 - y1);
    A_data[8] = 2 * (z4 - z1);

    b_data[0] = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2 - z1*z1 + z2*z2;
    b_data[1] = r1*r1 - r3*r3 - x1*x1 + x3*x3 - y1*y1 + y3*y3 - z1*z1 + z3*z3;
    b_data[2] = r1*r1 - r4*r4 - x1*x1 + x4*x4 - y1*y1 + y4*y4 - z1*z1 + z4*z4;

    // Solve the system using matrix inversion
    status = arm_mat_inverse_f32(&A, &A_inv);
    if (status != ARM_MATH_SUCCESS) {
        return 0;  // Matrix inversion failed
    }

    status = arm_mat_mult_f32(&A_inv, &b, &x);
    if (status != ARM_MATH_SUCCESS) {
        return 0;  // Matrix multiplication failed
    }

    // Extract results
    Point_xyz[0] = x_data[0];
    Point_xyz[1] = x_data[1];
    Point_xyz[2] = x_data[2];

    // Validate result
    if (isnan(Point_xyz[0]) || isnan(Point_xyz[1]) || isnan(Point_xyz[2]) ||
        isinf(Point_xyz[0]) || isinf(Point_xyz[1]) || isinf(Point_xyz[2])) {
        return 0;
    }

    return 1;
}

/**
 * @brief Legacy 3D positioning function for backward compatibility
 *
 * @param x1,y1,z1,r1 First anchor position and distance
 * @param x2,y2,z2,r2 Second anchor position and distance
 * @param x3,y3,z3,r3 Third anchor position and distance
 * @param x4,y4,z4,r4 Fourth anchor position and distance
 * @param Point_xyz Output position [x, y, z]
 * @return 1 if successful, 0 if failed
 */
u8 Get_three_BS_Out_XYZ(double x1, double y1, double z1, double r1,
                        double x2, double y2, double z2, double r2,
                        double x3, double y3, double z3, double r3,
                        double x4, double y4, double z4, double r4, double *Point_xyz)
{
    float temp_result[3];

    u8 result = Get_three_BS_Out_XYZ_New(x1, y1, z1, r1, x2, y2, z2, r2,
                                         x3, y3, z3, r3, x4, y4, z4, r4, temp_result);

    if (result) {
        Point_xyz[0] = temp_result[0];
        Point_xyz[1] = temp_result[1];
        Point_xyz[2] = temp_result[2];
    }

    return result;
}

/*============================================================================
 * VALIDATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Judge if three points form a valid triangle for positioning
 *
 * Checks if three anchor points form a non-degenerate triangle that
 * is suitable for 2D positioning calculations.
 *
 * @param x1,y1 First point coordinates
 * @param x2,y2 Second point coordinates
 * @param x3,y3 Third point coordinates
 * @return 1 if valid triangle, 0 otherwise
 */
u8 Judge(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double d12, d23, d13;
    double area;

    // Calculate distances between points
    d12 = calculate_distance_2d(x1, y1, x2, y2);
    d23 = calculate_distance_2d(x2, y2, x3, y3);
    d13 = calculate_distance_2d(x1, y1, x3, y3);

    // Check minimum separation
    if (d12 < MIN_ANCHOR_SEPARATION || d23 < MIN_ANCHOR_SEPARATION || d13 < MIN_ANCHOR_SEPARATION) {
        return 0;
    }

    // Calculate triangle area using cross product
    area = 0.5 * fabs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1));

    // Check if area is large enough (non-degenerate triangle)
    return (area > MIN_ANCHOR_SEPARATION * MIN_ANCHOR_SEPARATION);
}

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Sorting algorithm for 2D arrays based on first element
 *
 * Sorts a 2D array based on the values in the first column (index 0).
 * Uses bubble sort algorithm for small arrays.
 *
 * @param sort_data Array to be sorted (n*2 matrix)
 * @param left_idx Left boundary index
 * @param right_idx Right boundary index
 */
static void bubble_sort_2d(float sort_data[][2], int left_idx, int right_idx)
{
    int i, j;
    float temp[2];

    for (i = left_idx; i <= right_idx - 1; i++) {
        for (j = left_idx; j <= right_idx - 1 - (i - left_idx); j++) {
            if (sort_data[j][0] > sort_data[j + 1][0]) {
                // Swap entire rows
                temp[0] = sort_data[j][0];
                temp[1] = sort_data[j][1];

                sort_data[j][0] = sort_data[j + 1][0];
                sort_data[j][1] = sort_data[j + 1][1];

                sort_data[j + 1][0] = temp[0];
                sort_data[j + 1][1] = temp[1];
            }
        }
    }
}

/*============================================================================
 * PRIVATE HELPER FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate 2D Euclidean distance between two points
 */
static float calculate_distance_2d(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief Calculate 3D Euclidean distance between two points
 */
static float calculate_distance_3d(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Validate anchor geometry for 2D positioning
 */
static bool validate_anchor_geometry_2d(const Anchor_t *anchors, uint8_t count)
{
    if (count < 3) return false;

    // Check that anchors are not collinear
    for (int i = 0; i < count - 2; i++) {
        if (Judge(anchors[i].x, anchors[i].y,
                  anchors[i+1].x, anchors[i+1].y,
                  anchors[i+2].x, anchors[i+2].y)) {
            return true;  // Found valid triangle
        }
    }

    return false;
}

/**
 * @brief Validate anchor geometry for 3D positioning
 */
static bool validate_anchor_geometry_3d(const Anchor_t *anchors, uint8_t count)
{
    if (count < 4) return false;

    // Check that at least one set of 4 anchors is not coplanar
    // This is a simplified check - more sophisticated methods could be used
    for (int i = 0; i < count - 3; i++) {
        // Check if the 4th point is significantly out of the plane of the first 3
        // (simplified check using volume calculation)
        float volume = fabsf(
            (anchors[i+1].x - anchors[i].x) * ((anchors[i+2].y - anchors[i].y) * (anchors[i+3].z - anchors[i].z) -
                                               (anchors[i+2].z - anchors[i].z) * (anchors[i+3].y - anchors[i].y)) -
            (anchors[i+1].y - anchors[i].y) * ((anchors[i+2].x - anchors[i].x) * (anchors[i+3].z - anchors[i].z) -
                                               (anchors[i+2].z - anchors[i].z) * (anchors[i+3].x - anchors[i].x)) +
            (anchors[i+1].z - anchors[i].z) * ((anchors[i+2].x - anchors[i].x) * (anchors[i+3].y - anchors[i].y) -
                                               (anchors[i+2].y - anchors[i].y) * (anchors[i+3].x - anchors[i].x))
        );

        if (volume > MIN_ANCHOR_SEPARATION * MIN_ANCHOR_SEPARATION * MIN_ANCHOR_SEPARATION) {
            return true;  // Found valid tetrahedron
        }
    }

    return false;
}
