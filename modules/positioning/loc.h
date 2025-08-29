/**
 * @file    loc.h
 * @brief   UWB positioning algorithms and mathematical calculations header
 * 
 * This file contains function declarations for core positioning algorithms including:
 * - 2D/3D trilateration using least squares
 * - Taylor series expansion for position refinement  
 * - Distance calculations and validation
 * - Legacy positioning function interfaces
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef __LOC_H
#define __LOC_H

#include "data_types.h"
#include "system_config.h"
#include <arm_math.h>
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * MAIN POSITIONING ALGORITHMS
 *============================================================================*/

/**
 * @brief Calculate 2D position using Real-Time Location System algorithms
 * 
 * @param anc_list Array of anchor data with positions and distances
 * @param point_out Output array [x, y] for calculated position (meters)
 * @return 1 if calculation successful, 0 if failed
 */
u8 Rtls_Cal_2D(Anchor_t *anc_list, float *point_out);

/**
 * @brief Calculate 3D position using Real-Time Location System algorithms
 * 
 * @param anc_list Array of anchor data with positions and distances  
 * @param point_out Output array [x, y, z] for calculated position (meters)
 * @return 1 if calculation successful, 0 if failed
 */
u8 Rtls_Cal_3D(Anchor_t *anc_list, float *point_out);

/*============================================================================
 * DIRECT CALCULATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate 2D position from exactly 3 anchors using direct matrix inversion
 * 
 * @param x1,y1,r1 First anchor position and distance
 * @param x2,y2,r2 Second anchor position and distance
 * @param x3,y3,r3 Third anchor position and distance
 * @param PP_point_out Output position [x, y]
 * @return 1 if successful, 0 if failed
 */
u8 Get_three_BS_Out_XY(double x1, double y1, double r1, 
                       double x2, double y2, double r2,
                       double x3, double y3, double r3, double *PP_point_out);

/**
 * @brief Calculate 3D position from exactly 4 anchors using direct matrix inversion
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
                        double x4, double y4, double z4, double r4, double *Point_xyz);

/**
 * @brief Calculate 3D position using least squares method
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
                             double x4, double y4, double z4, double r4, float *Point_xyz);

/*============================================================================
 * VALIDATION FUNCTIONS  
 *============================================================================*/

/**
 * @brief Judge if three points form a valid triangle for positioning
 * 
 * @param x1,y1 First point coordinates
 * @param x2,y2 Second point coordinates
 * @param x3,y3 Third point coordinates
 * @return 1 if valid triangle, 0 otherwise
 */
u8 Judge(double x1, double y1, double x2, double y2, double x3, double y3);

#endif // __LOC_H
