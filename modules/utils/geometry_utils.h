#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "system_config.h"
#include "data_types.h"
#include <stdint.h>
#include <math.h>

// Geometric calculations
float calculate_distance_2d(const position_t* pos1, const position_t* pos2);
float calculate_distance_3d(const position_t* pos1, const position_t* pos2);
float calculate_geometric_diversity_score(const uint8_t* anchor_indices, uint8_t count,
                                        const discovered_anchor_t* anchor_list);

// Area and volume calculations
float calculate_triangle_area(const position_t* p1, const position_t* p2, const position_t* p3);
float calculate_tetrahedron_volume(const position_t* p1, const position_t* p2,
                                  const position_t* p3, const position_t* p4);

// Angle calculations
float calculate_angle_between_points(const position_t* center, const position_t* p1, const position_t* p2);
bool is_good_geometric_configuration(const uint8_t* anchor_indices, uint8_t count,
                                   const discovered_anchor_t* anchor_list);

// Position estimation utilities
float calculate_dop_estimate(const uint8_t* anchor_indices, uint8_t count,
                           const discovered_anchor_t* anchor_list);
bool check_anchor_spread(const uint8_t* anchor_indices, uint8_t count,
                        const discovered_anchor_t* anchor_list, float min_spread_cm);

#endif // GEOMETRY_UTILS_H
