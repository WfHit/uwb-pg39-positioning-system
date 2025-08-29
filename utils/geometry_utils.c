#include "geometry_utils.h"

float calculate_distance_2d(const position_t* pos1, const position_t* pos2)
{
    if(!pos1 || !pos2) return 0.0f;
    
    float dx = (float)(pos1->x_cm - pos2->x_cm);
    float dy = (float)(pos1->y_cm - pos2->y_cm);
    
    return sqrtf(dx * dx + dy * dy);
}

float calculate_distance_3d(const position_t* pos1, const position_t* pos2)
{
    if(!pos1 || !pos2) return 0.0f;
    
    float dx = (float)(pos1->x_cm - pos2->x_cm);
    float dy = (float)(pos1->y_cm - pos2->y_cm);
    float dz = (float)(pos1->z_cm - pos2->z_cm);
    
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

float calculate_geometric_diversity_score(const uint8_t* anchor_indices, uint8_t count, 
                                        const discovered_anchor_t* anchor_list)
{
    if(!anchor_indices || !anchor_list || count < 3) return 0.0f;
    
    float diversity_score = 0.0f;
    
    if(count == 3)
    {
        // Calculate triangle area for 3 anchors
        const position_t* p1 = &anchor_list[anchor_indices[0]].position;
        const position_t* p2 = &anchor_list[anchor_indices[1]].position;
        const position_t* p3 = &anchor_list[anchor_indices[2]].position;
        
        float area = calculate_triangle_area(p1, p2, p3);
        diversity_score = area / 10000.0f; // Normalize (assuming cm units)
    }
    else if(count >= 4)
    {
        // Calculate tetrahedron volume for 4+ anchors
        const position_t* p1 = &anchor_list[anchor_indices[0]].position;
        const position_t* p2 = &anchor_list[anchor_indices[1]].position;
        const position_t* p3 = &anchor_list[anchor_indices[2]].position;
        const position_t* p4 = &anchor_list[anchor_indices[3]].position;
        
        float volume = calculate_tetrahedron_volume(p1, p2, p3, p4);
        diversity_score = volume / 1000000.0f; // Normalize (assuming cm³ units)
        
        // Add bonus for additional anchors
        diversity_score *= (1.0f + (count - 4) * 0.1f);
    }
    
    // Apply penalty for poor anchor spread
    if(!check_anchor_spread(anchor_indices, count, anchor_list, 100.0f)) // 1m minimum spread
    {
        diversity_score *= 0.5f;
    }
    
    return diversity_score;
}

float calculate_triangle_area(const position_t* p1, const position_t* p2, const position_t* p3)
{
    if(!p1 || !p2 || !p3) return 0.0f;
    
    // Using cross product for area calculation
    float ax = (float)(p2->x_cm - p1->x_cm);
    float ay = (float)(p2->y_cm - p1->y_cm);
    float bx = (float)(p3->x_cm - p1->x_cm);
    float by = (float)(p3->y_cm - p1->y_cm);
    
    float cross_product = ax * by - ay * bx;
    return fabsf(cross_product) / 2.0f;
}

float calculate_tetrahedron_volume(const position_t* p1, const position_t* p2, 
                                  const position_t* p3, const position_t* p4)
{
    if(!p1 || !p2 || !p3 || !p4) return 0.0f;
    
    // Calculate vectors from p1 to other points
    float ax = (float)(p2->x_cm - p1->x_cm);
    float ay = (float)(p2->y_cm - p1->y_cm);
    float az = (float)(p2->z_cm - p1->z_cm);
    
    float bx = (float)(p3->x_cm - p1->x_cm);
    float by = (float)(p3->y_cm - p1->y_cm);
    float bz = (float)(p3->z_cm - p1->z_cm);
    
    float cx = (float)(p4->x_cm - p1->x_cm);
    float cy = (float)(p4->y_cm - p1->y_cm);
    float cz = (float)(p4->z_cm - p1->z_cm);
    
    // Calculate scalar triple product (a · (b × c))
    float cross_x = by * cz - bz * cy;
    float cross_y = bz * cx - bx * cz;
    float cross_z = bx * cy - by * cx;
    
    float scalar_triple = ax * cross_x + ay * cross_y + az * cross_z;
    
    return fabsf(scalar_triple) / 6.0f;
}

float calculate_angle_between_points(const position_t* center, const position_t* p1, const position_t* p2)
{
    if(!center || !p1 || !p2) return 0.0f;
    
    // Vectors from center to points
    float v1x = (float)(p1->x_cm - center->x_cm);
    float v1y = (float)(p1->y_cm - center->y_cm);
    float v1z = (float)(p1->z_cm - center->z_cm);
    
    float v2x = (float)(p2->x_cm - center->x_cm);
    float v2y = (float)(p2->y_cm - center->y_cm);
    float v2z = (float)(p2->z_cm - center->z_cm);
    
    // Dot product
    float dot_product = v1x * v2x + v1y * v2y + v1z * v2z;
    
    // Magnitudes
    float mag1 = sqrtf(v1x * v1x + v1y * v1y + v1z * v1z);
    float mag2 = sqrtf(v2x * v2x + v2y * v2y + v2z * v2z);
    
    if(mag1 == 0.0f || mag2 == 0.0f) return 0.0f;
    
    // Angle in radians
    float cos_angle = dot_product / (mag1 * mag2);
    
    // Clamp to avoid floating point errors
    if(cos_angle > 1.0f) cos_angle = 1.0f;
    if(cos_angle < -1.0f) cos_angle = -1.0f;
    
    return acosf(cos_angle);
}

bool is_good_geometric_configuration(const uint8_t* anchor_indices, uint8_t count,
                                   const discovered_anchor_t* anchor_list)
{
    if(!anchor_indices || !anchor_list || count < 3) return false;
    
    // Check minimum spread
    if(!check_anchor_spread(anchor_indices, count, anchor_list, 100.0f)) // 1m minimum
    {
        return false;
    }
    
    // Check for collinearity (2D case)
    if(count == 3)
    {
        float area = calculate_triangle_area(&anchor_list[anchor_indices[0]].position,
                                           &anchor_list[anchor_indices[1]].position,
                                           &anchor_list[anchor_indices[2]].position);
        return area > 1000.0f; // Minimum area of 100cm²
    }
    
    // Check for coplanarity (3D case)
    if(count >= 4)
    {
        float volume = calculate_tetrahedron_volume(&anchor_list[anchor_indices[0]].position,
                                                   &anchor_list[anchor_indices[1]].position,
                                                   &anchor_list[anchor_indices[2]].position,
                                                   &anchor_list[anchor_indices[3]].position);
        return volume > 100000.0f; // Minimum volume of 1000cm³
    }
    
    return true;
}

float calculate_dop_estimate(const uint8_t* anchor_indices, uint8_t count,
                           const discovered_anchor_t* anchor_list)
{
    if(!anchor_indices || !anchor_list || count < 3) return 999.0f; // High DOP = bad
    
    // Simplified DOP calculation based on geometric diversity
    float diversity = calculate_geometric_diversity_score(anchor_indices, count, anchor_list);
    
    if(diversity <= 0.0f) return 999.0f;
    
    // Convert diversity to DOP (lower is better)
    float dop = 10.0f / diversity;
    
    // Clamp to reasonable range
    if(dop > 999.0f) dop = 999.0f;
    if(dop < 0.1f) dop = 0.1f;
    
    return dop;
}

bool check_anchor_spread(const uint8_t* anchor_indices, uint8_t count,
                        const discovered_anchor_t* anchor_list, float min_spread_cm)
{
    if(!anchor_indices || !anchor_list || count < 2) return false;
    
    float max_distance = 0.0f;
    
    // Find maximum distance between any two anchors
    for(uint8_t i = 0; i < count; i++)
    {
        for(uint8_t j = i + 1; j < count; j++)
        {
            float distance = calculate_distance_3d(&anchor_list[anchor_indices[i]].position,
                                                  &anchor_list[anchor_indices[j]].position);
            if(distance > max_distance)
            {
                max_distance = distance;
            }
        }
    }
    
    return max_distance >= min_spread_cm;
}
