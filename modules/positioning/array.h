/**
 * @file    array.h
 * @brief   Dynamic array utility functions for positioning calculations
 * 
 * This header provides a simple dynamic array implementation for float values,
 * primarily used in matrix calculations for positioning algorithms. The arrays
 * automatically manage memory allocation and provide bounds-safe access methods.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef _ARRAY_H
#define _ARRAY_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * TYPE DEFINITIONS
 *============================================================================*/

/**
 * @brief Dynamic float array structure
 * 
 * This structure encapsulates a dynamically allocated array of float values
 * along with its size. Used primarily for temporary storage during matrix
 * operations in positioning calculations.
 */
typedef struct 
{
	float *array;       ///< Pointer to dynamically allocated float array
	uint16_t size;      ///< Number of elements in the array
} Array_t;

/*============================================================================
 * FUNCTION DECLARATIONS
 *============================================================================*/

/**
 * @brief Create a dynamic float array with specified size
 * @param init_size Initial size of the array (number of float elements)
 * @return Array_t structure with allocated memory and size information
 */
Array_t Array_create(uint16_t init_size);

/**
 * @brief Free memory allocated for dynamic array
 * @param a Pointer to Array_t structure to free
 */
void Array_free(Array_t* a);

/**
 * @brief Get pointer to array element at specified index
 * @param a Pointer to Array_t structure
 * @param idx Index of element to access (0-based)
 * @return Pointer to float value at specified index
 */
float* Array_get(Array_t *a, uint16_t idx);

/**
 * @brief Set value of array element at specified index
 * @param a Pointer to Array_t structure
 * @param idx Index of element to set (0-based)
 * @param value Float value to store at the specified index
 */
void Array_set(Array_t *a, uint16_t idx, float value);

/**
 * @brief Get the size of the array
 * @param a Pointer to Array_t structure
 * @return Number of float elements in the array
 */
uint16_t Array_size(const Array_t *a);

#endif // _ARRAY_H
