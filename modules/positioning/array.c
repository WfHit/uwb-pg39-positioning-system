/**
 * @file    array.c
 * @brief   Dynamic array utility functions for positioning calculations
 *
 * This file provides dynamic memory management for float arrays used in
 * matrix calculations and positioning algorithms. Provides a simple
 * interface for creating, accessing, and managing dynamically allocated arrays.
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "array.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/*============================================================================
 * ARRAY MANAGEMENT FUNCTIONS
 *============================================================================*/

/**
 * @brief Create a dynamic float array with specified size
 *
 * Allocates memory for a float array and initializes the Array_t structure.
 * The caller is responsible for calling Array_free() to release memory.
 *
 * @param init_size Initial size of the array (number of float elements)
 * @return Array_t structure with allocated memory and size information
 * @note Returns array with NULL pointer if allocation fails
 */
Array_t Array_create(uint16_t init_size)
{
	Array_t a;
	a.array = (float*)malloc(sizeof(float) * init_size);
	a.size = (a.array != NULL) ? init_size : 0;
	return a;
}

/**
 * @brief Free memory allocated for dynamic array
 *
 * Releases the memory allocated by Array_create() and resets the structure
 * to safe values. Safe to call multiple times on the same array.
 *
 * @param a Pointer to Array_t structure to free
 */
void Array_free(Array_t* a)
{
	if (a != NULL && a->array != NULL) {
		free(a->array);
		a->array = NULL;
		a->size = 0;
	}
}

/**
 * @brief Get pointer to array element at specified index
 *
 * Returns a pointer to the float value at the given index. No bounds
 * checking is performed - caller must ensure index is valid.
 *
 * @param a Pointer to Array_t structure
 * @param idx Index of element to access (0-based)
 * @return Pointer to float value at specified index
 * @warning No bounds checking performed - undefined behavior if idx >= size
 */
float* Array_get(Array_t *a, uint16_t idx)
{
	return &(a->array[idx]);
}

/**
 * @brief Set value of array element at specified index
 *
 * Sets the float value at the given index. No bounds checking is performed
 * - caller must ensure index is valid.
 *
 * @param a Pointer to Array_t structure
 * @param idx Index of element to set (0-based)
 * @param value Float value to store at the specified index
 * @warning No bounds checking performed - undefined behavior if idx >= size
 */
void Array_set(Array_t *a, uint16_t idx, float value)
{
	(a->array)[idx] = value;
}

/**
 * @brief Get the size of the array
 *
 * @param a Pointer to Array_t structure
 * @return Number of float elements in the array
 */
uint16_t Array_size(const Array_t *a)
{
	return (a != NULL) ? a->size : 0;
}

