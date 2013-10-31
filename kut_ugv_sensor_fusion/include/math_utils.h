/*
 * math_utils.h
 *
 * Some common mathematical functions, like getting minimum and maximum of
 * three integers.
 *
 * TODO: add more function
 * TODO: think more generic
 */
#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

/**
 * Return the greatest value of three passed arguments.
 *
 * Unsigned integer has been used, because RGB values cant be minus.
 */
unsigned int
get_maximum(unsigned int r, unsigned int g, unsigned int b);

/**
 * Return the lowest value of three passed arguments.
 *
 * Unsigned integer has been used, because RGB values cant be minus.
 */
unsigned int
get_minimum(unsigned int r, unsigned int g, unsigned int b);

#endif /* MATH_UTILS_H_ */
