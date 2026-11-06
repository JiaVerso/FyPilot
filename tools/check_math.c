/* Native regression checks for the math module copied from the QEMU project. */
#include "my_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    float source[] = {4, 7, 2, 6};
    float original[] = {4, 7, 2, 6};
    float inverse[4], product[4];
    math_matrixf_t a, saved, inv, prod;
    assert(my_matrix_init(&a, 2, 2, source) == MY_MATH_OK);
    assert(my_matrix_init(&saved, 2, 2, original) == MY_MATH_OK);
    assert(my_matrix_init(&inv, 2, 2, inverse) == MY_MATH_OK);
    assert(my_matrix_init(&prod, 2, 2, product) == MY_MATH_OK);
    assert(my_matrix_inverse(&a, &inv) == MY_MATH_OK);
    assert(fabsf(inverse[0] - 0.6f) < 1e-6f);
    assert(fabsf(inverse[1] + 0.7f) < 1e-6f);
    assert(fabsf(inverse[2] + 0.2f) < 1e-6f);
    assert(fabsf(inverse[3] - 0.4f) < 1e-6f);
    assert(my_matrix_multiply(&saved, &inv, &prod) == MY_MATH_OK);
    assert(fabsf(product[0] - 1.0f) < 1e-5f && fabsf(product[3] - 1.0f) < 1e-5f);
    assert(fabsf(product[1]) < 1e-5f && fabsf(product[2]) < 1e-5f);
    float singular[] = {1, 2, 2, 4};
    my_matrix_init(&a, 2, 2, singular);
    assert(my_matrix_inverse(&a, &inv) == MY_MATH_SINGULAR);
    my_matrix_dimensions(&a, 1, 2);
    assert(my_matrix_inverse(&a, &inv) == MY_MATH_SIZE_MISMATCH);
    assert(my_matrix_init(NULL, 1, 1, source) == MY_MATH_ARGUMENT_ERROR);
    assert(math_crc16(0xffff, "123456789", 9) == 0x29b1);
    puts("PASS: matrix inverse/product, singular/mismatched dimensions, null argument, CRC16");
    return 0;
}
