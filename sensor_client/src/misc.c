/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MANULYTICA_MISC_C_
#define MANULYTICA_MISC_C_


#include <zephyr.h>
#include <string.h>
#include <sys/printk.h>
#include <drivers/sensor.h>

#include <sys/byteorder.h>

/**
 * @brief Helper function for converting struct sensor_value to double.
 *
 * @param val A pointer to a sensor_value struct.
 * @return The converted value.
 */
float sensor_value_to_float(struct sensor_value *val)
{
	return (float)val->val1 + (float)val->val2 / 1000000; // 1000000
}


// 
// New buffer helper for signed values
// 
int16_t net_buf_signed_pull_le16(struct net_buf_simple *buf)
{
	int16_t val;

	val = UNALIGNED_GET((int16_t *)buf->data);
	net_buf_simple_pull(buf, sizeof(val));

	return sys_le16_to_cpu(val);
}

int32_t net_buf_signed_pull_le32(struct net_buf_simple *buf)
{
	int32_t val;

	val = UNALIGNED_GET((int32_t *)buf->data);
	net_buf_simple_pull(buf, sizeof(val));

	return sys_le32_to_cpu(val);
}
// 




int read_single_bit(unsigned char* buffer, unsigned int index)
{
    unsigned char c = buffer[index / 8]; //getting the byte which contains the bit
    unsigned int bit_position = index % 8; //getting the position of that bit within the byte

    return ((c >> (7 - bit_position)) & 1);
    //shifting that byte to the right with (7 - bit_position) will move the bit whose value you want to know at "the end" of the byte.
    //then, by doing bitwise AND with the new byte and 1 (whose binary representation is 00000001) will yield 1 or 0, depending on the value of the bit you need.
}



// unsigned createMask(unsigned a, unsigned b)
// {
//    unsigned r = 0;
//    for (unsigned i=a; i<=b; i++)
//        r |= 1 << i;

//    return r;
// }

uint16_t bitSpec(int start, int len) {
    return (~0U >> (16 - len)) << start;
}

// // Message Header Deconstruction //
// // Thoughts about sensor varables
// switch ((typeid(T)) {
//   case typeid(myClassA):
//     // handle that case
//     break;
//   case typeid(myClassB):
//     // handle that case
//     break;
//   case typeid(uint32_t):
//     // handle that case
//     break;
//   default:
//     // handle that case
// }



/* End of Define trap */
#endif 
