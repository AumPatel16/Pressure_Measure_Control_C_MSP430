/*
 * main.h
 *
 *  Created on: Jun. 2, 2021
 *      Author: richa
 */

#ifndef MAIN_H_
#define MAIN_H_




//STRUCTURES ------------------------------------------------------------------
typedef struct
{
        uint8_t lo;
        uint8_t ml;
        uint8_t mh;
        uint8_t hi;
 } HiLo_8_32_Struct;

 typedef struct
 {
         uint8_t lo;
         uint8_t ml;
         uint8_t mh;
         int8_t  hi;
  } HiLo_8_32_StructSigned;

typedef struct
{
        uint8_t lo;
        uint8_t hi;
} HiLo_8_16_Struct;

typedef struct
{
        uint8_t lo;
        int8_t hi;
} HiLo_8_16_StructSigned;

typedef struct
{
        uint16_t lo;
        uint16_t hi;
} HiLo_16_32_Struct;

typedef struct
{
        uint16_t lo;
        int16_t hi;
} HiLo_16_32_StructSigned;

typedef union
{
    uint32_t uval32;
    int32_t  val32;
    HiLo_8_32_Struct        uval8;
    HiLo_8_32_StructSigned   val8;
    HiLo_16_32_Struct       uval16;
    HiLo_16_32_StructSigned  val16;
} HiLo32UnionAll;


typedef union
{
    uint16_t uval16;
    HiLo_8_16_Struct uval8;
} HiLo16Union;

typedef union
{
    int16_t val16;
    HiLo_8_16_StructSigned val8;
} HiLo16UnionSigned;

//STRUCTURES END----------------------------------------------------------------------------











#endif /* MAIN_H_ */
