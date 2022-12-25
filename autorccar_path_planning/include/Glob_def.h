#ifndef __GLOB_DEF_H__
#define __GLOB_DEF_H__

//#define _CRT_SECURE_NO_WARNINGS


//extern "C"

//Type Definition
typedef signed char				s8;
typedef short					s16;
typedef long					s32;
typedef long long				s64;
typedef unsigned char			u8;
typedef unsigned short			u16;
typedef unsigned long			u32;
typedef unsigned long long		u64;


typedef enum COMPO_IDX { X = 0, Y, Z } Compo_Index;

typedef double					fType;

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>

#include "Glob_def.h"
#include "bezier_curve.h"

#define DEF_PI			3.14159265358979323846

#define  DEF_MEMCPY(a, b)   ( memcpy(a, b, sizeof(a)) )
#define  DEF_MEMSET(a, v)   ( memset(a, v, sizeof(a)) )

#define  DEF_SETFLAG(a)     ( a = SET ) // ������ �Լ� a�� 1�� ��
#define  DEF_RESETFLAG(a)   ( a = RESET )  // a�� 0�� ��  

#define  DEF_ABS(a)         (((a) < 0) ? (-(a)) : (a))
#define  DEF_MIN(a, b)      (((a) < (b)) ? (a) : (b))
#define  DEF_MAX(a, b)      (((a) > (b)) ? (a) : (b))
#define  DEF_SIGN(a)        (((a) < 0) ? (-1) : (1))
// malloc �����Ҵ�
#define  DEF_MALLOC_S8(a, S)        ( a = (s8*)malloc(sizeof(s8) * (S)) )
#define  DEF_MALLOC_U8(a, S)        ( a = (u8*)malloc(sizeof(u8) * (S)) )
#define  DEF_MALLOC_S16(a, S)       ( a = (s16*)malloc(sizeof(s16) * (S)) )
#define  DEF_MALLOC_U16(a, S)       ( a = (u16*)malloc(sizeof(u16) * (S)) )
#define  DEF_MALLOC_S32(a, S)       ( a = (s32*)malloc(sizeof(s32) * (S)) )
#define  DEF_MALLOC_U32(a, S)       ( a = (u32*)malloc(sizeof(u32) * (S)) )
#define  DEF_MALLOC_FLOAT(a, S)     ( a = (float*)malloc(sizeof(float) * (S)) )
#define  DEF_MALLOC_DOUBLE(a, S)    ( a = (double*)malloc(sizeof(double) * (S)) )
#define  DEF_MALLOC_FTYPE(a, S)         ( a = (fType*)malloc(sizeof(fType) * (S)) )

#define  DEF_SQ(a)              ((a) * (a))
#define  DEF_CUBE(a)			((a) * (a) * (a))

#define  DEF_DTORADIAN(a)       ((a) * (DEF_PI/180.0))
#define  DEF_RTODEGREE(a)       ((a) * (180.0/DEF_PI))

#define  DEF_RSHIFT(a, s)       ( (a) >> (s) )
#define  DEF_LSHIFT(a, s)       ( (a) << (s) )

#define  DEF_AND_OP(a, b)       ( (a) & (b) )
#define  DEF_OR_OP(a, b)        ( (a) | (b) )



#endif
