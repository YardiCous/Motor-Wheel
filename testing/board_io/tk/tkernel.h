#ifndef TKERNEL_H
#define TKERNEL_H
#include <stddef.h>
#include <stdint.h>
#define E_OK 0
#define TA_HLNG 1u
typedef struct { uint32_t intatr; void (*inthdr)(uint32_t); } T_DINT;
typedef struct { uint32_t hi; uint32_t lo; } SYSTIM;
int tk_def_int(uint32_t number, const T_DINT *config);
void EnableInt(uint32_t number, int level);
int tk_get_tim(SYSTIM *time);
int tk_set_flg(int32_t flag, uint32_t pattern);
#endif
