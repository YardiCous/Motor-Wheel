#ifndef TKERNEL_H
#define TKERNEL_H

#include <stdint.h>
#include <stddef.h>

#define TK_SUPPORT_SMP 0
#define E_OK 0
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define TA_HLNG 1u
#define TA_RNG3 2u
#define TA_TFIFO 4u
#define TA_WMUL 8u
#define TWF_ORW 1u
#define TWF_BITCLR 2u

typedef int32_t INT;
typedef uint32_t UINT;
typedef int32_t ER;
typedef int32_t ID;
typedef int32_t PRI;
typedef unsigned char UB;
typedef void (*FP)(INT, void *);
typedef struct
{
    uint32_t cycatr;
    void (*cychdr)(void *);
    uint32_t cyctim;
    uint32_t cycphs;
} T_CCYC;
typedef struct
{
    int32_t bufsz;
    int32_t maxmsz;
    uint32_t mbfatr;
} T_CMBF;
typedef struct
{
    uint32_t flgatr;
} T_CFLG;
typedef struct
{
    FP task;
    PRI itskpri;
    int32_t stksz;
    uint32_t tskatr;
} T_CTSK;

ID tk_cre_cyc(const T_CCYC *config);
ID tk_cre_mbf(const T_CMBF *config);
ID tk_cre_flg(const T_CFLG *config);
ID tk_cre_tsk(const T_CTSK *config);
ER tk_sta_tsk(ID task, INT code);
ER tk_sta_cyc(ID cyclic);
ER tk_slp_tsk(INT timeout);
ER tk_can_wup(ID task);
ER tk_wup_tsk(ID task);
ER tk_dly_tsk(UINT delay);
ER tk_snd_mbf(ID queue, const void *message, INT length, INT timeout);
INT tk_rcv_mbf(ID queue, void *message, INT timeout);
ER tk_clr_flg(ID flag, UINT pattern);
ER tk_wai_flg(ID flag, UINT pattern, UINT mode, UINT *result, INT timeout);

#endif
