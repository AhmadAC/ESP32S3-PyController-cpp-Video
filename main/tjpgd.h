#ifndef _TJPGDEC
#define _TJPGDEC

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JD_SZBUF        1024
#define JD_FORMAT       1   // 1: RGB565 Output
#define JD_USE_SCALE    0   // No scaling needed

typedef unsigned int UINT;
typedef unsigned char BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef int32_t LONG;
typedef int INT;

typedef enum {
    JDR_OK = 0, JDR_INTR, JDR_INP, JDR_MEM1, JDR_MEM2,
    JDR_PAR, JDR_FMT1, JDR_FMT2, JDR_FMT3
} JRESULT;

typedef struct {
    WORD left, right, top, bottom;
} JRECT;

typedef struct JDEC JDEC;
struct JDEC {
    UINT dctr;
    BYTE* dptr;
    BYTE* inbuf;
    BYTE dmsk;
    BYTE scale;
    BYTE msx, msy;
    BYTE qtid[3];
    int16_t dcv[3];
    WORD nrst;
    UINT width, height;
    BYTE* huffbits[2][2];
    WORD* huffcode[2][2];
    BYTE* huffdata[2][2];
    LONG* qttbl[4];
    void* workbuf;
    BYTE* mcubuf;
    void* pool;
    UINT sz_pool;
    UINT (*infunc)(JDEC*, BYTE*, UINT);
    void* device;
};

JRESULT jd_prepare (JDEC*, UINT(*)(JDEC*,BYTE*,UINT), void*, UINT, void*);
JRESULT jd_decomp (JDEC*, UINT(*)(JDEC*,void*,JRECT*), BYTE);

#ifdef __cplusplus
}
#endif
#endif