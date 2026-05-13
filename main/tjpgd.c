#include "tjpgd.h"
#include <string.h>

static const BYTE Zig[64] = {
    0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

static const WORD Ipsf[64] = {
    (WORD)(1.00000*8192), (WORD)(1.38704*8192), (WORD)(1.30656*8192), (WORD)(1.17588*8192), (WORD)(1.00000*8192), (WORD)(0.78570*8192), (WORD)(0.54120*8192), (WORD)(0.27590*8192),
    (WORD)(1.38704*8192), (WORD)(1.92388*8192), (WORD)(1.81226*8192), (WORD)(1.63099*8192), (WORD)(1.38704*8192), (WORD)(1.08979*8192), (WORD)(0.75066*8192), (WORD)(0.38268*8192),
    (WORD)(1.30656*8192), (WORD)(1.81226*8192), (WORD)(1.70711*8192), (WORD)(1.53636*8192), (WORD)(1.30656*8192), (WORD)(1.02656*8192), (WORD)(0.70711*8192), (WORD)(0.36048*8192),
    (WORD)(1.17588*8192), (WORD)(1.63099*8192), (WORD)(1.53636*8192), (WORD)(1.38268*8192), (WORD)(1.17588*8192), (WORD)(0.92388*8192), (WORD)(0.63638*8192), (WORD)(0.32442*8192),
    (WORD)(1.00000*8192), (WORD)(1.38704*8192), (WORD)(1.30656*8192), (WORD)(1.17588*8192), (WORD)(1.00000*8192), (WORD)(0.78570*8192), (WORD)(0.54120*8192), (WORD)(0.27590*8192),
    (WORD)(0.78570*8192), (WORD)(1.08979*8192), (WORD)(1.02656*8192), (WORD)(0.92388*8192), (WORD)(0.78570*8192), (WORD)(0.61732*8192), (WORD)(0.42522*8192), (WORD)(0.21677*8192),
    (WORD)(0.54120*8192), (WORD)(0.75066*8192), (WORD)(0.70711*8192), (WORD)(0.63638*8192), (WORD)(0.54120*8192), (WORD)(0.42522*8192), (WORD)(0.29290*8192), (WORD)(0.14932*8192),
    (WORD)(0.27590*8192), (WORD)(0.38268*8192), (WORD)(0.36048*8192), (WORD)(0.32442*8192), (WORD)(0.27590*8192), (WORD)(0.21678*8192), (WORD)(0.14932*8192), (WORD)(0.07612*8192)
};

static inline BYTE BYTECLIP(INT val) {
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    return (BYTE)val;
}

static void* alloc_pool(JDEC* jd, UINT nd) {
    char *rp = 0;
    nd = (nd + 3) & ~3;
    if (jd->sz_pool >= nd) {
        jd->sz_pool -= nd;
        rp = (char*)jd->pool;
        jd->pool = (void*)(rp + nd);
    }
    return (void*)rp;
}

static UINT create_qt_tbl(JDEC* jd, const BYTE* data, UINT ndata) {
    UINT i; BYTE d, z; LONG *pb;
    while (ndata) {
        if (ndata < 65) return JDR_FMT1;
        ndata -= 65;
        d = *data++;
        if (d & 0xF0) return JDR_FMT1;
        i = d & 3;
        pb = (LONG*)alloc_pool(jd, 64 * sizeof(LONG));
        if (!pb) return JDR_MEM1;
        jd->qttbl[i] = pb;
        for (i = 0; i < 64; i++) {
            z = Zig[i];
            pb[z] = (LONG)((DWORD)*data++ * Ipsf[z]);
        }
    }
    return JDR_OK;
}

static UINT create_huffman_tbl(JDEC* jd, const BYTE* data, UINT ndata) {
    UINT i, j, b, np, cls, num; BYTE d, *pb, *pd; WORD hc, *ph;
    while (ndata) {
        if (ndata < 17) return JDR_FMT1;
        ndata -= 17;
        d = *data++;
        cls = (d >> 4); num = d & 0x0F;
        if (d & 0xEE) return JDR_FMT1;
        pb = (BYTE*)alloc_pool(jd, 16);
        if (!pb) return JDR_MEM1;
        jd->huffbits[num][cls] = pb;
        for (np = i = 0; i < 16; i++) {
            pb[i] = b = *data++;
            np += b;
        }
        ph = (WORD*)alloc_pool(jd, np * sizeof(WORD));
        if (!ph) return JDR_MEM1;
        jd->huffcode[num][cls] = ph;
        hc = 0;
        for (j = i = 0; i < 16; i++) {
            b = pb[i];
            while (b--) ph[j++] = hc++;
            hc <<= 1;
        }
        if (ndata < np) return JDR_FMT1;
        ndata -= np;
        pd = (BYTE*)alloc_pool(jd, np);
        if (!pd) return JDR_MEM1;
        jd->huffdata[num][cls] = pd;
        for (i = 0; i < np; i++) {
            d = *data++;
            if (!cls && d > 11) return JDR_FMT1;
            *pd++ = d;
        }
    }
    return JDR_OK;
}

static INT bitext(JDEC* jd, UINT nbit) {
    BYTE msk, s, *dp; UINT dc, v, f;
    msk = jd->dmsk; dc = jd->dctr; dp = jd->dptr;
    s = *dp; v = f = 0;
    do {
        if (!msk) {
            if (!dc) {
                dp = jd->inbuf;
                dc = jd->infunc(jd, dp, JD_SZBUF);
                if (!dc) return 0 - (INT)JDR_INP;
            } else {
                dp++;
            }
            dc--;
            if (f) {
                f = 0;
                if (*dp != 0) return 0 - (INT)JDR_FMT1;
                *dp = s = 0xFF;
            } else {
                s = *dp;
                if (s == 0xFF) { f = 1; continue; }
            }
            msk = 0x80;
        }
        v <<= 1;
        if (s & msk) v++;
        msk >>= 1;
        nbit--;
    } while (nbit);
    jd->dmsk = msk; jd->dctr = dc; jd->dptr = dp;
    return (INT)v;
}

static INT huffext(JDEC* jd, const BYTE* hbits, const WORD* hcode, const BYTE* hdata) {
    BYTE msk, s, *dp; UINT dc, v, f, bl, nd;
    msk = jd->dmsk; dc = jd->dctr; dp = jd->dptr;
    s = *dp; v = f = 0; bl = 16;
    do {
        if (!msk) {
            if (!dc) {
                dp = jd->inbuf;
                dc = jd->infunc(jd, dp, JD_SZBUF);
                if (!dc) return 0 - (INT)JDR_INP;
            } else { dp++; }
            dc--;
            if (f) {
                f = 0;
                if (*dp != 0) return 0 - (INT)JDR_FMT1;
                *dp = s = 0xFF;
            } else {
                s = *dp;
                if (s == 0xFF) { f = 1; continue; }
            }
            msk = 0x80;
        }
        v <<= 1;
        if (s & msk) v++;
        msk >>= 1;
        for (nd = *hbits++; nd; nd--) {
            if (v == *hcode++) {
                jd->dmsk = msk; jd->dctr = dc; jd->dptr = dp;
                return *hdata;
            }
            hdata++;
        }
        bl--;
    } while (bl);
    return 0 - (INT)JDR_FMT1;
}

static void block_idct(LONG* src, BYTE* dst) {
    const LONG M13 = (LONG)(1.41421*4096), M2 = (LONG)(1.08239*4096), M4 = (LONG)(2.61313*4096), M5 = (LONG)(1.84776*4096);
    LONG v0, v1, v2, v3, v4, v5, v6, v7, t10, t11, t12, t13; UINT i;

    for (i = 0; i < 8; i++) {
        v0 = src[8 * 0]; v1 = src[8 * 2]; v2 = src[8 * 4]; v3 = src[8 * 6];
        t10 = v0 + v2; t12 = v0 - v2; t11 = (v1 - v3) * M13 >> 12;
        v3 += v1; t11 -= v3; v0 = t10 + v3; v3 = t10 - v3; v1 = t11 + t12; v2 = t12 - t11;

        v4 = src[8 * 7]; v5 = src[8 * 1]; v6 = src[8 * 5]; v7 = src[8 * 3];
        t10 = v5 - v4; t11 = v5 + v4; t12 = v6 - v7; v7 += v6;
        v5 = (t11 - v7) * M13 >> 12; v7 += t11; t13 = (t10 + t12) * M5 >> 12;
        v4 = t13 - (t10 * M2 >> 12); v6 = t13 - (t12 * M4 >> 12) - v7;
        v5 -= v6; v4 -= v5;

        src[8 * 0] = v0 + v7; src[8 * 7] = v0 - v7; src[8 * 1] = v1 + v6; src[8 * 6] = v1 - v6;
        src[8 * 2] = v2 + v5; src[8 * 5] = v2 - v5; src[8 * 3] = v3 + v4; src[8 * 4] = v3 - v4;
        src++;
    }

    src -= 8;
    for (i = 0; i < 8; i++) {
        v0 = src[0] + (128L << 8); v1 = src[2]; v2 = src[4]; v3 = src[6];
        t10 = v0 + v2; t12 = v0 - v2; t11 = (v1 - v3) * M13 >> 12;
        v3 += v1; t11 -= v3; v0 = t10 + v3; v3 = t10 - v3; v1 = t11 + t12; v2 = t12 - t11;

        v4 = src[7]; v5 = src[1]; v6 = src[5]; v7 = src[3];
        t10 = v5 - v4; t11 = v5 + v4; t12 = v6 - v7; v7 += v6;
        v5 = (t11 - v7) * M13 >> 12; v7 += t11; t13 = (t10 + t12) * M5 >> 12;
        v4 = t13 - (t10 * M2 >> 12); v6 = t13 - (t12 * M4 >> 12) - v7;
        v5 -= v6; v4 -= v5;

        dst[0] = BYTECLIP((v0 + v7) >> 8); dst[7] = BYTECLIP((v0 - v7) >> 8);
        dst[1] = BYTECLIP((v1 + v6) >> 8); dst[6] = BYTECLIP((v1 - v6) >> 8);
        dst[2] = BYTECLIP((v2 + v5) >> 8); dst[5] = BYTECLIP((v2 - v5) >> 8);
        dst[3] = BYTECLIP((v3 + v4) >> 8); dst[4] = BYTECLIP((v3 - v4) >> 8);
        dst += 8; src += 8;
    }
}

static JRESULT mcu_load(JDEC* jd) {
    LONG *tmp = (LONG*)jd->workbuf;
    UINT blk, nby, nbc, i, z, id, cmp; INT b, d, e; BYTE *bp;
    const BYTE *hb, *hd; const WORD *hc; const LONG *dqf;

    nby = jd->msx * jd->msy; nbc = 2; bp = jd->mcubuf;

    for (blk = 0; blk < nby + nbc; blk++) {
        cmp = (blk < nby) ? 0 : blk - nby + 1; id = cmp ? 1 : 0;
        hb = jd->huffbits[id][0]; hc = jd->huffcode[id][0]; hd = jd->huffdata[id][0];
        b = huffext(jd, hb, hc, hd);
        if (b < 0) return (JRESULT)(0 - b);
        d = jd->dcv[cmp];
        if (b) {
            e = bitext(jd, b);
            if (e < 0) return (JRESULT)(0 - e);
            b = 1 << (b - 1);
            if (!(e & b)) e -= (b << 1) - 1;
            d += e;
            jd->dcv[cmp] = (int16_t)d;
        }
        dqf = jd->qttbl[jd->qtid[cmp]];
        tmp[0] = d * dqf[0] >> 8;

        for (i = 1; i < 64; i++) tmp[i] = 0;
        hb = jd->huffbits[id][1]; hc = jd->huffcode[id][1]; hd = jd->huffdata[id][1];
        i = 1;
        do {
            b = huffext(jd, hb, hc, hd);
            if (b == 0) break;
            if (b < 0) return (JRESULT)(0 - b);
            z = (UINT)b >> 4;
            if (z) { i += z; if (i >= 64) return JDR_FMT1; }
            if (b &= 0x0F) {
                d = bitext(jd, b);
                if (d < 0) return (JRESULT)(0 - d);
                b = 1 << (b - 1);
                if (!(d & b)) d -= (b << 1) - 1;
                z = Zig[i];
                tmp[z] = d * dqf[z] >> 8;
            }
        } while (++i < 64);
        block_idct(tmp, bp);
        bp += 64;
    }
    return JDR_OK;
}

static JRESULT mcu_output(JDEC* jd, UINT (*outfunc)(JDEC*, void*, JRECT*), UINT x, UINT y) {
    const INT CVACC = (sizeof(INT) > 2) ? 1024 : 128;
    UINT ix, iy, mx, my, rx, ry; INT yy, cb, cr; BYTE *py, *pc, *rgb24; JRECT rect;

    mx = jd->msx * 8; my = jd->msy * 8;
    rx = (x + mx <= jd->width) ? mx : jd->width - x;
    ry = (y + my <= jd->height) ? my : jd->height - y;
    rect.left = x; rect.right = x + rx - 1;
    rect.top = y; rect.bottom = y + ry - 1;

    rgb24 = (BYTE*)jd->workbuf;
    for (iy = 0; iy < my; iy++) {
        pc = jd->mcubuf; py = pc + iy * 8;
        if (my == 16) { pc += 64 * 4 + (iy >> 1) * 8; if (iy >= 8) py += 64; }
        else { pc += mx * 8 + iy * 8; }
        for (ix = 0; ix < mx; ix++) {
            cb = pc[0] - 128; cr = pc[64] - 128;
            if (mx == 16) { if (ix == 8) py += 64 - 8; pc += ix & 1; }
            else { pc++; }
            yy = *py++;
            *rgb24++ = BYTECLIP(yy + ((INT)(1.402 * CVACC) * cr) / CVACC);
            *rgb24++ = BYTECLIP(yy - ((INT)(0.344 * CVACC) * cb + (INT)(0.714 * CVACC) * cr) / CVACC);
            *rgb24++ = BYTECLIP(yy + ((INT)(1.772 * CVACC) * cb) / CVACC);
        }
    }

    BYTE *s = (BYTE*)jd->workbuf;
    WORD w, *d = (WORD*)s;
    UINT n = rx * ry;
    do {
        w = (*s++ & 0xF8) << 8;
        w |= (*s++ & 0xFC) << 3;
        w |= *s++ >> 3;
        *d++ = w;
    } while (--n);

    return (JRESULT)outfunc(jd, jd->workbuf, &rect);
}

static JRESULT restart(JDEC* jd, WORD rstn) {
    UINT i, dc; WORD d; BYTE *dp;
    dp = jd->dptr; dc = jd->dctr; d = 0;
    for (i = 0; i < 2; i++) {
        if (!dc) {
            dp = jd->inbuf; dc = jd->infunc(jd, dp, JD_SZBUF);
            if (!dc) return JDR_INP;
        } else { dp++; }
        dc--; d = (d << 8) | *dp;
    }
    jd->dptr = dp; jd->dctr = dc; jd->dmsk = 0;
    if ((d & 0xFFD8) != 0xFFD0 || (d & 7) != (rstn & 7)) return JDR_FMT1;
    jd->dcv[2] = jd->dcv[1] = jd->dcv[0] = 0;
    return JDR_OK;
}

#define LDB_WORD(ptr) (WORD)(((WORD)*((BYTE*)(ptr))<<8)|(WORD)*(BYTE*)((ptr)+1))

JRESULT jd_prepare(JDEC* jd, UINT (*infunc)(JDEC*, BYTE*, UINT), void* pool, UINT sz_pool, void* dev) {
    BYTE *seg, b; WORD marker; DWORD ofs; UINT n, i, j, len; JRESULT rc;

    if (!pool) return JDR_PAR;
    jd->pool = pool; jd->sz_pool = sz_pool; jd->infunc = infunc; jd->device = dev; jd->nrst = 0;

    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            jd->huffbits[i][j] = 0; jd->huffcode[i][j] = 0; jd->huffdata[i][j] = 0;
        }
    }
    for (i = 0; i < 4; i++) jd->qttbl[i] = 0;

    jd->inbuf = seg = (BYTE*)alloc_pool(jd, JD_SZBUF);
    if (!seg) return JDR_MEM1;
    if (jd->infunc(jd, seg, 2) != 2) return JDR_INP;
    if (LDB_WORD(seg) != 0xFFD8) return JDR_FMT1;
    ofs = 2;

    for (;;) {
        if (jd->infunc(jd, seg, 4) != 4) return JDR_INP;
        marker = LDB_WORD(seg); len = LDB_WORD(seg + 2);
        if (len <= 2 || (marker >> 8) != 0xFF) return JDR_FMT1;
        len -= 2; ofs += 4 + len;

        switch (marker & 0xFF) {
        case 0xC0:
            if (len > JD_SZBUF) return JDR_MEM2;
            if (jd->infunc(jd, seg, len) != len) return JDR_INP;
            jd->width = LDB_WORD(seg+3); jd->height = LDB_WORD(seg+1);
            if (seg[5] != 3) return JDR_FMT3;
            for (i = 0; i < 3; i++) {
                b = seg[7 + 3 * i];
                if (!i) {
                    if (b != 0x11 && b != 0x22 && b != 0x21) return JDR_FMT3;
                    jd->msx = b >> 4; jd->msy = b & 15;
                } else { if (b != 0x11) return JDR_FMT3; }
                b = seg[8 + 3 * i];
                if (b > 3) return JDR_FMT3;
                jd->qtid[i] = b;
            }
            break;
        case 0xDD:
            if (len > JD_SZBUF) return JDR_MEM2;
            if (jd->infunc(jd, seg, len) != len) return JDR_INP;
            jd->nrst = LDB_WORD(seg);
            break;
        case 0xC4:
            if (len > JD_SZBUF) return JDR_MEM2;
            if (jd->infunc(jd, seg, len) != len) return JDR_INP;
            rc = create_huffman_tbl(jd, seg, len);
            if (rc) return rc;
            break;
        case 0xDB:
            if (len > JD_SZBUF) return JDR_MEM2;
            if (jd->infunc(jd, seg, len) != len) return JDR_INP;
            rc = create_qt_tbl(jd, seg, len);
            if (rc) return rc;
            break;
        case 0xDA:
            if (len > JD_SZBUF) return JDR_MEM2;
            if (jd->infunc(jd, seg, len) != len) return JDR_INP;
            if (!jd->width || !jd->height) return JDR_FMT1;
            if (seg[0] != 3) return JDR_FMT3;
            for (i = 0; i < 3; i++) {
                b = seg[2 + 2 * i];
                if (b != 0x00 && b != 0x11) return JDR_FMT3;
                b = i ? 1 : 0;
                if (!jd->huffbits[b][0] || !jd->huffbits[b][1]) return JDR_FMT1;
                if (!jd->qttbl[jd->qtid[i]]) return JDR_FMT1;
            }
            n = jd->msy * jd->msx;
            if (!n) return JDR_FMT1;
            len = n * 64 * 2 + 64;
            if (len < 256) len = 256;
            jd->workbuf = alloc_pool(jd, len);
            if (!jd->workbuf) return JDR_MEM1;
            jd->mcubuf = (BYTE*)alloc_pool(jd, (n + 2) * 64);
            if (!jd->mcubuf) return JDR_MEM1;
            jd->dptr = seg; jd->dctr = 0; jd->dmsk = 0;
            if (ofs %= JD_SZBUF) {
                jd->dctr = jd->infunc(jd, seg + ofs, JD_SZBUF - (UINT)ofs);
                jd->dptr = seg + ofs - 1;
            }
            return JDR_OK;
        default:
            if (jd->infunc(jd, 0, len) != len) return JDR_INP;
        }
    }
}

JRESULT jd_decomp(JDEC* jd, UINT (*outfunc)(JDEC*, void*, JRECT*), BYTE scale) {
    UINT x, y, mx, my; WORD rst, rsc; JRESULT rc;
    jd->scale = scale; mx = jd->msx * 8; my = jd->msy * 8;
    jd->dcv[2] = jd->dcv[1] = jd->dcv[0] = 0; rst = rsc = 0; rc = JDR_OK;
    for (y = 0; y < jd->height; y += my) {
        for (x = 0; x < jd->width; x += mx) {
            if (jd->nrst && rst++ == jd->nrst) {
                rc = restart(jd, rsc++);
                if (rc != JDR_OK) return rc;
                rst = 1;
            }
            rc = mcu_load(jd);
            if (rc != JDR_OK) return rc;
            rc = mcu_output(jd, outfunc, x, y);
            if (rc != JDR_OK) return rc;
        }
    }
    return rc;
}