// fpchk -- the FPU on the board against tools/fpref.py's vectors: every result bit and fflags,
// each op under its vector's rounding mode (frm, the instruction's rm dynamic).
//   fpchk fma <file>        lines "v sf df rm x a b c result flags" (formats 0 S, 1 D, 2 H)
//   fpchk div|sqrt <file>   lines "fmt rm a b result flags"
//   fpchk zfa <file>        lines "op fmt rm a b result flags": Zfa's fround, froundnx, fminm, fmaxm,
//                           fleq, fltq, fli (a is fli's table index)
#include <stdio.h>
#include <stdint.h>
#include <string.h>
typedef uint64_t u64;
#define F3(I) asm volatile("fsflags x0\n fmv.d.x ft1,%2\n fmv.d.x ft2,%3\n fmv.d.x ft3,%4\n " I " ft0,ft1,ft2,ft3\n fmv.x.d %0,ft0\n frflags %1" : "=r"(r), "=r"(fl) : "r"(a), "r"(b), "r"(c) : "ft0","ft1","ft2","ft3")
#define F2(I) asm volatile("fsflags x0\n fmv.d.x ft1,%2\n fmv.d.x ft2,%3\n " I " ft0,ft1,ft2\n fmv.x.d %0,ft0\n frflags %1" : "=r"(r), "=r"(fl) : "r"(a), "r"(b) : "ft0","ft1","ft2")
#define F1(I) asm volatile("fsflags x0\n fmv.d.x ft1,%2\n " I " ft0,ft1\n fmv.x.d %0,ft0\n frflags %1" : "=r"(r), "=r"(fl) : "r"(a) : "ft0","ft1")
#define XI(I) asm volatile("fsflags x0\n " I " ft0,%2\n fmv.x.d %0,ft0\n frflags %1" : "=r"(r), "=r"(fl) : "r"(a) : "ft0")
#define FX(I) asm volatile("fsflags x0\n fmv.d.x ft1,%2\n " I " %0,ft1\n frflags %1" : "=r"(r), "=r"(fl) : "r"(a) : "ft1")
#define XF(I) asm volatile("fsflags x0\n fmv.d.x ft1,%2\n fmv.d.x ft2,%3\n " I " %0,ft1,ft2\n frflags %1" : "=r"(r), "=r"(fl) : "r"(a), "r"(b) : "ft1","ft2")
// fli's index is an immediate: one instruction per entry (fli.d / fli.s, rs2 = 1, rs1 = the index)
#define FLI1(i) case i: if (d) asm volatile("fsflags x0\n .insn r 0x53, 0, 0x79, ft0, x" #i ", x1\n fmv.x.d %0,ft0\n frflags %1" : "=r"(r), "=r"(fl) :: "ft0"); \
                        else   asm volatile("fsflags x0\n .insn r 0x53, 0, 0x78, ft0, x" #i ", x1\n fmv.x.d %0,ft0\n frflags %1" : "=r"(r), "=r"(fl) :: "ft0"); break;
int main(int argc, char **argv) {
   FILE *f = fopen(argv[2], "r");
   int fma = !strcmp(argv[1], "fma"), sq = !strcmp(argv[1], "sqrt"), zfa = !strcmp(argv[1], "zfa");
   long n = 0, bad = 0;
   unsigned v, sf, df, rm, x, fmt, wf;
   u64 a, b, c, want, r, fl;
   for (;;) {
      if (fma) {
         if (fscanf(f, "%u %u %u %u %u %lx %lx %lx %lx %x", &v, &sf, &df, &rm, &x, &a, &b, &c, &want, &wf) != 10) break;
      } else if (zfa) {
         if (fscanf(f, "%u %u %u %lx %lx %lx %x", &v, &fmt, &rm, &a, &b, &want, &wf) != 7) break;
         sf = df = fmt; v += 30; c = 0; x = 0;
      } else {
         if (fscanf(f, "%u %u %lx %lx %lx %x", &fmt, &rm, &a, &b, &want, &wf) != 6) break;
         sf = df = fmt; v = sq ? 21 : 20; c = 0; x = 0;
      }
      asm volatile("fsrm %0" :: "r"((u64)rm));
      int d = df;
      switch (v) {
      case 0: if (d) F3("fmadd.d"); else F3("fmadd.s"); break;
      case 1: if (d) F3("fmsub.d"); else F3("fmsub.s"); break;
      case 2: if (d) F3("fnmsub.d"); else F3("fnmsub.s"); break;
      case 3: if (d) F3("fnmadd.d"); else F3("fnmadd.s"); break;
      case 4: b = c; if (d) F2("fadd.d"); else F2("fadd.s"); break;
      case 5: b = c; if (d) F2("fsub.d"); else F2("fsub.s"); break;
      case 6: if (d) F2("fmul.d"); else F2("fmul.s"); break;
      case 7: switch (sf * 3 + df) {   /* 0 S, 1 D, 2 H */
         case 1: F1("fcvt.d.s"); break;  case 3: F1("fcvt.s.d"); break;
         case 6: F1("fcvt.s.h"); break;  case 7: F1("fcvt.d.h"); break;
         case 2: F1("fcvt.h.s"); break;  default: F1("fcvt.h.d"); break; } break;
      case 8: switch (x * 2 + d) {   /* x = {il, iu} */
         case 0: XI("fcvt.s.w"); break;  case 1: XI("fcvt.d.w"); break;
         case 2: XI("fcvt.s.wu"); break; case 3: XI("fcvt.d.wu"); break;
         case 4: XI("fcvt.s.l"); break;  case 5: XI("fcvt.d.l"); break;
         case 6: XI("fcvt.s.lu"); break; default: XI("fcvt.d.lu"); break; } break;
      case 9: switch (x * 2 + sf) {
         case 0: FX("fcvt.w.s"); break;  case 1: FX("fcvt.w.d"); break;
         case 2: FX("fcvt.wu.s"); break; case 3: FX("fcvt.wu.d"); break;
         case 4: FX("fcvt.l.s"); break;  case 5: FX("fcvt.l.d"); break;
         case 6: FX("fcvt.lu.s"); break; default: FX("fcvt.lu.d"); break; } break;
      case 10: asm volatile("fsflags x0\n fmv.d.x ft1,%2\n fcvtmod.w.d %0,ft1,rtz\n frflags %1"   /* RTZ, its only rm */
                            : "=r"(r), "=r"(fl) : "r"(a) : "ft1"); break;
      case 20: if (d) F2("fdiv.d"); else F2("fdiv.s"); break;
      case 21: if (d) F1("fsqrt.d"); else F1("fsqrt.s"); break;
      case 30: if (d) F1("fround.d"); else F1("fround.s"); break;
      case 31: if (d) F1("froundnx.d"); else F1("froundnx.s"); break;
      case 32: if (d) F2("fminm.d"); else F2("fminm.s"); break;
      case 33: if (d) F2("fmaxm.d"); else F2("fmaxm.s"); break;
      case 34: if (d) XF("fleq.d"); else XF("fleq.s"); break;
      case 35: if (d) XF("fltq.d"); else XF("fltq.s"); break;
      default: switch (a) {
         FLI1(0) FLI1(1) FLI1(2) FLI1(3) FLI1(4) FLI1(5) FLI1(6) FLI1(7) FLI1(8) FLI1(9) FLI1(10) FLI1(11) FLI1(12) FLI1(13) FLI1(14) FLI1(15)
         FLI1(16) FLI1(17) FLI1(18) FLI1(19) FLI1(20) FLI1(21) FLI1(22) FLI1(23) FLI1(24) FLI1(25) FLI1(26) FLI1(27) FLI1(28) FLI1(29) FLI1(30) FLI1(31) } break;
      }
      if (r != want || fl != wf) {
         if (++bad <= 15)
            printf("DIFF v=%u sf=%u df=%u x=%u rm=%u a=%016lx b=%016lx c=%016lx got %016lx/%02lx want %016lx/%02x\n",
                   v, sf, df, x, rm, a, b, c, r, fl, want, wf);
      }
      n++;
   }
   printf("FPCHK %s: %ld vectors, %ld mismatches\n", argv[1], n, bad);
   return bad != 0;
}
