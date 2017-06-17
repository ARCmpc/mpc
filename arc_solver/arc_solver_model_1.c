/* This function was automatically generated by CasADi */#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
  #define CASADI_PREFIX(ID) arc_solver_model_1_ ## ID
#endif /* CODEGEN_PREFIX */

#include <math.h>

#include "arc_solver/include/arc_solver.h"

#define PRINTF printf
arc_solver_FLOAT CASADI_PREFIX(sq)(arc_solver_FLOAT x) { return x*x;}
#define sq(x) CASADI_PREFIX(sq)(x)

arc_solver_FLOAT CASADI_PREFIX(sign)(arc_solver_FLOAT x) { return x<0 ? -1 : x>0 ? 1 : x;}
#define sign(x) CASADI_PREFIX(sign)(x)

static const int CASADI_PREFIX(s0)[] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
#define s0 CASADI_PREFIX(s0)
static const int CASADI_PREFIX(s1)[] = {41, 1, 0, 41, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
#define s1 CASADI_PREFIX(s1)
static const int CASADI_PREFIX(s2)[] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
static const int CASADI_PREFIX(s3)[] = {1, 8, 0, 1, 2, 3, 4, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0};
#define s3 CASADI_PREFIX(s3)
static const int CASADI_PREFIX(s4)[] = {1, 8, 0, 0, 0, 1, 2, 2, 2, 2, 2, 0, 0};
#define s4 CASADI_PREFIX(s4)
static const int CASADI_PREFIX(s5)[] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
#define s5 CASADI_PREFIX(s5)
static const int CASADI_PREFIX(s6)[] = {6, 8, 0, 5, 9, 10, 11, 15, 18, 18, 18, 0, 1, 2, 3, 4, 0, 1, 3, 5, 0, 1, 0, 1, 2, 3, 0, 1, 3};
#define s6 CASADI_PREFIX(s6)
/* evaluate_stages */
int arc_solver_model_1(const arc_solver_FLOAT** arg, arc_solver_FLOAT** res) {
     arc_solver_FLOAT a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38;
         a0=arg[0] ? arg[0][2] : 0;
         a1=arg[1] ? arg[1][0] : 0;
  a1=(a0-a1);
         a2=sq(a1);
         a3=arg[1] ? arg[1][3] : 0;
  a2=(a3*a2);
         a4=arg[0] ? arg[0][3] : 0;
         a5=arg[1] ? arg[1][1] : 0;
  a5=(a4-a5);
         a6=sq(a5);
         a7=arg[1] ? arg[1][4] : 0;
  a6=(a7*a6);
  a2=(a2+a6);
  a6=arg[0] ? arg[0][5] : 0;
         a8=arg[1] ? arg[1][2] : 0;
  a8=(a6-a8);
         a9=sq(a8);
         a10=arg[1] ? arg[1][5] : 0;
  a9=(a10*a9);
  a2=(a2+a9);
  a9=arg[0] ? arg[0][6] : 0;
         a11=arg[0] ? arg[0][0] : 0;
  a9=(a9-a11);
         a12=sq(a9);
         a13=arg[1] ? arg[1][6] : 0;
  a12=(a13*a12);
  a2=(a2+a12);
  a12=arg[0] ? arg[0][7] : 0;
         a14=arg[0] ? arg[0][1] : 0;
  a12=(a12-a14);
         a15=sq(a12);
         a16=arg[1] ? arg[1][7] : 0;
  a15=(a16*a15);
  a2=(a2+a15);
  a15=sq(a11);
         a17=arg[1] ? arg[1][8] : 0;
  a15=(a17*a15);
  a2=(a2+a15);
  a15=sq(a14);
         a18=arg[1] ? arg[1][9] : 0;
  a15=(a18*a15);
  a2=(a2+a15);
  if (res[0]!=0) res[0][0]=a2;
  a2=(a11+a11);
  a2=(a2*a17);
  a9=(a9+a9);
  a9=(a9*a13);
  a2=(a2-a9);
  if (res[1]!=0) res[1][0]=a2;
  a2=(a14+a14);
  a2=(a2*a18);
  a12=(a12+a12);
  a12=(a12*a16);
  a2=(a2-a12);
  if (res[1]!=0) res[1][1]=a2;
  a1=(a1+a1);
  a1=(a1*a3);
  if (res[1]!=0) res[1][2]=a1;
  a5=(a5+a5);
  a5=(a5*a7);
  if (res[1]!=0) res[1][3]=a5;
  a8=(a8+a8);
  a8=(a8*a10);
  if (res[1]!=0) res[1][4]=a8;
  if (res[1]!=0) res[1][5]=a9;
  if (res[1]!=0) res[1][6]=a12;
  a12=arg[1] ? arg[1][11] : 0;
  a12=(a0-a12);
  a9=sq(a12);
  a8=arg[1] ? arg[1][12] : 0;
  a8=(a4-a8);
  a10=sq(a8);
  a9=(a9+a10);
  a10=arg[1] ? arg[1][31] : 0;
  a10=sq(a10);
  a9=(a9/a10);
  if (res[2]!=0) res[2][0]=a9;
  a12=(a12+a12);
  a12=(a12/a10);
  if (res[3]!=0) res[3][0]=a12;
  a8=(a8+a8);
  a8=(a8/a10);
  if (res[3]!=0) res[3][1]=a8;
  a8=cos(a6);
  a10=arg[0] ? arg[0][4] : 0;
  a12=(a10*a8);
  a9=1.0000000000000001e-01;
  a5=(a9*a11);
  a5=(a10+a5);
  a7=7.5000000000000000e-01;
  a1=(a14/a7);
  a3=tan(a1);
  a2=(a3*a10);
  a16=2.3558500000000002e+00;
  a2=(a2/a16);
  a18=(a9*a2);
  a18=(a6+a18);
  a13=cos(a18);
  a17=(a5*a13);
  a15=2.;
  a17=(a15*a17);
  a12=(a12+a17);
  a17=(a9*a11);
  a17=(a10+a17);
         a19=(a14/a7);
         a20=tan(a19);
         a21=(a20*a5);
  a21=(a21/a16);
         a22=(a9*a21);
  a22=(a6+a22);
         a23=cos(a22);
         a24=(a17*a23);
  a24=(a15*a24);
  a12=(a12+a24);
  a24=2.0000000000000001e-01;
         a25=(a24*a11);
  a25=(a10+a25);
         a26=(a14/a7);
         a27=tan(a26);
         a28=(a27*a17);
  a28=(a28/a16);
         a29=(a24*a28);
  a29=(a6+a29);
         a30=cos(a29);
         a31=(a25*a30);
  a12=(a12+a31);
  a31=3.3333333333333333e-02;
  a12=(a31*a12);
  a0=(a0+a12);
  if (res[4]!=0) res[4][0]=a0;
  a0=sin(a6);
  a12=(a10*a0);
         a32=sin(a18);
         a33=(a5*a32);
  a33=(a15*a33);
  a12=(a12+a33);
  a33=sin(a22);
         a34=(a17*a33);
  a34=(a15*a34);
  a12=(a12+a34);
  a34=sin(a29);
         a35=(a25*a34);
  a12=(a12+a35);
  a12=(a31*a12);
  a4=(a4+a12);
  if (res[4]!=0) res[4][1]=a4;
  a4=(a15*a11);
  a4=(a11+a4);
  a12=(a15*a11);
  a4=(a4+a12);
  a4=(a4+a11);
  a4=(a31*a4);
  a4=(a10+a4);
  if (res[4]!=0) res[4][2]=a4;
  a21=(a15*a21);
  a2=(a2+a21);
  a28=(a15*a28);
  a2=(a2+a28);
  a7=(a14/a7);
  a28=tan(a7);
  a21=(a28*a25);
  a21=(a21/a16);
  a2=(a2+a21);
  a2=(a31*a2);
  a2=(a6+a2);
  if (res[4]!=0) res[4][3]=a2;
  if (res[4]!=0) res[4][4]=a11;
  if (res[4]!=0) res[4][5]=a14;
  a14=(a9*a13);
  a14=(a15*a14);
  a11=(a9*a23);
  a2=(a9*a20);
  a21=4.2447524248148222e-01;
  a2=(a21*a2);
  a16=(a9*a2);
  a4=sin(a22);
  a12=(a4*a16);
  a12=(a17*a12);
  a11=(a11-a12);
  a11=(a15*a11);
  a14=(a14+a11);
  a11=(a24*a30);
  a12=(a9*a27);
  a12=(a21*a12);
  a35=(a24*a12);
         a36=sin(a29);
         a37=(a36*a35);
  a37=(a25*a37);
  a11=(a11-a37);
  a14=(a14+a11);
  a14=(a31*a14);
  if (res[5]!=0) res[5][0]=a14;
  a14=(a9*a32);
  a14=(a15*a14);
  a11=(a9*a33);
  a22=cos(a22);
  a16=(a22*a16);
  a16=(a17*a16);
  a11=(a11+a16);
  a11=(a15*a11);
  a14=(a14+a11);
  a11=(a24*a34);
  a29=cos(a29);
  a35=(a29*a35);
  a35=(a25*a35);
  a11=(a11+a35);
  a14=(a14+a11);
  a14=(a31*a14);
  if (res[5]!=0) res[5][1]=a14;
  if (res[5]!=0) res[5][2]=a24;
  a2=(a15*a2);
  a12=(a15*a12);
  a2=(a2+a12);
  a12=(a24*a28);
  a12=(a21*a12);
  a2=(a2+a12);
  a2=(a31*a2);
  if (res[5]!=0) res[5][3]=a2;
  a2=1.;
  if (res[5]!=0) res[5][4]=a2;
  a1=cos(a1);
  a1=sq(a1);
  a12=1.3333333333333333e+00;
  a1=(a12/a1);
  a1=(a10*a1);
  a1=(a21*a1);
  a14=(a9*a1);
  a11=sin(a18);
  a35=(a11*a14);
  a35=(a5*a35);
  a35=(a15*a35);
  a19=cos(a19);
  a19=sq(a19);
  a19=(a12/a19);
  a19=(a5*a19);
  a19=(a21*a19);
  a16=(a9*a19);
  a37=(a4*a16);
  a37=(a17*a37);
  a37=(a15*a37);
  a35=(a35+a37);
  a26=cos(a26);
  a26=sq(a26);
  a26=(a12/a26);
  a26=(a17*a26);
  a26=(a21*a26);
  a37=(a24*a26);
         a38=(a36*a37);
  a38=(a25*a38);
  a35=(a35+a38);
  a35=(a31*a35);
  a35=(-a35);
  if (res[5]!=0) res[5][5]=a35;
  a18=cos(a18);
  a14=(a18*a14);
  a14=(a5*a14);
  a14=(a15*a14);
  a16=(a22*a16);
  a16=(a17*a16);
  a16=(a15*a16);
  a14=(a14+a16);
  a37=(a29*a37);
  a37=(a25*a37);
  a14=(a14+a37);
  a14=(a31*a14);
  if (res[5]!=0) res[5][6]=a14;
  a19=(a15*a19);
  a1=(a1+a19);
  a26=(a15*a26);
  a1=(a1+a26);
  a7=cos(a7);
  a7=sq(a7);
  a12=(a12/a7);
  a12=(a25*a12);
  a12=(a21*a12);
  a1=(a1+a12);
  a1=(a31*a1);
  if (res[5]!=0) res[5][7]=a1;
  if (res[5]!=0) res[5][8]=a2;
  if (res[5]!=0) res[5][9]=a2;
  if (res[5]!=0) res[5][10]=a2;
  a3=(a21*a3);
  a1=(a9*a3);
  a12=(a11*a1);
  a12=(a5*a12);
  a13=(a13-a12);
  a13=(a15*a13);
  a8=(a8+a13);
  a20=(a21*a20);
  a9=(a9*a20);
  a13=(a4*a9);
  a13=(a17*a13);
  a23=(a23-a13);
  a23=(a15*a23);
  a8=(a8+a23);
  a27=(a21*a27);
  a24=(a24*a27);
  a23=(a36*a24);
  a23=(a25*a23);
  a30=(a30-a23);
  a8=(a8+a30);
  a8=(a31*a8);
  if (res[5]!=0) res[5][11]=a8;
  a1=(a18*a1);
  a1=(a5*a1);
  a32=(a32+a1);
  a32=(a15*a32);
  a0=(a0+a32);
  a9=(a22*a9);
  a9=(a17*a9);
  a33=(a33+a9);
  a33=(a15*a33);
  a0=(a0+a33);
  a24=(a29*a24);
  a24=(a25*a24);
  a34=(a34+a24);
  a0=(a0+a34);
  a0=(a31*a0);
  if (res[5]!=0) res[5][12]=a0;
  if (res[5]!=0) res[5][13]=a2;
  a20=(a15*a20);
  a3=(a3+a20);
  a27=(a15*a27);
  a3=(a3+a27);
  a21=(a21*a28);
  a3=(a3+a21);
  a3=(a31*a3);
  if (res[5]!=0) res[5][14]=a3;
  a3=sin(a6);
  a3=(a10*a3);
  a11=(a5*a11);
  a11=(a15*a11);
  a3=(a3+a11);
  a4=(a17*a4);
  a4=(a15*a4);
  a3=(a3+a4);
  a36=(a25*a36);
  a3=(a3+a36);
  a3=(a31*a3);
  a3=(-a3);
  if (res[5]!=0) res[5][15]=a3;
  a6=cos(a6);
  a10=(a10*a6);
  a5=(a5*a18);
  a5=(a15*a5);
  a10=(a10+a5);
  a17=(a17*a22);
  a15=(a15*a17);
  a10=(a10+a15);
  a25=(a25*a29);
  a10=(a10+a25);
  a31=(a31*a10);
  if (res[5]!=0) res[5][16]=a31;
  if (res[5]!=0) res[5][17]=a2;
  return 0;
}

int arc_solver_model_1_init(int *f_type, int *n_in, int *n_out, int *sz_arg, int* sz_res) {
  *f_type = 1;
  *n_in = 2;
  *n_out = 6;
  *sz_arg = 2;
  *sz_res = 6;
  return 0;
}

int arc_solver_model_1_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row) {
  const int* s;
  switch (i) {
    case 0:
      s = s0; break;
    case 1:
      s = s1; break;
    case 2:
    case 4:
      s = s2; break;
    case 3:
      s = s3; break;
    case 5:
      s = s4; break;
    case 6:
      s = s5; break;
    case 7:
      s = s6; break;
    default:
      return 1;
  }

  *nrow = s[0];
  *ncol = s[1];
  *colind = s + 2;
  *row = s + 2 + (*ncol + 1);
  return 0;
}

int arc_solver_model_1_work(int *sz_iw, int *sz_w) {
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 39;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
