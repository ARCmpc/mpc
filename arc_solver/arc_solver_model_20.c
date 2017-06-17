/* This function was automatically generated by CasADi */#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
  #define CASADI_PREFIX(ID) arc_solver_model_20_ ## ID
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
/* evaluate_stages */
int arc_solver_model_20(const arc_solver_FLOAT** arg, arc_solver_FLOAT** res) {
     arc_solver_FLOAT a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17;
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
  a6=(a6-a8);
  a8=sq(a6);
         a9=arg[1] ? arg[1][5] : 0;
  a8=(a9*a8);
  a2=(a2+a8);
  a8=arg[0] ? arg[0][6] : 0;
         a10=arg[0] ? arg[0][0] : 0;
  a8=(a8-a10);
         a11=sq(a8);
         a12=arg[1] ? arg[1][6] : 0;
  a11=(a12*a11);
  a2=(a2+a11);
  a11=arg[0] ? arg[0][7] : 0;
         a13=arg[0] ? arg[0][1] : 0;
  a11=(a11-a13);
         a14=sq(a11);
         a15=arg[1] ? arg[1][7] : 0;
  a14=(a15*a14);
  a2=(a2+a14);
  a14=sq(a10);
         a16=arg[1] ? arg[1][8] : 0;
  a14=(a16*a14);
  a2=(a2+a14);
  a14=sq(a13);
         a17=arg[1] ? arg[1][9] : 0;
  a14=(a17*a14);
  a2=(a2+a14);
  if (res[0]!=0) res[0][0]=a2;
  a10=(a10+a10);
  a10=(a10*a16);
  a8=(a8+a8);
  a8=(a8*a12);
  a10=(a10-a8);
  if (res[1]!=0) res[1][0]=a10;
  a13=(a13+a13);
  a13=(a13*a17);
  a11=(a11+a11);
  a11=(a11*a15);
  a13=(a13-a11);
  if (res[1]!=0) res[1][1]=a13;
  a1=(a1+a1);
  a1=(a1*a3);
  if (res[1]!=0) res[1][2]=a1;
  a5=(a5+a5);
  a5=(a5*a7);
  if (res[1]!=0) res[1][3]=a5;
  a6=(a6+a6);
  a6=(a6*a9);
  if (res[1]!=0) res[1][4]=a6;
  if (res[1]!=0) res[1][5]=a8;
  if (res[1]!=0) res[1][6]=a11;
  a11=arg[1] ? arg[1][11] : 0;
  a0=(a0-a11);
  a11=sq(a0);
  a8=arg[1] ? arg[1][12] : 0;
  a4=(a4-a8);
  a8=sq(a4);
  a11=(a11+a8);
  a8=arg[1] ? arg[1][31] : 0;
  a8=sq(a8);
  a11=(a11/a8);
  if (res[2]!=0) res[2][0]=a11;
  a0=(a0+a0);
  a0=(a0/a8);
  if (res[3]!=0) res[3][0]=a0;
  a4=(a4+a4);
  a4=(a4/a8);
  if (res[3]!=0) res[3][1]=a4;
  return 0;
}

int arc_solver_model_20_init(int *f_type, int *n_in, int *n_out, int *sz_arg, int* sz_res) {
  *f_type = 1;
  *n_in = 2;
  *n_out = 4;
  *sz_arg = 2;
  *sz_res = 4;
  return 0;
}

int arc_solver_model_20_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row) {
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
    default:
      return 1;
  }

  *nrow = s[0];
  *ncol = s[1];
  *colind = s + 2;
  *row = s + 2 + (*ncol + 1);
  return 0;
}

int arc_solver_model_20_work(int *sz_iw, int *sz_w) {
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 18;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
