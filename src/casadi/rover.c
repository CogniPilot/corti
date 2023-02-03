/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) rover_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_sq CASADI_PREFIX(sq)

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s1[15] = {1, 6, 0, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0};

/* rover:(t,T,PX[1x6],PY[1x6],L)->(x,y,psi,V,delta) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[2]? arg[2][0] : 0;
  a1=1.;
  a2=arg[0]? arg[0][0] : 0;
  a3=arg[1]? arg[1][0] : 0;
  a4=(a2/a3);
  a5=(a1-a4);
  a5=(a0*a5);
  a6=arg[2]? arg[2][1] : 0;
  a7=(a6*a4);
  a5=(a5+a7);
  a7=(a1-a4);
  a5=(a5*a7);
  a7=(a1-a4);
  a7=(a6*a7);
  a8=arg[2]? arg[2][2] : 0;
  a9=(a8*a4);
  a7=(a7+a9);
  a9=(a7*a4);
  a5=(a5+a9);
  a9=(a1-a4);
  a5=(a5*a9);
  a9=(a1-a4);
  a7=(a7*a9);
  a9=(a1-a4);
  a9=(a8*a9);
  a10=arg[2]? arg[2][3] : 0;
  a11=(a10*a4);
  a9=(a9+a11);
  a11=(a9*a4);
  a7=(a7+a11);
  a11=(a7*a4);
  a5=(a5+a11);
  a11=(a1-a4);
  a5=(a5*a11);
  a11=(a1-a4);
  a7=(a7*a11);
  a11=(a1-a4);
  a9=(a9*a11);
  a11=(a1-a4);
  a11=(a10*a11);
  a12=arg[2]? arg[2][4] : 0;
  a13=(a12*a4);
  a11=(a11+a13);
  a13=(a11*a4);
  a9=(a9+a13);
  a13=(a9*a4);
  a7=(a7+a13);
  a13=(a7*a4);
  a5=(a5+a13);
  a13=(a1-a4);
  a5=(a5*a13);
  a13=(a1-a4);
  a7=(a7*a13);
  a13=(a1-a4);
  a9=(a9*a13);
  a13=(a1-a4);
  a11=(a11*a13);
  a13=(a1-a4);
  a13=(a12*a13);
  a14=arg[2]? arg[2][5] : 0;
  a15=(a14*a4);
  a13=(a13+a15);
  a13=(a13*a4);
  a11=(a11+a13);
  a11=(a11*a4);
  a9=(a9+a11);
  a9=(a9*a4);
  a7=(a7+a9);
  a7=(a7*a4);
  a5=(a5+a7);
  if (res[0]!=0) res[0][0]=a5;
  a5=arg[3]? arg[3][0] : 0;
  a7=(a2/a3);
  a4=(a1-a7);
  a4=(a5*a4);
  a9=arg[3]? arg[3][1] : 0;
  a11=(a9*a7);
  a4=(a4+a11);
  a11=(a1-a7);
  a4=(a4*a11);
  a11=(a1-a7);
  a11=(a9*a11);
  a13=arg[3]? arg[3][2] : 0;
  a15=(a13*a7);
  a11=(a11+a15);
  a15=(a11*a7);
  a4=(a4+a15);
  a15=(a1-a7);
  a4=(a4*a15);
  a15=(a1-a7);
  a11=(a11*a15);
  a15=(a1-a7);
  a15=(a13*a15);
  a16=arg[3]? arg[3][3] : 0;
  a17=(a16*a7);
  a15=(a15+a17);
  a17=(a15*a7);
  a11=(a11+a17);
  a17=(a11*a7);
  a4=(a4+a17);
  a17=(a1-a7);
  a4=(a4*a17);
  a17=(a1-a7);
  a11=(a11*a17);
  a17=(a1-a7);
  a15=(a15*a17);
  a17=(a1-a7);
  a17=(a16*a17);
  a18=arg[3]? arg[3][4] : 0;
  a19=(a18*a7);
  a17=(a17+a19);
  a19=(a17*a7);
  a15=(a15+a19);
  a19=(a15*a7);
  a11=(a11+a19);
  a19=(a11*a7);
  a4=(a4+a19);
  a19=(a1-a7);
  a4=(a4*a19);
  a19=(a1-a7);
  a11=(a11*a19);
  a19=(a1-a7);
  a15=(a15*a19);
  a19=(a1-a7);
  a17=(a17*a19);
  a19=(a1-a7);
  a19=(a18*a19);
  a20=arg[3]? arg[3][5] : 0;
  a21=(a20*a7);
  a19=(a19+a21);
  a19=(a19*a7);
  a17=(a17+a19);
  a17=(a17*a7);
  a15=(a15+a17);
  a15=(a15*a7);
  a11=(a11+a15);
  a11=(a11*a7);
  a4=(a4+a11);
  if (res[1]!=0) res[1][0]=a4;
  a4=5.;
  a5=(a9-a5);
  a5=(a4*a5);
  a5=(a5/a3);
  a11=(a2/a3);
  a7=(a1-a11);
  a7=(a5*a7);
  a9=(a13-a9);
  a9=(a4*a9);
  a9=(a9/a3);
  a15=(a9*a11);
  a7=(a7+a15);
  a15=(a1-a11);
  a7=(a7*a15);
  a15=(a1-a11);
  a15=(a9*a15);
  a13=(a16-a13);
  a13=(a4*a13);
  a13=(a13/a3);
  a17=(a13*a11);
  a15=(a15+a17);
  a17=(a15*a11);
  a7=(a7+a17);
  a17=(a1-a11);
  a7=(a7*a17);
  a17=(a1-a11);
  a15=(a15*a17);
  a17=(a1-a11);
  a17=(a13*a17);
  a16=(a18-a16);
  a16=(a4*a16);
  a16=(a16/a3);
  a19=(a16*a11);
  a17=(a17+a19);
  a19=(a17*a11);
  a15=(a15+a19);
  a19=(a15*a11);
  a7=(a7+a19);
  a19=(a1-a11);
  a7=(a7*a19);
  a19=(a1-a11);
  a15=(a15*a19);
  a19=(a1-a11);
  a17=(a17*a19);
  a19=(a1-a11);
  a19=(a16*a19);
  a20=(a20-a18);
  a20=(a4*a20);
  a20=(a20/a3);
  a18=(a20*a11);
  a19=(a19+a18);
  a19=(a19*a11);
  a17=(a17+a19);
  a17=(a17*a11);
  a15=(a15+a17);
  a15=(a15*a11);
  a7=(a7+a15);
  a0=(a6-a0);
  a0=(a4*a0);
  a0=(a0/a3);
  a15=(a2/a3);
  a11=(a1-a15);
  a11=(a0*a11);
  a6=(a8-a6);
  a6=(a4*a6);
  a6=(a6/a3);
  a17=(a6*a15);
  a11=(a11+a17);
  a17=(a1-a15);
  a11=(a11*a17);
  a17=(a1-a15);
  a17=(a6*a17);
  a8=(a10-a8);
  a8=(a4*a8);
  a8=(a8/a3);
  a19=(a8*a15);
  a17=(a17+a19);
  a19=(a17*a15);
  a11=(a11+a19);
  a19=(a1-a15);
  a11=(a11*a19);
  a19=(a1-a15);
  a17=(a17*a19);
  a19=(a1-a15);
  a19=(a8*a19);
  a10=(a12-a10);
  a10=(a4*a10);
  a10=(a10/a3);
  a18=(a10*a15);
  a19=(a19+a18);
  a18=(a19*a15);
  a17=(a17+a18);
  a18=(a17*a15);
  a11=(a11+a18);
  a18=(a1-a15);
  a11=(a11*a18);
  a18=(a1-a15);
  a17=(a17*a18);
  a18=(a1-a15);
  a19=(a19*a18);
  a18=(a1-a15);
  a18=(a10*a18);
  a14=(a14-a12);
  a4=(a4*a14);
  a4=(a4/a3);
  a14=(a4*a15);
  a18=(a18+a14);
  a18=(a18*a15);
  a19=(a19+a18);
  a19=(a19*a15);
  a17=(a17+a19);
  a17=(a17*a15);
  a11=(a11+a17);
  a17=atan2(a7,a11);
  if (res[2]!=0) res[2][0]=a17;
  a17=casadi_sq(a11);
  a15=casadi_sq(a7);
  a17=(a17+a15);
  a17=sqrt(a17);
  if (res[3]!=0) res[3][0]=a17;
  a15=arg[4]? arg[4][0] : 0;
  a19=4.;
  a5=(a9-a5);
  a5=(a19*a5);
  a5=(a5/a3);
  a18=(a2/a3);
  a14=(a1-a18);
  a5=(a5*a14);
  a9=(a13-a9);
  a9=(a19*a9);
  a9=(a9/a3);
  a14=(a9*a18);
  a5=(a5+a14);
  a14=(a1-a18);
  a5=(a5*a14);
  a14=(a1-a18);
  a9=(a9*a14);
  a13=(a16-a13);
  a13=(a19*a13);
  a13=(a13/a3);
  a14=(a13*a18);
  a9=(a9+a14);
  a14=(a9*a18);
  a5=(a5+a14);
  a14=(a1-a18);
  a5=(a5*a14);
  a14=(a1-a18);
  a9=(a9*a14);
  a14=(a1-a18);
  a13=(a13*a14);
  a20=(a20-a16);
  a20=(a19*a20);
  a20=(a20/a3);
  a20=(a20*a18);
  a13=(a13+a20);
  a13=(a13*a18);
  a9=(a9+a13);
  a9=(a9*a18);
  a5=(a5+a9);
  a11=(a11*a5);
  a0=(a6-a0);
  a0=(a19*a0);
  a0=(a0/a3);
  a2=(a2/a3);
  a5=(a1-a2);
  a0=(a0*a5);
  a6=(a8-a6);
  a6=(a19*a6);
  a6=(a6/a3);
  a5=(a6*a2);
  a0=(a0+a5);
  a5=(a1-a2);
  a0=(a0*a5);
  a5=(a1-a2);
  a6=(a6*a5);
  a8=(a10-a8);
  a8=(a19*a8);
  a8=(a8/a3);
  a5=(a8*a2);
  a6=(a6+a5);
  a5=(a6*a2);
  a0=(a0+a5);
  a5=(a1-a2);
  a0=(a0*a5);
  a5=(a1-a2);
  a6=(a6*a5);
  a1=(a1-a2);
  a8=(a8*a1);
  a4=(a4-a10);
  a19=(a19*a4);
  a19=(a19/a3);
  a19=(a19*a2);
  a8=(a8+a19);
  a8=(a8*a2);
  a6=(a6+a8);
  a6=(a6*a2);
  a0=(a0+a6);
  a7=(a7*a0);
  a11=(a11-a7);
  a11=(a11/a17);
  a15=(a15*a11);
  a15=(a15/a17);
  a15=atan(a15);
  if (res[4]!=0) res[4][0]=a15;
  return 0;
}

int rover(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

int rover_alloc_mem(void) {
  return 0;
}

int rover_init_mem(int mem) {
  return 0;
}

void rover_free_mem(int mem) {
}

int rover_checkout(void) {
  return 0;
}

void rover_release(int mem) {
}

void rover_incref(void) {
}

void rover_decref(void) {
}

casadi_int rover_n_in(void) { return 5;}

casadi_int rover_n_out(void) { return 5;}

casadi_real rover_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* rover_name_in(casadi_int i){
  switch (i) {
    case 0: return "t";
    case 1: return "T";
    case 2: return "PX";
    case 3: return "PY";
    case 4: return "L";
    default: return 0;
  }
}

const char* rover_name_out(casadi_int i){
  switch (i) {
    case 0: return "x";
    case 1: return "y";
    case 2: return "psi";
    case 3: return "V";
    case 4: return "delta";
    default: return 0;
  }
}

const casadi_int* rover_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    case 4: return casadi_s0;
    default: return 0;
  }
}

const casadi_int* rover_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    case 3: return casadi_s0;
    case 4: return casadi_s0;
    default: return 0;
  }
}

int rover_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
