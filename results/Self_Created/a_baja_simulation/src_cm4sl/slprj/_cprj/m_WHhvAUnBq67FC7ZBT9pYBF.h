#ifndef __WHhvAUnBq67FC7ZBT9pYBF_h__
#define __WHhvAUnBq67FC7ZBT9pYBF_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef typedef_InstanceStruct_WHhvAUnBq67FC7ZBT9pYBF
#define typedef_InstanceStruct_WHhvAUnBq67FC7ZBT9pYBF

typedef struct {
  SimStruct *S;
  PyObject *namespaceDict;
  PyGILState_STATE GIL;
  void *emlrtRootTLSGlobal;
  real_T (*b_y0)[3];
} InstanceStruct_WHhvAUnBq67FC7ZBT9pYBF;

#endif                                 /* typedef_InstanceStruct_WHhvAUnBq67FC7ZBT9pYBF */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_WHhvAUnBq67FC7ZBT9pYBF(SimStruct *S, int_T method,
  void* data);

#endif
