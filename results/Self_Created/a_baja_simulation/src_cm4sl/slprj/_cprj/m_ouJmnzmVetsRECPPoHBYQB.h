#ifndef __ouJmnzmVetsRECPPoHBYQB_h__
#define __ouJmnzmVetsRECPPoHBYQB_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef typedef_InstanceStruct_ouJmnzmVetsRECPPoHBYQB
#define typedef_InstanceStruct_ouJmnzmVetsRECPPoHBYQB

typedef struct {
  SimStruct *S;
  PyObject *namespaceDict;
  PyGILState_STATE GIL;
  void *emlrtRootTLSGlobal;
  real_T (*b_y0)[3];
} InstanceStruct_ouJmnzmVetsRECPPoHBYQB;

#endif                                 /* typedef_InstanceStruct_ouJmnzmVetsRECPPoHBYQB */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_ouJmnzmVetsRECPPoHBYQB(SimStruct *S, int_T method,
  void* data);

#endif
