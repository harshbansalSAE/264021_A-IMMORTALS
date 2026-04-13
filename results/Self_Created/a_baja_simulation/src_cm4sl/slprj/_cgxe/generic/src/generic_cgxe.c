/* Include files */

#include "generic_cgxe.h"
#include "m_ouJmnzmVetsRECPPoHBYQB.h"

unsigned int cgxe_generic_method_dispatcher(SimStruct* S, int_T method, void
  * data)
{
  if (ssGetChecksum0(S) == 1136306472 &&
      ssGetChecksum1(S) == 1809472304 &&
      ssGetChecksum2(S) == 3349506629 &&
      ssGetChecksum3(S) == 879182699) {
    method_dispatcher_ouJmnzmVetsRECPPoHBYQB(S, method, data);
    return 1;
  }

  return 0;
}
