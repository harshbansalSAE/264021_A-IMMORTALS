/* Include files */

#include "modelInterface.h"
#include "m_ouJmnzmVetsRECPPoHBYQB.h"
#include <string.h>
#include "mwstringutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void cgxe_mdl_start(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance);
static void cgxe_mdl_initialize(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void cgxe_mdl_outputs(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void cgxe_mdl_update(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void cgxe_mdl_derivative(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void cgxe_mdl_enable(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void cgxe_mdl_disable(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void cgxe_mdl_terminate(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);
static void CheckPythonError(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *pyObjsToRelease[], int32_T numObjToRelease);
static void PyObj_marshalIn(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *pyToMarshal, PyObject *pyOwner, real_T outputVal[3]);
static PyObject *getPyNamespaceDict(void);
static void assignToPyDict(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  PyObject *dict, char_T *key, real_T val[3]);
static PyObject *marshalOut(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, real_T data[3]);
static void execPyScript(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  char_T *script, PyObject *ns);
static PyObject *getPyDictVal(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *dict, char_T *key);
static void b_assignToPyDict(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *dict, char_T *key, real_T val[3]);
static void b_execPyScript(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  char_T *script, PyObject *ns);
static PyObject *b_getPyDictVal(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *dict, char_T *key);
static void c_execPyScript(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  char_T *script, PyObject *ns);
static int32_T _s32_s64_(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  int64_T b);
static void init_simulink_io_address(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance);

/* Function Definitions */
static void cgxe_mdl_start(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance)
{
  init_simulink_io_address(moduleInstance);
  cgxertSetSimStateCompliance(moduleInstance->S, 2);
}

static void cgxe_mdl_initialize(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  PyObject *r;
  real_T dv[3];
  real_T output[3];
  int32_T i;
  cgxertInitMLPythonIFace();
  moduleInstance->GIL = PyGILState_Ensure();
  moduleInstance->namespaceDict = getPyNamespaceDict();
  for (i = 0; i < 3; i++) {
    output[i] = 0.0;
  }

  assignToPyDict(moduleInstance, moduleInstance->namespaceDict, "radar_out",
                 output);
  execPyScript(moduleInstance,
               "import socket\nimport struct\nimport math\nimport numpy as np\nfrom sklearn.cluster import DBSCAN\n\n# Define variables in the g"
               "lobal scope immediately\nglobal tcp_sock, buffer, last_data\ntcp_sock = None\nbuffer = b\"\"\nlast_data = [0.0, 0.0, 0.0]\n\n# C"
               "onstants\nRSI_HDR_FMT = \"<hhiiifi\"\nRADAR_HDR_FMT = \"<iiiiii\"\nPOINT_FMT = \"<fffff\"\nCM_TEXT_HDR_SIZE = 64\nRSI_HDR_SIZE ="
               " struct.calcsize(RSI_HDR_FMT)\nRADAR_HDR_SIZE = struct.calcsize(RADAR_HDR_FMT)\nPOINT_SIZE = struct.calcsize(POINT_FMT)\n\nHOST "
               "= \"127.0.0.1\"\nPORT = 2212\n\ndef connect_carmaker():\n    global tcp_sock, buffer\n    try:\n        # Close existing if it e"
               "xists\n        if tcp_sock:\n            tcp_sock.close()\n        \n        tcp_sock = socket.socket(socket.AF_INET, socket.SOC"
               "K_STREAM)\n        tcp_sock.settimeout(0.002) \n        tcp_sock.connect((HOST, PORT))\n        buffer = b\"\"\n        return T"
               "rue\n    except Exception:\n        tcp_sock = None\n        return False\n\n# Initial attempt\nconnect_carmaker()",
               moduleInstance->namespaceDict);
  r = getPyDictVal(moduleInstance, moduleInstance->namespaceDict, "radar_out");
  PyObj_marshalIn(moduleInstance, r, NULL, dv);
  for (i = 0; i < 3; i++) {
    (*moduleInstance->b_y0)[i] = dv[i];
  }

  Py_DecRef(r);
  PyGILState_Release(moduleInstance->GIL);
}

static void cgxe_mdl_outputs(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  PyObject *r;
  real_T dv[3];
  int32_T i;
  moduleInstance->GIL = PyGILState_Ensure();
  b_assignToPyDict(moduleInstance, moduleInstance->namespaceDict, "radar_out",
                   *moduleInstance->b_y0);
  b_execPyScript(moduleInstance,
                 "# Safety check: Define variables if the namespace was cleared\nif \'tcp_sock\' not in globals():\n    global tcp_sock, buffer, l"
                 "ast_data\n    tcp_sock = None\n    buffer = b\"\"\n    last_data = [0.0, 0.0, 0.0]\n\n# 1. Handle Reconnection logic\nif tcp_soc"
                 "k is None:\n    # Try to connect inside the function defined in Initialize\n    try:\n        # Calling the function from Initia"
                 "lize tab\n        connect_carmaker() \n    except:\n        pass\n    radar_out = last_data # Return [0,0,0] while trying to con"
                 "nect\nelse:\n    # 2. Receive Data\n    try:\n        chunk = tcp_sock.recv(16384)\n        if not chunk:\n            tcp_sock."
                 "close()\n            tcp_sock = None\n            last_data = [0.0, 0.0, 0.0]\n        else:\n            buffer += chunk\n    e"
                 "xcept socket.timeout:\n        pass \n    except Exception:\n        tcp_sock = None\n        last_data = [0.0, 0.0, 0.0]\n\n   "
                 " # 3. Parse Buffer (Same logic as before)\n    while len(buffer) >= 64:\n        if not buffer.startswith(b\"*\"):\n            "
                 "idx = buffer.find(b\"*\", 1)\n            buffer = buffer[idx:] if idx != -1 else b\"\"\n            continue\n        \n       "
                 " header_text = buffer[:64].decode(\'ascii\', \'ignore\').split()\n        if len(header_text) >= 3 and header_text[0] == \"*Rada"
                 "rRSI\":\n            msg_bytes = int(header_text[2])\n            total_len = 64 + msg_bytes\n            if len(buffer) < total"
                 "_len: break\n            \n            binary = buffer[64:total_len]\n            buffer = buffer[total_len:]\n            \n   "
                 "         # Extract data and run clustering\n            _, _, _, _, _, _, n_sens = struct.unpack_from(\"<hhiiifi\", binary, 0)\n"
                 "            offset = 24 # RSI_HDR_SIZE\n            \n            all_pts = []\n            for _ in range(max(0, n_sens)):\n   "
                 "             sid, otype, n_det, _, _, _ = struct.unpack_from(\"<iiiiii\", binary, offset)\n                offset += 24\n       "
                 "         for _ in range(n_det):\n                    c0, c1, c2, pwr, vel = struct.unpack_from(\"<fffff\", binary, offset)\n    "
                 "                offset += 20\n                    if otype == 0: x, y = c0, c1\n                    else: x, y = c0*math.cos(c2)"
                 "*math.cos(c1), c0*math.cos(c2)*math.sin(c1)\n                    if pwr >= -95.0 and -11.0 <= vel <= -5.0:\n                    "
                 "    all_pts.append([x, y, vel, pwr])\n\n            if all_pts:\n                pts_np = np.array(all_pts)\n                clu"
                 "stering = DBSCAN(eps=2.5, min_samples=1).fit(pts_np[:, :2])\n                targets = []\n                for cid in set(cluste"
                 "ring.labels_):\n                    if cid == -1: continue\n                    idx = np.where(clustering.labels_ == cid)[0]\n  "
                 "                  targets.append([np.mean(pts_np[idx, 0]), np.mean(pts_np[idx, 1]), np.mean(pts_np[idx, 2])])\n                \n"
                 "                if targets:\n                    closest = min(targets, key=lambda t: (t[0]**2 + t[1]**2)**0.5)\n               "
                 "     last_data = [float(len(targets)), (closest[0]**2 + closest[1]**2)**0.5, closest[2]]\n        else:\n            buffer = bu"
                 "ffer[64:]\n\n    radar_out = last_data",
                 moduleInstance->namespaceDict);
  r = b_getPyDictVal(moduleInstance, moduleInstance->namespaceDict, "radar_out");
  PyObj_marshalIn(moduleInstance, r, NULL, dv);
  for (i = 0; i < 3; i++) {
    (*moduleInstance->b_y0)[i] = dv[i];
  }

  Py_DecRef(r);
  PyGILState_Release(moduleInstance->GIL);
}

static void cgxe_mdl_update(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_derivative(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_enable(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_disable(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_terminate(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  moduleInstance->GIL = PyGILState_Ensure();
  c_execPyScript(moduleInstance,
                 "global tcp_sock\nif tcp_sock is not None:\n    tcp_sock.close()\n    tcp_sock = None",
                 moduleInstance->namespaceDict);
  Py_DecRef(moduleInstance->namespaceDict);
  PyGILState_Release(moduleInstance->GIL);
}

static void CheckPythonError(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *pyObjsToRelease[], int32_T numObjToRelease)
{
  PyObject *pMsg;
  PyObject *pTraceback = NULL;
  PyObject *pType = NULL;
  PyObject *pValue = NULL;
  PyObject *sep = NULL;
  PyObject *tracebackList = NULL;
  PyObject *tracebackModule = NULL;
  int32_T i;
  int32_T idx;
  char_T *cMsg;
  void *slString;
  i = suStringStackSize();
  PyErr_Fetch(&pType, &pValue, &pTraceback);
  PyErr_NormalizeException(&pType, &pValue, &pTraceback);
  if (pType != NULL) {
    if (pTraceback != NULL) {
      tracebackModule = PyImport_ImportModule("traceback");
      tracebackList = PyObject_CallMethod(tracebackModule, "format_exception",
        "OOO", pType, pValue, pTraceback);
      sep = PyUnicode_FromString("");
      pMsg = PyUnicode_Join(sep, tracebackList);
    } else if (pValue != NULL) {
      pMsg = PyObject_Str(pValue);
    } else {
      pMsg = PyObject_Str(pType);
    }

    cMsg = (char_T *)PyUnicode_AsUTF8(pMsg);
    if (cMsg == NULL) {
      cMsg =
        "Simulink encountered an error when converting a python error message to UTF-8";
      PyErr_Clear();
    } else {
      slString = suAddStackString(cMsg);
      cMsg = suToCStr(slString);
    }

    if (sep != NULL) {
      Py_DecRef(sep);
    }

    if (tracebackList != NULL) {
      Py_DecRef(tracebackList);
    }

    if (tracebackModule != NULL) {
      Py_DecRef(tracebackModule);
    }

    if (pMsg != NULL) {
      Py_DecRef(pMsg);
    }

    pMsg = pType;
    if (pMsg != NULL) {
      Py_DecRef(pMsg);
    }

    pMsg = pValue;
    if (pMsg != NULL) {
      Py_DecRef(pMsg);
    }

    pMsg = pTraceback;
    if (pMsg != NULL) {
      Py_DecRef(pMsg);
    }

    for (idx = 0; idx < numObjToRelease; idx++) {
      pMsg = pyObjsToRelease[idx];
      if (pMsg != NULL) {
        Py_DecRef(pMsg);
      }
    }

    PyGILState_Release(moduleInstance->GIL);
    cgxertReportError(moduleInstance->S, -1, -1,
                      "Simulink:CustomCode:PythonRuntimeError", 3, 1, strlen
                      (cMsg), cMsg);
  }

  suMoveReturnedStringsToTopOfCallerStack(i, 0);
}

static void PyObj_marshalIn(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *pyToMarshal, PyObject *pyOwner, real_T outputVal[3])
{
  PyObject *b_pyObjArray[1];
  PyObject *pyObjArray[1];
  PyObject *objToRelease;
  PyObject *tmpItem;
  Py_buffer pyBuffer;
  Py_buffer view;
  real_T b_outputVal;
  Py_ssize_t len;
  int32_T i;
  char_T *r;
  uint8_T isRowMajor;
  void *buf;
  void *r1;
  i = suStringStackSize();
  if (pyOwner == NULL) {
    objToRelease = pyToMarshal;
  } else {
    objToRelease = pyOwner;
  }

  if (PyList_Check(pyToMarshal) || PyTuple_Check(pyToMarshal)) {
    for (len = 0; len < 3; len++) {
      tmpItem = PySequence_GetItem(pyToMarshal, len);
      if (tmpItem == NULL) {
        Py_DecRef(objToRelease);
        PyGILState_Release(moduleInstance->GIL);
        cgxertReportError(moduleInstance->S, -1, -1,
                          "Simulink:CustomCode:PythonIdxOutOfRange", 3, 1, 0, "");
      }

      Py_DecRef(tmpItem);
      b_outputVal = PyFloat_AsDouble(tmpItem);
      if (!(objToRelease == NULL)) {
        tmpItem = objToRelease;
      }

      b_pyObjArray[0U] = tmpItem;
      CheckPythonError(moduleInstance, b_pyObjArray, 1);
      outputVal[len] = b_outputVal;
    }
  } else {
    PyObject_GetBuffer(pyToMarshal, &view, 156U);
    pyObjArray[0U] = objToRelease;
    CheckPythonError(moduleInstance, pyObjArray, 1);
    pyBuffer = view;
    if (pyBuffer.ndim != 1) {
      PyGILState_Release(moduleInstance->GIL);
      PyBuffer_Release(&pyBuffer);
      cgxertReportError(moduleInstance->S, -1, -1,
                        "Simulink:CustomCode:PythonBufferWrongNDims", 3, 1, 0,
                        "");
    }

    r = pyBuffer.format;
    buf = suAddStackString((char_T *)"d");
    r1 = suAddStackString(r);
    len = suStrcmp(buf, r1);
    if (len != 0) {
      PyGILState_Release(moduleInstance->GIL);
      PyBuffer_Release(&pyBuffer);
      cgxertReportError(moduleInstance->S, -1, -1,
                        "Simulink:CustomCode:PythonBufferWrongType", 3, 1, 0, "");
    }

    if (pyBuffer.shape[0] != 3) {
      PyGILState_Release(moduleInstance->GIL);
      PyBuffer_Release(&pyBuffer);
      cgxertReportError(moduleInstance->S, -1, -1,
                        "Simulink:CustomCode:PythonBufferWrongShape", 3, 1, 0,
                        "");
    }

    len = view.len;
    buf = view.buf;
    isRowMajor = PyBuffer_IsContiguous(&view, 'C');
    if (isRowMajor == 1) {
      memcpy(&outputVal[0], buf, (uint32_T)len);
    } else {
      memcpy(&outputVal[0], buf, (uint32_T)len);
    }

    PyBuffer_Release(&view);
  }

  suMoveReturnedStringsToTopOfCallerStack(i, 0);
}

static PyObject *getPyNamespaceDict(void)
{
  return PyDict_Copy(PyModule_GetDict(PyImport_AddModule("__main__")));
}

static void assignToPyDict(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  PyObject *dict, char_T *key, real_T val[3])
{
  PyObject *pyObj;
  if (dict != NULL) {
    pyObj = marshalOut(moduleInstance, val);
    PyDict_SetItemString(dict, key, pyObj);
    Py_DecRef(pyObj);
  }
}

static PyObject *marshalOut(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, real_T data[3])
{
  PyObject *b_pyObjArray[2];
  PyObject *pyObjArray[1];
  PyObject *asarrayFcn;
  PyObject *memoryView;
  PyObject *numpyModule;
  PyObject *pyInput;
  Py_buffer view;
  Py_ssize_t shape[1] = { 3 };

  Py_ssize_t stride[1];
  Py_ssize_t itemSize;
  itemSize = PyBuffer_SizeFromFormat("d");
  stride[0U] = itemSize;
  view.buf = (void *)data;
  view.obj = 0;
  view.len = _s32_s64_(moduleInstance, 3LL * (int64_T)itemSize);
  view.itemsize = itemSize;
  view.readonly = 0;
  view.ndim = 1;
  view.format = "d";
  view.shape = &shape[0U];
  view.strides = &stride[0U];
  view.suboffsets = 0;
  view.internal = 0;
  memoryView = PyMemoryView_FromBuffer(&view);
  numpyModule = PyImport_ImportModule("numpy");
  if (numpyModule == NULL) {
    PyErr_Clear();
    pyInput = memoryView;
  } else {
    asarrayFcn = PyObject_GetAttrString(numpyModule, "asarray");
    pyObjArray[0U] = numpyModule;
    CheckPythonError(moduleInstance, pyObjArray, 1);
    pyInput = PyObject_CallFunctionObjArgs(asarrayFcn, memoryView, NULL);
    b_pyObjArray[0U] = numpyModule;
    b_pyObjArray[1U] = asarrayFcn;
    CheckPythonError(moduleInstance, b_pyObjArray, 2);
    Py_DecRef(memoryView);
    Py_DecRef(asarrayFcn);
    Py_DecRef(numpyModule);
  }

  return pyInput;
}

static void execPyScript(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  char_T *script, PyObject *ns)
{
  PyObject *pyObjArray[2];
  PyObject *codeObject;
  PyObject *originalNamespace;
  PyObject *unusedEvalResult;
  Py_ssize_t i;
  Py_ssize_t numKeysInModifiedNs;
  if (ns != NULL) {
    codeObject = Py_CompileString(script, "Python Code Block", 257);
    CheckPythonError(moduleInstance, NULL, 0);
    originalNamespace = PyDict_Copy(ns);
    unusedEvalResult = PyEval_EvalCode(codeObject, ns, ns);
    pyObjArray[0U] = codeObject;
    pyObjArray[1U] = unusedEvalResult;
    CheckPythonError(moduleInstance, pyObjArray, 2);
    Py_DecRef(codeObject);
    if (unusedEvalResult != NULL) {
      Py_DecRef(unusedEvalResult);
    }

    codeObject = PyDict_Keys(ns);
    numKeysInModifiedNs = PyList_Size(codeObject);
    for (i = 0; i < numKeysInModifiedNs; i++) {
      unusedEvalResult = PySequence_GetItem(codeObject, i);
      CheckPythonError(moduleInstance, NULL, 0);
      if ((PyDict_Contains(originalNamespace, unusedEvalResult) == 0) &&
          (!PyModule_Check(PyDict_GetItem(ns, unusedEvalResult)))) {
        PyDict_DelItem(ns, unusedEvalResult);
      }

      Py_DecRef(unusedEvalResult);
    }

    Py_DecRef(codeObject);
    Py_DecRef(originalNamespace);
  }
}

static PyObject *getPyDictVal(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *dict, char_T *key)
{
  PyObject *b_value;
  b_value = PyDict_GetItemString(dict, key);
  CheckPythonError(moduleInstance, NULL, 0);
  Py_IncRef(b_value);
  return b_value;
}

static void b_assignToPyDict(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *dict, char_T *key, real_T val[3])
{
  PyObject *pyObj;
  if (dict != NULL) {
    pyObj = marshalOut(moduleInstance, val);
    PyDict_SetItemString(dict, key, pyObj);
    Py_DecRef(pyObj);
  }
}

static void b_execPyScript(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  char_T *script, PyObject *ns)
{
  PyObject *pyObjArray[2];
  PyObject *codeObject;
  PyObject *originalNamespace;
  PyObject *unusedEvalResult;
  Py_ssize_t i;
  Py_ssize_t numKeysInModifiedNs;
  if (ns != NULL) {
    codeObject = Py_CompileString(script, "Python Code Block", 257);
    CheckPythonError(moduleInstance, NULL, 0);
    originalNamespace = PyDict_Copy(ns);
    unusedEvalResult = PyEval_EvalCode(codeObject, ns, ns);
    pyObjArray[0U] = codeObject;
    pyObjArray[1U] = unusedEvalResult;
    CheckPythonError(moduleInstance, pyObjArray, 2);
    Py_DecRef(codeObject);
    if (unusedEvalResult != NULL) {
      Py_DecRef(unusedEvalResult);
    }

    codeObject = PyDict_Keys(ns);
    numKeysInModifiedNs = PyList_Size(codeObject);
    for (i = 0; i < numKeysInModifiedNs; i++) {
      unusedEvalResult = PySequence_GetItem(codeObject, i);
      CheckPythonError(moduleInstance, NULL, 0);
      if ((PyDict_Contains(originalNamespace, unusedEvalResult) == 0) &&
          (!PyModule_Check(PyDict_GetItem(ns, unusedEvalResult)))) {
        PyDict_DelItem(ns, unusedEvalResult);
      }

      Py_DecRef(unusedEvalResult);
    }

    Py_DecRef(codeObject);
    Py_DecRef(originalNamespace);
  }
}

static PyObject *b_getPyDictVal(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance, PyObject *dict, char_T *key)
{
  PyObject *b_value;
  b_value = PyDict_GetItemString(dict, key);
  CheckPythonError(moduleInstance, NULL, 0);
  Py_IncRef(b_value);
  return b_value;
}

static void c_execPyScript(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  char_T *script, PyObject *ns)
{
  PyObject *pyObjArray[2];
  PyObject *codeObject;
  PyObject *originalNamespace;
  PyObject *unusedEvalResult;
  if (ns != NULL) {
    codeObject = Py_CompileString(script, "Python Code Block", 257);
    CheckPythonError(moduleInstance, NULL, 0);
    originalNamespace = PyDict_Copy(ns);
    unusedEvalResult = PyEval_EvalCode(codeObject, ns, ns);
    pyObjArray[0U] = codeObject;
    pyObjArray[1U] = unusedEvalResult;
    CheckPythonError(moduleInstance, pyObjArray, 2);
    Py_DecRef(codeObject);
    if (unusedEvalResult != NULL) {
      Py_DecRef(unusedEvalResult);
    }

    Py_DecRef(originalNamespace);
  }
}

static int32_T _s32_s64_(InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance,
  int64_T b)
{
  int32_T a;
  a = (int32_T)b;
  if ((int64_T)a != b) {
    emlrtIntegerOverflowWarningOrError2018b(NULL,
      moduleInstance->emlrtRootTLSGlobal);
  }

  return a;
}

static void init_simulink_io_address(InstanceStruct_ouJmnzmVetsRECPPoHBYQB
  *moduleInstance)
{
  moduleInstance->emlrtRootTLSGlobal = (void *)cgxertGetEMLRTCtx
    (moduleInstance->S);
  moduleInstance->b_y0 = (real_T (*)[3])cgxertGetOutputPortSignal
    (moduleInstance->S, 0);
}

/* CGXE Glue Code */
static void mdlOutputs_ouJmnzmVetsRECPPoHBYQB(SimStruct *S, int_T tid)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_outputs(moduleInstance);
}

static void mdlInitialize_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_initialize(moduleInstance);
}

static void mdlUpdate_ouJmnzmVetsRECPPoHBYQB(SimStruct *S, int_T tid)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_update(moduleInstance);
}

static void mdlDerivatives_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_derivative(moduleInstance);
}

static void mdlTerminate_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_terminate(moduleInstance);
  free((void *)moduleInstance);
}

static void mdlEnable_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_enable(moduleInstance);
}

static void mdlDisable_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_disable(moduleInstance);
}

static void mdlStart_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
  InstanceStruct_ouJmnzmVetsRECPPoHBYQB *moduleInstance =
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB *)calloc(1, sizeof
    (InstanceStruct_ouJmnzmVetsRECPPoHBYQB));
  moduleInstance->S = S;
  cgxertSetRuntimeInstance(S, (void *)moduleInstance);
  ssSetmdlOutputs(S, mdlOutputs_ouJmnzmVetsRECPPoHBYQB);
  ssSetmdlInitializeConditions(S, mdlInitialize_ouJmnzmVetsRECPPoHBYQB);
  ssSetmdlUpdate(S, mdlUpdate_ouJmnzmVetsRECPPoHBYQB);
  ssSetmdlDerivatives(S, mdlDerivatives_ouJmnzmVetsRECPPoHBYQB);
  ssSetmdlTerminate(S, mdlTerminate_ouJmnzmVetsRECPPoHBYQB);
  ssSetmdlEnable(S, mdlEnable_ouJmnzmVetsRECPPoHBYQB);
  ssSetmdlDisable(S, mdlDisable_ouJmnzmVetsRECPPoHBYQB);
  cgxe_mdl_start(moduleInstance);

  {
    uint_T options = ssGetOptions(S);
    options |= SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE;
    ssSetOptions(S, options);
  }
}

static void mdlProcessParameters_ouJmnzmVetsRECPPoHBYQB(SimStruct *S)
{
}

void method_dispatcher_ouJmnzmVetsRECPPoHBYQB(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_ouJmnzmVetsRECPPoHBYQB(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_ouJmnzmVetsRECPPoHBYQB(S);
    break;

   default:
    /* Unhandled method */
    /*
       sf_mex_error_message("Stateflow Internal Error:\n"
       "Error calling method dispatcher for module: ouJmnzmVetsRECPPoHBYQB.\n"
       "Can't handle method %d.\n", method);
     */
    break;
  }
}

mxArray *cgxe_ouJmnzmVetsRECPPoHBYQB_BuildInfoUpdate(void)
{
  mxArray * mxBIArgs;
  mxArray * elem_1;
  mxArray * elem_2;
  mxArray * elem_3;
  double * pointer;
  mxBIArgs = mxCreateCellMatrix(1,3);
  elem_1 = mxCreateDoubleMatrix(0,0, mxREAL);
  pointer = mxGetPr(elem_1);
  mxSetCell(mxBIArgs,0,elem_1);
  elem_2 = mxCreateDoubleMatrix(0,0, mxREAL);
  pointer = mxGetPr(elem_2);
  mxSetCell(mxBIArgs,1,elem_2);
  elem_3 = mxCreateCellMatrix(1,0);
  mxSetCell(mxBIArgs,2,elem_3);
  return mxBIArgs;
}

mxArray *cgxe_ouJmnzmVetsRECPPoHBYQB_fallback_info(void)
{
  const char* fallbackInfoFields[] = { "fallbackType", "incompatiableSymbol" };

  mxArray* fallbackInfoStruct = mxCreateStructMatrix(1, 1, 2, fallbackInfoFields);
  mxArray* fallbackType = mxCreateString("incompatibleFunction");
  mxArray* incompatibleSymbol = mxCreateString("PyList_Check");
  mxSetFieldByNumber(fallbackInfoStruct, 0, 0, fallbackType);
  mxSetFieldByNumber(fallbackInfoStruct, 0, 1, incompatibleSymbol);
  return fallbackInfoStruct;
}
