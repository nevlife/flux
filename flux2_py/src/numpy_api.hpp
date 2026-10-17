#ifndef FLUX2_PY_NUMPY_API_HPP
#define FLUX2_PY_NUMPY_API_HPP

// numpy arrays and scalars made through numpy's C API. nb::ndarray hands an array to numpy by
// calling numpy.array() from C++, which costs ten times the array itself. The translation unit
// that defines FLUX2_PY_IMPORT_NUMPY holds the API table; the others reach it through the symbol.

#include <nanobind/nanobind.h>

#define PY_ARRAY_UNIQUE_SYMBOL flux2_py_ARRAY_API
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#ifndef FLUX2_PY_IMPORT_NUMPY
#define NO_IMPORT_ARRAY
#endif
#include "flux2/channel.hpp"

#include <numpy/arrayobject.h>

#include <cstddef>

namespace flux2_py
{

namespace nb = nanobind;

// numpy's C API passes its objects as PyObject *; the casts between the views live here.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
inline PyArrayObject * as_array(nb::handle a)
{
  return reinterpret_cast<PyArrayObject *>(a.ptr());
}
inline PyArray_Descr * as_descr(nb::handle d)
{
  return reinterpret_cast<PyArray_Descr *>(d.ptr());
}
inline nb::object own(PyArray_Descr * d)
{
  return nb::steal(reinterpret_cast<PyObject *>(d));
}
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

// numpy 2 hides the field behind an accessor that numpy 1 (Jazzy) does not have.
inline std::size_t elsize(PyArray_Descr * d)
{
#if NPY_ABI_VERSION >= 0x02000000
  return static_cast<std::size_t>(PyDataType_ELSIZE(d));
#else
  return static_cast<std::size_t>(d->elsize);
#endif
}

// numpy has no bfloat16: such a frame reads as its uint16 bits.
inline int npy_type(flux2::DType d)
{
  switch (d) {
    case flux2::DType::U8:
      return NPY_UINT8;
    case flux2::DType::I8:
      return NPY_INT8;
    case flux2::DType::U16:
    case flux2::DType::BF16:
      return NPY_UINT16;
    case flux2::DType::I16:
      return NPY_INT16;
    case flux2::DType::U32:
      return NPY_UINT32;
    case flux2::DType::I32:
      return NPY_INT32;
    case flux2::DType::U64:
      return NPY_UINT64;
    case flux2::DType::I64:
      return NPY_INT64;
    case flux2::DType::F16:
      return NPY_HALF;
    case flux2::DType::F32:
      return NPY_FLOAT32;
    case flux2::DType::F64:
      return NPY_FLOAT64;
  }
  throw std::runtime_error("flux2: unknown dtype in frame");
}

// A view of `data` that `owner` keeps alive. `descr` is borrowed. Strides are in bytes; null is
// C-contiguous.
inline nb::object numpy_view(
  void * data, int ndim, npy_intp * shape, PyArray_Descr * descr, nb::handle owner, bool writable,
  npy_intp * strides = nullptr)
{
  Py_INCREF(descr);  // NewFromDescr steals it
  const int flags =
    (strides == nullptr ? NPY_ARRAY_C_CONTIGUOUS : 0) | (writable ? NPY_ARRAY_WRITEABLE : 0);
  PyObject * a =
    PyArray_NewFromDescr(&PyArray_Type, descr, ndim, shape, strides, data, flags, nullptr);
  if (a == nullptr) throw nb::python_error();
  Py_INCREF(owner.ptr());
  if (PyArray_SetBaseObject(as_array(a), owner.ptr()) != 0) {
    Py_DECREF(a);
    throw nb::python_error();
  }
  return nb::steal(a);
}

inline nb::object numpy_view(
  const void * data, std::size_t n, PyArray_Descr * descr, nb::handle owner, bool writable)
{
  npy_intp shape = static_cast<npy_intp>(n);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): read-only unless `writable`
  return numpy_view(const_cast<void *>(data), 1, &shape, descr, owner, writable);
}

inline nb::object numpy_scalar(const void * p, PyArray_Descr * descr)
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): PyArray_Scalar copies out of p
  PyObject * s = PyArray_Scalar(const_cast<void *>(p), descr, nullptr);
  if (s == nullptr) throw nb::python_error();
  return nb::steal(s);
}

}  // namespace flux2_py

#endif  // FLUX2_PY_NUMPY_API_HPP
