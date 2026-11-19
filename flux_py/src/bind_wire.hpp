#ifndef FLUX_PY_BIND_WIRE_HPP
#define FLUX_PY_BIND_WIRE_HPP

#include "flux/wire.hpp"

#include <nanobind/nanobind.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

// flux.wire's Reader, Writer and WireError (a subclass of `error`, flux.Error), for generated
// Python adapters.
void bind_wire(nanobind::module_ & m, nanobind::handle error);

namespace flux
{
class Frame;
}

namespace flux_py
{

// Raised as flux.UsageError: a call the state does not allow (a released frame, an ended loan).
struct UsageError : std::logic_error
{
  using std::logic_error::logic_error;
};

// Raised as flux.InvalidArgument.
struct ArgumentError : std::invalid_argument
{
  using std::invalid_argument::invalid_argument;
};

// `name` in `m`, a subclass of flux.Error and the Python exception `also`.
inline auto error_subclass(
  nanobind::module_ & m, const char * name, nanobind::handle error,
  PyObject * also) -> nanobind::object
{
  const std::string full = nanobind::cast<std::string>(m.attr("__name__")) + "." + name;
  const nanobind::object bases = nanobind::steal(PyTuple_Pack(2, error.ptr(), also));
  nanobind::object cls = nanobind::steal(PyErr_NewException(full.c_str(), bases.ptr(), nullptr));
  m.attr(name) = cls;
  return cls;
}

// n `dtype` elements at device address p, as a flux.wire.DeviceSpan holding `owner`.
auto device_span(
  nanobind::handle owner, const void * p, std::size_t n, nanobind::handle dtype, int device,
  bool readonly) -> nanobind::object;

// The view a take result's array holds and its stream's device (-1 for none); null for others.
auto frame_of(nanobind::handle buf, int * device) -> const flux::Frame *;

using TensorAt = std::array<std::size_t, 7>;

// A tensor message's fields, copied out first: the writer may still change the frame.
struct TensorFields
{
  const std::uint8_t * data;
  flux::wire::DlType dtype;
  std::vector<std::int64_t> shape;
  std::vector<std::int64_t> strides;
};

// WireError when they do not fit the frame.
auto read_tensor(const flux::wire::Reader & w, const TensorAt & at) -> TensorFields;

// A DLPack capsule of `t` with its first element at `addr`, holding `owner`.
auto tensor_dlpack(
  const TensorFields & t, std::uintptr_t addr, nanobind::handle owner, int device_type,
  int device_id) -> nanobind::object;

}  // namespace flux_py

#endif  // FLUX_PY_BIND_WIRE_HPP
