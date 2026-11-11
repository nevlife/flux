#ifndef FLUX_PY_BIND_WIRE_HPP
#define FLUX_PY_BIND_WIRE_HPP

#include "flux/wire.hpp"

#include <nanobind/nanobind.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// flux.wire's Reader, Writer and WireError, for generated Python adapters.
void bind_wire(nanobind::module_ & m);

namespace flux_py
{

using TensorAt = std::array<std::size_t, 7>;

// A tensor message's fields, copied out of the frame before tensor_fault() checks them: the
// writer may still change the frame.
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
