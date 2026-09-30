#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER flux

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "flux_tp.h"

#if !defined(FLUX_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define FLUX_TP_H

#include <lttng/tracepoint.h>
#include <stdint.h>

LTTNG_UST_TRACEPOINT_EVENT(
  flux, channel_init,
  LTTNG_UST_TP_ARGS(
    const void *, channel_arg, uint64_t, dev_arg, uint64_t, ino_arg, const char *, segment_arg, int,
    publisher_arg, uint32_t, slot_count_arg, uint32_t, slot_size_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)
                        lttng_ust_field_integer(uint64_t, dev, dev_arg)
                          lttng_ust_field_integer(uint64_t, ino, ino_arg)
                            lttng_ust_field_string(segment, segment_arg)
                              lttng_ust_field_integer(int, publisher, publisher_arg)
                                lttng_ust_field_integer(uint32_t, slot_count, slot_count_arg)
                                  lttng_ust_field_integer(uint32_t, slot_size, slot_size_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, claim,
  LTTNG_UST_TP_ARGS(
    const void *, channel_arg, uint32_t, slot_arg, int, result_arg, int, reclaimed_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)
                        lttng_ust_field_integer(uint32_t, slot, slot_arg)
                          lttng_ust_field_integer(int, result, result_arg)
                            lttng_ust_field_integer(int, reclaimed, reclaimed_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, commit,
  LTTNG_UST_TP_ARGS(
    const void *, channel_arg, uint64_t, ticket_arg, uint32_t, slot_arg, uint64_t, nbytes_arg, int,
    woke_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)
                        lttng_ust_field_integer(uint64_t, ticket, ticket_arg)
                          lttng_ust_field_integer(uint32_t, slot, slot_arg)
                            lttng_ust_field_integer(uint64_t, nbytes, nbytes_arg)
                              lttng_ust_field_integer(int, woke, woke_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, take,
  LTTNG_UST_TP_ARGS(
    const void *, channel_arg, uint64_t, ticket_arg, uint32_t, slot_arg, uint64_t, lost_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)
                        lttng_ust_field_integer(uint64_t, ticket, ticket_arg)
                          lttng_ust_field_integer(uint32_t, slot, slot_arg)
                            lttng_ust_field_integer(uint64_t, lost, lost_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, release,
  LTTNG_UST_TP_ARGS(const void *, channel_arg, uint64_t, ticket_arg, int, fenced_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)
                        lttng_ust_field_integer(uint64_t, ticket, ticket_arg)
                          lttng_ust_field_integer(int, fenced, fenced_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, refused, LTTNG_UST_TP_ARGS(const void *, channel_arg, int, reason_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)
                        lttng_ust_field_integer(int, reason, reason_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, wake, LTTNG_UST_TP_ARGS(const void *, executor_arg, uint32_t, events_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, executor, (uintptr_t)executor_arg)
                        lttng_ust_field_integer(uint32_t, events, events_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, callback_start, LTTNG_UST_TP_ARGS(const void *, executor_arg, uint32_t, entry_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, executor, (uintptr_t)executor_arg)
                        lttng_ust_field_integer(uint32_t, entry, entry_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, callback_end,
  LTTNG_UST_TP_ARGS(const void *, executor_arg, uint32_t, entry_arg, int, delivered_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, executor, (uintptr_t)executor_arg)
                        lttng_ust_field_integer(uint32_t, entry, entry_arg)
                          lttng_ust_field_integer(int, delivered, delivered_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, py_gil, LTTNG_UST_TP_ARGS(const void *, executor_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, executor, (uintptr_t)executor_arg)))

LTTNG_UST_TRACEPOINT_EVENT(
  flux, py_ready, LTTNG_UST_TP_ARGS(const void *, channel_arg),
  LTTNG_UST_TP_FIELDS(lttng_ust_field_integer_hex(uintptr_t, channel, (uintptr_t)channel_arg)))

#endif  // FLUX_TP_H

#include <lttng/tracepoint-event.h>
