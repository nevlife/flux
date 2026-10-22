/* Handles from _new/_create/_open go to their _free (NULL ok); other pointers must be valid. View
 * and loan storage is caller memory, zeroed before first use; an empty one's accessors give NULL/0.
 * A view keeps its mapping past the channel handle; a commit empties its loan. timeout_ns < 0 waits
 * forever. flux_last_error() is per thread. Callbacks must not unwind. One call at a time per
 * channel: others get *_IN_USE or false; getters are unchecked, must not overlap; commits may. */

#ifndef FLUX_H
#define FLUX_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define FLUX_ABI_VERSION 4

#define FLUX_MAX_DIMS 8

#define FLUX_NO_SCHEMA 0

#define FLUX_MAX_LABEL_LEN 240

#define FLUX_MAX_DRAIN 64

#define FLUX_DEFAULT_TICK_NS 100000000

#define FLUX_DEFAULT_MAX_SUBSCRIBERS 16

#define FLUX_MAX_SUBSCRIBERS 4096

#define FLUX_NO_STAMP -1

#define FLUX_HEAP_BASE 2147483648

#define FLUX_DTYPE_U8 0

#define FLUX_DTYPE_I8 1

#define FLUX_DTYPE_U16 2

#define FLUX_DTYPE_I16 3

#define FLUX_DTYPE_U32 4

#define FLUX_DTYPE_I32 5

#define FLUX_DTYPE_U64 6

#define FLUX_DTYPE_I64 7

#define FLUX_DTYPE_F16 8

#define FLUX_DTYPE_F32 9

#define FLUX_DTYPE_F64 10

#define FLUX_DTYPE_BF16 11

#define FLUX_RELEASE_WAIT 0

#define FLUX_RELEASE_DEFERRED 1

enum flux_status
#if defined(__cplusplus) || __STDC_VERSION__ >= 202311L
  : int32_t
#endif // defined(__cplusplus) || __STDC_VERSION__ >= 202311L
 {
  FLUX_OK = 0,
  FLUX_INVALID_ARGUMENT = 1,
  FLUX_MISMATCH = 2,
  // Not there yet (no publisher). Retrying may help.
  FLUX_NOT_READY = 3,
  FLUX_PUBLISHER_EXISTS = 4,
  FLUX_SUBSCRIBERS_FULL = 5,
  FLUX_UNSUPPORTED = 6,
  // A callback returned failure; the caller kept its own error.
  FLUX_CALLBACK = 7,
  FLUX_IO = 8,
  FLUX_IN_USE = 9,
};
#ifndef __cplusplus
#if __STDC_VERSION__ >= 202311L
typedef enum flux_status flux_status;
#else
typedef int32_t flux_status;
#endif // __STDC_VERSION__ >= 202311L
#endif // __cplusplus

enum flux_published
#if defined(__cplusplus) || __STDC_VERSION__ >= 202311L
  : int32_t
#endif // defined(__cplusplus) || __STDC_VERSION__ >= 202311L
 {
  FLUX_PUBLISHED_OK = 0,
  FLUX_PUBLISHED_BACKPRESSURE = 1,
  FLUX_PUBLISHED_TOO_LARGE = 2,
  FLUX_PUBLISHED_NOT_PUBLISHER = 3,
  // An unknown dtype, or a commit of an empty loan (already committed or aborted).
  FLUX_PUBLISHED_INVALID_ARGUMENT = 4,
  FLUX_PUBLISHED_IN_USE = 5,
  // The payload is device memory; loan and write through `flux_loan_device_ptr`.
  FLUX_PUBLISHED_WRONG_DEVICE = 6,
  // The stream could not be waited for. The slot stays claimed and is never reused.
  FLUX_PUBLISHED_FENCE_FAILED = 7,
  FLUX_PUBLISHED_GPU_WEDGED = 8,
};
#ifndef __cplusplus
#if __STDC_VERSION__ >= 202311L
typedef enum flux_published flux_published;
#else
typedef int32_t flux_published;
#endif // __STDC_VERSION__ >= 202311L
#endif // __cplusplus

enum flux_route
#if defined(__cplusplus) || __STDC_VERSION__ >= 202311L
  : int32_t
#endif // defined(__cplusplus) || __STDC_VERSION__ >= 202311L
 {
  FLUX_ROUTE_NONE = 0,
  FLUX_ROUTE_SHM_DIRECT = 1,
  FLUX_ROUTE_SHM_REGISTERED = 2,
  FLUX_ROUTE_DEVICE_HANDLE = 3,
};
#ifndef __cplusplus
#if __STDC_VERSION__ >= 202311L
typedef enum flux_route flux_route;
#else
typedef int32_t flux_route;
#endif // __STDC_VERSION__ >= 202311L
#endif // __cplusplus

typedef struct flux_announcement flux_announcement;

typedef struct flux_channel flux_channel;

typedef struct flux_executor flux_executor;

// Stop, interrupt and wake from any thread. Outlives its executor: the calls then do nothing.
typedef struct flux_handle flux_handle;

typedef struct flux_partitioned flux_partitioned;

typedef struct flux_schema flux_schema;

typedef struct flux_schemas flux_schemas;

// A CUDA stream declared on channels; NULL where a function takes one means a host channel.
typedef struct flux_stream flux_stream;

// Every pointer inside stays valid until `flux_topic_list_free`.
typedef struct flux_topic_list flux_topic_list;

typedef struct flux_watch flux_watch;

typedef struct flux_memory_policy {
  bool precommit;
  // Implies precommit.
  bool lock;
} flux_memory_policy;

// `replay` 0 is volatile, N is `transient_local(N)`.
typedef struct flux_qos {
  uint32_t depth;
  uint32_t replay;
  uint32_t max_borrow;
} flux_qos;

// A claimed slot in caller memory, never copied. Commit it while no call is on its channel.
typedef struct flux_loan {
  uint64_t opaque[16];
} flux_loan;

// Experimental arena handle of a live loan; `base` is null without one (`FLUX_ARENA_BYTES`).
typedef struct flux_heap {
  uint8_t *base;
  size_t size;
  const void *channel;
  uint32_t slot;
} flux_heap;

// A borrowed frame in caller memory, never copied. A move copies the bytes and zeroes the source.
typedef struct flux_view {
  uint64_t opaque[16];
} flux_view;

// Same bytes as the frame header in shared memory; `dtype` is a `FLUX_DTYPE_*`.
typedef struct flux_frame_meta {
  uint8_t ndim;
  uint8_t dtype;
  uint8_t reserved0[2];
  uint32_t itemsize;
  uint64_t nbytes;
  uint64_t shape[FLUX_MAX_DIMS];
} flux_frame_meta;

typedef struct flux_refused {
  uint64_t max_borrow;
  uint64_t no_reader;
  uint64_t contended;
  uint64_t bad_frame;
  uint64_t fence;
} flux_refused;

typedef struct flux_fence_wait {
  uint64_t commit_ns;
  uint64_t commit_waits;
  uint64_t commit_max_ns;
  uint64_t release_ns;
  uint64_t release_waits;
  uint64_t release_max_ns;
} flux_fence_wait;

// `attach`, `deliver_one`, `channel` required; `drop` runs once, on release or on a failed add.
typedef struct flux_source {
  void *ctx;
  // 1 attached, 0 nothing to attach to yet, negative failed.
  int32_t (*attach)(void *ctx);
  // 1 delivered a frame, 0 found none, negative failed.
  int32_t (*deliver_one)(void *ctx);
  // NULL until attached and after an orphaned channel is dropped. Valid until the next call.
  const struct flux_channel *(*channel)(void *ctx);
  void (*drop)(void *ctx);
  // Optional: while `channel` is NULL, the watch the last attach armed; else retried per tick.
  const struct flux_watch *(*watch)(void *ctx);
  // The handle `flux_trace_endpoint_init` named this source's endpoint by, or NULL.
  const void *endpoint;
} flux_source;

// 0 is success; anything else ends the spin with `FLUX_CALLBACK`.
typedef int32_t (*flux_hook)(void *ctx);

typedef struct flux_gpu_platform {
  flux_route route;
  int32_t device;
  bool integrated;
  bool pageable_access;
  bool host_register;
  bool vmm;
  bool posix_fd_handle;
} flux_gpu_platform;

typedef struct flux_manifest_entry {
  const char *signpost;
  // The exact key; NULL or "" when the caller has none.
  const char *key;
  const char *label;
  bool publisher;
} flux_manifest_entry;

typedef struct flux_channel_stats {
  bool live;
  uint32_t slot_size;
  uint32_t slot_count;
  uint32_t max_subscribers;
  uint32_t storage_kind;
  uint32_t epoch;
  uint64_t fingerprint;
  uint64_t publish_seq;
  int32_t waiters;
} flux_channel_stats;

typedef struct flux_endpoint_info {
  uint32_t pid;
  bool publisher;
  const char *label;
} flux_endpoint_info;

typedef struct flux_topic_info {
  const char *signpost;
  const char *key;
  uint64_t fingerprint;
  bool key_exact;
  const struct flux_endpoint_info *endpoints;
  size_t endpoint_count;
} flux_topic_info;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

uint32_t flux_abi_version(void);

flux_status flux_channel_new(uint32_t slot_size, uint32_t slot_count, struct flux_channel **out);

// The publisher of signpost `name`, for up to `max_subscribers` subscribing processes.
flux_status flux_channel_create(const char *name,
                                uint32_t slot_size,
                                uint32_t slot_count,
                                uint32_t max_subscribers,
                                uint64_t fingerprint,
                                struct flux_memory_policy mem,
                                const struct flux_stream *stream,
                                int32_t stamp_offset,
                                struct flux_channel **out);

flux_status flux_watch_open(const char *name, struct flux_watch **out);

void flux_watch_free(struct flux_watch *w);

uint32_t flux_watch_epoch(const struct flux_watch *w);

uint32_t flux_watch_arm(const struct flux_watch *w);

bool flux_watch_wait(const struct flux_watch *w, int64_t timeout_ns);

// After a first success through `w`, starts at its publisher's oldest frame. `w` is not kept.
flux_status flux_channel_open_watched(const struct flux_watch *w,
                                      uint64_t fingerprint,
                                      struct flux_qos q,
                                      struct flux_memory_policy mem,
                                      const struct flux_stream *stream,
                                      int32_t stamp_offset,
                                      struct flux_channel **out);

struct flux_qos flux_qos_default(void);

// A subscriber of signpost `name`. `FLUX_NOT_READY` until a publisher is up.
flux_status flux_channel_open(const char *name,
                              uint64_t fingerprint,
                              struct flux_memory_policy mem,
                              const struct flux_stream *stream,
                              int32_t stamp_offset,
                              struct flux_channel **out);

void flux_channel_free(struct flux_channel *ch);

flux_published flux_channel_publish(struct flux_channel *ch, const uint8_t *data, size_t len);

flux_published flux_channel_publish_typed(struct flux_channel *ch,
                                          const uint8_t *data,
                                          size_t len,
                                          uint8_t dtype,
                                          const uint64_t *shape_,
                                          size_t ndim);

// The whole slot as one u8 run. False when no slot is free, this is a subscriber, or in use.
bool flux_channel_loan(struct flux_channel *ch, struct flux_loan *out);

// The shape is checked at commit. False also for an unknown dtype.
bool flux_channel_loan_typed(struct flux_channel *ch,
                             uint8_t dtype,
                             const uint64_t *shape_,
                             size_t ndim,
                             struct flux_loan *out);

uint8_t *flux_loan_data(struct flux_loan *l, size_t *capacity);

void *flux_loan_stream(const struct flux_loan *l);

// 0 when the channel declared no stream, as for `flux_view_device_ptr`.
uint64_t flux_loan_device_ptr(const struct flux_loan *l);

flux_published flux_loan_commit(struct flux_loan *l);

struct flux_heap flux_loan_heap(const struct flux_loan *l);

// Arena memory the frame owns until its slot is claimed again; `off` gets its arena offset.
uint8_t *flux_heap_alloc(const struct flux_heap *h, size_t bytes, size_t align, uint64_t *off);

// Gives the frame arena memory the hook placed; `release(ctx)` runs when the slot is reclaimed.
bool flux_heap_adopt(const struct flux_heap *h,
                     const uint8_t *p,
                     size_t bytes,
                     void (*release)(void*),
                     void *ctx,
                     uint64_t *off);

const uint8_t *flux_arena_resolve(const uint8_t *frame, size_t len, size_t *size);

// Routes this thread's allocations to the arena under `libflux_heaphook.so`; false without it.
bool flux_arena_scope_enter(void);

void flux_arena_scope_leave(void);

// Shrinks a one-dimensional loan to `nbytes`.
flux_published flux_loan_commit_len(struct flux_loan *l, size_t nbytes);

// Ends the loan (its slot's old frame is gone either way) or the mapping a commit kept.
void flux_loan_abort(struct flux_loan *l);

bool flux_loan_valid(const struct flux_loan *l);

// The oldest frame newer than the cursor. A view already in `out` is released first.
bool flux_channel_take(struct flux_channel *ch, struct flux_view *out);

bool flux_channel_take_blocking(struct flux_channel *ch, int64_t timeout_ns, struct flux_view *out);

const uint8_t *flux_view_data(const struct flux_view *v, size_t *len);

void *flux_view_stream(const struct flux_view *v);

// The frame's device address, 0 when the channel declared no stream.
uint64_t flux_view_device_ptr(const struct flux_view *v);

// Points into `v`, so moving the view moves it. Size a typed read from `dtype`, not `itemsize`.
const struct flux_frame_meta *flux_view_meta(const struct flux_view *v);

uint64_t flux_view_ticket(const struct flux_view *v);

const void *flux_view_trace_channel(const struct flux_view *v);

// `CLOCK_MONOTONIC` at commit.
uint64_t flux_view_publish_ns(const struct flux_view *v);

void flux_view_release(struct flux_view *v);

bool flux_view_valid(const struct flux_view *v);

flux_status flux_channel_set_qos(struct flux_channel *ch, struct flux_qos q);

struct flux_qos flux_channel_qos(const struct flux_channel *ch);

struct flux_refused flux_channel_refused(const struct flux_channel *ch);

// Releases refused because their borrow was already given back: a `flux_view` released twice.
uint64_t flux_channel_misreleased(const struct flux_channel *ch);

// Commits and releases that could not wait for the stream; each holds its slot (see `flux_channel_set_fence_timeout`).
uint64_t flux_channel_fence_failed(const struct flux_channel *ch);

struct flux_fence_wait flux_channel_fence_wait(const struct flux_channel *ch);

// 0 = no bound (default). One that runs out is a failed fence until its work is done.
void flux_channel_set_fence_timeout(const struct flux_channel *ch, uint64_t ns);

// `FLUX_RELEASE_DEFERRED`: a release returns at once; its slot goes back once the device is done.
flux_status flux_channel_set_release_mode(const struct flux_channel *ch, int32_t mode);

// Orders the view's channel stream after what `s` queued so far, on the device, not the host.
bool flux_view_after(const struct flux_view *v, const struct flux_stream *s);

// The payload is device memory: views and loans have no host bytes.
bool flux_channel_device_backed(const struct flux_channel *ch);

void *flux_channel_stream(const struct flux_channel *ch);

bool flux_channel_ready(const struct flux_channel *ch);

bool flux_channel_can_borrow(const struct flux_channel *ch);

uint64_t flux_channel_lost(const struct flux_channel *ch);

uint64_t flux_channel_dropped(const struct flux_channel *ch);

uint64_t flux_channel_cursor(const struct flux_channel *ch);

uint32_t flux_channel_attach_generation(const struct flux_channel *ch);

// Why a restarted publisher refused this subscriber; takes find nothing until it restarts again.
flux_status flux_channel_attach_status(struct flux_channel *ch);

bool flux_channel_orphaned(const struct flux_channel *ch);

bool flux_channel_inherited(const struct flux_channel *ch);

uint32_t flux_channel_wake_seq(const struct flux_channel *ch);

uint32_t flux_channel_slot_size(const struct flux_channel *ch);

uint32_t flux_channel_slot_count(const struct flux_channel *ch);

bool flux_channel_payload_readonly(const struct flux_channel *ch);

bool flux_channel_pages_committed(const struct flux_channel *ch);

bool flux_channel_pages_locked(const struct flux_channel *ch);

// Sample `wake_seq` before the take that finds nothing. False only on a timeout.
bool flux_channel_wait(const struct flux_channel *ch, uint32_t last_seq, int64_t timeout_ns);

void flux_stamp_offset_register(uint64_t fingerprint, int32_t stamp_offset);

int32_t flux_stamp_offset(uint64_t fingerprint);

flux_status flux_signpost_name(const char *key,
                               uint64_t fingerprint,
                               char *out,
                               size_t cap,
                               size_t *len);

// Waits with a thread per source below Linux 5.16. `FLUX_UNSUPPORTED` where seccomp forbids it.
flux_status flux_executor_new(struct flux_executor **out);

void flux_executor_free(struct flux_executor *ex);

// Equal priorities start in registration order, then take turns. Not while spinning. At most 127.
flux_status flux_executor_add(struct flux_executor *ex, struct flux_source src, int32_t priority);

size_t flux_executor_size(struct flux_executor *ex);

flux_status flux_executor_set_pass_budget(struct flux_executor *ex, uint32_t n);

uint32_t flux_executor_pass_budget(struct flux_executor *ex);

// Work the next pass would find, read without a syscall.
bool flux_executor_ready(struct flux_executor *ex);

// The last pass ran out of budget, not of frames: skip the next wait.
bool flux_executor_has_more(struct flux_executor *ex);

flux_status flux_executor_on_pass(struct flux_executor *ex, flux_hook f, void *ctx);

flux_status flux_executor_dispatch(struct flux_executor *ex, size_t *delivered);

// `yield_now` is asked between callbacks, never before the first.
flux_status flux_executor_dispatch_yielding(struct flux_executor *ex,
                                            bool (*yield_now)(void *ctx),
                                            void *ctx,
                                            size_t *delivered);

// Must never overlap a dispatch.
flux_status flux_executor_wait_for_work(struct flux_executor *ex, int64_t timeout_ns);

flux_status flux_executor_spin(struct flux_executor *ex, int64_t tick_ns);

// Also ends once `*run` is atomically set false. `run` must outlive the call.
flux_status flux_executor_spin_while(struct flux_executor *ex, bool *run, int64_t tick_ns);

void flux_executor_clear_interrupt(struct flux_executor *ex);

struct flux_handle *flux_executor_handle(struct flux_executor *ex);

void flux_handle_free(struct flux_handle *h);

// Held if no spin is running yet, so a stop racing the start of a spin is not lost.
void flux_handle_stop(const struct flux_handle *h);

// Ends the current wait, not the loop: `spin` runs `on_pass` and goes on.
void flux_handle_interrupt(const struct flux_handle *h);

// Breaks the wait so a pass runs; with nothing ready, `spin` waits on.
void flux_handle_wake(const struct flux_handle *h);

bool flux_handle_is_spinning(const struct flux_handle *h);

flux_status flux_partitioned_new(struct flux_partitioned **out);

void flux_partitioned_free(struct flux_partitioned *px);

// Takes `ex`: its handle is freed here and must not be used again. Take its `flux_handle` first.
void flux_partitioned_add(struct flux_partitioned *px, struct flux_executor *ex, size_t *index);

// A thread running `run` until `stop`, which may come first: `run` then returns at once.
flux_status flux_partitioned_add_external(struct flux_partitioned *px,
                                          flux_hook run,
                                          flux_hook stop,
                                          void *ctx,
                                          size_t *index);

// Runs on the partition's own thread before its loop, on every spin.
flux_status flux_partitioned_on_thread_start(struct flux_partitioned *px,
                                             size_t index,
                                             flux_hook init,
                                             void *ctx);

size_t flux_partitioned_size(struct flux_partitioned *px);

// The first failure on any partition stops every partition and is returned here.
flux_status flux_partitioned_spin(struct flux_partitioned *px, int64_t tick_ns);

struct flux_handle *flux_partitioned_handle(struct flux_partitioned *px);

flux_status flux_stream_create(struct flux_stream **out);

// The caller's `CUstream`, outliving its channels and views; NULL is `FLUX_INVALID_ARGUMENT`.
flux_status flux_stream_from_native(void *native, struct flux_stream **out);

void flux_stream_free(struct flux_stream *s);

void *flux_stream_native(const struct flux_stream *s);

int32_t flux_stream_device(const struct flux_stream *s);

// Until all work queued on the stream so far has run; false if the wait failed.
bool flux_stream_wait(const struct flux_stream *s);

// Fills `out` always; `FLUX_UNSUPPORTED` (reason in `flux_last_error()`) for no route.
flux_status flux_gpu_probe(int32_t device, struct flux_gpu_platform *out);

// The sticky `CUresult` after which every GPU channel refuses until a restart; 0 while none.
int32_t flux_gpu_wedged(void);

int32_t flux_gpu_current_device(void);

// Lists this endpoint until freed. A failure costs visibility only.
flux_status flux_announce(const struct flux_manifest_entry *entry, struct flux_announcement **out);

void flux_announcement_free(struct flux_announcement *a);

// A missing or unreadable channel reads as not live.
flux_status flux_read_channel_stats(const char *signpost, struct flux_channel_stats *out);

// Sorted by signpost. Free with `flux_topic_list_free`.
struct flux_topic_list *flux_enumerate_topics(void);

size_t flux_topic_list_size(const struct flux_topic_list *l);

const struct flux_topic_info *flux_topic_list_get(const struct flux_topic_list *l, size_t i);

void flux_topic_list_free(struct flux_topic_list *l);

// Every .msg on `AMENT_PREFIX_PATH` that parses. `FLUX_UNSUPPORTED` if a directory is unreadable.
flux_status flux_schemas_load(struct flux_schemas **out);

void flux_schemas_free(struct flux_schemas *s);

size_t flux_schemas_size(const struct flux_schemas *s);

// NULL when no installed type has this fingerprint.
const struct flux_schema *flux_schemas_find(const struct flux_schemas *s, uint64_t fingerprint);

const char *flux_schema_ros_type(const struct flux_schema *s);

// The frame as the ROS message's CDR. `FLUX_MISMATCH` for a frame that does not match.
flux_status flux_schema_cdr_size(const struct flux_schema *s,
                                 const uint8_t *data,
                                 size_t len,
                                 size_t *out);

// `*written` is the CDR size even past `cap`, which is `FLUX_INVALID_ARGUMENT` (frames change).
flux_status flux_schema_write_cdr(const struct flux_schema *s,
                                  const uint8_t *data,
                                  size_t len,
                                  uint8_t *out,
                                  size_t cap,
                                  size_t *written);

// Why this thread's last failed call failed. Valid until its next failed call.
const char *flux_last_error(void);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* FLUX_H */
