//! How large and how many slots a publisher gets when its caller passes 0. Every default and every
//! rule for them is here; the bindings pass 0 through.

use crate::layout::{MIN_SLOT_COUNT, PAYLOAD_ALIGN, align_up};
use crate::qos::DEFAULT_DEPTH;

/// 0: `slot_size` decides.
pub const DEFAULT_SLOT_SIZE: u32 = 0;
/// The slot of a publisher with no generated type given no size: its payload is the whole frame.
pub const UNTYPED_SLOT_SIZE: u32 = 8 << 20;
/// Room a typed slot keeps after its scalar block for the message's own strings; its arrays go to
/// the arena.
pub const SLOT_STRING_BYTES: u32 = 4096;
pub const DEFAULT_SLOT_COUNT: u32 = 0;

/// A publisher left at count 0 starts with memory for one slot to write and a default depth to
/// read, and adds slots up to its count (`Channel::claim`).
pub const START_SLOTS: u32 = 1 + DEFAULT_DEPTH;

/// What the slots of one publisher may take, by where their payload lives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bound {
    pub budget_bytes: u64,
    pub max_slots: u32,
}

pub const HOST_BOUND: Bound = Bound {
    budget_bytes: 256 << 20,
    max_slots: 256,
};
/// Device memory is scarcer and its frames are mostly large.
pub const DEVICE_BOUND: Bound = Bound {
    budget_bytes: 256 << 20,
    max_slots: 32,
};

/// The slot of a generated type with `scalar_bytes` of scalar block: that, the string room and the
/// arena trailer the Writer keeps clear (GPU one when the channel declares a stream), in pages.
// The trailers and the page are constants far below u32::MAX.
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub const fn typed_slot_size(scalar_bytes: u32, gpu: bool) -> u32 {
    let trailer = if gpu {
        crate::arena::GPU_TRAILER_BYTES
    } else {
        crate::arena::TRAILER_BYTES
    };
    // The Writer keeps the trailer on an 8-byte boundary: up to 7 bytes before it.
    let bytes = scalar_bytes + SLOT_STRING_BYTES + trailer as u32 + 7;
    let page = PAYLOAD_ALIGN as u32;
    bytes.div_ceil(page) * page
}

/// `asked`, or for 0 the slot of the generated type with `scalar_bytes` (None: not one).
#[must_use]
pub const fn slot_size(asked: u32, scalar_bytes: Option<u32>, gpu_stream: bool) -> u32 {
    match (asked, scalar_bytes) {
        (0, Some(scalar)) => typed_slot_size(scalar, gpu_stream),
        (0, None) => UNTYPED_SLOT_SIZE,
        (n, _) => n,
    }
}

/// `bound`'s budget over the slot, within `MIN_SLOT_COUNT..=max_slots`.
#[must_use]
pub fn max_slots(slot_size: u32, bound: Bound) -> u32 {
    let stride = align_up(slot_size as usize, PAYLOAD_ALIGN) as u64;
    let n = (bound.budget_bytes / stride.max(1))
        .clamp(u64::from(MIN_SLOT_COUNT), u64::from(bound.max_slots));
    u32::try_from(n).unwrap_or(bound.max_slots)
}

/// The slots of `count` that have memory from the start: all of them when the caller asked for a
/// count, else `START_SLOTS`.
#[must_use]
pub fn start_slots(asked: u32, count: u32) -> u32 {
    if asked != 0 {
        return count;
    }
    START_SLOTS.min(count)
}

/// `asked`, or for 0 the slots a publisher of `slot_size` gets, its payload on the device or not.
#[must_use]
pub fn slot_count(asked: u32, slot_size: u32, device_payload: bool) -> u32 {
    if asked != 0 {
        return asked;
    }
    let bound = if device_payload {
        DEVICE_BOUND
    } else {
        HOST_BOUND
    };
    max_slots(slot_size, bound)
}

#[cfg(test)]
mod tests {
    use super::{
        DEVICE_BOUND, HOST_BOUND, START_SLOTS, UNTYPED_SLOT_SIZE, max_slots, slot_count, slot_size,
        start_slots, typed_slot_size,
    };

    #[test]
    fn a_size_of_zero_is_the_types_or_the_untyped_one() {
        assert_eq!(slot_size(0, Some(100), false), typed_slot_size(100, false));
        assert_eq!(slot_size(0, None, true), UNTYPED_SLOT_SIZE);
        assert_eq!(slot_size(4096, Some(100), false), 4096);
    }

    /// The table the defaults were chosen by: an Imu, a ZED 1920x1200 bgr8 frame, the untyped slot.
    #[test]
    fn the_bounds_give_the_counts_they_were_chosen_for() {
        assert_eq!(max_slots(8192, HOST_BOUND), 256);
        assert_eq!(max_slots(6_912_000, HOST_BOUND), 38);
        assert_eq!(max_slots(UNTYPED_SLOT_SIZE, HOST_BOUND), 32);
        assert_eq!(max_slots(UNTYPED_SLOT_SIZE, DEVICE_BOUND), 32);
        assert_eq!(max_slots(1 << 20, DEVICE_BOUND), 32);
        assert_eq!(max_slots(u32::MAX, HOST_BOUND), 2);
    }

    #[test]
    fn a_count_of_zero_is_the_bound_of_where_the_payload_lives() {
        assert_eq!(slot_count(0, 8192, false), 256);
        assert_eq!(slot_count(0, 8192, true), 32);
        assert_eq!(slot_count(0, 200 << 20, true), 2);
        assert_eq!(slot_count(5, 8192, false), 5);
    }

    #[test]
    fn a_count_left_to_flux_starts_at_the_default_depth_and_an_asked_one_whole() {
        assert_eq!(START_SLOTS, 9);
        assert_eq!(start_slots(0, 256), 9);
        assert_eq!(start_slots(0, 2), 2);
        assert_eq!(start_slots(5, 5), 5);
    }
}
