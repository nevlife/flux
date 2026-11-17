//! How large and how many slots a publisher gets when its caller passes 0. Every default and every
//! rule for them is here; the bindings pass 0 through.

use crate::channel::Alloc;
use crate::error::{Error, Result};
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

/// What the slots of one publisher may take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bound {
    pub budget_bytes: u64,
    pub max_slots: u32,
}

pub const HOST_BOUND: Bound = Bound {
    budget_bytes: 256 << 20,
    max_slots: 256,
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

/// The default slot of an `Alloc::Slot` publisher, sized for a frame's arrays, not its scalars: an
/// HD image (1280x720 BGRA, 3.52 MiB) fits.
pub const SLOT_ALLOC_SLOT_SIZE: u32 = 4 << 20;
pub const SLOT_ALLOC_MAX_SLOT_SIZE: u32 = 16 << 20;
/// Slots an `Alloc::Slot` publisher starts with: a subscriber holding seven frames to pair them
/// with another topic, the one being written, the newest in flight, and room to spare.
pub const SLOT_ALLOC_START_SLOTS: u32 = 12;
pub const SLOT_ALLOC_MAX_SLOTS: u32 = 16;

/// A publisher's slot size, the slots its layout reserves, and those with memory at start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub slot_size: u32,
    pub slot_count: u32,
    pub start: u32,
    /// Slots past `start` are added off the publish path (`channel::filler`).
    pub fixed: bool,
}

/// The slots of a publisher. `Arena` as `slot_size`, `slot_count` and `start_slots`. `Slot`, and a
/// GPU memory channel (a stream, no generated type: its payload is the frame's data), take the
/// `SLOT_ALLOC_*` value for each 0 and reserve `max_count` slots, `count` of them at start; a
/// `count` given alone is all the slots.
pub fn plan(
    alloc: Alloc,
    size: u32,
    count: u32,
    max_size: u32,
    max_count: u32,
    scalar_bytes: Option<u32>,
    gpu_stream: bool,
) -> Result<Plan> {
    let fixed = alloc == Alloc::Slot || (gpu_stream && scalar_bytes.is_none());
    if !fixed {
        let slot_size = slot_size(size, scalar_bytes, gpu_stream);
        let slot_count = slot_count(count, slot_size);
        return Ok(Plan {
            slot_size,
            slot_count,
            start: start_slots(count, slot_count),
            fixed: false,
        });
    }
    let or = |n: u32, default: u32| if n == 0 { default } else { n };
    let (size, max_size) = (
        or(size, SLOT_ALLOC_SLOT_SIZE),
        or(max_size, SLOT_ALLOC_MAX_SLOT_SIZE),
    );
    // A count given without a maximum is the count, as an Arena publisher's.
    let reserve = match (count, max_count) {
        (_, m) if m != 0 => m,
        (c, _) if c != 0 => c,
        _ => SLOT_ALLOC_MAX_SLOTS,
    };
    let (count, max_count) = (
        or(count, SLOT_ALLOC_START_SLOTS),
        or(max_count, SLOT_ALLOC_MAX_SLOTS),
    );
    if size > max_size {
        return Err(Error::InvalidArgument(format!(
            "slot_size {size} is past max_slot_size {max_size}"
        )));
    }
    if count > max_count {
        return Err(Error::InvalidArgument(format!(
            "slot_count {count} is past max_slot_count {max_count}"
        )));
    }
    Ok(Plan {
        slot_size: size,
        slot_count: reserve,
        start: count,
        fixed: true,
    })
}

/// `asked`, or for 0 the slots a publisher of `slot_size` gets.
#[must_use]
pub fn slot_count(asked: u32, slot_size: u32) -> u32 {
    if asked != 0 {
        return asked;
    }
    max_slots(slot_size, HOST_BOUND)
}

#[cfg(test)]
mod tests {
    use super::{
        HOST_BOUND, Plan, SLOT_ALLOC_MAX_SLOTS, SLOT_ALLOC_SLOT_SIZE, SLOT_ALLOC_START_SLOTS,
        START_SLOTS, UNTYPED_SLOT_SIZE, max_slots, plan, slot_count, slot_size, start_slots,
        typed_slot_size,
    };
    use crate::channel::Alloc;

    #[test]
    fn a_slot_plan_takes_its_defaults_and_keeps_within_its_maxima() {
        let defaults = plan(Alloc::Slot, 0, 0, 0, 0, Some(100), false).unwrap();
        assert_eq!(
            defaults,
            Plan {
                slot_size: SLOT_ALLOC_SLOT_SIZE,
                slot_count: SLOT_ALLOC_MAX_SLOTS,
                start: SLOT_ALLOC_START_SLOTS,
                fixed: true,
            }
        );
        let asked = plan(Alloc::Slot, 32 << 20, 20, 64 << 20, 24, None, false).unwrap();
        assert_eq!(
            (asked.slot_size, asked.slot_count, asked.start),
            (32 << 20, 24, 20)
        );
        let fixed = plan(Alloc::Slot, 0, 4, 0, 0, None, false).unwrap();
        assert_eq!((fixed.slot_count, fixed.start), (4, 4));
        assert!(fixed.fixed);
        assert!(plan(Alloc::Slot, 17 << 20, 0, 0, 0, None, false).is_err());
        assert!(plan(Alloc::Slot, 0, 17, 0, 0, None, false).is_err());
        // A GPU memory channel's payload is its data: it takes the same defaults.
        assert_eq!(
            plan(Alloc::Arena, 0, 0, 0, 0, None, true).unwrap(),
            defaults
        );
        // Arena keeps the type's slot and the budgeted count.
        let arena = plan(Alloc::Arena, 0, 0, 0, 0, Some(100), false).unwrap();
        assert_eq!(arena.slot_size, slot_size(0, Some(100), false));
        assert_eq!(arena.start, START_SLOTS);
        assert!(!arena.fixed);
    }

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
        assert_eq!(max_slots(u32::MAX, HOST_BOUND), 2);
    }

    #[test]
    fn a_count_of_zero_is_the_bound_over_the_slot() {
        assert_eq!(slot_count(0, 8192), 256);
        assert_eq!(slot_count(0, 200 << 20), 2);
        assert_eq!(slot_count(5, 8192), 5);
    }

    #[test]
    fn a_count_left_to_flux_starts_at_the_default_depth_and_an_asked_one_whole() {
        assert_eq!(START_SLOTS, 9);
        assert_eq!(start_slots(0, 256), 9);
        assert_eq!(start_slots(0, 2), 2);
        assert_eq!(start_slots(5, 5), 5);
    }
}
