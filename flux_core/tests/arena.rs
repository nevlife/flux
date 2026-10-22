use flux_core::arena::{HEAP_BASE, TRAILER_BYTES};
use flux_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, MemoryPolicy, NO_SCHEMA, Published, signpost_name,
};

fn pair(test: &str, slots: u32) -> (Channel, Channel) {
    // SAFETY: set before any thread reads it.
    unsafe { std::env::set_var("FLUX_ARENA_BYTES", (64u32 << 20).to_string()) };
    let name = signpost_name(&format!("/a{}_{test}", std::process::id()), NO_SCHEMA).unwrap();
    let p = Channel::create(
        &name,
        4096,
        slots,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap();
    let s = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    (p, s)
}

/// One frame: an 8-byte offset into the arena, then a run there longer than the slot.
fn publish(p: &mut Channel, fill: u8, n: usize) -> u64 {
    let mut w = p.loan().unwrap();
    let heap = w.heap().unwrap();
    // SAFETY: w is live.
    let (dst, off) = unsafe { heap.alloc(n, 64) }.unwrap();
    // SAFETY: heap_alloc handed out n writable bytes.
    unsafe { std::ptr::write_bytes(dst, fill, n) };
    w.data_mut()[..8].copy_from_slice(&(HEAP_BASE + off).to_le_bytes());
    assert_eq!(w.commit_len(8), Published::Ok);
    off
}

#[test]
fn a_frame_carries_a_run_past_its_slot_and_frees_it_when_the_slot_comes_back() {
    let (mut p, mut s) = pair("run", 2);
    let off = publish(&mut p, 7, 1 << 20);
    let v = s.take().unwrap();
    assert_eq!(v.data().len(), 8 + TRAILER_BYTES);
    let heap = v.heap().expect("the trailer names the arena");
    let at = u64::from_le_bytes(v.data()[..8].try_into().unwrap()) - HEAP_BASE;
    assert_eq!(at, off);
    let run = &heap[usize::try_from(at).unwrap()..][..1 << 20];
    assert!(run.iter().all(|&b| b == 7));

    // Held: the other slot takes the next frame, and this run stays put.
    let other = publish(&mut p, 9, 1 << 20);
    assert_ne!(other, off);
    assert!(run.iter().all(|&b| b == 7));
    drop(v);
    // Both slots claimed again: the first run was freed, so the arena hands its offset out again.
    let again = publish(&mut p, 1, 1 << 20);
    assert_eq!(again, off);
}
