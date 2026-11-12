//! A frame with fields in its publisher's arena, made into one that holds them itself, on
//! hand-built frames of test/msg: the copy converts to the same CDR as the frame read with its arena.

mod common;

use common::test_registry;
use flux_gen::cdr::{cdr_size, cdr_size_in, write_cdr, write_cdr_in};
use flux_gen::flatten::flatten_message;
use flux_gen::inline::{gpu_fingerprint, has_gpu_data, inline_arena, with_data};
use flux_gen::layout::{Layout, build_layout};
use flux_gen::wire::{HEAP_POS, UNIT, WIRE_GPU, WIRE_HEAP};

fn layout(name: &str) -> Layout {
    let reg = test_registry();
    build_layout(&flatten_message(reg.get(name).unwrap(), &reg).unwrap()).unwrap()
}

/// The descriptor of `n` elements at arena byte `at`.
fn in_arena(at: usize, n: usize) -> [u8; 8] {
    let mut d = [0u8; 8];
    let w = WIRE_HEAP + u32::try_from(at / UNIT).unwrap();
    d[..4].copy_from_slice(&w.to_ne_bytes());
    d[4..].copy_from_slice(&u32::try_from(n).unwrap().to_ne_bytes());
    d
}

fn cdr_of(layout: &Layout, frame: &[u8], heap: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; cdr_size_in(layout, frame, heap).unwrap()];
    write_cdr_in(layout, frame, heap, &mut out).unwrap();
    out
}

fn same_cdr(layout: &Layout, frame: &[u8], heap: &[u8]) {
    let flat = inline_arena(layout, frame, heap, &[]).expect("inlined");
    let mut out = vec![0u8; cdr_size(layout, &flat).unwrap()];
    write_cdr(layout, &flat, &mut out).unwrap();
    assert_eq!(out, cdr_of(layout, frame, heap), "{}", layout.name);
}

#[test]
fn a_string_and_a_column_in_the_arena_come_into_the_frame() {
    let l = layout("Labeled");
    let mut frame = vec![0u8; 16];
    frame[..8].copy_from_slice(&in_arena(32, 5));
    frame[8..16].copy_from_slice(&in_arena(64, 2));
    let mut heap = vec![0u8; 128];
    heap[32..37].copy_from_slice(b"hello");
    heap[64..72].copy_from_slice(&1.5f64.to_ne_bytes());
    heap[72..80].copy_from_slice(&2.5f64.to_ne_bytes());
    let flat = inline_arena(&l, &frame, &heap, &[]).unwrap();
    assert_eq!(&flat[16..21], b"hello");
    let values = u32::from_ne_bytes(flat[8..12].try_into().unwrap()) as usize;
    assert!(values < HEAP_POS && values.is_multiple_of(8), "{values}");
    assert_eq!(&flat[values..values + 8], &1.5f64.to_ne_bytes());
    same_cdr(&l, &frame, &heap);
}

/// Elements in the arena hold descriptors of their own, some to the arena and some to the frame.
#[test]
fn a_jagged_array_in_the_arena_is_walked_into_the_frame() {
    let l = layout("TensorList");
    // The frame: the tensors descriptor, then element 0's shape inside the frame itself.
    let mut frame = vec![0u8; 16];
    frame[..8].copy_from_slice(&in_arena(0, 2));
    frame[8..12].copy_from_slice(&3u32.to_ne_bytes());
    let mut heap = vec![0u8; 256];
    // Element 0: shape [3] in the frame at 8, data in the arena.
    heap[..4].copy_from_slice(&8u32.to_ne_bytes());
    heap[4..8].copy_from_slice(&1u32.to_ne_bytes());
    heap[8..16].copy_from_slice(&in_arena(64, 2));
    // Element 1: both in the arena.
    heap[16..24].copy_from_slice(&in_arena(96, 1));
    heap[24..32].copy_from_slice(&in_arena(128, 1));
    heap[64..68].copy_from_slice(&1.0f32.to_ne_bytes());
    heap[68..72].copy_from_slice(&2.0f32.to_ne_bytes());
    heap[96..100].copy_from_slice(&7u32.to_ne_bytes());
    heap[128..132].copy_from_slice(&9.0f32.to_ne_bytes());
    same_cdr(&l, &frame, &heap);
}

#[test]
fn a_frame_without_arena_fields_comes_back_as_it_was() {
    let l = layout("Labeled");
    let mut frame = vec![0u8; 16];
    frame[..4].copy_from_slice(&16u32.to_ne_bytes());
    frame[4..8].copy_from_slice(&2u32.to_ne_bytes());
    frame.extend_from_slice(b"ok");
    assert_eq!(inline_arena(&l, &frame, &[], &[]).unwrap(), frame);
}

#[test]
fn a_field_in_gpu_memory_past_the_arena_or_read_twice_is_refused() {
    let l = layout("Labeled");
    let heap = vec![0u8; 64];
    let mut gpu = vec![0u8; 16];
    gpu[..4].copy_from_slice(&WIRE_GPU.to_ne_bytes());
    gpu[4..8].copy_from_slice(&1u32.to_ne_bytes());
    assert!(inline_arena(&l, &gpu, &heap, &[]).is_err(), "GPU memory");
    let mut past = vec![0u8; 16];
    past[..8].copy_from_slice(&in_arena(48, 32));
    assert!(
        inline_arena(&l, &past, &heap, &[]).is_err(),
        "past the arena"
    );
    // Labeled has one string; Names has a string array, whose entries can all name one run.
    let n = layout("Names");
    let dyn_at = n.root.placed[1].offset;
    let mut twice = vec![0u8; n.root.stride];
    twice[dyn_at..dyn_at + 8].copy_from_slice(&in_arena(0, 3));
    let mut heap = vec![0u8; 64];
    for i in 0..3 {
        heap[i * 8..i * 8 + 8].copy_from_slice(&in_arena(32, 32));
    }
    assert!(
        inline_arena(&n, &twice, &heap, &[]).is_err(),
        "the same bytes three times"
    );
}

/// A field in the GPU arena comes from the host copy a CPU subscriber made of its block.
#[test]
fn a_column_in_the_gpu_arena_comes_from_its_host_copy() {
    let l = layout("Labeled");
    let mut frame = vec![0u8; 16];
    frame[..4].copy_from_slice(&16u32.to_ne_bytes());
    frame[4..8].copy_from_slice(&2u32.to_ne_bytes());
    // values: 2 float64 at GPU arena byte 512, inside a copied block that starts at 256.
    let w = WIRE_GPU + u32::try_from(512 / UNIT).unwrap();
    frame[8..12].copy_from_slice(&w.to_ne_bytes());
    frame[12..16].copy_from_slice(&2u32.to_ne_bytes());
    frame.extend_from_slice(b"ok");
    let mut block = vec![0u8; 512];
    block[256..264].copy_from_slice(&4.5f64.to_ne_bytes());
    let gpu = [(flux_gen::wire::GPU_POS + 256, &block[..])];
    let flat = inline_arena(&l, &frame, &[], &gpu).unwrap();
    let at = u32::from_ne_bytes(flat[8..12].try_into().unwrap()) as usize;
    assert_eq!(&flat[at..at + 8], &4.5f64.to_ne_bytes());
    assert!(
        inline_arena(&l, &frame, &[], &[]).is_err(),
        "no copy of the block"
    );
}

/// A GPU memory channel's aux (a frame with `data` empty) and payload make one frame of its type.
#[test]
fn an_aux_frame_and_its_payload_make_one_frame() {
    let l = layout("DlTensor");
    assert!(has_gpu_data(&l));
    assert!(!has_gpu_data(&layout("Labeled")), "no data column of bytes");
    // The aux: the scalar block with every descriptor empty, as a Builder leaves it.
    let aux = vec![0u8; l.root.stride];
    let data: Vec<u8> = (0..100u8).collect();
    let full = with_data(&l, &aux, &data).unwrap();
    let mut want = vec![0u8; cdr_size(&l, &full).unwrap()];
    write_cdr(&l, &full, &mut want).unwrap();
    assert_eq!(
        &want[want.len() - 100..],
        &data[..],
        "data ends the message"
    );
    assert!(
        with_data(&l, &aux[..4], &data).is_err(),
        "aux shorter than the scalar block"
    );
}

#[test]
fn a_gpu_fingerprint_is_its_types_and_no_other() {
    let a = gpu_fingerprint(0x1234);
    assert_eq!(a, gpu_fingerprint(0x1234));
    assert_ne!(a, gpu_fingerprint(0x1235));
    assert_ne!(a, 0x1234);
}
