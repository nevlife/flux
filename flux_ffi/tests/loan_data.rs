//! What C++ `WriteSlot` does: `data()`, then `capacity()`, then a write through the first
//! pointer. Both accessors call `flux_loan_data`. Run under Miri to check the aliasing:
//! `cargo +nightly miri test -p flux_ffi --test loan_data`.

use flux_ffi::*;

#[test]
fn a_second_loan_data_keeps_the_first_pointer_valid() {
    let mut ch: *mut flux_channel = std::ptr::null_mut();
    let mut l = flux_loan { opaque: [0; 16] };
    let mut cap = 0usize;
    let (l, cap) = (&raw mut l, &raw mut cap);
    // SAFETY: every pointer is to a live local; the loan is ended before the channel is freed.
    let made = unsafe { flux_channel_new(64, 2, &raw mut ch) };
    assert_eq!(made, flux_status::FLUX_OK);
    // SAFETY: as above.
    assert!(unsafe { flux_channel_loan(ch, l) });
    // SAFETY: as above.
    let p = unsafe { flux_loan_data(l, cap) };
    // SAFETY: the loan's capacity is 64 bytes.
    unsafe { p.write(1) };
    // SAFETY: as above.
    assert_eq!(unsafe { flux_loan_data(l, cap) }, p);
    // SAFETY: as above.
    let rest = unsafe { p.add(1) };
    // SAFETY: bytes 1..64 of the loan.
    unsafe { rest.write_bytes(7, 63) };
    // SAFETY: as above.
    let sent = unsafe { flux_loan_commit(l) };
    assert_eq!(sent, flux_published::FLUX_PUBLISHED_OK);
    // SAFETY: as above.
    assert!(unsafe { flux_loan_data(l, cap) }.is_null());
    // SAFETY: as above.
    unsafe { flux_loan_abort(l) };
    // SAFETY: as above.
    unsafe { flux_channel_free(ch) };
}
