use flux_core::{Channel, DType, Error, Published, QoS};

fn frame(ch: &mut Channel) -> Vec<u8> {
    ch.take().expect("a frame").data().to_vec()
}

#[test]
fn publish_then_take_round_trips() {
    let mut ch = Channel::new(64, 4).unwrap();
    assert!(!ch.ready());
    assert_eq!(ch.publish(b"hello"), Published::Ok);
    assert!(ch.ready());
    let v = ch.take().unwrap();
    assert_eq!(v.data(), b"hello");
    assert_eq!(v.meta().shape(), &[5]);
    assert_eq!(v.ticket(), 1);
    drop(v);
    assert!(ch.take().is_none());
}

#[test]
fn depth_one_skips_to_newest_and_counts_lost() {
    let mut ch = Channel::new(8, 4).unwrap();
    ch.set_qos(QoS::new(1).unwrap()).unwrap();
    for b in [b"a", b"b", b"c"] {
        assert_eq!(ch.publish(b), Published::Ok);
    }
    assert_eq!(frame(&mut ch), b"c");
    assert_eq!(ch.lost(), 2);
}

#[test]
fn deeper_queue_delivers_in_order() {
    let mut ch = Channel::new(8, 4).unwrap();
    ch.set_qos(QoS::new(3).unwrap()).unwrap();
    for b in [b"a", b"b", b"c"] {
        assert_eq!(ch.publish(b), Published::Ok);
    }
    assert_eq!(frame(&mut ch), b"a");
    assert_eq!(frame(&mut ch), b"b");
    assert_eq!(frame(&mut ch), b"c");
    assert_eq!(ch.lost(), 0);
}

#[test]
fn too_large_and_mismatched_frames_are_refused() {
    let mut ch = Channel::new(4, 2).unwrap();
    assert_eq!(ch.publish(b"12345"), Published::TooLarge);
    assert_eq!(
        ch.publish_typed(&[0; 4], DType::U16, &[3]),
        Published::TooLarge
    );
    assert!(Published::TooLarge.faulted());
    assert!(!Published::Backpressure.faulted());
}

#[test]
fn typed_publish_carries_shape_and_dtype() {
    let mut ch = Channel::new(64, 2).unwrap();
    assert_eq!(
        ch.publish_typed(&[0; 24], DType::F32, &[2, 3]),
        Published::Ok
    );
    let v = ch.take().unwrap();
    assert_eq!(v.meta().dtype(), Some(DType::F32));
    assert_eq!(v.meta().shape(), &[2, 3]);
    assert_eq!(v.data().len(), 24);
}

#[test]
fn loan_commit_len_publishes_the_prefix() {
    let mut ch = Channel::new(16, 2).unwrap();
    let mut w = ch.loan().unwrap();
    assert_eq!(w.capacity(), 16);
    w.data_mut()[..3].copy_from_slice(b"xyz");
    assert_eq!(w.commit_len(3), Published::Ok);
    assert_eq!(frame(&mut ch), b"xyz");
}

#[test]
fn aborted_loan_publishes_nothing() {
    let mut ch = Channel::new(16, 2).unwrap();
    drop(ch.loan().unwrap());
    assert!(ch.take().is_none());
    assert_eq!(ch.publish(b"ok"), Published::Ok);
    assert_eq!(frame(&mut ch), b"ok");
}

/// The claim takes the older slot; the abort must neither leave the old ticket on the loan's
/// bytes nor leave the slot claimed, which would make the next publish overwrite "bbbb".
#[test]
fn an_aborted_loan_over_an_unread_frame_hides_it_and_frees_its_slot() {
    let mut ch = Channel::new(8, 2).unwrap();
    ch.set_qos(QoS::new(2).unwrap()).unwrap();
    assert_eq!(ch.publish(b"aaaa"), Published::Ok);
    assert_eq!(ch.publish(b"bbbb"), Published::Ok);
    let mut w = ch.loan().unwrap();
    w.data_mut()[..4].copy_from_slice(b"ZZZZ");
    drop(w);
    assert_eq!(ch.publish(b"cccc"), Published::Ok);
    assert_eq!(frame(&mut ch), b"bbbb");
    assert_eq!(frame(&mut ch), b"cccc");
    assert_eq!(ch.lost(), 1, "aaaa went with the abort");
}

/// "b" is overwritten while "a" is held, so the next take skips from ticket 1 to 3.
#[test]
fn a_frame_overwritten_inside_the_depth_window_counts_as_lost() {
    let mut ch = Channel::new(8, 2).unwrap();
    ch.set_qos(QoS::new(2).unwrap().max_borrow(2).unwrap())
        .unwrap();
    assert_eq!(ch.publish(b"a"), Published::Ok);
    assert_eq!(ch.publish(b"b"), Published::Ok);
    let held = ch.take().unwrap();
    assert_eq!(held.data(), b"a");
    assert_eq!(ch.publish(b"c"), Published::Ok);
    assert_eq!(frame(&mut ch), b"c");
    assert_eq!(ch.lost(), 1);
}

/// Loans committed out of claim order: the next overwrite still takes the oldest frame's slot.
#[test]
fn the_oldest_frame_is_overwritten_first_whatever_order_loans_commit_in() {
    let mut ch = Channel::new(8, 3).unwrap();
    ch.set_qos(QoS::new(3).unwrap()).unwrap();
    let mut a = ch.loan().unwrap();
    let mut b = ch.loan().unwrap();
    a.data_mut()[0] = b'a';
    b.data_mut()[0] = b'b';
    assert_eq!(b.commit_len(1), Published::Ok);
    assert_eq!(a.commit_len(1), Published::Ok);
    assert_eq!(ch.publish(b"c"), Published::Ok);
    assert_eq!(ch.publish(b"d"), Published::Ok);
    assert_eq!(frame(&mut ch), b"a");
    assert_eq!(frame(&mut ch), b"c");
    assert_eq!(frame(&mut ch), b"d");
    assert_eq!(ch.lost(), 1);
}

/// Claim and take cost must not grow with the slot count (they scanned every slot before).
#[test]
fn publish_and_take_do_not_scan_every_slot() {
    let mut ch = Channel::new(8, 65_535).unwrap();
    let t0 = std::time::Instant::now();
    for _ in 0..2000 {
        assert_eq!(ch.publish(b"x"), Published::Ok);
        assert!(ch.take().is_some());
    }
    let each = t0.elapsed() / 2000;
    assert!(
        each < std::time::Duration::from_micros(20),
        "{each:?} per cycle"
    );
}

#[test]
fn a_heap_channel_too_large_to_allocate_is_an_error() {
    assert!(matches!(
        Channel::new(u32::MAX, 65_535),
        Err(Error::Io { .. })
    ));
}

#[test]
fn held_views_cause_backpressure() {
    let mut ch = Channel::new(8, 2).unwrap();
    ch.set_qos(QoS::new(2).unwrap().max_borrow(2).unwrap())
        .unwrap();
    assert_eq!(ch.publish(b"a"), Published::Ok);
    assert_eq!(ch.publish(b"b"), Published::Ok);
    let a = ch.take().unwrap();
    let b = ch.take().unwrap();
    assert_eq!(ch.publish(b"c"), Published::Backpressure);
    assert_eq!(ch.dropped(), 1);
    drop((a, b));
    assert_eq!(ch.publish(b"c"), Published::Ok);
}

#[test]
fn max_borrow_refuses_a_third_view() {
    let mut ch = Channel::new(8, 4).unwrap();
    ch.set_qos(QoS::new(3).unwrap().max_borrow(2).unwrap())
        .unwrap();
    for b in [b"a", b"b", b"c"] {
        assert_eq!(ch.publish(b), Published::Ok);
    }
    let _a = ch.take().unwrap();
    let _b = ch.take().unwrap();
    assert!(!ch.can_borrow());
    assert!(ch.take().is_none());
    assert_eq!(ch.refused().max_borrow, 1);
}

#[test]
fn view_outlives_its_channel() {
    let mut ch = Channel::new(8, 2).unwrap();
    assert_eq!(ch.publish(b"keep"), Published::Ok);
    let v = ch.take().unwrap();
    drop(ch);
    assert_eq!(v.data(), b"keep");
}

#[test]
fn qos_rejects_bad_values() {
    assert!(QoS::new(0).is_err());
    assert!(QoS::default().max_borrow(0).is_err());
    let mut ch = Channel::new(8, 2).unwrap();
    assert!(ch.set_qos(QoS::new(2).unwrap().transient_local(3)).is_err());
}

#[test]
fn one_slot_is_refused() {
    assert!(matches!(Channel::new(8, 1), Err(Error::InvalidArgument(_))));
}
