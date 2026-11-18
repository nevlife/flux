//! Image slots on the integrated GPU this host has; every test returns at once elsewhere.

use super::{gpu, read_back};
use crate::channel::{Alloc, PublisherConfig, soon};
use crate::gpu::{Stream, probe};
use crate::layout::NO_SCHEMA;
use crate::qos::{MemoryPolicy, QoS};
use crate::{Channel, Error, Published};

fn integrated() -> bool {
    let p = probe(0);
    p.integrated && crate::gpu::driver::Driver::get().host_export.is_ok()
}

fn image_publisher(
    name: &str,
    count: u32,
    max_count: u32,
    stream: Stream,
) -> crate::Result<Channel> {
    let config = PublisherConfig {
        alloc: Alloc::Slot,
        slot_size: 3 << 20,
        slot_count: count,
        max_slot_count: max_count,
        image_slots: true,
        ..PublisherConfig::default()
    };
    Channel::create_config(
        name,
        &config,
        NO_SCHEMA,
        MemoryPolicy::default(),
        stream,
        -1,
        None,
    )
}

fn subscriber(name: &str, stream: Stream, borrow: u32) -> Channel {
    let mut s = Channel::open_on(name, NO_SCHEMA, MemoryPolicy::default(), stream, -1).unwrap();
    s.set_qos(QoS::new(4).unwrap().max_borrow(borrow).unwrap())
        .unwrap();
    s
}

/// The `/proc/self/maps` line of the mapping that starts at `p`.
fn mapping_at(p: *const u8) -> Option<String> {
    let maps = std::fs::read_to_string("/proc/self/maps").ok()?;
    let at = format!("{:x}-", p as usize);
    maps.lines().find(|l| l.starts_with(&at)).map(str::to_owned)
}

#[test]
fn an_image_slot_is_a_dmabuf_mapped_in_the_publisher_and_the_subscriber() {
    if !integrated() {
        return;
    }
    let name = super::channel::topic("img_map");
    let mut publisher = image_publisher(&name, 4, 0, Stream::default()).unwrap();
    assert_eq!(publisher.slots_ready(), 4);
    let mut sub = subscriber(&name, Stream::default(), 1);
    let mut slot = publisher.loan().unwrap();
    assert_eq!(slot.capacity(), 4 << 20);
    let at = slot.data_mut().as_ptr();
    let maps = mapping_at(at).expect("the publisher's slot is a mapping of its own");
    assert!(maps.contains("dmabuf"), "{maps}");
    slot.data_mut()[..3].copy_from_slice(&[1, 2, 3]);
    assert_eq!(slot.commit_len(3), Published::Ok);
    let view = sub.take().unwrap();
    assert_eq!(view.data(), &[1, 2, 3]);
    let maps =
        mapping_at(view.data().as_ptr()).expect("the subscriber's slot is a mapping of its own");
    assert!(maps.contains("dmabuf") && maps.contains(" r--s "), "{maps}");
}

#[test]
fn a_gpu_subscriber_reads_an_image_slot_through_its_registration() {
    if !integrated() {
        return;
    }
    let Some((_, copy)) = gpu() else {
        return;
    };
    let name = super::channel::topic("img_gpu");
    let mut publisher = image_publisher(&name, 4, 0, Stream::default()).unwrap();
    let mut sub = subscriber(&name, Stream::create().unwrap(), 1);
    for k in 0..8u8 {
        let mut slot = publisher.loan().unwrap();
        slot.data_mut()[..4096].fill(k + 10);
        assert_eq!(slot.commit_len(4096), Published::Ok);
        let view = sub.take().unwrap();
        assert_ne!(view.device_ptr(), 0);
        assert_eq!(read_back(copy, view.device_ptr(), 4096), vec![k + 10; 4096]);
    }
}

#[test]
fn a_subscriber_reads_an_image_slot_the_publisher_added_after_it_attached() {
    if !integrated() {
        return;
    }
    let name = super::channel::topic("img_grow");
    let mut publisher = image_publisher(&name, 2, 4, Stream::default()).unwrap();
    let mut sub = subscriber(&name, Stream::default(), 4);
    let mut held = Vec::new();
    for k in 0..2u8 {
        assert_eq!(publisher.publish(&[k]), Published::Ok);
        held.push(sub.take().unwrap());
    }
    assert!(publisher.loan().is_none(), "both slots are held");
    assert!(soon(|| publisher.slots_ready() == 3));
    assert_eq!(publisher.publish(&[9]), Published::Ok);
    assert_eq!(sub.take().map(|got| got.data().to_vec()), Some(vec![9]));
    assert_eq!(held[0].data(), &[0]);
}

#[test]
fn a_frame_of_an_image_slot_stays_readable_once_its_publisher_is_gone() {
    if !integrated() {
        return;
    }
    let name = super::channel::topic("img_gone");
    let mut publisher = image_publisher(&name, 4, 0, Stream::default()).unwrap();
    let mut sub = subscriber(&name, Stream::default(), 1);
    assert_eq!(publisher.publish(&[5; 1000]), Published::Ok);
    let view = sub.take().unwrap();
    drop(publisher);
    assert_eq!(view.data(), &[5; 1000][..]);
}

#[test]
fn image_slots_need_fixed_slots() {
    if !integrated() {
        return;
    }
    let config = PublisherConfig {
        image_slots: true,
        ..PublisherConfig::default()
    };
    let name = super::channel::topic("img_arena");
    let made = Channel::create_config(
        &name,
        &config,
        NO_SCHEMA,
        MemoryPolicy::default(),
        Stream::default(),
        -1,
        None,
    );
    assert!(
        matches!(made, Err(Error::InvalidArgument(_))),
        "{:?}",
        made.err()
    );
}

fn format_publisher(
    name: &str,
    format: crate::ImageFormat,
    slot_size: u32,
) -> crate::Result<Channel> {
    let config = PublisherConfig {
        alloc: Alloc::Slot,
        slot_size,
        slot_count: 4,
        image_format: format,
        ..PublisherConfig::default()
    };
    Channel::create_config(
        name,
        &config,
        NO_SCHEMA,
        MemoryPolicy::default(),
        Stream::default(),
        -1,
        None,
    )
}

fn bgra(width: u32, height: u32) -> crate::ImageFormat {
    crate::ImageFormat {
        encoding: crate::ImageEncoding::Bgra8,
        width,
        height,
    }
}

/// The publisher declares the format, a slot holds one frame of it, and both ends see the layout
/// and a dmabuf for the slot.
#[test]
fn an_image_format_sizes_the_slot_and_reaches_the_subscriber() {
    if !integrated() {
        return;
    }
    let name = super::channel::topic("img_fmt");
    let mut publisher = format_publisher(&name, bgra(848, 480), 0).unwrap();
    let mut sub = subscriber(&name, Stream::default(), 1);
    let slot = publisher.loan().unwrap();
    assert_eq!(
        slot.capacity(),
        2 << 20,
        "848x480 at a pitch of 3584 rounds up to 2 MiB"
    );
    let layout = slot.image_layout().unwrap();
    assert_eq!(
        (layout.planes, layout.pitch[0], layout.bytes),
        (1, 3584, 3584 * 480)
    );
    let mine = slot.dmabuf_fd().unwrap();
    assert_eq!(slot.commit_len(layout.bytes as usize), Published::Ok);
    let view = sub.take().unwrap();
    assert_eq!(view.image_layout(), Some(layout));
    let theirs = view.dmabuf_fd().unwrap();
    let target = |fd| std::fs::read_link(format!("/proc/self/fd/{fd}")).unwrap();
    assert!(
        target(theirs).to_string_lossy().contains("dmabuf"),
        "{:?}",
        target(theirs)
    );
    assert_ne!(
        mine, theirs,
        "each end holds its own descriptor of the slot"
    );
}

#[test]
fn a_slot_smaller_than_a_frame_of_the_format_is_refused() {
    if !integrated() {
        return;
    }
    let name = super::channel::topic("img_small");
    let made = format_publisher(&name, bgra(1920, 1200), 4 << 20);
    assert!(
        matches!(made, Err(Error::InvalidArgument(_))),
        "{:?}",
        made.err()
    );
    let odd = crate::ImageFormat {
        encoding: crate::ImageEncoding::Nv12,
        width: 1921,
        height: 1080,
    };
    let made = format_publisher(&name, odd, 0);
    assert!(
        matches!(made, Err(Error::InvalidArgument(_))),
        "{:?}",
        made.err()
    );
}

/// A byte-image slot has no layout to report.
#[test]
fn image_slots_without_a_format_have_no_layout() {
    if !integrated() {
        return;
    }
    let name = super::channel::topic("img_nofmt");
    let mut publisher = image_publisher(&name, 4, 0, Stream::default()).unwrap();
    let slot = publisher.loan().unwrap();
    assert_eq!(slot.image_layout(), None);
    assert!(slot.dmabuf_fd().is_some());
    assert_eq!(slot.commit_len(1), Published::Ok);
}

/// A generated type's frame starts with its scalars, so its image is not where the layout says.
#[test]
fn an_image_format_on_a_typed_channel_is_refused() {
    if !integrated() {
        return;
    }
    let config = PublisherConfig {
        alloc: Alloc::Slot,
        image_format: bgra(640, 480),
        ..PublisherConfig::default()
    };
    let name = super::channel::topic("img_typed");
    let made = Channel::create_config(
        &name,
        &config,
        NO_SCHEMA,
        MemoryPolicy::default(),
        Stream::default(),
        -1,
        Some(64),
    );
    assert!(
        matches!(made, Err(Error::InvalidArgument(_))),
        "{:?}",
        made.err()
    );
}
