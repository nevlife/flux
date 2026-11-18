//! An image slot's format and the layout flux gives it: rows of a pitch that is a multiple of
//! 256 bytes, which VIC requires of every plane, the planes one after another.

use crate::error::{Error, Result};

/// The row pitch VIC takes: `NvBufSurfaceImport` refuses any other.
pub const IMAGE_PITCH_ALIGN: u32 = 256;

/// Wire ABI: append only. The names are `sensor_msgs/Image` encodings.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ImageEncoding {
    #[default]
    None = 0,
    Bgra8 = 1,
    Rgba8 = 2,
    Mono8 = 3,
    Mono16 = 4,
    Nv12 = 5,
    Uyvy = 6,
    Yuyv = 7,
}

impl ImageEncoding {
    pub fn from_u32(v: u32) -> Option<ImageEncoding> {
        use ImageEncoding as E;
        [
            E::None,
            E::Bgra8,
            E::Rgba8,
            E::Mono8,
            E::Mono16,
            E::Nv12,
            E::Uyvy,
            E::Yuyv,
        ]
        .into_iter()
        .find(|e| *e as u32 == v)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImageFormat {
    pub encoding: ImageEncoding,
    pub width: u32,
    pub height: u32,
}

/// Where each plane of a frame starts in its slot and how long its rows are.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImageLayout {
    pub planes: u32,
    pub width: [u32; 2],
    pub height: [u32; 2],
    /// Bytes per row, the `step` a `sensor_msgs/Image` in this slot carries.
    pub pitch: [u32; 2],
    pub offset: [u32; 2],
    /// Bytes of a frame, every plane.
    pub bytes: u32,
}

impl ImageFormat {
    pub fn is_some(&self) -> bool {
        self.encoding != ImageEncoding::None
    }

    /// The layout, or why the format has none.
    pub fn layout(&self) -> Result<ImageLayout> {
        use ImageEncoding as E;
        let (w, h) = (self.width, self.height);
        let bad = |why: &str| {
            Err(Error::InvalidArgument(format!(
                "a {:?} image of {w}x{h} {why}",
                self.encoding
            )))
        };
        if w == 0 || h == 0 {
            return bad("has no pixels");
        }
        // (plane width in pixels, bytes per pixel, plane height) for each plane.
        let planes: &[(u32, u32, u32)] = match self.encoding {
            E::None => return bad("has no encoding"),
            E::Bgra8 | E::Rgba8 => &[(w, 4, h)],
            E::Mono8 => &[(w, 1, h)],
            E::Mono16 => &[(w, 2, h)],
            E::Uyvy | E::Yuyv if w % 2 == 0 => &[(w, 2, h)],
            E::Nv12 if w % 2 == 0 && h % 2 == 0 => &[(w, 1, h), (w / 2, 2, h / 2)],
            E::Uyvy | E::Yuyv | E::Nv12 => return bad("needs an even width and height"),
        };
        #[allow(clippy::cast_possible_truncation, reason = "one or two planes")]
        let mut l = ImageLayout {
            planes: planes.len() as u32,
            ..ImageLayout::default()
        };
        let mut at: u64 = 0;
        for (i, &(pw, bpp, ph)) in planes.iter().enumerate() {
            let pitch = u64::from(pw) * u64::from(bpp);
            let pitch = pitch.next_multiple_of(u64::from(IMAGE_PITCH_ALIGN));
            let (Ok(pitch), Ok(offset)) = (u32::try_from(pitch), u32::try_from(at)) else {
                return bad("does not fit a slot");
            };
            l.width[i] = pw;
            l.height[i] = ph;
            l.pitch[i] = pitch;
            l.offset[i] = offset;
            at += u64::from(pitch) * u64::from(ph);
        }
        let Ok(bytes) = u32::try_from(at) else {
            return bad("does not fit a slot");
        };
        l.bytes = bytes;
        Ok(l)
    }
}

#[cfg(test)]
mod tests {
    use super::{ImageEncoding as E, ImageFormat};

    fn pitches(e: E, w: u32, h: u32) -> Vec<u32> {
        let l = ImageFormat {
            encoding: e,
            width: w,
            height: h,
        }
        .layout()
        .unwrap();
        l.pitch[..l.planes as usize].to_vec()
    }

    /// The pitches `NvBufSurfaceCreate` picked on Orin (L4T R39.2.1) for the same images.
    #[test]
    fn the_pitch_is_the_one_the_jetson_library_picks() {
        assert_eq!(pitches(E::Bgra8, 1920, 1200), [7680]);
        assert_eq!(pitches(E::Bgra8, 848, 480), [3584]);
        assert_eq!(pitches(E::Bgra8, 1344, 376), [5376]);
        assert_eq!(pitches(E::Mono8, 1920, 1080), [2048]);
        assert_eq!(pitches(E::Mono8, 640, 480), [768]);
        assert_eq!(pitches(E::Mono16, 1344, 376), [2816]);
        assert_eq!(pitches(E::Nv12, 1920, 1200), [2048, 2048]);
        assert_eq!(pitches(E::Nv12, 848, 480), [1024, 1024]);
        assert_eq!(pitches(E::Uyvy, 848, 480), [1792]);
    }

    #[test]
    fn nv12_has_its_chroma_plane_after_the_luma_rows() {
        let l = ImageFormat {
            encoding: E::Nv12,
            width: 1280,
            height: 720,
        }
        .layout()
        .unwrap();
        assert_eq!(l.offset, [0, 1280 * 720]);
        assert_eq!(l.height, [720, 360]);
        assert_eq!(l.bytes, 1280 * 720 * 3 / 2);
    }

    #[test]
    fn an_odd_size_of_a_subsampled_format_and_no_encoding_are_refused() {
        for (e, w, h) in [
            (E::Nv12, 1921, 1080),
            (E::Nv12, 1920, 1081),
            (E::Yuyv, 641, 480),
            (E::None, 640, 480),
            (E::Bgra8, 0, 480),
        ] {
            let f = ImageFormat {
                encoding: e,
                width: w,
                height: h,
            };
            assert!(f.layout().is_err(), "{f:?}");
        }
    }
}
