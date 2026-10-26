use std::mem::MaybeUninit;

/// A value kept in caller memory. The tag is the first byte, so zeroed storage is empty.
#[repr(C)]
pub(crate) struct Stored<T> {
    live: u8,
    value: MaybeUninit<T>,
}

impl<T> Stored<T> {
    pub(crate) fn get(&self) -> Option<&T> {
        // SAFETY: live is set only after value is written.
        (self.live != 0).then(|| unsafe { self.value.assume_init_ref() })
    }

    pub(crate) fn take(&mut self) -> Option<T> {
        if self.live == 0 {
            return None;
        }
        self.live = 0;
        // SAFETY: live was set, and clearing it first means the value is read once.
        Some(unsafe { self.value.assume_init_read() })
    }

    /// Drops what was held first: a view or loan left in place is released, not leaked.
    pub(crate) fn put(&mut self, v: Option<T>) {
        drop(self.take());
        if let Some(v) = v {
            self.value.write(v);
            self.live = 1;
        }
    }
}

/// `stored`, shared instead of exclusive; same contract.
pub(crate) unsafe fn stored_ref<'a, T, S>(p: *const S) -> &'a Stored<T> {
    // SAFETY: guaranteed by the caller.
    unsafe { &*p.cast::<Stored<T>>() }
}

/// # Safety
/// `p` points at zeroed or previously stored `S` that is laid out as `Stored<T>` (checked where
/// the storage type is declared) and is not aliased for 'a.
pub(crate) unsafe fn stored<'a, T, S>(p: *mut S) -> &'a mut Stored<T> {
    // SAFETY: guaranteed by the caller.
    unsafe { &mut *p.cast::<Stored<T>>() }
}
