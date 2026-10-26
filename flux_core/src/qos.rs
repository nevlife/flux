//! Consumer `QoS`. Names follow the ROS 2 `rmw` `QoS` where flux has the same knob.

use crate::error::{Error, Result};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Durability {
    /// 0 = volatile, N = `transient_local(N)`.
    pub replay: u32,
}

impl Durability {
    pub const fn volatile() -> Durability {
        Durability { replay: 0 }
    }
    pub const fn transient_local(n: u32) -> Durability {
        Durability { replay: n }
    }
}

/// A setter refuses a value wrong on its own. `validate` is the one check that needs two fields.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QoS {
    depth: u32,
    durability: Durability,
    max_borrow: u32,
}

impl Default for QoS {
    fn default() -> Self {
        QoS {
            depth: 8,
            durability: Durability::volatile(),
            max_borrow: 1,
        }
    }
}

impl QoS {
    /// `depth`: frames this consumer may fall behind the newest.
    pub fn new(depth: u32) -> Result<QoS> {
        QoS::default().keep_last(depth)
    }

    fn keep_last(mut self, depth: u32) -> Result<QoS> {
        if depth == 0 {
            return Err(Error::InvalidArgument(
                "depth must be >= 1; depth=1 means the newest frame only".into(),
            ));
        }
        self.depth = depth;
        Ok(self)
    }

    #[must_use]
    pub fn transient_local(mut self, n: u32) -> QoS {
        self.durability = Durability::transient_local(n);
        self
    }

    /// Views held at once. A held view keeps its slot from being rewritten.
    pub fn max_borrow(mut self, n: u32) -> Result<QoS> {
        if n == 0 {
            return Err(Error::InvalidArgument(
                "max_borrow must be >= 1: a consumer that may hold no view can never take one"
                    .into(),
            ));
        }
        self.max_borrow = n;
        Ok(self)
    }

    pub fn depth(&self) -> u32 {
        self.depth
    }
    pub fn durability(&self) -> Durability {
        self.durability
    }
    pub fn borrow_limit(&self) -> u32 {
        self.max_borrow
    }

    pub(crate) fn validate(&self) -> Result<()> {
        if self.durability.replay > self.depth {
            return Err(Error::InvalidArgument(
                "durability=transient_local(n) needs n <= depth: replayed frames arrive through \
                 the same lag window"
                    .into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MemoryPolicy {
    pub precommit: bool,
    /// Implies `precommit`: mlock populates what it locks.
    pub lock: bool,
}
