use std::fmt;
use std::io;

#[derive(Debug)]
pub enum Error {
    /// The caller's own value is wrong.
    InvalidArgument(String),
    /// Another process wrote something this build cannot use. Retrying does not help.
    Mismatch(String),
    /// Not there yet (no publisher, segment not ready). Retrying may help.
    NotReady(String),
    PublisherExists(String),
    SubscribersFull(String),
    /// The kernel or the host refuses something flux needs.
    Unsupported(String),
    /// A `Source` or hook returned an error; the executor stopped on it.
    Callback(Box<dyn std::error::Error + Send + Sync>),
    Io {
        what: &'static str,
        name: String,
        source: io::Error,
    },
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub(crate) fn io(what: &'static str, name: &str, source: io::Error) -> Error {
        Error::Io {
            what,
            name: name.to_owned(),
            source,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Callback(e) => write!(f, "flux: callback failed: {e}"),
            Error::InvalidArgument(m)
            | Error::Mismatch(m)
            | Error::NotReady(m)
            | Error::Unsupported(m) => {
                write!(f, "flux: {m}")
            }
            Error::PublisherExists(name) => {
                write!(f, "flux: '{name}' already has a live publisher")
            }
            Error::SubscribersFull(name) => {
                write!(f, "flux: '{name}' has no free subscriber number")
            }
            Error::Io { what, name, source } => write!(f, "flux: {what} '{name}': {source}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io { source, .. } => Some(source),
            Error::Callback(e) => Some(e.as_ref()),
            _ => None,
        }
    }
}
