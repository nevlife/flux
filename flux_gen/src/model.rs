use std::path::PathBuf;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArrayKind {
    Scalar,
    Fixed,
    /// `T[<=N]`: parsed so flatten can name it, never rides the wire.
    Bounded,
    Dynamic,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Field {
    pub name: String,
    /// A primitive, "string"/"wstring", or a message name.
    pub type_name: String,
    pub kind: ArrayKind,
    /// N for Fixed and Bounded, else 0.
    pub size: usize,
    /// `@optional`: parsed so flatten can name it, never rides the wire.
    pub optional: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ConstValue {
    Str(String),
    Bool(bool),
    Int(i128),
    Float(f64),
}

/// `TYPE NAME=value`: a named symbol beside the adapter, never wire data or fingerprinted.
#[derive(Clone, Debug, PartialEq)]
pub struct Constant {
    pub name: String,
    pub type_name: String,
    pub value: ConstValue,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Message {
    pub name: String,
    pub fields: Vec<Field>,
    /// Bare type references resolve against this package first.
    pub package: Option<String>,
    pub path: Option<PathBuf>,
    pub constants: Vec<Constant>,
}

impl Message {
    pub fn ident(&self) -> String {
        match &self.package {
            Some(p) => format!("{p}/{}", self.name),
            None => self.name.clone(),
        }
    }
}
