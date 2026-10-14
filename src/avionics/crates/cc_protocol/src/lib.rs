//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! CC <-> board line protocol (doc/specs/cc-protocol.md), wire-identical to src/glider/cc_protocol.py.
//!
//! One newline-delimited message per line: `<command> [params...]`. Tokens are whitespace-separated, so
//! there is NO quoting or escaping. A param value is one of:
//!  * a bare token -> a simple value with no spaces (e.g. `3000`, `taster`, `192.168.10.1`)
//!  * `base64:<data>` -> anything else: spaces, quotes, JSON, binary
//!
//! Both sides know each command's schema, so the parser does not guess types: a bare token stays a string
//! and the receiver converts numerics itself. Named params are `key=value`; everything else is positional.
//! The command is lowercased; values keep their case. [`parse`] handles requests and responses
//! (ok/err/pong/iam) alike.

#![no_std]

extern crate alloc;

use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use base64::Engine;

/// The prefix that marks a base64-carried value.
const PREFIX: &str = "base64:";

/// A parsed protocol message (request or response).
///
/// A board receives `command params` (Control has stripped the routing board id), so `args` are the
/// positional params and `named` the `key=value` ones.
#[derive(Debug, PartialEq, Eq)]
pub struct Message {
    /// The first token, lowercased; None for an empty line.
    pub command: Option<String>,
    /// The positional params, decoded.
    pub args: Vec<String>,
    /// The `key=value` params, decoded.
    pub named: BTreeMap<String, String>,
}

/// Why a line or token would not decode.
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    /// A `base64:` token whose payload is not valid base64.
    Base64,
    /// A `base64:` token whose payload is not UTF-8.
    Utf8,
}

/// Is `text` safe to send bare: non-empty, not prefixed, and only characters that never need escaping?
fn is_simple(text: &str) -> bool {
    !text.is_empty()
        && !text.starts_with(PREFIX)
        && text
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._-/+:".contains(&byte))
}

/// Encode a value into one whitespace-free wire token: bare when already safe, else `base64:`-prefixed.
///
/// Integers and booleans go through their `to_string()` / `"true"` forms first, exactly as the Python side
/// does, and are always safe.
pub fn encode(text: &str) -> String {
    if is_simple(text) {
        return text.to_string();
    }
    let mut token = String::from(PREFIX);
    base64::engine::general_purpose::STANDARD.encode_string(text, &mut token);
    token
}

/// Decode a wire token back to a string (base64-decoded when `base64:`-prefixed, else the token as-is).
pub fn decode(token: &str) -> Result<String, Error> {
    match token.strip_prefix(PREFIX) {
        None => Ok(token.to_string()),
        Some(data) => {
            let bytes = base64::engine::general_purpose::STANDARD
                .decode(data)
                .map_err(|_| Error::Base64)?;
            String::from_utf8(bytes).map_err(|_| Error::Utf8)
        }
    }
}

/// Parse a protocol line (request or response); the command is None for an empty line.
pub fn parse(line: &str) -> Result<Message, Error> {
    let mut tokens = line.split_whitespace();
    let mut message = Message {
        command: None,
        args: Vec::new(),
        named: BTreeMap::new(),
    };
    let Some(command) = tokens.next() else {
        return Ok(message);
    };
    message.command = Some(command.to_lowercase());
    for token in tokens {
        if token.starts_with(PREFIX) {
            message.args.push(decode(token)?); // an encoded positional (may contain '=')
            continue;
        }
        match token.find('=') {
            Some(equals) if equals > 0 => {
                message
                    .named
                    .insert(token[..equals].to_string(), decode(&token[equals + 1..])?);
            }
            _ => message.args.push(decode(token)?),
        }
    }
    Ok(message)
}

/// Build a protocol line from a command, its positional values and its `key=value` params.
pub fn build(command: &str, args: &[&str], named: &[(&str, &str)]) -> String {
    let mut line = String::from(command);
    for value in args {
        line.push(' ');
        line.push_str(&encode(value));
    }
    for (key, value) in named {
        line.push(' ');
        line.push_str(key);
        line.push('=');
        line.push_str(&encode(value));
    }
    line
}
