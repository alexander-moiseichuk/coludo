//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The Glider Command Language: the operator's console commands, and the registry the board's components add
//! theirs to when they activate. After the moisture project's bench consoles, written exactly as the CC protocol
//! writes a line (doc/specs/cc-protocol.md): positional words, then `name=value` named parameters.
//!
//! ```text
//! command [sub] [word ...] name=value name=value
//! video start bars fps=25 kbps=8000
//! pace seconds=10 source=gptimer
//! panic confirm
//! ```
//!
//! **Components own their commands.** A component registers its commands when it activates (the SD card when it
//! mounts: `sdtest`; the camera when the config declares one and the card mounts: `video ...`; the diagnostics at
//! boot: `load`, `pace`; a driver when its part answers) and withdraws them when it goes. Linking or removing a
//! component therefore needs no edit anywhere else and no feature switch: `help` lists exactly what this board runs
//! right now.
//!
//! **One declaration per command:** its words, what it does, its named parameters (kind, range, default, help) and
//! its bare words. The parser, the validation and `help` all read it, so a command cannot accept what its help
//! does not list, and a typo is answered with the names it does take instead of being ignored.
//!
//! **Values** follow the CC encoding: a value with a space or `=` travels as `name=base64:<data>`, which
//! tools/avionics_console.py does for you, and the CC parser decodes it before a command sees it. Replies are CC lines (`ok ...` /
//! `err <code> ...`), so the contract runner and the hub parse them unchanged; the console tool renders them.
//!
//! **A command that takes the board away needs the word `confirm`** (moisture's `clear confirm`): a stray paste must
//! not panic the flight computer.
//!
//! **One point of injection.** The CC executor (commands/mod.rs) is the only dispatcher: the console and the hub
//! both hand it their lines, tagged with their [`Origin`]. It answers a CC command from its own table and hands
//! anything else here, so a GCL command is reachable from the hub as from the console -- under the same gates. Each
//! command declares its [`Gate`] (a change runs only in SETTING, as the CC commands that change the board do) and
//! its [`Reach`] (a command whose arguments must never cross the network -- `wifi` carries a password, and the hub
//! logs every line -- is console only).
//!
//! The registry is behind a lock: the console reads it a line at a time, components write it at activation --
//! neither is a hot path, and nothing in the flight loop ever touches it.

use std::collections::BTreeMap;
use std::sync::RwLock;

use serde_json::json;

/// What a named parameter accepts.
#[derive(Clone, Copy)]
pub enum Kind {
    /// A whole number in `low..=high`.
    Integer {
        /// The smallest value accepted.
        low: i64,
        /// The largest value accepted.
        high: i64,
    },
    /// Any text (a password, an SSID).
    Text,
    /// One of a fixed set of words.
    Choice(&'static [&'static str]),
}

/// One named parameter of a command: `name=value`.
pub struct Parameter {
    /// Its name, as typed before the `=`.
    pub name: &'static str,
    /// What it accepts.
    pub kind: Kind,
    /// Its value when not given; None makes it required.
    pub default: Option<&'static str>,
    /// One line for `help`.
    pub about: &'static str,
}

/// One bare word a command takes (`bars`, `confirm`).
pub struct Word {
    /// The word.
    pub name: &'static str,
    /// One line for `help`.
    pub about: &'static str,
}

/// The word that arms a command which takes the board away.
pub const CONFIRM: Word = Word {
    name: "confirm",
    about: "required: this command takes the board away",
};

/// Where a command may run (the CC commands' rule, doc/specs/avionics-port.md §6).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Gate {
    /// In every stage: a read, or a command the operator recovers with.
    Open,
    /// Only in SETTING: it changes the board, loads a core, or takes the board away.
    Setting,
    /// Only in SETTING and disarmed: a bus diagnostic, which holds a bus owner off its parts while it runs, so never
    /// in the last seconds on the pad (M9).
    Disarmed,
}

/// Who may send a command.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Reach {
    /// The console and the hub alike.
    Everywhere,
    /// The USB console only: its arguments must never cross the network (the hub logs every line).
    Console,
}

/// Where a line came from.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Origin {
    /// The USB console (UART0).
    Console,
    /// The CC hub, over Wi-Fi.
    Hub,
}

/// What a command does with its validated arguments; it answers one CC line.
pub type Run = Box<dyn Fn(&Arguments) -> String + Send + Sync>;

/// One command, as a component registers it.
pub struct Command {
    /// The component that registered it; [`withdraw`] removes a component's commands by this name.
    pub owner: &'static str,
    /// Its words: the command, and its subcommand when it has one (`video start`).
    pub words: &'static str,
    /// One line for `help`.
    pub about: &'static str,
    /// Where it may run.
    pub gate: Gate,
    /// Who may send it.
    pub reach: Reach,
    /// Its named parameters.
    pub parameters: &'static [Parameter],
    /// Its bare words; [`CONFIRM`] among them makes the command need it.
    pub bare_words: &'static [Word],
    /// What it does.
    pub run: Run,
}

/// A command's arguments, validated against its declaration and completed with the defaults.
pub struct Arguments {
    /// Each named parameter's value as text (defaults filled in).
    values: BTreeMap<&'static str, String>,
    /// The bare words given.
    words: Vec<&'static str>,
}

impl Arguments {
    /// An Integer parameter's value. Only declared names reach here and those were validated, so a miss is a
    /// declaration bug: it reads as 0 rather than taking the console down.
    pub fn integer(&self, name: &str) -> i64 {
        self.values.get(name).and_then(|value| value.parse().ok()).unwrap_or(0)
    }

    /// A Text or Choice parameter's value ('' for an undeclared name).
    pub fn text(&self, name: &str) -> &str {
        self.values.get(name).map_or("", String::as_str)
    }

    /// Whether the bare word was given.
    pub fn word(&self, name: &str) -> bool {
        self.words.contains(&name)
    }
}

/// The registered commands, in registration order.
static REGISTRY: RwLock<Vec<Command>> = RwLock::new(Vec::new());

/// The CC commands' names, listed by `help` beside the operator commands (they keep plan B's grammar).
static CC_NAMES: RwLock<Vec<&'static str>> = RwLock::new(Vec::new());

/// Add a component's commands; a command whose words are already taken replaces the old one (a component that
/// re-activates re-registers).
pub fn register(commands: Vec<Command>) {
    let Ok(mut registry) = REGISTRY.write() else {
        return; // a poisoned lock: a panic elsewhere already rebooted the board
    };
    for command in commands {
        registry.retain(|registered| registered.words != command.words);
        registry.push(command);
    }
}

/// Remove every command a component registered (it deactivated, or its part went away).
pub fn withdraw(owner: &str) {
    if let Ok(mut registry) = REGISTRY.write() {
        registry.retain(|command| command.owner != owner);
    }
}

/// Tell `help` which CC commands exist (the CC table is the binary's; plain names keep the dependency one-way).
pub fn set_cc_names(names: Vec<&'static str>) {
    if let Ok(mut cc_names) = CC_NAMES.write() {
        *cc_names = names;
    }
}

/// Answer one line if it names a GCL command (or `help`); None when it names none, so the caller answers
/// `err badcmd`.
///
/// * `origin` - who sent it: a [`Reach::Console`] command refuses the hub.
/// * `refusal` - why a change must not run now (the stage is not SETTING), or None in SETTING: a [`Gate::Setting`]
///   command answers `err unsafe <refusal>`, the CC commands' own answer.
/// * `diagnostic_refusal` - why a bus diagnostic must not run now (not SETTING, or armed), or None: a
///   [`Gate::Disarmed`] command answers `err unsafe <diagnostic_refusal>`.
pub fn handle(line: &str, origin: Origin, refusal: Option<&str>, diagnostic_refusal: Option<&str>) -> Option<String> {
    let message = cc_protocol::parse(line).ok()?;
    let name = message.command.as_deref()?;
    let registry = REGISTRY.read().ok()?;
    if name == "help" || name == "?" {
        return Some(help(&registry, &message.args));
    }
    // the longest match, whatever the registration order: `panic log` before a bare `panic`
    let sub = message.args.first().map(String::as_str);
    let found = registry
        .iter()
        .find(|command| {
            command
                .words
                .split_once(' ')
                .is_some_and(|(word, rest)| word == name && Some(rest) == sub)
        })
        .map(|command| (command, 1))
        .or_else(|| {
            registry
                .iter()
                .find(|command| command.words == name)
                .map(|command| (command, 0))
        });
    let Some((command, consumed)) = found else {
        // a known command word with a wrong or missing subcommand: say which ones exist
        let subcommands = subcommands(&registry, name);
        return (!subcommands.is_empty()).then(|| {
            failure(&format!(
                "{name} takes a subcommand: {} (help {name})",
                subcommands.join(", ")
            ))
        });
    };
    if command.reach == Reach::Console && origin != Origin::Console {
        return Some(cc_protocol::build(
            "err",
            &[
                "unsafe",
                &format!(
                    "{} is console only: its arguments must not cross the network",
                    command.words
                ),
            ],
            &[],
        ));
    }
    let refused = match command.gate {
        Gate::Open => None,
        Gate::Setting => refusal,
        Gate::Disarmed => diagnostic_refusal,
    };
    if let Some(refusal) = refused {
        return Some(cc_protocol::build("err", &["unsafe", refusal], &[]));
    }
    Some(match parse(command, &message, consumed) {
        Ok(arguments) => (command.run)(&arguments),
        Err(problem) => failure(&problem),
    })
}

/// The subcommands registered under a command word (`video` -> start, stop, stats).
fn subcommands(registry: &[Command], name: &str) -> Vec<&'static str> {
    registry
        .iter()
        .filter_map(|command| command.words.split_once(' '))
        .filter(|(word, _)| *word == name)
        .map(|(_, sub)| sub)
        .collect()
}

/// Validate a line's arguments against the command's declaration and fill in the defaults. The CC parser has
/// already split them: `name=value` tokens (values decoded) into `named`, every other token into `args`.
fn parse(command: &Command, message: &cc_protocol::Message, consumed: usize) -> Result<Arguments, String> {
    let mut values = BTreeMap::new();
    for (name, value) in &message.named {
        let Some(parameter) = command.parameters.iter().find(|parameter| parameter.name == name) else {
            return Err(format!("{} takes no {name}= ({})", command.words, listing(command)));
        };
        check(parameter, value)?;
        values.insert(parameter.name, value.clone());
    }
    for parameter in command.parameters {
        if !values.contains_key(parameter.name) {
            let default = parameter
                .default
                .ok_or_else(|| format!("{} needs {}= ({})", command.words, parameter.name, parameter.about))?;
            values.insert(parameter.name, default.to_string());
        }
    }
    let mut words = Vec::new();
    for token in message.args.iter().skip(consumed) {
        let Some(word) = command.bare_words.iter().find(|word| word.name == token) else {
            return Err(format!(
                "{} takes no word {token} ({})",
                command.words,
                listing(command)
            ));
        };
        words.push(word.name);
    }
    let needs_confirm = command.bare_words.iter().any(|word| word.name == CONFIRM.name);
    if needs_confirm && !words.contains(&CONFIRM.name) {
        return Err(format!(
            "{} needs the word confirm: it takes the board away",
            command.words
        ));
    }
    Ok(Arguments { values, words })
}

/// Whether `value` is one `parameter` accepts; the reason when it is not.
fn check(parameter: &Parameter, value: &str) -> Result<(), String> {
    match parameter.kind {
        Kind::Integer { low, high } => match value.parse::<i64>() {
            Ok(number) if (low..=high).contains(&number) => Ok(()),
            _ => Err(format!(
                "{}= takes a whole number {low}..{high}, not {value}",
                parameter.name
            )),
        },
        Kind::Text => Ok(()),
        Kind::Choice(choices) if choices.contains(&value) => Ok(()),
        Kind::Choice(choices) => Err(format!(
            "{}= takes {}, not {value}",
            parameter.name,
            choices.join(" or ")
        )),
    }
}

/// What a command takes, for an error line: `it takes bars width= height= ...`.
fn listing(command: &Command) -> String {
    let words = command.bare_words.iter().map(|word| word.name.to_string());
    let parameters = command
        .parameters
        .iter()
        .map(|parameter| format!("{}=", parameter.name));
    let all: Vec<String> = words.chain(parameters).collect();
    if all.is_empty() {
        "it takes nothing".into()
    } else {
        format!("it takes {}", all.join(" "))
    }
}

/// `help [command]`: the registered commands with their parameters and their component, plus the CC commands by
/// name. JSON, so the console tool can render it and complete from it.
fn help(registry: &[Command], wanted: &[String]) -> String {
    let wanted = wanted.join(" ");
    let commands: Vec<_> = registry
        .iter()
        .filter(|command| {
            wanted.is_empty() || command.words == wanted || command.words.starts_with(&format!("{wanted} "))
        })
        .map(|command| {
            let parameters: Vec<_> = command
                .parameters
                .iter()
                .map(|parameter| {
                    json!({"name": parameter.name, "default": parameter.default, "about": parameter.about,
                           "accepts": accepts(parameter.kind)})
                })
                .collect();
            let words: Vec<_> = command
                .bare_words
                .iter()
                .map(|word| json!({"name": word.name, "about": word.about}))
                .collect();
            json!({"command": command.words, "component": command.owner, "about": command.about,
                   "setting_only": command.gate != Gate::Open,
                   "disarmed_only": command.gate == Gate::Disarmed, "console_only": command.reach == Reach::Console,
                   "parameters": parameters, "words": words})
        })
        .collect();
    if commands.is_empty() && !wanted.is_empty() {
        return failure(&format!("no command {wanted} on this board (help lists them)"));
    }
    let cc = CC_NAMES.read().map(|names| names.clone()).unwrap_or_default();
    let body = json!({"grammar": "command [sub] [word ...] name=value ...", "commands": commands, "cc": cc});
    cc_protocol::build("ok", &[&body.to_string()], &[])
}

/// What a parameter kind accepts, for `help`.
fn accepts(kind: Kind) -> String {
    match kind {
        Kind::Integer { low, high } => format!("{low}..{high}"),
        Kind::Text => "text".into(),
        Kind::Choice(choices) => choices.join("|"),
    }
}

/// A GCL failure line: `err badargs <why>`, the CC code for malformed arguments.
fn failure(problem: &str) -> String {
    cc_protocol::build("err", &["badargs", problem], &[])
}
