//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The card's layout (doc/specs/avionics-port.md "SD and flash file layout", the session object): the ONE place
//! that maps (session, kind) to a directory, so no consumer builds a path of its own. Every path is relative to the
//! card's root (`/sdcard` on the board, the first partition) with `/` between its parts:
//!
//! ```text
//! sessions.csv                     GLOBAL_INDEX: every boot's session.csv rows (wrapped lines, no routing)
//! <session>/recordings/            RECORDINGS: the ring_saver's files of the session
//!   <session>_<stream>.csv  recorder.log  session.csv
//! <session>/<kind>/                any later kind, beside recordings/
//! videos/<session>/                VIDEOS: the camera's files of the session (the second partition's mount point)
//! _<label>/recordings/             an earlier boot's RESERVED label, escaped (see below)
//! ```
//!
//! Session first: each session keeps its own directory, so FAT never holds many files in one directory and a
//! damaged directory costs one session, not the collection. Videos are the one kind kept apart, under `videos/`:
//! on a two-partition card that directory is the second partition's mount point (`/sdcard/videos`), so video and the
//! ring_saver have a volume each; on a one-partition card it is a plain directory beside the sessions.
//!
//! **A session may not take a name the card's root holds for something else** ([`RESERVED`], in any case: FAT folds
//! case): `videos`, or its recordings would land in the videos directory, on the videos partition; and the M4
//! firmware's `recordings` and `video`, which the tools still read on a card that firmware wrote (a session
//! `recordings` would be one directory in both layouts). [`is_reserved`] says so where the label is chosen: the
//! firmware's session prefix falls back to the boot id with a warning. A label is checked only there, for the running
//! boot; the ring_saver also files earlier boots' blocks by their header's label, and the M4 firmware allowed these
//! names, so the layout itself files a reserved session under `_<label>` (`_videos/recordings/`). No session this
//! firmware names begins with `_` (a label is letters, digits and `-`, a boot id digits, the dated form begins with
//! a digit), so that directory is no other session's, and nothing of a session ever lands on the videos partition.
//!
//! Pure and host-tested: the saver routes every line through [`file_path`], the camera asks [`directory`] for its
//! directory, and the host tests and tools read the card by the same names.

use std::string::String;

/// The global index at the card's root: every boot's session.csv rows, as recorderd kept the Luckfox's
/// `/userdata/recordings/session.csv`. Its lines carry the routing `session.csv` (recorder_wire's `SESSION_INDEX`):
/// only the file's name differs, so it never reads as one session's index.
pub const GLOBAL_INDEX: &str = "sessions.csv";

/// The ring_saver's kind: `<session>/recordings`, which holds the session's streams, recorder.log and session.csv.
pub const RECORDINGS: &str = "recordings";

/// The camera's kind, kept apart: `videos/<session>`. Also the name of the second partition's mount point under the
/// card's root, so no session may take it ([`is_reserved`]).
pub const VIDEOS: &str = "videos";

/// The names at the card's root that are no session's: [`VIDEOS`], the videos partition's mount point, and the M4
/// firmware's root directories `recordings` and `video` (its `recordings/<session>/`, `video/<session>/`), which
/// tools/board_pull.sh still reads on a card that firmware wrote. Compared in any case ([`is_reserved`]).
pub const RESERVED: [&str; 3] = [VIDEOS, "recordings", "video"];

/// What a reserved session's directory begins with, before its label ([`directory`]): a byte no session this
/// firmware names begins with.
pub const RESERVED_ESCAPE: char = '_';

/// The directory of `session`'s `kind`, relative to the card's root: `videos/<session>` for [`VIDEOS`],
/// `<session>/<kind>` for [`RECORDINGS`] and any later kind. A reserved session ([`is_reserved`], an earlier boot's
/// label) is `_<session>` in either: `_videos/recordings`.
///
/// Args:
/// * `session` - the session label (the boot id `%06u`, a `recorder.session` label, or the dated form).
/// * `kind` - [`RECORDINGS`], [`VIDEOS`], or a later kind's name.
///
/// Returns the directory, without a trailing `/`.
pub fn directory(session: &str, kind: &str) -> String {
    let mut path = String::new();
    push_directory(&mut path, session, kind);
    path
}

/// Builds in `path` (emptied first) the path of the file `name` in `session`'s `kind` directory: [`directory`], then
/// `/<name>`. The ring_saver builds every line's path in one reused String this way, with no allocation once it has
/// grown.
///
/// Args:
/// * `path` - where the path is built; what it held is dropped.
/// * `session` - the session label.
/// * `kind` - [`RECORDINGS`], [`VIDEOS`], or a later kind's name.
/// * `name` - the file's name in that directory.
///
/// Returns nothing: the path is in `path`.
pub fn file_path(path: &mut String, session: &str, kind: &str, name: &str) {
    path.clear();
    push_directory(path, session, kind);
    path.push('/');
    path.push_str(name);
}

/// Whether `label` may not name a session: it is one of [`RESERVED`] in any case. FAT folds case, so `Videos` would
/// share the videos directory on a one-partition card, and on a two-partition card make a directory beside the mount
/// point that the board's HTTP listing names twice.
pub fn is_reserved(label: &str) -> bool {
    RESERVED.iter().any(|name| label.eq_ignore_ascii_case(name))
}

/// Appends `session`'s `kind` directory to `path`.
fn push_directory(path: &mut String, session: &str, kind: &str) {
    if kind == VIDEOS {
        path.push_str(VIDEOS);
        path.push('/');
        push_session(path, session);
    } else {
        push_session(path, session);
        path.push('/');
        path.push_str(kind);
    }
}

/// Appends `session`'s directory name to `path`: the session, after [`RESERVED_ESCAPE`] when it is reserved.
fn push_session(path: &mut String, session: &str) {
    if is_reserved(session) {
        path.push(RESERVED_ESCAPE);
    }
    path.push_str(session);
}

#[cfg(test)]
mod tests {
    use super::{GLOBAL_INDEX, RECORDINGS, VIDEOS, directory, file_path, is_reserved};

    #[test]
    fn each_kind_has_its_directory_and_the_index_its_place_at_the_root() {
        assert_eq!(directory("000276", RECORDINGS), "000276/recordings");
        assert_eq!(directory("000276", VIDEOS), "videos/000276");
        assert_eq!(
            directory("flightday", "photos"),
            "flightday/photos",
            "a later kind beside recordings"
        );
        assert_eq!(
            directory("20000101_000006_898573", RECORDINGS),
            "20000101_000006_898573/recordings"
        );
        assert_eq!(GLOBAL_INDEX, "sessions.csv");
        assert!(!GLOBAL_INDEX.contains('/'), "at the card's root");
    }

    #[test]
    fn a_file_path_is_built_in_place_and_drops_what_the_string_held() {
        let mut path = String::from("left over from the last line");
        file_path(&mut path, "000276", RECORDINGS, "000276_health.csv");
        assert_eq!(path, "000276/recordings/000276_health.csv");
        file_path(&mut path, "000276", RECORDINGS, "recorder.log");
        assert_eq!(path, "000276/recordings/recorder.log");
        file_path(&mut path, "000276", VIDEOS, "5321.h264");
        assert_eq!(path, "videos/000276/5321.h264");
        // negative: a kind named like a file is still a directory, never a file at the session's root
        file_path(&mut path, "000276", "session.csv", "x");
        assert_eq!(path, "000276/session.csv/x");
    }

    #[test]
    fn the_roots_other_names_are_reserved_in_any_case() {
        for reserved in [
            "videos",
            "Videos",
            "VIDEOS",
            "vIdEoS",
            "recordings",
            "Recordings",
            "video",
            "VIDEO",
        ] {
            assert!(is_reserved(reserved), "{reserved}");
        }
        // negative: labels that merely contain or resemble them, a later kind, ids and the escaped form may
        for allowed in [
            "videos2",
            "my-videos",
            "recording",
            "recordings1",
            "photos",
            "000276",
            "flightday",
            "",
        ] {
            assert!(!is_reserved(allowed), "{allowed}");
        }
        assert!(!is_reserved("_videos"), "the escaped form is no reserved name");
    }

    #[test]
    fn an_earlier_boots_reserved_label_is_filed_under_its_escape_never_on_the_videos_partition() {
        assert_eq!(directory("videos", RECORDINGS), "_videos/recordings");
        assert_eq!(directory("Videos", RECORDINGS), "_Videos/recordings", "its case kept");
        assert_eq!(directory("recordings", RECORDINGS), "_recordings/recordings");
        assert_eq!(directory("video", "photos"), "_video/photos");
        assert_eq!(directory("videos", VIDEOS), "videos/_videos");
        let mut path = String::new();
        file_path(&mut path, "videos", RECORDINGS, "videos_health.csv");
        assert_eq!(path, "_videos/recordings/videos_health.csv");
        assert!(!path.starts_with("videos/"), "under the videos mount point");
        // negative: a session that only resembles one is its own directory
        assert_eq!(directory("videos2", RECORDINGS), "videos2/recordings");
        assert_eq!(directory("_videos", RECORDINGS), "_videos/recordings");
    }
}
