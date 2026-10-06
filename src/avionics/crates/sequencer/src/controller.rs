//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight stages, from src/glider controller.py (`Stage`): their ids, their operator-facing names, which of
//! them are on the ground and which airborne. The rest of plan B's Controller (bring-up, the device directory,
//! arming, the operator hold) belongs to the firmware's control thread and its `StageCell`, not to this crate.

/// One flight stage. The ids are plan B's, so a warm-start crumb reads the same on both firmwares; 0 is the
/// NVS "no checkpoint" sentinel and never a live stage.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// On the pad: the sequencer is free to detect the launch.
    Setting = 1,
    /// Powered boost.
    Boosting = 2,
    /// Unpowered glide after separation.
    Gliding = 3,
    /// The endgame: the approach and the landing.
    Landing = 4,
    /// Landed; the flight is over.
    Done = 5,
}

impl Stage {
    /// Every stage, in id order.
    pub const ALL: [Stage; 5] = [
        Stage::Setting,
        Stage::Boosting,
        Stage::Gliding,
        Stage::Landing,
        Stage::Done,
    ];

    /// The operator-facing name (`stage`, `health`, `whoami`, and the stage column of sequencer.csv).
    pub fn name(self) -> &'static str {
        match self {
            Stage::Setting => "setting",
            Stage::Boosting => "boosting",
            Stage::Gliding => "gliding",
            Stage::Landing => "landing",
            Stage::Done => "done",
        }
    }

    /// The stage an operator names, None for a name that is not a stage.
    pub fn from_name(name: &str) -> Option<Stage> {
        Stage::ALL.into_iter().find(|stage| stage.name() == name)
    }

    /// The stage with this id, None for 0 (the sentinel) or an id past the last stage.
    pub fn from_id(id: u8) -> Option<Stage> {
        Stage::ALL.into_iter().find(|stage| *stage as u8 == id)
    }

    /// Nothing is flying: the stages active commands may run in (src/glider cc_client `_GROUND`).
    pub fn ground(self) -> bool {
        matches!(self, Stage::Setting | Stage::Done)
    }

    /// Off the ground: BOOSTING through LANDING (plan B `Stage.airborne`, the union of the powered boost and the
    /// passive glide). The stage machine's RSO backstop bounds exactly these: it once tested "not DONE" instead,
    /// which let a stale timer force DONE from SETTING on a board sitting on the pad.
    pub fn airborne(self) -> bool {
        !self.ground()
    }
}

#[cfg(test)]
mod tests {
    use super::Stage;

    #[test]
    fn names_round_trip_and_ids_match_plan_b() {
        for (index, stage) in Stage::ALL.into_iter().enumerate() {
            assert_eq!(stage as u8, index as u8 + 1);
            assert_eq!(Stage::from_name(stage.name()), Some(stage));
            assert_eq!(Stage::from_id(stage as u8), Some(stage));
        }
    }

    #[test]
    fn rejects_the_sentinel_and_unknown_names() {
        assert_eq!(Stage::from_id(0), None);
        assert_eq!(Stage::from_id(6), None);
        assert_eq!(Stage::from_name("auto"), None);
        assert_eq!(Stage::from_name("Setting"), None);
    }

    #[test]
    fn only_setting_and_done_are_ground() {
        let ground: [bool; 5] = Stage::ALL.map(Stage::ground);
        assert_eq!(ground, [true, false, false, false, true]);
    }

    #[test]
    fn airborne_is_boosting_through_landing() {
        let airborne: [bool; 5] = Stage::ALL.map(Stage::airborne);
        assert_eq!(airborne, [false, true, true, true, false]);
    }
}
