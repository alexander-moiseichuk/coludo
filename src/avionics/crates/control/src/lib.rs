//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight math: one module per src/glider file it ports, f32 throughout (decision D18), and no clock read
//! anywhere -- a function that needs the time takes it as an argument, so the host tests drive it.
//!
//! The modules, each named after the plan B file it ports:
//!  * [`commons`] -- the shared primitives the flight math uses: the clamp, the servo neutral, the fin table and
//!    the barometric altitude (the stage machine's half of commons.py is in crates/sequencer);
//!  * [`pid`] -- one control axis: an angle error in, a fin deflection command out, with back-calculation;
//!  * [`mixer`] -- the three axis commands mixed into a whole-degree angle per fin (elevons and a rudder);
//!  * [`airspeed`] -- the hybrid airspeed estimate (pitot, accelerometer backbone, GNSS corrector);
//!  * [`governor`] -- the dynamic-pressure fin governor: the estimate's throttle and the mixer's 1/v^2 cap;
//!  * [`navigation`] -- the landing-zone geometry the Mission resolves its target and gates with;
//!  * [`wind`] -- the wind triangle from the GNSS ground velocity, the heading and the airspeed;
//!  * [`drift`] -- the GNSS pad drift (tasks/gnss_calib.py), frozen at launch to de-bias the wind;
//!  * [`flight`] -- the flight law (tasks/flight.py): governor, stage gate, guidance, PIDs, mixer, wind feed;
//!  * [`guidance`] -- the stage-dependent guidance law: the attitude setpoints and the heading error;
//!  * [`attitude`] -- the primary attitude: the complementary filter (gyro, gravity, track, magnetometer), its
//!    blind window and the handover to a source above or below it.
//!
//! And one with no plan B file, the port's own:
//!  * [`change`] -- a barometer's climb rate, `change`: the least-squares slope of its altitude over half a second.

pub mod airspeed;
pub mod attitude;
pub mod change;
pub mod commons;
pub mod drift;
pub mod flight;
pub mod governor;
pub mod guidance;
pub mod mixer;
pub mod navigation;
pub mod pid;
pub mod wind;
