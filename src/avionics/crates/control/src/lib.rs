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
//!  * [`guidance`] -- the stage-dependent guidance law: the attitude setpoints and the heading error.

pub mod airspeed;
pub mod commons;
pub mod governor;
pub mod guidance;
pub mod mixer;
pub mod navigation;
pub mod pid;
pub mod wind;
