//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight math: one module per src/glider file it ports, f32 throughout (decision D18), and no clock read
//! anywhere -- a function that needs the time takes it as an argument, so the host tests drive it.
//!
//! For now only [`navigation`], the landing-zone geometry the Mission resolves its target and gates with.

pub mod navigation;
