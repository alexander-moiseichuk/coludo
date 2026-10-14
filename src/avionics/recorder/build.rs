//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! ESP-IDF's link arguments and cfgs for the flight recorder (esp-idf-sys, through embuild).

fn main() {
    embuild::espidf::sysenv::output();
}
