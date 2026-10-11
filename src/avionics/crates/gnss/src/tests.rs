//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the GNSS receiver, positive and negative, mostly on test_gnss.py's own sentences and the bursts it
//! captured on the 2026-10-04 bench. The plan B comparison itself is tests/golden.rs.

use super::drivers::{atgm336h, neo6mv2};
use super::{
    Antenna, Event, Gnss, Malformed, Sentence, SkyFrame, SkyRow, Update, checksum_ok, degrees_e7, hundredths, nmea,
    python_float, python_int,
};
use control::navigation::Position;
use sequencer::Stage;

/// A valid NMEA sentence around `body`, without the line ending (test_gnss's `_line`).
fn line(body: &str) -> String {
    String::from_utf8(nmea(body)).unwrap().trim_end().to_owned()
}

/// test_gnss's canned RMC: 48 07.038 N, 011 31.000 E, 22.4 knots, course 84.4.
const RMC: &str = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";

/// test_gnss's ATGM burst with a fix (session 5, the small active antenna).
const ATGM_BURST: [&str; 10] = [
    "$GNGSA,A,3,01,02,04,07,08,09,14,17,22,30,,,1.6,0.9,1.3,1*3C",
    "$GNGSA,A,3,11,19,21,22,29,36,39,,,,,,1.6,0.9,1.3,4*37",
    "$GPGSV,4,1,13,01,37,138,26,02,48,105,20,04,16,182,19,07,73,337,27,0*60",
    "$GPGSV,4,2,13,08,48,033,20,09,32,214,40,14,23,294,38,17,21,228,40,0*65",
    "$GPGSV,4,3,13,22,09,281,32,27,11,043,,30,36,320,40,194,,,25,0*69",
    "$GPGSV,4,4,13,197,,,26,0*5C",
    "$BDGSV,2,1,07,11,66,159,26,19,07,202,30,21,05,299,30,22,15,252,44,0*73",
    "$BDGSV,2,2,07,29,48,192,40,36,59,003,26,39,23,312,37,0*4B",
    "$GNRMC,001847.900,A,4807.03800,N,01131.00000,E,0.00,0.00,051026,,,A,V*0B",
    "$GPTXT,01,01,01,ANTENNA OK*35",
];

/// The same ATGM cold, before its first fix: mode 1, nothing used, one GPS satellite.
const ATGM_COLD: [&str; 4] = [
    "$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,1*01",
    "$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,4*04",
    "$GPGSV,1,1,01,01,,,39,0*6F",
    "$BDGSV,1,1,00,0*74",
];

/// A NEO-6M burst (session 6): NMEA 2.3, one GSA with no system id, GSV without a signal id.
const NEO_BURST: [&str; 4] = [
    "$GPGSA,A,3,07,02,08,01,46,30,09,14,17,04,27,22,1.99,0.98,1.74*06",
    "$GPGSV,3,1,12,01,43,130,27,02,50,092,20,04,09,178,32,07,81,335,25*79",
    "$GPGSV,3,2,12,08,41,035,22,09,26,208,39,14,27,300,42,17,27,233,38*72",
    "$GPGSV,3,3,12,22,13,286,40,27,05,045,12,30,43,319,40,46,29,248,43*79",
];

/// A second, in microseconds.
const SECOND: u64 = 1_000_000;

/// Parse every sentence at `now_us`.
fn feed(gnss: &mut Gnss, sentences: &[&str], now_us: u64) {
    for sentence in sentences {
        gnss.parse(sentence, now_us);
    }
}

/// A sky row from its eight columns.
fn row(columns: [i32; 8]) -> SkyRow {
    let antenna = [Antenna::Unknown, Antenna::Ok, Antenna::Open, Antenna::Short][columns[7] as usize];
    SkyRow {
        in_view: columns[0],
        used: columns[1],
        mode: columns[2],
        cn0: [columns[3], columns[4], columns[5], columns[6]],
        antenna,
    }
}

#[test]
fn checksums_verify_and_refuse() {
    assert!(checksum_ok(&line("GPRMC,123519,A")));
    assert!(!checksum_ok("$GPRMC,123519,A*00"));
    for sentence in ATGM_BURST.iter().chain(&ATGM_COLD).chain(&NEO_BURST) {
        assert!(checksum_ok(sentence), "{sentence}");
    }
    assert!(!checksum_ok("$GPRMC,123519,A"), "no star: unverifiable");
    assert!(!checksum_ok("$GPRMC,123519,A*"), "nothing after the star");
    assert!(!checksum_ok("*"));
    assert!(!checksum_ok(""));
    assert!(!checksum_ok("$GPRMC*G1"), "not hex");
    assert!(checksum_ok("$*00"), "nothing between: XOR 0");
}

#[test]
fn checksums_read_the_hex_as_python_int_does() {
    // 'A' ^ 'B' is 3: one digit, either case, a sign and spaces are int(.., 16)'s
    assert!(checksum_ok("$AB*3"));
    assert!(checksum_ok("$AB*03"));
    assert!(checksum_ok("$AB* 3"));
    assert!(checksum_ok("$AB*3 "));
    assert!(checksum_ok("$AB*+3"));
    assert!(!checksum_ok("$AB*-3"));
    assert!(checksum_ok("$*-0"));
    assert!(checksum_ok(
        "$GNRMC,001847.900,A,4807.03800,N,01131.00000,E,0.00,0.00,051026,,,A,V*0b"
    ));
    assert!(
        checksum_ok(&(line(RMC) + "XYZ")),
        "only two characters after the star count"
    );
    // a sentence that is not ASCII fails, whatever its bytes XOR to
    assert!(!checksum_ok("$GPTXT,\u{e9}*00"));
}

#[test]
fn degrees_parse_ddmm_into_units() {
    assert_eq!(degrees_e7("4807.038", "N"), Ok(Some(481_173_000)));
    assert_eq!(degrees_e7("01131.000", "W"), Ok(Some(-115_166_667)));
    assert_eq!(degrees_e7("01131.00000", "E"), Ok(Some(115_166_667)));
    assert_eq!(degrees_e7("9000.0000", "S"), Ok(Some(-900_000_000)));
    assert_eq!(degrees_e7("18000.0000", "W"), Ok(Some(-1_800_000_000)));
    assert_eq!(degrees_e7("4807.", "N"), Ok(Some(481_166_667)));
    assert_eq!(degrees_e7("123.5", "N"), Ok(Some(13_916_667)), "one degree digit");
    assert_eq!(
        degrees_e7("4875.0", "N"),
        Ok(Some(492_500_000)),
        "minutes past 60 as they come"
    );
    assert_eq!(
        degrees_e7("4807.038", "s"),
        Ok(Some(481_173_000)),
        "only S and W are negative"
    );
    assert_eq!(degrees_e7("", "N"), Ok(None));
    // half a unit rounds up, a third rounds down, and a tenth decimal is beyond a unit
    assert_eq!(degrees_e7("0000.000003", "N"), Ok(Some(1)));
    assert_eq!(degrees_e7("0000.000002", "S"), Ok(Some(0)));
    assert_eq!(degrees_e7("4807.0380000009", "N"), Ok(Some(481_173_000)));
}

#[test]
fn degrees_refuse_what_is_not_ddmm() {
    for value in [
        "4807",
        "48.5",
        "7.5",
        ".5",
        "-4807.038",
        "+4807.038",
        "4807.038e1",
        "48a7.038",
        "4807.0a8",
        "4807..0",
        " 4807.038",
        "21500.0",
        "99959.99999",
    ] {
        assert_eq!(degrees_e7(value, "N"), Err(Malformed), "{value}");
    }
    assert_eq!(
        degrees_e7("21400.0", "W"),
        Ok(Some(-2_140_000_000)),
        "just inside an i32"
    );
}

#[test]
fn fields_read_as_micropython_reads_them() {
    assert_eq!(python_int(" 08\t"), Ok(8));
    assert_eq!(python_int("+3"), Ok(3));
    assert_eq!(python_int("1_0"), Err(Malformed), "underscores are Python's, not here");
    assert_eq!(python_int("3.0"), Err(Malformed));
    assert_eq!(python_int(""), Err(Malformed));
    assert_eq!(python_float(" 22.4 "), Ok(22.4));
    assert_eq!(python_float("1e2"), Ok(100.0));
    assert!(python_float("nan").unwrap().is_nan());
    assert_eq!(python_float("east"), Err(Malformed));
}

#[test]
fn hdop_reads_as_exact_hundredths() {
    assert_eq!(hundredths("0.29"), Ok(29), "plan B's float gave 28");
    assert_eq!(hundredths("1.15"), Ok(115), "plan B's float gave 114");
    assert_eq!(hundredths("0.9"), Ok(90));
    assert_eq!(hundredths("99.99"), Ok(9999));
    assert_eq!(hundredths("1.239"), Ok(123), "truncated, as int() truncates");
    assert_eq!(hundredths("5."), Ok(500));
    assert_eq!(hundredths(".5"), Ok(50));
    assert_eq!(hundredths("-0.5"), Ok(-50));
    for text in ["", ".", "-", "nan", "1e1", "1.2.3", "x"] {
        assert_eq!(hundredths(text), Err(Malformed), "{text}");
    }
}

#[test]
fn command_sentences_are_framed_with_their_checksum() {
    assert_eq!(
        nmea("PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0"),
        b"$PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0*02\r\n"
    );
    assert_eq!(nmea("PUBX,40,GSA,0,50,0,0,0,0"), b"$PUBX,40,GSA,0,50,0,0,0,0*7B\r\n");
    assert_eq!(nmea(""), b"$*00\r\n");
}

#[test]
fn the_atgm_init_is_the_bench_s() {
    assert_eq!(
        atgm336h::commands(10)[4],
        b"$PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0*02\r\n"
    );
    let expected = |bodies: [&str; 5]| bodies.map(nmea);
    assert_eq!(
        atgm336h::commands(5),
        expected([
            "PCAS03,5,0,0,0,1,0,0,0,0,0,,,0,0",
            "PCAS02,200",
            "PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0",
            "PMTK220,200",
            "PCAS03,5,0,50,50,1,0,0,50,0,0,,,0,0",
        ])
    );
    assert_eq!(
        atgm336h::commands(1),
        expected([
            "PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0",
            "PCAS02,1000",
            "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0",
            "PMTK220,1000",
            "PCAS03,1,0,10,10,1,0,0,10,0,0,,,0,0",
        ])
    );
    assert_eq!(
        atgm336h::commands(0),
        atgm336h::commands(1),
        "no rate is 1 Hz, never a divide by zero"
    );
    assert_eq!(atgm336h::commands(-7), atgm336h::commands(1));
    assert_eq!(
        atgm336h::commands(20)[4],
        nmea("PCAS03,20,0,99,99,1,0,0,99,0,0,,,0,0"),
        "capped at 99"
    );
    assert_eq!(
        atgm336h::commands(i32::MAX)[4],
        nmea(&format!("PCAS03,{},0,99,99,1,0,0,99,0,0,,,0,0", i32::MAX))
    );
    let switch = atgm336h::sky_switch(10);
    assert_eq!(switch.off, b"$PCAS03,10,0,0,0,1,0,0,0,0,0,,,0,0*32\r\n");
    assert_eq!(
        switch.frame(SkyFrame::On),
        b"$PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0*02\r\n"
    );
    assert_eq!(
        atgm336h::sky_switch(5).on,
        b"$PCAS03,5,0,50,50,1,0,0,50,0,0,,,0,0*33\r\n"
    );
}

#[test]
fn ubx_frames_carry_the_fletcher_checksum() {
    let rate = neo6mv2::ubx(0x06, 0x08, &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]);
    assert_eq!(
        rate,
        [
            0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39
        ]
    );
    assert_eq!(
        neo6mv2::ubx(0x0A, 0x04, &[]),
        [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34]
    );
}

#[test]
#[should_panic(expected = "fits its u16 length")]
fn a_ubx_payload_past_its_length_field_is_refused() {
    neo6mv2::ubx(0x06, 0x08, &vec![0; 65536]);
}

#[test]
fn the_neo_init_is_the_bench_s_plus_the_airborne_model() {
    let mut navigation = vec![0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x01, 0x00, 0x08];
    navigation.extend_from_slice(&[0; 33]);
    navigation.extend_from_slice(&[0x57, 0xF8]);
    let frames = neo6mv2::commands(5);
    assert_eq!(frames[7], navigation);
    assert_eq!(frames[8], b"$PUBX,40,GSA,0,50,0,0,0,0*7B\r\n");
    assert_eq!(frames[9], b"$PUBX,40,GSV,0,50,0,0,0,0*6C\r\n");
    assert_eq!(frames[1], nmea("PUBX,40,GGA,0,5,0,0,0,0"));
    assert_eq!(frames[6], neo6mv2::ubx(0x06, 0x08, &[200, 0, 1, 0, 1, 0]));
    assert_eq!(neo6mv2::commands(0), neo6mv2::commands(1));
    assert_eq!(
        neo6mv2::commands(25)[8],
        nmea("PUBX,40,GSA,0,250,0,0,0,0"),
        "a byte still holds 250"
    );
    assert_eq!(
        neo6mv2::commands(30)[9],
        nmea("PUBX,40,GSV,0,255,0,0,0,0"),
        "capped at a byte"
    );
    assert_eq!(neo6mv2::commands(30)[6], neo6mv2::ubx(0x06, 0x08, &[33, 0, 1, 0, 1, 0]));
    let switch = neo6mv2::sky_switch(10);
    assert_eq!(
        switch.off,
        b"$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"
    );
    assert_eq!(
        switch.on,
        b"$PUBX,40,GSA,0,100,0,0,0,0*4F\r\n$PUBX,40,GSV,0,100,0,0,0,0*58\r\n"
    );
}

#[test]
fn rmc_pushes_the_fix_speed_and_course() {
    let mut gnss = Gnss::new();
    let update = gnss.parse(&line(RMC), 0);
    let position = Position {
        latitude_e7: 481_173_000,
        longitude_e7: 115_166_667,
    };
    assert_eq!(update.position, Some(position));
    assert!((update.speed_ms.unwrap() - 22.4 * 0.514444).abs() < 1e-5);
    assert_eq!(update.course_deg, Some(84.4));
    let fix_row = update.fix_row.unwrap();
    assert_eq!(
        (fix_row.position, fix_row.speed_knots, fix_row.course_deg),
        (position, 22.4, 84.4)
    );
    assert!(gnss.fix());
    // an empty speed is 0 and an empty course pushes none
    let update = gnss.parse(&line("GNRMC,1,A,3352.12345,S,15112.54321,W,,,051026,,,A"), 0);
    assert_eq!((update.speed_ms, update.course_deg), (Some(0.0), None));
    assert_eq!(update.fix_row.unwrap().course_deg, 0.0);
}

#[test]
fn a_void_or_unverified_rmc_moves_nothing() {
    let mut gnss = Gnss::new();
    gnss.parse(&line(RMC), 0);
    let void = gnss.parse(&line("GPRMC,123520,V,,,,,,,230394,,"), 0);
    assert_eq!(void.position, None);
    assert!(!gnss.fix());
    assert_eq!(
        gnss.parse("$GPRMC,123521,A,1234.000,N,01234.000,E,0,0,230394,,*00", 0),
        Update::default()
    );
    assert!(!gnss.fix(), "a bad checksum changes nothing");
    assert_eq!(gnss.parse(&line(RMC)[1..], 0), Update::default(), "no '$'");
}

#[test]
fn a_malformed_field_keeps_what_came_before_it() {
    let mut gnss = Gnss::new();
    // the fix flag is set before the latitude raises
    let update = gnss.parse(&line("GPRMC,1,A,48x7.038,N,01131.000,E,022.4,084.4,230394,003.1,W"), 0);
    assert_eq!(update, Update::default());
    assert!(gnss.fix());
    // the position goes out before the speed raises; nothing after it
    let update = gnss.parse(&line("GPRMC,1,A,4807.038,N,01131.000,E,fast,084.4,230394,003.1,W"), 0);
    assert!(update.position.is_some());
    assert_eq!((update.speed_ms, update.fix_row), (None, None));
    // GGA's quality parses, its satellites raise: the quality stays
    gnss.parse(&line("GPGGA,1,4807.038,N,01131.000,E,2,y,0.9,545.4,M,46.9,M,,"), 0);
    assert_eq!((gnss.fix_quality(), gnss.satellites()), (2, 0));
}

#[test]
fn gga_gives_the_altitude_the_elevation_and_the_quality() {
    let mut gnss = Gnss::new();
    let first = gnss.parse(
        &line("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
        0,
    );
    assert_eq!((first.altitude_m, first.elevation_m), (Some(545.4), Some(0.0)));
    assert_eq!((gnss.fix_quality(), gnss.satellites(), gnss.hdop()), (1, 8, 0.9));
    let gga_row = first.gga_row.unwrap();
    assert_eq!((gga_row.quality, gga_row.satellites, gga_row.hdop_centi), (1, 8, 90));
    let second = gnss.parse(
        &line("GPGGA,123520,4807.038,N,01131.000,E,1,08,0.29,550.4,M,46.9,M,,"),
        0,
    );
    assert!((second.elevation_m.unwrap() - 5.0).abs() < 1e-3);
    assert_eq!(second.gga_row.unwrap().hdop_centi, 29);
    // a quality-only GGA (no fix yet): zeros in the row, no altitude pushed
    let quality = gnss.parse(&line("GPGGA,123530,,,,,0,00,99.99,,M,,M,,"), 0);
    assert_eq!((quality.altitude_m, quality.elevation_m), (None, None));
    let gga_row = quality.gga_row.unwrap();
    assert_eq!(
        (gga_row.altitude_m, gga_row.elevation_m, gga_row.hdop_centi),
        (0.0, 0.0, 9999)
    );
    // too few fields: only the arrival counts
    assert_eq!(gnss.parse(&line("GPGGA,1,2,3"), 0).gga_row, None);
}

#[test]
fn outages_are_evented_once_per_edge_per_sentence() {
    let mut gnss = Gnss::new();
    assert!(gnss.sweep(100 * SECOND).is_empty(), "never heard: nothing lost");
    gnss.parse(&line(RMC), 10 * SECOND);
    gnss.parse(
        &line("GPGGA,1,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
        10 * SECOND,
    );
    assert!(gnss.sweep(11 * SECOND).is_empty(), "1 s quiet is jitter");
    assert!(gnss.sweep(13 * SECOND).is_empty(), "3 s exactly is not past the gap");
    // the milliseconds are floored (plan B's `// 1000`): 4000.999 ms is 4000, never rounded up
    let lost: Vec<String> = gnss.sweep(14 * SECOND + 999).iter().map(Event::to_string).collect();
    assert_eq!(lost, ["RMC lost (quiet 4000 ms)", "GGA lost (quiet 4000 ms)"]);
    assert!(gnss.sweep(15 * SECOND).is_empty(), "no spam while out");
    let back = gnss.parse(&line(RMC), 15 * SECOND + 1500);
    assert_eq!(
        back.events,
        [Event::Back {
            sentence: Sentence::Rmc,
            after_ms: 5001
        }]
    );
    assert_eq!(back.events[0].to_string(), "RMC back after 5001 ms");
    assert!(gnss.sweep(16 * SECOND).is_empty(), "GGA is still out, already evented");
}

#[test]
fn the_outage_gap_is_strictly_past_three_seconds() {
    let mut gnss = Gnss::new();
    gnss.parse(&line(RMC), SECOND);
    assert!(gnss.sweep(4 * SECOND).is_empty(), "exactly 3 s");
    assert_eq!(
        gnss.sweep(4 * SECOND + 1),
        [Event::Lost {
            sentence: Sentence::Rmc,
            quiet_ms: 3000
        }],
        "1 us past: lost, the milliseconds floored"
    );
}

#[test]
fn every_field_count_gate_sits_where_plan_b_s_does() {
    let mut gnss = Gnss::new();
    // RMC and GGA need more than 9 fields; a shorter one is refused but still marks its arrival
    gnss.parse(&line(RMC), 0);
    gnss.parse(&line("GPGGA,1,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"), 0);
    assert_eq!(gnss.sweep(4 * SECOND).len(), 2, "both out");
    let short_rmc = gnss.parse(&line("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4"), 5 * SECOND);
    assert_eq!(
        (short_rmc.position, short_rmc.fix_row),
        (None, None),
        "9 fields: refused"
    );
    assert_eq!(short_rmc.events[0].to_string(), "RMC back after 5000 ms", "yet arrived");
    let short_gga = gnss.parse(&line("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9"), 5 * SECOND);
    assert_eq!(
        (short_gga.altitude_m, short_gga.gga_row),
        (None, None),
        "9 fields: refused"
    );
    assert_eq!(short_gga.events[0].to_string(), "GGA back after 5000 ms", "yet arrived");
    let rmc = gnss.parse(
        &line("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394"),
        5 * SECOND,
    );
    assert!(rmc.position.is_some() && rmc.fix_row.is_some(), "10 fields: taken");
    let gga = gnss.parse("$GPGGA,1,4807.038,N,01131.000,E,1,08,0.9,545.4*42", 5 * SECOND);
    assert_eq!(gga.altitude_m, Some(545.4), "10 fields: taken");
    // GSV needs more than 3: a NEO-6M with nothing in view sends 4, which empties its talker's view
    feed(&mut gnss, &NEO_BURST, 6 * SECOND);
    assert_eq!(gnss.settle(7 * SECOND).unwrap().0.in_view, 12);
    gnss.parse(&line("GPGSV,1,1"), 8 * SECOND);
    assert_eq!(gnss.settle(9 * SECOND), None, "3 fields: not the sky, no burst");
    gnss.parse("$GPGSV,1,1,00*79", 10 * SECOND);
    let (empty, _first) = gnss.settle(11 * SECOND).unwrap();
    assert_eq!((empty.in_view, empty.cn0), (0, [0; 4]), "4 fields: nothing in view");
    // GSA needs more than 14: twelve PRN slots after the mode
    gnss.parse(&line("GPGSA,A,3,05,06,07,,,,,,,,"), 12 * SECOND);
    assert_eq!(
        (gnss.sky_row().used, gnss.settle(13 * SECOND)),
        (12, None),
        "14 fields: refused"
    );
    gnss.parse(&line("GPGSA,A,3,05,06,07,,,,,,,,,"), 14 * SECOND);
    assert_eq!(
        gnss.settle(15 * SECOND).unwrap().0.used,
        3,
        "15 fields: taken, replacing GP's 12"
    );
}

#[test]
fn a_gsa_is_keyed_by_its_system_id_else_by_its_talker() {
    let mut gnss = Gnss::new();
    // NMEA 2.3 (no id, 18 fields): two talkers are two systems, and their counts sum
    gnss.parse(&line("GPGSA,A,3,01,02,03,04,,,,,,,,,1.5,0.9,1.2"), 0);
    gnss.parse(&line("BDGSA,A,2,11,12,13,,,,,,,,,,1.6,1.0,1.3"), 0);
    assert_eq!((gnss.sky_row().used, gnss.sky_row().mode), (7, 3));
    // the same talker again replaces its own count
    gnss.parse(&line("GPGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2"), 0);
    assert_eq!(gnss.sky_row().used, 5);
    // an NMEA 4.1 id keys its own system, apart from the talker-keyed ones
    gnss.parse(&line("GNGSA,A,3,05,06,07,08,09,,,,,,,,1.5,0.9,1.2,1"), 0);
    assert_eq!(gnss.sky_row().used, 10);
    gnss.parse(&line("GNGSA,A,1,,,,,,,,,,,,,9.9,9.9,9.9,1"), 0);
    assert_eq!(gnss.sky_row().used, 5, "system 1 lost its fix; GP and BD stand");
}

#[test]
fn a_burst_ends_at_exactly_half_a_second_of_quiet() {
    let mut gnss = Gnss::new();
    gnss.parse(NEO_BURST[0], SECOND);
    assert_eq!(gnss.settle(SECOND + 499_999), None, "1 us short: still arriving");
    assert!(gnss.settle(SECOND + 500_000).is_some(), "half a second: over");
    assert_eq!(gnss.settle(SECOND + 500_001), None, "spent");
}

#[test]
fn a_burst_makes_one_sky_row_once_it_is_quiet() {
    let mut gnss = Gnss::new();
    assert_eq!(gnss.settle(5 * SECOND), None, "no diagnostics, no row (the HITL case)");
    assert_eq!(gnss.sky_row(), SkyRow::default());
    feed(&mut gnss, &ATGM_COLD, 10 * SECOND);
    assert_eq!(
        gnss.settle(10 * SECOND + 100_000),
        None,
        "the burst may still be arriving"
    );
    let (cold, first) = gnss.settle(11 * SECOND).unwrap();
    assert_eq!(cold, row([1, 0, 1, 39, 0, 0, 0, 0]));
    assert_eq!(
        first.unwrap().to_string(),
        "first sky view: 1 in view, 0 used, mode 1, C/N0 39/0/0/0 dB-Hz, antenna unknown"
    );
    assert_eq!(gnss.settle(12 * SECOND), None, "nothing new");
    let events: Vec<Event> = ATGM_BURST
        .iter()
        .flat_map(|sentence| gnss.parse(sentence, 20 * SECOND).events)
        .collect();
    assert_eq!(
        events,
        [Event::Antenna {
            now: Antenna::Ok,
            was: Antenna::Unknown
        }]
    );
    let (fixed, first) = gnss.settle(21 * SECOND).unwrap();
    assert_eq!(fixed, row([20, 17, 3, 44, 40, 40, 40, 1]));
    assert_eq!(first, None, "only the first burst is evented");
    assert_eq!(gnss.cn0_mean(), 41.0);
}

#[test]
fn a_refused_configuration_is_one_event_per_family() {
    let mut gnss = Gnss::new();
    let refused = |gnss: &mut Gnss, family: &str| {
        let body = format!("GPTXT,01,01,01,{family} inv format");
        let line = nmea(&body);
        gnss.line(&line, 1_000_000).events
    };
    let first = refused(&mut gnss, "PCAS");
    assert_eq!(first, [Event::Rejected { family: "PCAS" }]);
    assert!(
        first[0]
            .to_string()
            .starts_with("the module refused a PCAS sentence (inv format)")
    );
    assert!(refused(&mut gnss, "PCAS").is_empty(), "once per family");
    assert_eq!(refused(&mut gnss, "PMTK"), [Event::Rejected { family: "PMTK" }]);
    // another text is no refusal, and a refusal is no sky sentence: no burst pends
    assert!(refused(&mut gnss, "XYZW").is_empty());
    assert!(
        gnss.line(&nmea("GPTXT,01,01,01,PCASinv format"), 1_000_000)
            .events
            .is_empty()
    );
    assert!(gnss.settle(10_000_000).is_none());
}

#[test]
fn the_antenna_text_events_only_a_change() {
    let mut gnss = Gnss::new();
    let event = |gnss: &mut Gnss, sentence: &str| gnss.parse(sentence, 0).events;
    assert_eq!(event(&mut gnss, "$GPTXT,01,01,01,ANTENNA OK*35").len(), 1);
    assert!(event(&mut gnss, "$GPTXT,01,01,01,ANTENNA OK*35").is_empty());
    assert_eq!(
        event(&mut gnss, "$GPTXT,01,01,01,ANTENNA OPEN*25")[0].to_string(),
        "antenna OPEN (was OK)"
    );
    assert!(event(&mut gnss, "$GPTXT,01,01,02,ANTSTATUS=INIT*25").is_empty());
    assert!(
        event(&mut gnss, "$GPTXT,01,01,01,ANTENNA OK*00").is_empty(),
        "bad checksum"
    );
    assert_eq!(gnss.sky_row().antenna, Antenna::Open);
}

#[test]
fn a_gsv_group_counts_only_whole_and_in_order() {
    let mut gnss = Gnss::new();
    feed(&mut gnss, &ATGM_BURST, 0);
    let before = gnss.sky_row();
    feed(
        &mut gnss,
        &[
            "$GPGSV,4,2,13,08,48,033,20,09,32,214,40,14,23,294,38,17,21,228,40,0*65",
            &line("GPGSV,3,1,11,01,43,131,50,02,50,092,50,04,09,179,50,07,81,335,50,0"),
            &line("GPGSV,3,3,11,22,13,287,50,27,05,045,50,30,43,319,50,0"),
            &line("GPGSV,2,1,05,01,43,131,51,02,50,092,51,04,09,179,51,07,81,335,51,0"),
            "$GPGSV,2,2,05,08,41,035,51,0*00",
            &line("BDGSV,2,2,07,29,48,192,52,36,59,003,52,39,23,312,52,0"),
        ],
        0,
    );
    assert_eq!(gnss.sky_row(), before, "no partial group replaced a view");
    // in view but untracked is no signal; a system losing its fix drops its count, the best mode stays
    gnss.parse(&line("GPGSV,1,1,02,01,43,131,,02,50,092,"), 0);
    gnss.parse(&line("GNGSA,A,1,,,,,,,,,,,,,9.9,9.9,9.9,4"), 0);
    assert_eq!(gnss.sky_row(), row([9, 10, 3, 44, 40, 37, 30, 1]));
}

#[test]
fn the_neo_burst_has_one_system_and_no_antenna() {
    let mut gnss = Gnss::new();
    feed(&mut gnss, &NEO_BURST, 0);
    assert_eq!(gnss.settle(SECOND).unwrap().0, row([12, 12, 3, 43, 42, 40, 40, 0]));
}

#[test]
fn sky_sentences_restart_the_quiet_clock_and_other_texts_do_not() {
    for (sentence, restarts) in [
        (NEO_BURST[0], true),
        (NEO_BURST[1], true),
        ("$GPTXT,01,01,01,ANTENNA OK*35", true),
        ("$GPTXT,01,01,02,ANTSTATUS=INIT*25", false),
    ] {
        let mut gnss = Gnss::new();
        gnss.parse(NEO_BURST[0], 0);
        gnss.parse(sentence, SECOND);
        assert_eq!(gnss.settle(SECOND + 200_000).is_some(), !restarts, "{sentence}");
        assert_eq!(gnss.settle(2 * SECOND).is_some(), restarts, "{sentence}");
    }
}

#[test]
fn the_diagnostics_run_on_the_pad_only() {
    let mut gnss = Gnss::new();
    let mut writes = Vec::new();
    for stage in [
        Stage::Setting,
        Stage::Setting,
        Stage::Boosting,
        Stage::Boosting,
        Stage::Gliding,
        Stage::Landing,
        Stage::Done,
        Stage::Done,
    ] {
        writes.push(gnss.sky_window(stage));
    }
    assert_eq!(
        writes,
        [
            None,
            None,
            Some((SkyFrame::Off, Event::SkyOff)),
            None,
            None,
            None,
            Some((SkyFrame::On, Event::SkyOn)),
            None
        ]
    );
    assert_eq!(Event::SkyOff.to_string(), "sky diagnostics off for flight");
    // a launch called off: SETTING straight to DONE stays on the pad
    assert_eq!(gnss.sky_window(Stage::Setting), None);
    assert!(gnss.diagnosing());
}

#[test]
fn a_tick_sweeps_switches_and_settles_in_that_order() {
    // a warm start into GLIDING: the init left the diagnostics on, the first tick switches them off
    let mut gnss = Gnss::new();
    gnss.parse(&line(RMC), SECOND);
    feed(&mut gnss, &ATGM_COLD, SECOND);
    let tick = gnss.tick(5 * SECOND, Stage::Gliding);
    assert_eq!(tick.write, Some(SkyFrame::Off));
    let events: Vec<String> = tick.events.iter().map(Event::to_string).collect();
    assert_eq!(
        events,
        [
            "RMC lost (quiet 4000 ms)",
            "sky diagnostics off for flight",
            "first sky view: 1 in view, 0 used, mode 1, C/N0 39/0/0/0 dB-Hz, antenna unknown"
        ]
    );
    assert_eq!(tick.sky_row, Some(row([1, 0, 1, 39, 0, 0, 0, 0])));
    let quiet = gnss.tick(6 * SECOND, Stage::Gliding);
    assert_eq!((quiet.write, quiet.sky_row, quiet.events.len()), (None, None, 0));
}

#[test]
fn a_line_is_counted_decoded_stripped_and_followed_by_a_sweep() {
    let mut gnss = Gnss::new();
    let raw = format!("  {}\r\n", line(RMC));
    assert!(gnss.line(raw.as_bytes(), SECOND).position.is_some());
    assert_eq!(gnss.lines(), 1);
    assert_eq!(
        gnss.line(b"", 2 * SECOND),
        Update::default(),
        "an empty read is no line"
    );
    assert_eq!(gnss.lines(), 1);
    let mut noise = b"\xff\xfe".to_vec();
    noise.extend_from_slice(line(RMC).as_bytes());
    assert_eq!(gnss.line(&noise, 2 * SECOND), Update::default(), "not UTF-8: dropped");
    assert_eq!(gnss.lines(), 2);
    let swept = gnss.line(b"$GPTXT,01,01,02,ANTSTATUS=INIT*25\r\n", 6 * SECOND);
    assert_eq!(
        swept.events,
        [Event::Lost {
            sentence: Sentence::Rmc,
            quiet_ms: 5000
        }]
    );
}
