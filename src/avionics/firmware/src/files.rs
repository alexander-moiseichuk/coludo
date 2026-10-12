//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! HTTP access to the microSD and to the flash ring (read-only; the one write is an OTA image): how recordings and logs come off the board over Wi-Fi
//! -- on the pad, and after landing. Nothing is ever written.
//! * `GET /sd/<path>` returns a file, or a directory's listing (one `<name>;<bytes>` line per entry). Paths are
//!   confined to the card, laid out as crates/recorder's layout says: a session's recordings at
//!   `/sd/<session>/recordings/`, its videos at `/sd/videos/<session>/`, the global index `/sd/sessions.csv`. On a
//!   two-partition card `videos` is the videos partition's mount point, which partition 1's root does not hold, so
//!   the root's listing names it itself. A file another task holds open for writing (the ring_saver appending a
//!   batch, the video writer) answers `503` with `Retry-After: 1`: FatFs's lock table (CONFIG_FATFS_FS_LOCK) refuses
//!   the open (FR_LOCKED, EACCES through the VFS), as it refuses a full table (ENFILE). A saver batch holds a file for
//!   a moment, so the client asks again; the video file being recorded stays busy until `video stop`.
//!
//!   A file's reply carries its size as of the open in `X-Coludo-Bytes`, the one proof a client has that the file
//!   came whole: esp-idf-svc 0.53 drops the Content-Length and sends the body chunked, and when a read fails
//!   mid-file it still closes the reply as if it were whole. The listing cannot stand in for it: a session the
//!   board is still saving grows between the listing and the read. The size at the open is the size sent, since the
//!   lock table keeps every writer out until the reader closes the file. tools/board_pull.sh checks it.
//! * `GET /ring[?boot=N]` streams every valid block of the `log` partition decoded to wire lines,
//!   `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`, in block_sequence order: a byte-exact wire capture, the lines the
//!   ring_saver would file (doc/specs/avionics-port.md "CC tees and the HTTP exits"). `boot=N` keeps only that
//!   boot's blocks.
//! * `GET /ring/image` streams the raw 24 MiB partition, as `esptool read_flash 0x800000 0x1800000` reads it.
//! * `POST /ota?sha256=<hex>` writes a firmware image into the next app slot (M15, crate::ota): the one write this
//!   server takes, on the pad and disarmed only.
//! * `GET /camera/preview.jpg` is the camera's latest preview (M7b): 720x384, made once a second while the camera
//!   records on the pad (404 before the first), for the operator to check its aim.
//! * `GET /beacon` is the recovery beacon's page as the access point serves it at DONE (crate::beacon): the bench sees
//!   it before then.
//!
//! tools/ring_split.py turns either into session directories: the recovery path when the card failed in flight. The
//! ring keeps changing while it streams (the writer appends, the eraser erases), so either is a snapshot taken over
//! some seconds; a block erased meanwhile is left out, and the tools drop the duplicates a pull may share with the SD.
//! A block that stays unreadable is left out too (`/ring/image` sends it as zeros, so the image keeps its size), and
//! recorder.log says so.

use std::fs::File;
use std::io::Read;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::server::{Configuration, EspHttpConnection, EspHttpServer, Request};
use esp_idf_svc::io::Write;
use esp_idf_svc::sys::EspError;
use flash_ring::header::{BlockHeader, HEADER_BYTES, Session};
use flash_ring::{BLOCK_BYTES, Flash, Slot};
use recorder::{Decoder, Recorder};

use crate::recorder::{Partition, RingView};
use firmware::allocator::InternalBuffer;
use firmware::sd;
use recorder::layout;

/// The port the file server listens on (the hub owns 1234/1235, its dashboard 8080).
const PORT: u16 = 8000;
/// Bytes per read/send.
const CHUNK: usize = 32 << 10;
/// How many times a ring block read is tried before the block is given up: a flash call failing, which a second try
/// usually passes (as the bring-up scan tries again).
const READ_ATTEMPTS: u32 = 3;

/// Start the server; it serves for as long as the returned handle lives.
pub fn serve(beacon: crate::beacon::Beacon) -> anyhow::Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&Configuration {
        http_port: PORT,
        uri_match_wildcard: true,
        stack_size: 8192,
        // core 0 with the rest of the link (spec "Threads, cores, priorities"): unpinned, httpd could decode a whole
        // `/ring` on core 1, the flight core
        core: Some(Core::Core0),
        // stack in PSRAM (avionics-port.md §4.7, internal RAM budget): it reads the card and the ring with
        // esp_flash_read, never esp_mmu_map/unmap, the one runtime path that turns the cache off (no OTA here: M15's
        // runs elsewhere); its Rust allocations go to PSRAM too (crate::allocator)
        task_caps: esp_idf_svc::sys::MALLOC_CAP_SPIRAM | esp_idf_svc::sys::MALLOC_CAP_8BIT,
        ..Default::default()
    })?;
    // firmware OTA (M15): the image into the next app slot, on the pad and disarmed (crate::ota)
    server.fn_handler::<anyhow::Error, _>("/ota", Method::Post, crate::ota::receive)?;
    // the camera's aim on the pad (M7b): the latest preview JPEG, made once a second while the camera records in
    // SETTING; 404 before the first
    server.fn_handler::<anyhow::Error, _>("/camera/preview.jpg", Method::Get, |request| {
        let Some((jpeg, uptime)) = firmware::video::preview() else {
            request
                .into_status_response(404)?
                .write_all(b"no preview: the camera records none (video start, on the pad)\n")?;
            return Ok(());
        };
        let uptime = uptime.to_string();
        request
            .into_response(
                200,
                None,
                &[
                    ("Content-Type", "image/jpeg"),
                    ("Cache-Control", "no-store"),
                    ("X-Coludo-Uptime-Us", &uptime),
                ],
            )?
            .write_all(&jpeg)?;
        Ok(())
    })?;
    // the recovery beacon's page as the AP will serve it at DONE (crate::beacon), for the bench before then
    server.fn_handler::<anyhow::Error, _>("/beacon", Method::Get, move |request| {
        let page = beacon.page();
        request
            .into_response(200, None, &[("Content-Type", "text/html; charset=utf-8")])?
            .write_all(page.as_bytes())?;
        Ok(())
    })?;
    server.fn_handler::<anyhow::Error, _>("/sd/*", Method::Get, |request| {
        let relative = request
            .uri()
            .trim_start_matches("/sd")
            .split('?')
            .next()
            .unwrap_or("")
            .to_string();
        if relative.split('/').any(|part| part == "..") {
            request.into_status_response(400)?.write_all(b"no ..\n")?;
            return Ok(());
        }
        let path = format!("{}{relative}", sd::MOUNT);
        let Ok(metadata) = std::fs::metadata(&path) else {
            request.into_status_response(404)?.write_all(b"not found\n")?;
            return Ok(());
        };
        if metadata.is_dir() {
            let entries = match std::fs::read_dir(&path) {
                Ok(entries) => entries,
                Err(error) if busy(&error) => return answer_busy(request),
                Err(error) => return Err(error.into()),
            };
            let mut listing = String::new();
            let mut videos_listed = false;
            for entry in entries.flatten() {
                let bytes = entry.metadata().map(|metadata| metadata.len()).unwrap_or(0);
                let name = entry.file_name().to_string_lossy().into_owned();
                videos_listed |= name.eq_ignore_ascii_case(layout::VIDEOS); // FAT folds case
                listing.push_str(&format!("{name};{bytes}\n"));
            }
            // the videos partition is mounted at the root's `videos`, a mount point partition 1 does not list
            if relative.trim_matches('/').is_empty() && sd::videos_mounted() && !videos_listed {
                listing.push_str(&format!("{};0\n", layout::VIDEOS));
            }
            request.into_ok_response()?.write_all(listing.as_bytes())?;
            return Ok(());
        }
        // opened before the answer starts, so a busy file still gets its 503 status line
        let mut file = match File::open(&path) {
            Ok(file) => file,
            Err(error) if busy(&error) => return answer_busy(request),
            Err(error) => return Err(error.into()),
        };
        // the size as of the open: the lock table keeps every writer out until the file is closed again. The
        // Content-Length goes nowhere (esp-idf-svc 0.53), so X-Coludo-Bytes carries it to the client (see the
        // module notes)
        let length = file
            .metadata()
            .map_or(metadata.len(), |metadata| metadata.len())
            .to_string();
        let mut response = request.into_response(
            200,
            None,
            &[
                ("Content-Type", "application/octet-stream"),
                ("Content-Length", &length),
                (SIZE_HEADER, &length),
            ],
        )?;
        let mut buffer = vec![0u8; CHUNK];
        loop {
            let read = file.read(&mut buffer)?;
            if read == 0 {
                break;
            }
            response.write_all(&buffer[..read])?;
        }
        Ok(())
    })?;
    server.fn_handler::<anyhow::Error, _>("/ring", Method::Get, |request| {
        let (Some(view), Some(recorder)) = (crate::recorder::ring(), crate::recorder::get()) else {
            request
                .into_status_response(503)?
                .write_all(b"the ring is not up: the bring-up scan has not finished, or found no log partition\n")?;
            return Ok(());
        };
        let Ok(boot) = filter(request.uri(), "boot") else {
            request
                .into_status_response(400)?
                .write_all(b"/ring takes only boot=<id>\n")?;
            return Ok(());
        };
        let mut response = request.into_response(200, None, &[("Content-Type", "text/plain")])?;
        decode_ring(view, recorder, boot, &mut |text| Ok(response.write_all(text)?))
    })?;
    // the panic log (crate::panics_logger): every record as text, newest first, or only seq=N's
    server.fn_handler::<anyhow::Error, _>("/panic", Method::Get, |request| {
        let Ok(seq) = filter(request.uri(), "seq") else {
            request
                .into_status_response(400)?
                .write_all(b"/panic takes only seq=<record>\n")?;
            return Ok(());
        };
        match crate::panics_logger::texts(seq) {
            // a record asked for by its seq that no slot holds
            Some(text) if text.is_empty() && seq.is_some() => request
                .into_status_response(404)?
                .write_all(b"no such record in the panic log\n")?,
            // every record (none: an empty body), or the one asked for
            Some(text) => request
                .into_response(200, None, &[("Content-Type", "text/plain")])?
                .write_all(&text)?,
            None => request
                .into_status_response(503)?
                .write_all(b"the panic log is not open (see the boot log)\n")?,
        }
        Ok(())
    })?;
    server.fn_handler::<anyhow::Error, _>("/ring/image", Method::Get, |request| {
        let Some(view) = crate::recorder::ring() else {
            request
                .into_status_response(503)?
                .write_all(b"the ring is not up: the bring-up scan has not finished, or found no log partition\n")?;
            return Ok(());
        };
        let size = view.partition.size();
        let length = size.to_string();
        // esp-idf-svc 0.53 drops the Content-Length (its httpd_resp_set_len is commented out) and sends the body
        // chunked, and on a handler's error it still sends the closing chunk: a body cut short would arrive as a
        // whole, well-formed reply, which no client can tell from a smaller partition. So the image always goes
        // out whole, every slot at its offset (below)
        let mut response = request.into_response(
            200,
            None,
            &[
                ("Content-Type", "application/octet-stream"),
                ("Content-Length", &length),
            ],
        )?;
        // read a block at a time into internal RAM, which esp_flash fills directly, and send in PSRAM chunks: this
        // thread's own allocations go to PSRAM (crate::allocator), and a read into PSRAM bounces through internal DMA
        // memory, the kind esp_hosted needs, allocated anew per call
        let mut block = InternalBuffer::zeroed(BLOCK_BYTES as usize)
            .ok_or_else(|| anyhow::anyhow!("no internal RAM for a block buffer"))?;
        let mut image = Vec::with_capacity(CHUNK);
        for offset in (0..size).step_by(BLOCK_BYTES as usize) {
            if let Err(error) = read_retrying(&view.partition, offset, &mut block) {
                // zeros carry no block magic, so every reader (the scan, tools/ring_split.py) counts the slot
                // foreign, as the writer counts a slot it cannot read: the block is lost to this pull, said so in
                // recorder.log, and the slots behind it keep their offsets
                log::warn!("files :: /ring/image: the block at {offset:#x} is unreadable ({error}), sent as zeros");
                block.fill(0);
            }
            image.extend_from_slice(&block);
            if image.len() >= CHUNK {
                response.write_all(&image)?;
                image.clear();
            }
        }
        response.write_all(&image)?;
        Ok(())
    })?;
    Ok(server)
}

/// The header a file's reply carries its size in, bytes as of the open (see the module notes).
const SIZE_HEADER: &str = "X-Coludo-Bytes";

/// errno EACCES: FatFs's FR_LOCKED through the VFS, a file another task holds open for writing.
const LOCKED: i32 = 13;

/// errno ENFILE: FatFs's lock table, or the VFS's `max_files` handles, are all taken.
const NO_HANDLE: i32 = 23;

/// Whether an open failed only because the card is busy with that file, or with too many: worth asking again.
fn busy(error: &std::io::Error) -> bool {
    matches!(error.raw_os_error(), Some(LOCKED | NO_HANDLE))
}

/// Answers `503` with `Retry-After: 1`: the file is open for writing elsewhere, or no handle is free right now.
fn answer_busy(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    request.into_response(503, None, &[("Retry-After", "1")])?.write_all(
        b"busy: the file is being written (the ring_saver or the video), or no handle is free; try again\n",
    )?;
    Ok(())
}

/// The `<key>=N` of a URI's query (`/ring?boot=N`, `/panic?seq=N`): None when there is no query. Errors: the query
/// names anything else, or N is not a number.
fn filter(uri: &str, key: &str) -> Result<Option<u32>, ()> {
    let Some((_, query)) = uri.split_once('?') else {
        return Ok(None);
    };
    if query.is_empty() {
        return Ok(None);
    }
    let value = query
        .strip_prefix(key)
        .and_then(|rest| rest.strip_prefix('='))
        .ok_or(())?;
    value.parse().map(Some).map_err(|_| ())
}

/// Decode the ring's blocks into wire lines, oldest first, handing the text to `send` about [`CHUNK`] at a time.
///
/// The blocks come from the block table (closed ones and the writer's open one) in block_sequence order. Each is read
/// whole into internal RAM, but a block the table showed open only as far as [`open_length`] allows; and each is
/// checked against the table by its header's sequence, since the eraser may have erased it and the writer opened
/// another there since the table was read. A torn chunk is a loss only when no later block of its boot follows it
/// (the decoder's rule): the next block in sequence order tells, since a boot's blocks are numbered one after
/// another and a later boot always numbers past them.
///
/// Args:
/// * `view` - the flash ring.
/// * `recorder` - for the writer position.
/// * `boot` - keep only this boot's blocks; None keeps every one.
/// * `send` - takes the text.
///
/// Errors: `send` failed (the client went away).
fn decode_ring(
    view: &RingView,
    recorder: &Recorder,
    boot: Option<u32>,
    send: &mut dyn FnMut(&[u8]) -> anyhow::Result<()>,
) -> anyhow::Result<()> {
    let geometry = view.table.geometry();
    // sized for every slot at once (72 KiB), rather than grown
    let mut blocks: Vec<(u32, u32, bool)> = Vec::with_capacity(geometry.blocks() as usize);
    blocks.extend((0..geometry.blocks()).filter_map(|slot| match view.table.slot(slot) {
        Slot::Valid { block_sequence, .. } => Some((block_sequence, slot, false)),
        Slot::Open { block_sequence, .. } => Some((block_sequence, slot, true)),
        _ => None,
    }));
    blocks.sort_unstable_by_key(|&(block_sequence, _, _)| block_sequence);
    let mut decoder = Decoder::new();
    // the boot (session, boot id) the decoder holds: it remembers each file's column list and the newest uptime per
    // boot, for as long as it lives (in PSRAM: this thread's allocations go there, crate::allocator). A boot's blocks
    // come one after another in sequence order, so once the next boot starts the last one's memory is never asked
    // for again: a fresh decoder per boot prints the same lines and holds one boot's files at a time, however many
    // boots the ring keeps
    let mut decoding: Option<(Session, u32)> = None;
    // the flash reads' destinations in internal RAM, which esp_flash fills directly (see /ring/image)
    let mut block = InternalBuffer::zeroed(BLOCK_BYTES as usize)
        .ok_or_else(|| anyhow::anyhow!("no internal RAM for a block buffer"))?;
    let mut next =
        InternalBuffer::zeroed(HEADER_BYTES).ok_or_else(|| anyhow::anyhow!("no internal RAM for a header"))?;
    let mut text = Vec::with_capacity(2 * CHUNK);
    for (index, &(block_sequence, slot, open)) in blocks.iter().enumerate() {
        let length = if open {
            open_length(recorder, block_sequence)
        } else {
            BLOCK_BYTES as usize
        };
        let block = &mut block[..length];
        let offset = geometry.slot_offset(slot);
        if let Err(error) = read_retrying(&view.partition, offset, block) {
            // left out, as an erased block would be; recorder.log says so, since the pull cannot
            log::warn!("files :: /ring: block {block_sequence} at {offset:#x} is unreadable ({error}), left out");
            continue;
        }
        let Ok(header) = BlockHeader::decode(block) else {
            continue;
        };
        if header.block_sequence != block_sequence || boot.is_some_and(|boot| boot != header.boot_id) {
            continue;
        }
        if decoding != Some((header.session, header.boot_id)) {
            decoder = Decoder::new();
            decoding = Some((header.session, header.boot_id));
        }
        let boot_goes_on = open
            || blocks.get(index + 1).is_some_and(|&(_, next_slot, _)| {
                read_retrying(&view.partition, geometry.slot_offset(next_slot), &mut next).is_ok()
                    && BlockHeader::decode(&next)
                        .is_ok_and(|next| next.boot_id == header.boot_id && next.session == header.session)
            });
        // a block of another layout version, or one a cut damaged since, prints nothing
        let _ = decoder.block(block, boot_goes_on, |line| text.extend_from_slice(line.wire));
        if text.len() >= CHUNK {
            send(&text)?;
            text.clear();
        }
    }
    send(&text)
}

/// How many bytes of a block the table showed open (the writer's, this boot) are safe to read now.
///
/// The writer keeps going while the table is walked, sorted and the blocks before this one are read, so its position
/// is read only now, and compared: the writer moves forward only, and publishes a position in a block only after it
/// closed every earlier one.
/// * the writer is past this block: it closed the block since the walk, so the block is whole, every chunk it will
///   ever hold programmed, and is read whole. Reading the header alone would leave out a closed block's records with
///   no loss line, while later blocks of the boot went out;
/// * the writer is in this block: up to its published offset, past which it may be programming;
/// * the writer has published nothing in this block yet (it opened it since its last publish): the header alone.
fn open_length(recorder: &Recorder, block_sequence: u32) -> usize {
    match recorder.writer_position() {
        Some(writer) if writer.block_sequence > block_sequence => BLOCK_BYTES as usize,
        Some(writer) if writer.block_sequence == block_sequence => {
            (writer.offset as usize).clamp(HEADER_BYTES, BLOCK_BYTES as usize)
        }
        _ => HEADER_BYTES,
    }
}

/// Reads `buffer.len()` bytes of the ring at `offset`, trying up to [`READ_ATTEMPTS`] times.
///
/// Errors: the last attempt's.
fn read_retrying(partition: &Partition, offset: u32, buffer: &mut [u8]) -> Result<(), EspError> {
    let mut attempt = 1;
    loop {
        match partition.read(offset, buffer) {
            Err(_) if attempt < READ_ATTEMPTS => attempt += 1,
            result => return result,
        }
    }
}
