//! Read-only HTTP access to the microSD: `GET /sd/<path>` returns a file, or a directory's listing (one
//! `<name>;<bytes>` line per entry). How recordings and logs come off the board over Wi-Fi -- on the pad, and
//! after landing. Paths are confined to the card; nothing is ever written.

use std::fs::File;
use std::io::Read;

use esp_idf_svc::http::Method;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::Write;

use firmware::sd;

/// The port the file server listens on (the hub owns 1234/1235, its dashboard 8080).
const PORT: u16 = 8000;
/// Bytes per read/send.
const CHUNK: usize = 32 << 10;

/// Start the server; it serves for as long as the returned handle lives.
pub fn serve() -> anyhow::Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&Configuration {
        http_port: PORT,
        uri_match_wildcard: true,
        stack_size: 8192,
        ..Default::default()
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
            let mut listing = String::new();
            for entry in std::fs::read_dir(&path)?.flatten() {
                let bytes = entry.metadata().map(|metadata| metadata.len()).unwrap_or(0);
                listing.push_str(&format!("{};{bytes}\n", entry.file_name().to_string_lossy()));
            }
            request.into_ok_response()?.write_all(listing.as_bytes())?;
            return Ok(());
        }
        let length = metadata.len().to_string();
        let mut response = request.into_response(
            200,
            None,
            &[
                ("Content-Type", "application/octet-stream"),
                ("Content-Length", &length),
            ],
        )?;
        let mut file = File::open(&path)?;
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
    Ok(server)
}
