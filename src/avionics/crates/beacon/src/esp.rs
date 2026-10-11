//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The beacon's access point on ESP-IDF (the `esp` feature), for both ends: the glider at DONE (firmware/src/beacon.rs)
//! and the remote recorder (src/avionics/recorder). Each passes its own thread and server settings: the glider's
//! stacks go to PSRAM on core 0, the C6 has neither PSRAM nor a second core.
//!
//! The page opens by itself (a captive portal, operator 2026-10-11): the AP's address is [`ADDRESS`], its DHCP server
//! offers the beacon as the DNS server, a DNS responder answers every name with the beacon, and every other path
//! redirects to the page, so a phone's connectivity check (Android's /generate_204, iOS's /hotspot-detect.html,
//! Windows' /connecttest.txt) finds a portal and the phone opens it. No DHCP option 114 (RFC 8910): phones take it
//! only as an HTTPS captive-portal API (RFC 8908), which a beacon cannot serve. The server's wildcard matching must be
//! on: without it the probes got 404 (no portal, only "no internet") and the page stayed shut.

use std::net::UdpSocket;

use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use esp_idf_svc::io::Write as _;
use esp_idf_svc::sys;
use esp_idf_svc::wifi::{AccessPointConfiguration, AuthMethod, BlockingWifi, Configuration, EspWifi};

use crate::{ADDRESS, Radio, dns_answer};

/// Phones at once.
const MAXIMUM_CONNECTIONS: u16 = 4;

/// The DNS responder's stack.
const DNS_STACK: usize = 4 << 10;

/// The page's port: a phone opens the access point's address as it is.
const PORT: u16 = 80;

/// The access point `radio` names up on `wifi` (a station dropped first; started when it is not), at its TX cap, and
/// addressed for the portal.
///
/// Errors: a name or password too long for the driver, or a driver call that failed.
pub fn start(wifi: &mut BlockingWifi<EspWifi<'static>>, radio: &Radio) -> anyhow::Result<()> {
    let _ = wifi.wifi_mut().disconnect();
    let access_point = AccessPointConfiguration {
        ssid: radio
            .ssid
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("{} is longer than 32 bytes", radio.ssid))?,
        password: radio
            .password
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("the password is longer than 64 bytes"))?,
        channel: radio.channel,
        auth_method: if radio.password.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        },
        max_connections: MAXIMUM_CONNECTIONS,
        ..Default::default()
    };
    wifi.set_configuration(&Configuration::AccessPoint(access_point))?;
    if !wifi.is_started()? {
        wifi.start()?;
    }
    // the cap in quarter-dBm, which IDF takes from 8 to 84 (2..21 dBm)
    let quarters = (i16::from(radio.power_dbm) * 4).clamp(8, 84) as i8;
    // SAFETY: a plain driver call on the started Wi-Fi.
    sys::esp!(unsafe { sys::esp_wifi_set_max_tx_power(quarters) })?;
    portal(wifi.wifi().ap_netif().handle())
}

/// The AP `netif` re-addressed to [`ADDRESS`], its DHCP server offering the beacon as the DNS server (the server
/// stopped for the change, then started again).
fn portal(netif: *mut sys::esp_netif_t) -> anyhow::Result<()> {
    let address = sys::esp_ip4_addr {
        addr: u32::from_le_bytes(ADDRESS),
    };
    let information = sys::esp_netif_ip_info_t {
        ip: address,
        netmask: sys::esp_ip4_addr {
            addr: u32::from_le_bytes([255, 255, 255, 0]),
        },
        gw: address,
    };
    let mut dns = sys::esp_netif_dns_info_t::default();
    dns.ip.u_addr.ip4 = address;
    dns.ip.type_ = 0; // IPADDR_TYPE_V4
    let mut offer_dns: u8 = 1;
    // SAFETY: plain esp_netif calls on the AP's live netif; the option value outlives the call.
    unsafe {
        let _ = sys::esp_netif_dhcps_stop(netif);
        sys::esp!(sys::esp_netif_set_ip_info(netif, &information))?;
        sys::esp!(sys::esp_netif_set_dns_info(
            netif,
            sys::esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN,
            &mut dns
        ))?;
        sys::esp!(sys::esp_netif_dhcps_option(
            netif,
            sys::esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
            sys::esp_netif_dhcp_option_id_t_ESP_NETIF_DOMAIN_NAME_SERVER,
            (&mut offer_dns as *mut u8).cast(),
            1
        ))?;
        sys::esp!(sys::esp_netif_dhcps_start(netif))?;
    }
    Ok(())
}

/// The DNS responder on its own thread (`thread`'s core, priority and stack placement): every A query answered with
/// [`ADDRESS`], every other one with no record, forever.
///
/// Errors: port 53 would not bind, or the thread would not spawn.
pub fn spawn_dns(thread: ThreadSpawnConfiguration) -> anyhow::Result<()> {
    let socket = UdpSocket::bind(("0.0.0.0", 53))?;
    ThreadSpawnConfiguration {
        name: Some(c"beacon_dns"),
        stack_size: DNS_STACK,
        ..thread
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(DNS_STACK).spawn(move || {
        let mut query = [0u8; 512];
        loop {
            let Ok((length, sender)) = socket.recv_from(&mut query) else {
                continue;
            };
            if let Some(answer) = dns_answer(&query[..length]) {
                let _ = socket.send_to(&answer, sender);
            }
        }
    });
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// The page server on port 80 (`server`'s task settings, its wildcard matching turned on): `/` answers `page()`,
/// every other path redirects to it, so the phones' connectivity checks find a portal. It lives as long as the value.
///
/// Errors: the server would not start.
pub fn serve(
    server: HttpConfiguration,
    page: impl Fn() -> String + Send + Sync + 'static,
) -> anyhow::Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&HttpConfiguration {
        http_port: PORT,
        uri_match_wildcard: true,
        ..server
    })?;
    server.fn_handler::<anyhow::Error, _>("/", Method::Get, move |request| {
        let page = page();
        request
            .into_response(200, None, &[("Content-Type", "text/html; charset=utf-8")])?
            .write_all(page.as_bytes())?;
        Ok(())
    })?;
    let [a, b, c, d] = ADDRESS;
    let location = format!("http://{a}.{b}.{c}.{d}/");
    server.fn_handler::<anyhow::Error, _>("/*", Method::Get, move |request| {
        request
            .into_response(302, Some("Found"), &[("Location", location.as_str())])?
            .write_all(format!("the beacon's page: {location}\n").as_bytes())?;
        Ok(())
    })?;
    Ok(server)
}
