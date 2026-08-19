use std::{fs, net::Ipv4Addr, path::Path};

use serde::Deserialize;

const DEFAULT_CONFIG: &str = "config/default.toml";
const LOCAL_CONFIG: &str = "config/local.toml";

#[derive(Debug, Deserialize)]
struct Config {
    wifi: WifiConfig,
    http: HttpConfig,
}

#[derive(Debug, Deserialize)]
struct WifiConfig {
    ssid: String,
    password: String,
    channel: u8,
}

#[derive(Debug, Deserialize)]
struct HttpConfig {
    address: String,
    port: u16,
}

#[derive(Debug, Default, Deserialize)]
struct ConfigOverride {
    #[serde(default)]
    wifi: WifiConfigOverride,
    #[serde(default)]
    http: HttpConfigOverride,
}

#[derive(Debug, Default, Deserialize)]
struct WifiConfigOverride {
    ssid: Option<String>,
    password: Option<String>,
    channel: Option<u8>,
}

#[derive(Debug, Default, Deserialize)]
struct HttpConfigOverride {
    address: Option<String>,
    port: Option<u16>,
}

fn main() {
    generate_config();
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

fn generate_config() {
    println!("cargo:rerun-if-changed={DEFAULT_CONFIG}");
    println!("cargo:rerun-if-changed={LOCAL_CONFIG}");

    let mut config: Config =
        toml::from_str(&fs::read_to_string(DEFAULT_CONFIG).expect("read config/default.toml"))
            .expect("parse config/default.toml");

    if Path::new(LOCAL_CONFIG).exists() {
        let local: ConfigOverride =
            toml::from_str(&fs::read_to_string(LOCAL_CONFIG).expect("read config/local.toml"))
                .expect("parse config/local.toml");
        if let Some(value) = local.wifi.ssid {
            config.wifi.ssid = value;
        }
        if let Some(value) = local.wifi.password {
            config.wifi.password = value;
        }
        if let Some(value) = local.wifi.channel {
            config.wifi.channel = value;
        }
        if let Some(value) = local.http.address {
            config.http.address = value;
        }
        if let Some(value) = local.http.port {
            config.http.port = value;
        }
    }

    validate_config(&config);
    let address: Ipv4Addr = config
        .http
        .address
        .parse()
        .expect("http.address must be an IPv4 address");
    let octets = address.octets();
    let generated = format!(
        "pub const WIFI_SSID: &str = {:?};\n\
         pub const WIFI_PASSWORD: &str = {:?};\n\
         pub const WIFI_CHANNEL: u8 = {};\n\
         pub const HTTP_ADDRESS: [u8; 4] = {:?};\n\
         pub const HTTP_PORT: u16 = {};\n",
        config.wifi.ssid, config.wifi.password, config.wifi.channel, octets, config.http.port,
    );
    let out_dir = std::env::var_os("OUT_DIR").expect("OUT_DIR is set by Cargo");
    fs::write(Path::new(&out_dir).join("app_config.rs"), generated)
        .expect("write generated app config");
}

fn validate_config(config: &Config) {
    assert!(
        !config.wifi.ssid.is_empty() && config.wifi.ssid.len() <= 32,
        "wifi.ssid must contain 1..=32 bytes"
    );
    assert!(
        (8..=63).contains(&config.wifi.password.len()),
        "wifi.password must contain 8..=63 bytes for WPA2-PSK"
    );
    assert!(
        (1..=13).contains(&config.wifi.channel),
        "wifi.channel must be in 1..=13"
    );
    assert!(config.http.port != 0, "http.port must not be zero");
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                what if what.starts_with("_defmt_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`"
                    );
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                what if what.starts_with("esp_rtos_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler."
                    );
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!(
                        "💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests"
                    );
                    eprintln!();
                }
                "free"
                | "malloc"
                | "calloc"
                | "get_free_internal_heap_size"
                | "malloc_internal"
                | "realloc_internal"
                | "calloc_internal"
                | "free_internal" => {
                    eprintln!();
                    eprintln!(
                        "💡 Did you forget the `esp-alloc` dependency or didn't enable the `compat` feature on it?"
                    );
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
