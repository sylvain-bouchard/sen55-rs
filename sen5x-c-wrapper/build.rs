fn main() {
    // Tell Cargo to re-run this script only if files inside the c_src directory change
    println!("cargo:rerun-if-changed=hal/esp-idf");
    println!("cargo:rerun-if-changed=hal/mock");
    println!("cargo:rerun-if-changed=hal/desktop");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_MOCK");

    let mut build = cc::Build::new();

    build
        .include("hal/esp-idf")
        .define("SEN5X_I2C", None)
        .file("hal/esp-idf/sen5x_i2c.c")
        .file("hal/esp-idf/sensirion_common.c")
        .file("hal/esp-idf/sensirion_i2c.c")
        .warnings(false);

    // Target-specific configurations
    let target = std::env::var("TARGET").unwrap_or_default();

    if std::env::var("CARGO_FEATURE_MOCK").is_ok() {
        println!("cargo:warning=Building with SEN5x mock HAL");
        build.file("hal/mock/sensirion_i2c_hal_mock.c");
    } else if target.contains("esp") {
        println!("cargo:warning=Building with ESP-IDF SEN5x HAL");
        build.file("hal/esp-idf/sensirion_i2c_hal.c");
    } else {
        println!("cargo:warning=Building with desktop SEN5x HAL");
        build.file("hal/desktop/sensirion_i2c_hal_mock.c");
    }

    build.compile("sensirion_c");
}
