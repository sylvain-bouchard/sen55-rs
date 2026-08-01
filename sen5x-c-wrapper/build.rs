fn main() {
    // Tell Cargo to re-run this script only if files inside the c_src directory change
    println!("cargo:rerun-if-changed=embedded-i2c-sen5x");
    println!("cargo:rerun-if-changed=tests/mocks");

    let mut build = cc::Build::new();

    build
        .include("embedded-i2c-sen5x")
        .define("SEN5X_I2C", None)
        .file("embedded-i2c-sen5x/sen5x_i2c.c")
        .file("embedded-i2c-sen5x/sensirion_common.c")
        .file("embedded-i2c-sen5x/sensirion_i2c.c")
        .warnings(false);

    // Target-specific configurations
    let target = std::env::var("TARGET").unwrap_or_default();
    
    if target.contains("thumbv7em-none-eabihf") {
        // When compiling for bare-metal Cortex-M4F, ensure we don't accidentally
        // pull in host-specific operating system configurations or components.
        build.flag("-ffreestanding");
        // Optional: Ensure hardware floating point extensions match if needed
        build.flag("-mfloat-abi=hard");
        build.flag("-mfpu=fpv4-sp-d16");
    } else {
        // Only include the desktop HAL mock when running local host tests
        build.file("tests/mocks/sensirion_i2c_hal_mock.c");
    }

    build.compile("sensirion_c");
}