fn main() {
    // Tell Cargo to re-run this script only if files inside the c_src directory change
    println!("cargo:rerun-if-changed=embedded-i2c-sen5x");
    println!("cargo:rerun-if-changed=tests/mocks");

    // Initialize the C compiler utility instance
    cc::Build::new()
        // Include the target folder path so the C preprocessor can locate header files
        .include("embedded-i2c-sen5x")
        // Statically compile the explicit driver and common protocol components
        .file("embedded-i2c-sen5x/sen5x_i2c.c")
        .file("embedded-i2c-sen5x/sensirion_common.c")
        .file("embedded-i2c-sen5x/sensirion_i2c.c")
        .file("tests/mocks/sensirion_i2c_hal_mock.c")
        // Suppress benign legacy C compiler warnings to keep the build output exceptionally clean
        .warnings(false)
        // Compile the target source collection into an archive object file named 'libsensirion_c.a'
        .compile("sensirion_c");
}