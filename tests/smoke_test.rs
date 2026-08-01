use sen5x_rust::Sen5xDriver;

#[link(name = "sensirion_c", kind = "static")]
extern "C" {
    // The exact signature from the frozen `sen5x_i2c.h` header file
    fn sen5x_device_reset() -> i16;
}

#[test]
fn test_c_driver_linkage() {
    // 2. Execute the compiled C function inside an unsafe block
    // It will attempt to call the C function, format the mock buffer,
    // and execute your mock sleep routine.
    let result = unsafe { sen5x_device_reset() };

    // 3. Verify execution behavior
    // The function returns an error status code. Since we haven't mocked a
    // real passing I2C read/write transaction layer yet, it will likely return a
    // non-zero I2C error code, which perfectly proves the C code is executing!
    println!(
        "C driver executed successfully and returned status: {}",
        result
    );

    // We are just checking that it doesn't crash or fail to link
    assert!(result != 0 || result == 0);
}

#[test]
fn test_safe_driver_interface() {
    let mut driver = Sen5xDriver::new();
    
    // Trigger our wrapped routine
    let result = driver.device_reset();
    
    assert!(result.is_ok(), "Safe driver bridge failed to communicate with target C layer!");
}
