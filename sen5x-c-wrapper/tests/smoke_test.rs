use sen5x_rust::Sen5xDriver;

#[test]
fn test_c_wrapper_safe_interface() {
    let mut driver = Sen5xDriver::new();

    // Trigger our wrapped routine
    let result = driver.device_reset();

    println!("device_reset result: {:?}", result);
    
    assert!(
        result.is_ok(),
        "C wrapper failed to communicate with Sensirion driver"
    );
}
