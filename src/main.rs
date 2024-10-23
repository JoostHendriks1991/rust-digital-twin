use can_socket::tokio::CanSocket;
use canopen_tokio::CanOpenSocket;

mod eds;
mod controller;

use crate::controller::MotorController;
use crate::eds::parse_eds;

#[tokio::main]
async fn main() -> Result<(), ()> {

    // Initialize the logging system.
    env_logger::builder()
        .filter_module(module_path!(), log::LevelFilter::Info)
        .parse_default_env()
        .init();

    // Open the CAN bus.
    log::info!("Opening CAN bus on interface vcan0");
    let socket = CanSocket::bind("vcan0").map_err(|e| {
        log::error!("Failed to create CAN socket for interface vcan0: {e}")
    })?;
    
    log::info!("CAN bus on interface vcan0 opened");
    let mut bus = CanOpenSocket::new(socket);

    // Build motor controller
    let controller_data = parse_eds().unwrap();
    
    // Initialize motor controller
    let controller = MotorController::initialize(&mut bus, controller_data).await;

    println!("Eds file: {}", controller.unwrap().eds_data.file_info.file_name);

    Ok(())
}
