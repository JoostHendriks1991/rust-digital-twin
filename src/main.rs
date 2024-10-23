use can_socket::tokio::CanSocket;

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

    // Build motor controller
    let controller_data = parse_eds().unwrap();
    
    // Initialize motor controller
    let mut controller = MotorController::initialize(socket, controller_data).await.unwrap();

    println!("Eds file: {}", controller.eds_data.file_info.file_name);

    MotorController::start_socket(&mut controller).await;


    Ok(())
}
