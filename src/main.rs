use can_socket::tokio::CanSocket;
use cia402_runner::MotorController;
use std::path::PathBuf;
use tokio::task;
use std::sync::Arc;
use tokio::sync::Mutex;
use futures::future;

mod eds;
mod config;
mod cia301;
mod nmt;
mod sdo;
mod cia402_runner;

use crate::cia301::Node;
use crate::config::Config;

#[derive(clap::Parser)]
struct Options {
    /// The path of the configuration file to use.
    #[clap(long, short)]
    #[clap(value_name = "CONFIG.toml")]
    config: PathBuf,
}

#[tokio::main]
async fn main() {

    // Initialize the logging system.
    env_logger::builder()
        .filter_module(module_path!(), log::LevelFilter::Info)
        .parse_default_env()
        .init();


    // Run the server and set a non-zero exit code if we had an error.
    do_main(clap::Parser::parse()).await.ok();

}

async fn do_main(options: Options) -> Result<(), ()> {

    // Read the configuration file.
    let config = Config::read_from_file(&options.config)?;

    let speed_factor = config.general.speed_factor;

    // Initialze controllers
    let mut controllers = Vec::new();

    // Build nodes from eds files and bind socket
    for node in config.node.iter() {

        // Bind socket
        let socket = CanSocket::bind(&config.bus.interface).map_err(|e| {
            log::error!("Failed to create CAN socket for interface {}: {e}", &config.bus.interface)
        })?;
        log::info!("CAN bus on interface {} opened for node {}", &config.bus.interface, node.node_id);

        // Parse eds data
        let node_id = node.node_id;
        let node_data = eds::parse_eds(&node_id, &node.eds_file).unwrap();

        // Initialize node
        let controller = Arc::new(Mutex::new(
            MotorController::initialize(Node::new(socket, node_id, node_data))
        ));
        controllers.push(controller);

    }

    let mut futures = Vec::new();

    // Start nodes
    for controller in controllers.iter() {
        let controller_clone: Arc<Mutex<MotorController>>  = Arc::clone(&controller);
        futures.push(
            task::spawn(async move {
                loop {

                    tokio::time::sleep(tokio::time::Duration::from_micros(1)).await;

                    let mut controller = controller_clone.lock().await;

                    controller.node.socket_listener().await;

                }
            })
        );
        let controller_clone: Arc<Mutex<MotorController>>  = Arc::clone(&controller);
        futures.push(
            task::spawn(async move {
                loop {

                    tokio::time::sleep(tokio::time::Duration::from_micros(1)).await;

                    let mut controller = controller_clone.lock().await;
                        
                    controller.update_controller(&speed_factor).await;
                    
                }
            })
        );
    }

    future::join_all(futures).await;
    
    Ok(())
}

