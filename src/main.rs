use can_socket::tokio::CanSocket;
use std::path::PathBuf;
use tokio::task;
use std::sync::Arc;
use tokio::sync::Mutex;
use futures::future;

mod eds;
mod config;
mod controller;

use crate::controller::MotorController;
use crate::eds::parse_eds;
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
    
    // Initialize motor controllers
    let mut controllers = Vec::new();

    // Build motor controllers
    for node in config.node.iter() {

        // Start socket
        let socket = CanSocket::bind(&config.bus.interface).map_err(|e| {
            log::error!("Failed to create CAN socket for interface {}: {e}", &config.bus.interface)
        })?;
        log::info!("CAN bus on interface {} opened for node {}", &config.bus.interface, node.node_id);

        // Parse eds data
        let controller_data = parse_eds(node.node_id).unwrap();

        // Initialize controller
        let controller= Arc::new(Mutex::new(
            MotorController::initialize(socket, node.node_id, controller_data).await.unwrap()
        ));
        controllers.push(controller);
    }

    let mut futures = Vec::new();

    for controller in controllers.iter() {
        let controller_clone: Arc<Mutex<MotorController>>  = Arc::clone(controller);
        futures.push(
            task::spawn(async move {
            let mut controller = controller_clone.lock().await;
            controller.start_socket().await;
            })
        );
    }

    future::join_all(futures).await;
    Ok(())
}

