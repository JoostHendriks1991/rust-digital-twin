use can_socket::tokio::CanSocket;
use cia402_runner::{MotorController, Message};
use std::path::PathBuf;
use tokio::task;
use std::sync::Arc;
use tokio::sync::Mutex;
use futures::future;
use tokio::sync::mpsc;

mod eds;
mod config;
mod cia301;
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
    
    // Initialize nodes
    let mut nodes = Vec::new();

    // Initialze controllers
    let mut controllers = Vec::new();

    // Build nodes from eds files and bind socket
    for node in config.node.iter() {

        // Construct mpsc channels
        let (node_tx, controller_rx ) = mpsc::channel::<Message>(100);
        let (controller_tx, node_rx ) = mpsc::channel::<Message>(100);

        // Bind socket
        let socket = CanSocket::bind(&config.bus.interface).map_err(|e| {
            log::error!("Failed to create CAN socket for interface {}: {e}", &config.bus.interface)
        })?;
        log::info!("CAN bus on interface {} opened for node {}", &config.bus.interface, node.node_id);

        // Parse eds data
        let node_id = node.node_id;
        let node_data = eds::parse_eds(&node_id, &node.eds_file).unwrap();

        // Initialize node
        let node = Arc::new(Mutex::new(
            Node::new(node_tx, node_rx, socket, node_id, node_data).unwrap()
        ));
        nodes.push(node);

        // Initialize controller
        let controller = Arc::new(Mutex::new(
            MotorController::initialize(node_id, controller_rx, controller_tx).unwrap()
        ));
        controllers.push(controller);

    }

    let mut futures = Vec::new();

    // Start nodes
    for node in nodes.iter() {
        let node_clone: Arc<Mutex<Node>>  = Arc::clone(node);
        futures.push(
            task::spawn(async move {
            let mut node = node_clone.lock().await;
            node.start_socket().await;
            })
        );
    }

    // Start controllers
    for controller in controllers.iter() {
        let controller_clone: Arc<Mutex<MotorController>>  = Arc::clone(controller);
        futures.push(
            task::spawn(async move {
            let mut controller = controller_clone.lock().await;
            controller.run(&speed_factor).await;
            })
        );
    }

    future::join_all(futures).await;
    
    Ok(())
}

