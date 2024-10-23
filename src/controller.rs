use can_socket::tokio::CanSocket;
use can_socket::CanFrame;

use crate::eds::EDSData;

pub struct MotorController {
    pub eds_data: EDSData,
    pub socket: CanSocket,
    pub message_buffer: Vec<CanFrame>
}


impl MotorController {
    /// Initialize the motor controller.
    pub async fn initialize(
        socket: CanSocket,
        eds_data: EDSData,
    ) -> Result<Self, ()> {
        let controller = Self {
            eds_data,
            socket,
            message_buffer: Vec::new(),
        };

        Ok(controller)
    }

    pub async fn start_socket(&mut self) {
        loop {
            let message = self.socket.recv().await.ok();
            if message.is_some() {
                println!("Received Message: {:?}", message);
                self.message_buffer.push(message.unwrap())
            }
        }

    } 

}