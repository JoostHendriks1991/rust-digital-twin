use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::{nmt::{NmtCommand, NmtState}, CanOpenSocket};

use crate::eds::EDSData;

pub struct MotorController {
    pub node_id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: CanSocket,
}


impl MotorController {
    /// Initialize the motor controller.
    pub async fn initialize(
        socket: CanSocket,
        node_id: u8,
        eds_data: EDSData,
    ) -> Result<Self, ()> {
        let controller = Self {
            node_id,
            eds_data,
            nmt_state: NmtState::Initializing,
            socket,
        };

        Ok(controller)
    }

    pub async fn start_socket(&mut self) {

        // Start receiving frames over socket
        loop {
            if let Some(frame) = self.socket.recv().await.ok() {
                let node_id = (frame.id().as_u32() & 0x7F) as u8;
                let cob_id = (frame.id().as_u32() >> 4) & 0xFF;
                let cob_id_hex = format!("{:X}", cob_id);
                match (cob_id, node_id) {
                    (0x00, 0) => self.parse_nmt_command(&frame.data()).await,
                    _ => {},
                }
                if self.node_id == node_id {
                    println!("Node {} received data: {:?}", node_id, frame.data());
                }
            }
        }

    }

    async fn parse_nmt_command(&mut self, data: &[u8]) {

        // Check if the data the correct size
        if !(data.len() == 2) {
            log::error!("Received incorrect frame data length for NMT state change");
        }

        // Extract data
        let requested_state = data[0];
        let addressed_node = data[1];

        // Label NMT command based on requested state
        let nmt_command = match requested_state {
            0x01 => NmtCommand::Start,
            0x02 => NmtCommand::Stop,
            0x80 => NmtCommand::GoToPreOperational,
            0x81 => NmtCommand::Reset,
            0x82 => NmtCommand::ResetCommunication,
            _ => panic!("Unexpected requested state: {:#X}", requested_state),
        };

        // Change NMT state
        if addressed_node == self.node_id {

            let new_nmt_state = match nmt_command {
                NmtCommand::Start => NmtState::Operational,
                NmtCommand::Stop => NmtState::Stopped,
			    NmtCommand::GoToPreOperational => NmtState::PreOperational,
			    NmtCommand::Reset => NmtState::Initializing,
			    NmtCommand::ResetCommunication => NmtState::Initializing,
            };

            if new_nmt_state != self.nmt_state {
                self.nmt_state = new_nmt_state;
                self.send_new_nmt_state().await;
            }

        }

    }

    pub async fn send_new_nmt_state(&mut self) {

        let cob = u16::from_str_radix("700", 16).unwrap();
        let cob_id = CanId::new_base(cob | self.node_id as u16).unwrap();

        let data: [u8; 1] = [match self.nmt_state {
            NmtState::Initializing => 0x00,
            NmtState::Stopped => 0x04,
            NmtState::Operational => 0x05,
            NmtState::PreOperational => 0x7f,
        }];

        let frame = &CanFrame::new(
            cob_id,
            &data,
            None,
        )
        .unwrap();

        if let Err(_) = self.socket.send(frame).await {
            log::error!("Error sending frame");
        }

        log::info!("New NMT State node {}: {}", self.node_id, self.nmt_state);

    }

}