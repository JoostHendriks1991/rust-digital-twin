use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::nmt::{NmtCommand, NmtState};

use crate::eds::EDSData;

pub struct MotorController {
    pub node_id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: CanSocket,
}

enum ServerCommand {
	/// The server is uploading a segment.
	SegmentUpload,

	/// The server has downloaded the segment.
	SegmentDownload,

	/// The server accepts the upload request.
	InitiateUpload,

	/// The server accepts the download request.
	InitiateDownload,

	/// The server is aborting the transfer.
	AbortTransfer,

    /// Unknown server command.
    Unknown,
}

impl ServerCommand {
    fn get_server_command(value: u8) -> ServerCommand {
        match value {
            0 => ServerCommand::SegmentUpload,
            1 => ServerCommand::SegmentDownload,
            2 => ServerCommand::InitiateUpload,
            3 => ServerCommand::InitiateDownload,
            4 => ServerCommand::AbortTransfer,
            _ => ServerCommand::Unknown,
        }
    }
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

            // Check if a frame is received
            if let Some(frame) = self.socket.recv().await.ok() {

                // Extract id and cob_id
                let node_id = (frame.id().as_u32() & 0x7F) as u8;
                let function_code = frame.id().as_u32() & (0x0F << 7);
                let _cob_id_hex = format!("{:X}", function_code);

                // Parse frame
                if node_id == 0 {
                    match function_code {
                        0x000 => self.parse_nmt_command(&frame.data()).await,
                        _ => {},
                    }
                } else if node_id == self.node_id {
                    match function_code {
                        0x580 => self.parse_sdo_downlaod().await,
                        0x600 => self.parse_sdo_upload(&frame.data()).await,
                        _ => {},
                    }
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

            self.nmt_state = match nmt_command {
                NmtCommand::Start => NmtState::Operational,
                NmtCommand::Stop => NmtState::Stopped,
			    NmtCommand::GoToPreOperational => NmtState::PreOperational,
			    NmtCommand::Reset => NmtState::Initializing,
			    NmtCommand::ResetCommunication => NmtState::Initializing,
            };
            self.send_new_nmt_state().await;

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

    async fn parse_sdo_downlaod(&mut self) {

        println!("Sdo download");

    }

    async fn parse_sdo_upload(&mut self, data: &[u8]) {

        if data.len() > 8 {
            log::error!("Data length too long")
        };
        
		let size_set = (data[0] & (1 << 0)) != 0;
        let expedited = (data[0] & (1 << 1)) != 0;
        let n = (data[0] & (1 << 2)) != 0;
        let ccs = (data[0] >> 5) & 0b111;

        let command = ServerCommand::get_server_command(ccs);

        match command {
            ServerCommand::InitiateUpload => {
                self.update_register(data).await;
                self.send_sdo_response().await;
            }
            _ => {},
        }

    }

    async fn update_register(&mut self, data: &[u8]) {

        let index = u16::from_le_bytes([data[1], data[2]]);
        let sub_index = data[3];
        let value = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);

    }

    async fn send_sdo_response(&mut self) {

        let data: [u8; 8] = [0; 8];

        let cob = u16::from_str_radix("600", 16).unwrap();
        let cob_id = CanId::new_base(cob | self.node_id as u16).unwrap();

        let frame = &CanFrame::new(
            cob_id,
            &data,
            None,
        )
        .unwrap();

        if let Err(_) = self.socket.send(frame).await {
            log::error!("Error sending frame");
        }

    }


}