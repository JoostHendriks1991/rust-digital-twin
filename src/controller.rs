use std::collections::HashMap;

use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::nmt::{NmtCommand, NmtState};

use crate::eds::{DataType, DataValue, EDSData, ObjectType};

pub struct MotorController {
    pub node_id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: CanSocket,
}

#[derive(Debug)]
enum ServerCommand {
    
	/// The server is uploading a segment.
	UploadSegmentResponse = 0,

	/// The server has downloaded the segment.
	DownloadSegmentResponse = 1,

	/// The server accepts the upload request.
	InitiateUploadResponse = 2,

	/// The server accepts the download request.
	InitiateDownloadResponse = 3,

	/// The server is aborting the transfer.
	AbortTransfer = 4,

    /// Unknown server command.
    Unknown = 5,
}

#[derive(Debug)]
enum ClientCommand {

	/// Download a segment to the server.
	SegmentDownload = 0,

	/// Initiate a download to the server.
	InitiateDownload = 1,

	/// Initiate an upload from the server.
	InitiateUpload = 2,

	/// Request the server to upload a segment.
	SegmentUpload = 3,

	/// Tell the server we are aborting the transfer.
	AbortTransfer = 4,

    /// Unknown client command.
    Unknown = 5,
}

impl ServerCommand {
    fn server_command(value: u8) -> ServerCommand {
        match value {
            0 => ServerCommand::UploadSegmentResponse,
            1 => ServerCommand::DownloadSegmentResponse,
            2 => ServerCommand::InitiateUploadResponse,
            3 => ServerCommand::InitiateDownloadResponse,
            4 => ServerCommand::AbortTransfer,
            _ => ServerCommand::Unknown,
        }
    }
}

impl ClientCommand {
    fn client_command(value: u8) -> ClientCommand {
        match value {
            0 => ClientCommand::SegmentDownload,
            1 => ClientCommand::InitiateDownload,
            2 => ClientCommand::InitiateUpload,
            3 => ClientCommand::SegmentUpload,
            4 => ClientCommand::AbortTransfer,
            _ => ClientCommand::Unknown,
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
                        0x080 => self.parse_sync().await,
                        _ => {},
                    }
                } else if node_id == self.node_id {
                    match function_code {
                        0x080 => self.parse_emcy().await,
                        0x600 => self.parse_sdo_client_request(&frame.data()).await,
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

    async fn parse_sdo_client_request(&mut self, data: &[u8]) {

        if data.len() > 8 {
            log::error!("Data length too long")
        };

		let size_set = (data[0] & (1 << 0)) != 0;
        let expedited = (data[0] & (1 << 1)) != 0;
        let n = (data[0] & (1 << 2)) != 0;
        let ccs = (data[0] >> 5) & 0b111;

        let command = ClientCommand::client_command(ccs);
        self.sdo_response(&command,data).await;

    }

    async fn sdo_response(&mut self, command: &ClientCommand, input_data: &[u8]) {

        let input_index = u16::from_le_bytes([input_data[1], input_data[2]]);
        let input_sub_index = input_data[3];

        for object in self.eds_data.od.iter_mut() {

            match object {

                ObjectType::Var(content) => {

                    if (content.index == input_index) && (content.sub_index == input_sub_index) {

                        log::debug!("Object var: {:?}", content);

                        let mut data: [u8; 8] = [0; 8];
                        let mut scs = ServerCommand::Unknown;
                        let mut n = 0;
                        let mut s = 0;
                        let mut e = 0;

                        match command {
                            ClientCommand::InitiateUpload => {

                                s = 1;
                                e = 1;
                                scs = ServerCommand::InitiateUploadResponse;

                                match content.value {
                                    DataValue::Integer8(value) => {
                                        n = 3;
                                        data[4] = value as u8;
                                    }
                                    DataValue::Integer16(value) => {
                                        n = 2;
                                        data[4..6].copy_from_slice(&value.to_le_bytes());
                                    }
                                    DataValue::Integer32(value) => data[4..].copy_from_slice(&value.to_le_bytes()),
                                    DataValue::Unsigned8(value) => {
                                        n = 3;
                                        data[4] = value;
                                    }
                                    DataValue::Unsigned16(value) => {
                                        n = 2;
                                        data[4..6].copy_from_slice(&value.to_le_bytes());
                                    }
                                    DataValue::Unsigned32(value) => data[4..].copy_from_slice(&value.to_le_bytes()),
                                    _ => log::error!("Data type not implemented for initiate upload"),
                                };
                            }
                            ClientCommand::InitiateDownload => {

                                // Update value with incoming data
                                match content.value {
                                    DataValue::Integer8(_) => content.value = DataValue::Integer8(input_data[4] as i8),
                                    DataValue::Integer16(_) => content.value = DataValue::Integer16(i16::from_le_bytes([input_data[4], input_data[5]])),
                                    DataValue::Integer32(_) => content.value = DataValue::Integer32(i32::from_le_bytes([input_data[4], input_data[5], input_data[6], input_data[7]])),
                                    DataValue::Unsigned8(_) => content.value = DataValue::Unsigned8(input_data[4]),
                                    DataValue::Unsigned16(_) => content.value = DataValue::Unsigned16(u16::from_le_bytes([input_data[4], input_data[5]])),
                                    DataValue::Unsigned32(_) => content.value = DataValue::Unsigned32(u32::from_le_bytes([input_data[4], input_data[5], input_data[6], input_data[7]])),
                                    _ => log::error!("Data type not implemented for initiate download"),
                                }

                                s = 0;
                                e = 0;
                                scs = ServerCommand::InitiateDownloadResponse;
                                
                            }
                            _ => log::error!("Client command not implemented"),
                        }

                        data[0] = data[0] | (scs as u8 & 0b111) << 5;
                        data[0] = data[0] | (n & 0b11) << 2;
                        data[0] = data[0] | e << 1;
                        data[0] = data[0] | s << 0;

                        data[1..3].copy_from_slice(&content.index.to_le_bytes());

                        data[3] = content.sub_index;
        
                        let cob = u16::from_str_radix("580", 16).unwrap();
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

                _ => {},
            }

        }

    }

    async fn parse_sync(&self) {

        let mut active_tpdo: HashMap<u16, bool> = HashMap::new();
        let mut tpdo_objects: Vec<Vec<u32>> = Vec::new();

        for object in self.eds_data.od.iter() {

            let mut tpdo_enabled = false;
            let mut sync_type = 0x0;
            let mut tpdo_object: Vec<u32> = Vec::new();

            match object {

                ObjectType::Var(content) => {

                    let base_index = content.index & 0xFF00;

                    if base_index == 0x1800 {

                        let pdo_number = content.index & 0x7F;

                        for i in 0..8 {

                            if i == pdo_number {
                                
                                if content.sub_index == 1 {

                                    match content.value {
                                        DataValue::Unsigned32(value) => {
                                            tpdo_enabled = (value & (1 << 31)) != 0;
                                        }
                                        _ => {},
                                    }

                                }

                                if content.sub_index == 2 {
                                    match content.value {
                                        DataValue::Unsigned8(value) => {
                                            sync_type = value;
                                        }
                                        _ => {},
                                    }
                                }
                                let tpdo_active = tpdo_enabled && (sync_type == 0);
                                println!("Node: {}, tpdo: {i}, enabled: {}, sync_type: {}, active: {}", self.node_id, tpdo_enabled, sync_type, tpdo_active);
                                active_tpdo.insert(i, tpdo_active);

                            }

                        }

                    }

                }
                _ => {},

            }

        }
        // println!("Tpdo active: {:?}", active_tpdo);
        //     for object in self.eds_data.od.iter() {

        //         let mut tpdo_enabled = false;
        //         let mut sync_type = 0x0;
        //         let mut tpdo_object: Vec<u32> = Vec::new();
    
        //         match object {
    
        //             ObjectType::Var(content) => {

        //             if base_index == 0x1A00 {

        //                 for i in 0..tpdo_active.len() {

        //                     println!("{}", tpdo_active[i]);

        //                     let pdo_number = content.index & 0x7F;

        //                     if i as u16 == pdo_number {
        //                         let mut active_objects = 0;
        //                         if content.sub_index == 1 {
        //                             match content.value {
        //                                 DataValue::Unsigned8(value) => {
        //                                     println!("{}", value);
        //                                     active_objects = value;
        //                                 }
        //                                 _ => {},
        //                             }
        //                         }
    
        //                         for i in 1..active_objects {
        //                             if content.sub_index == i {
        //                                 match content.value {
        //                                     DataValue::Unsigned32(value) => {
        //                                         println!("Object: {}", value);
        //                                         tpdo_object.push(value);
        //                                     }
        //                                     _ => {},
        //                                 }
        //                             }
        //                         }
        //                     } 
        //                 }

        //             }
        
        //         }

        //         _ => {},

        //     }
        //     tpdo_objects.push(tpdo_object);

        // }

        // for (i, tpdo_indices) in tpdo_objects.iter().enumerate() {

        //     let mut tpdo_data: Vec<u8> = Vec::new();

        //     for tpdo_index in tpdo_indices {

        //         for object in self.eds_data.od.iter() {

        //             match object {
        
        //                 ObjectType::Var(content) => {
        
        //                     let base_index = content.index & 0xFF00;
        //                     let index_tpdo = ((tpdo_index >> 16) & 0xFFFF) as u16;
        //                     let data_type = (tpdo_index & 0xFFFF) as u16;

        //                     if index_tpdo == base_index {
        //                         match data_type {
        //                             0x0008 => match content.value {
        //                                 DataValue::Unsigned8(value) => {
        //                                     tpdo_data.extend(&value.to_le_bytes())
        //                                 }
        //                                 DataValue::Integer8(value) => {
        //                                     tpdo_data.extend(&value.to_le_bytes())
        //                                 }
        //                                 _ => {},
        //                             }
        //                             0x0010 => match content.value {
        //                                 DataValue::Unsigned16(value) => {
        //                                     tpdo_data.extend(&value.to_le_bytes())
        //                                 }
        //                                 DataValue::Integer16(value) => {
        //                                     tpdo_data.extend(&value.to_le_bytes())
        //                                 }
        //                                 _ => {},
        //                             }
        //                             0x0020 => match content.value {
        //                                 DataValue::Unsigned32(value) => {
        //                                     tpdo_data.extend(&value.to_le_bytes())
        //                                 }
        //                                 DataValue::Integer32(value) => {
        //                                     tpdo_data.extend(&value.to_le_bytes())
        //                                 }
        //                                 _ => {},
        //                             }
        //                             _ => {},
        //                         }
        //                     }
                        
        //                 }
        //                 _ => {},
                    
        //             }
        
        //         }

        //     }

        //     tpdo_data.resize(8, 0);

        //     if i > 4 {
        //         log::error!("Index too high");
        //         break;
        //     }

        //     let functions_code = u16::from_str_radix(format!("{i}80").as_str(), 16).unwrap();
        //     let cob_id = CanId::new_base(functions_code | self.node_id as u16).unwrap();

        //     let frame = &CanFrame::new(
        //         cob_id,
        //         &tpdo_data.as_slice(),
        //         None,
        //     )
        //     .unwrap();

        //     if let Err(_) = self.socket.send(frame).await {
        //         log::error!("Error sending frame");
        //     }

    }

    async fn parse_emcy(&mut self) {

        println!("Emcy");

    }

}