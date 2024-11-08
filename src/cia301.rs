use std::collections::{HashMap, BTreeMap};

use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::nmt::{NmtCommand, NmtState};

use crate::eds::{DataValue, EDSData, ObjectType};
use crate::cia402_runner::{Command, State};

pub struct Node {
    pub node_id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: CanSocket,
    pub motor_controller: MotorController,
}

pub struct MotorController {
    pub mode_of_operation: i8,
    pub command: Command,
    pub state: State,
}

#[derive(Debug)]
enum ServerCommand {
    
	/// The server is uploading a segment.
	_UploadSegmentResponse = 0,

	/// The server has downloaded the segment.
	_DownloadSegmentResponse = 1,

	/// The server accepts the upload request.
	InitiateUploadResponse = 2,

	/// The server accepts the download request.
	InitiateDownloadResponse = 3,

	/// The server is aborting the transfer.
	_AbortTransfer = 4,

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


impl Node {
    /// Initialize the motor controller.
    pub async fn initialize(
        socket: CanSocket,
        node_id: u8,
        eds_data: EDSData,
    ) -> Result<Self, ()> {
        let node = Self {
            node_id,
            eds_data,
            nmt_state: NmtState::Initializing,
            socket,
            motor_controller: MotorController {
                mode_of_operation: 0,
                command: Command::None,
                state: State::NotReadyToSwitchOn,
            }
        };

        Ok(node)
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
                        0x200 => self.parse_rpdo(&1, &frame.data()).await,
                        0x300 => self.parse_rpdo(&2, &frame.data()).await,
                        0x400 => self.parse_rpdo(&3, &frame.data()).await,
                        0x500 => self.parse_rpdo(&4, &frame.data()).await,
                        0x600 => self.parse_sdo_client_request(&frame.data()).await,
                        _ => {},
                    }
                }

                self.update_controller().await;
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

    async fn parse_rpdo(&mut self, rpdo_number: &u8, input_data: &[u8]) {

        let mut enabled_sub_indices: u8 = 0;
        let mut rpdo_indices: HashMap<u8, u32> = HashMap::new();

        let rpdo_index = match rpdo_number {
            1 => 0x1600,
            2 => 0x1601,
            3 => 0x1602,
            4 => 0x1603,
            _ => panic!("Rpdo not implemented")
        };

        for object in self.eds_data.od.iter() {

            match object {

                ObjectType::Var(content) => {

                    if content.index == rpdo_index {
                        if content.sub_index == 0 {
                            match content.value {
                                DataValue::Unsigned8(value) => {
                                    enabled_sub_indices = value;
                                }
                                _ => {},
                            }
                        } else {
                            match content.value {
                                DataValue::Unsigned32(value) => {
                                    rpdo_indices.insert(content.sub_index, value);
                                }
                                _ => {},
                            }
                        }
                    }


                    
                }
                _ => {},
            }
        }

        let mut data = input_data;

        for i in 0..enabled_sub_indices {

            if let Some(rpdo_index_value) = rpdo_indices.get(&(i + 1)) {


                let index_to_set = (rpdo_index_value >> 16) as u16;
                let sub_index_to_set = ((rpdo_index_value >> 8) & 0xFF) as u8;
                let data_type = (rpdo_index_value & 0xFF) as u8;

                for object in self.eds_data.od.iter_mut() {

                    match object {
        
                        ObjectType::Var(content) => {
                            if content.index == index_to_set && content.sub_index == sub_index_to_set {

                                // Break loop when there is no data left
                                if data == &[] {
                                    break;
                                }

                                match (data_type, &content.value) {
                                    (0x08, DataValue::Unsigned8(_)) => {
                                        content.value = DataValue::Unsigned8(data[0]);
                                        data = drop_front(data, 1);
                                    }
                                    (0x08, DataValue::Integer8(_)) => {
                                        content.value = DataValue::Integer8(data[0] as i8);
                                        data = drop_front(data, 1);
                                    }
                                    (0x10, DataValue::Unsigned16(_)) => {
                                        content.value = DataValue::Unsigned16(u16::from_le_bytes([data[0], data[1]]));
                                        data = drop_front(data, 2);
                                    }
                                    (0x10, DataValue::Integer16(_)) => {
                                        content.value = DataValue::Integer16(i16::from_le_bytes([input_data[0], input_data[1]]));
                                        data = drop_front(data, 2);
                                    }
                                    (0x20, DataValue::Unsigned32(_)) => {
                                        content.value = DataValue::Unsigned32(u32::from_le_bytes([input_data[0], input_data[1], input_data[2], input_data[3]]));
                                        data = drop_front(data, 4);
                                    }
                                    (0x20, DataValue::Integer32(_)) => {
                                        content.value = DataValue::Integer32(i32::from_le_bytes([input_data[0], input_data[1], input_data[2], input_data[3]]));
                                        data = drop_front(data, 4);
                                    }
                                    _ => log::error!("Data type not implemented. Data type: 0x{:X}, data value: {:?}", data_type, content.value)
                                };
                            }
                        }
                        _ => {},
                    }
                }
            }

        }
    }

    async fn parse_sync(&self) {

        let mut tpdos_enabled: BTreeMap<u16, bool> = BTreeMap::new();
        let mut tpdos_sync_type: BTreeMap<u16, u8> = BTreeMap::new();
        let mut number_of_entries: BTreeMap<u16, u8> = BTreeMap::new();
        let mut tpdo_objects: BTreeMap<&u16, BTreeMap<u8, u32>> = BTreeMap::new();

        for object in self.eds_data.od.iter() {

            match object {

                ObjectType::Var(content) => {

                    let base_index = content.index & 0xFF00;
                    let pdo_number = content.index & 0x7F;

                    if base_index == 0x1800 {

                        for i in 0..8 {

                            if i == pdo_number {
                                
                                if content.sub_index == 1 {

                                    match content.value {
                                        DataValue::Unsigned32(value) => {
                                            let tpdo_enabled = !((value & (1 << 31)) != 0);
                                            tpdos_enabled.insert(i, tpdo_enabled);
                                        }
                                        _ => {},
                                    }

                                }

                                if content.sub_index == 2 {
                                    match content.value {
                                        DataValue::Unsigned8(value) => {
                                            tpdos_sync_type.insert(i, value);
                                        }
                                        _ => {},
                                    }
                                }

                            }

                        }

                    }

                    if base_index == 0x1A00 {

                        for i in 0..8 {

                            if i == pdo_number {
                                
                                if content.sub_index == 0 {

                                    match content.value {
                                        DataValue::Unsigned8(value) => {
                                            number_of_entries.insert(i, value);
                                        }
                                        _ => {},
                                    }
                                }
                            }
                        }
                    }

                }
                _ => {},

            }

        }

        for object in self.eds_data.od.iter() {

    
            match object {

                ObjectType::Var(content) => {

                    let base_index = content.index & 0xFF00;

                    if base_index == 0x1A00 {

                        for tpdo_number in tpdos_enabled.keys() {
                            if let (Some(tpdo_enabled), Some(sync_type), Some(number_of_entries)) = (tpdos_enabled.get(tpdo_number), tpdos_sync_type.get(tpdo_number), number_of_entries.get(tpdo_number)) {
                                if *tpdo_enabled && (*sync_type == 255) {
                                    if content.index == (base_index | tpdo_number) {
                                        for i in 1..(*number_of_entries + 1) {
                                            if content.sub_index == i {
                                                match content.value {
                                                    DataValue::Unsigned32(value) => {
                                                        tpdo_objects.entry(tpdo_number)
                                                            .or_insert_with(BTreeMap::new)
                                                            .insert(content.sub_index, value);  
                                                    }
                                                    _ => {},
                                                }
                                            }  
                                        }                                 

                                    }
                                }
                            }
                        }
                    
                    }

                }
                _ => {},

            }

        }

        for tpdo_object_nr in tpdo_objects.keys() {

            let mut data_to_send: Vec<u8> = Vec::new();

            if let Some(tpdo_sub_indices) = tpdo_objects.get(tpdo_object_nr) {

                let tpdo_number = tpdo_object_nr;

                for sub_index in tpdo_sub_indices.keys() {

                    if let Some(tpdo_content) = tpdo_sub_indices.get(sub_index) {

                        for object in self.eds_data.od.iter() {

                            match object {
                
                                ObjectType::Var(content) => {
                
                                    let index_to_find = (tpdo_content >> 16) as u16;
                                    let data_type = (tpdo_content & 0xFF) as u8;
        
                                    if index_to_find == content.index {
                                        match data_type {
                                            0x08 => match content.value {
                                                DataValue::Unsigned8(value) => {
                                                    data_to_send.extend(&value.to_le_bytes())
                                                }
                                                DataValue::Integer8(value) => {
                                                    data_to_send.extend(&value.to_le_bytes())
                                                }
                                                _ => {},
                                            }
                                            0x10 => match content.value {
                                                DataValue::Unsigned16(value) => {
                                                    data_to_send.extend(&value.to_le_bytes())
                                                }
                                                DataValue::Integer16(value) => {
                                                    data_to_send.extend(&value.to_le_bytes())
                                                }
                                                _ => {},
                                            }
                                            0x20 => match content.value {
                                                DataValue::Unsigned32(value) => {
                                                    data_to_send.extend(&value.to_le_bytes())
                                                }
                                                DataValue::Integer32(value) => {
                                                    data_to_send.extend(&value.to_le_bytes())
                                                }
                                                _ => {},
                                            }
                                            _ => {},
                                        }
                                    }
                                
                                }
                                _ => {},
                            
                            }
                
                        }
        
                    }
                
                }
    
                let functions_code = u16::from_str_radix(format!("{}80", *tpdo_number + 1).as_str(), 16).unwrap();
                let cob_id = CanId::new_base(functions_code | self.node_id as u16).unwrap();
    
                let frame = &CanFrame::new(
                    cob_id,
                    &data_to_send.as_slice(),
                    None,
                )
                .unwrap();

                if let Err(_) = self.socket.send(frame).await {
                    log::error!("Error sending frame");
                }
            }
        }
    }

    async fn parse_emcy(&mut self) {

        println!("Emcy");

    }

}

fn drop_front(slice: &[u8], count: usize) -> &[u8] {
    if count > slice.len() {
        &[]
    } else {
        &slice[count..]
    }
}