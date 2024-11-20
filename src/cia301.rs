use std::collections::BTreeMap;
use tokio::sync::mpsc;

use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::nmt::{NmtCommand, NmtState};

use crate::cia402_runner::{Message, MessagaType};
use crate::eds::{DataValue, EDSData};

pub struct Node {
    tx: mpsc::Sender<Message>,
    rx: mpsc::Receiver<Message>,
    pub node_id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: CanSocket,
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

    pub fn new(
        tx: mpsc::Sender<Message>,
        rx: mpsc::Receiver<Message>,
        socket: CanSocket,
        node_id: u8,
        eds_data: EDSData,
    ) -> Result<Self, ()> {
        let node = Self {
            tx,
            rx,
            node_id,
            eds_data,
            nmt_state: NmtState::Initializing,
            socket,
        };
        Ok(node)
    }

    pub async fn start_socket(&mut self) {

        // Start receiving frames over socket
        loop {

            // Check if a frame is received
            if let Some(frame) = self.socket.recv().await.ok() {
                self.handle_frame(frame).await;
            }

            match self.rx.try_recv() {
                Ok(data) => self.update_od(data).await,
                Err(_) => {},
            }

        }

    }

    async fn handle_frame(&mut self, frame: CanFrame) {

        // Extract id and cob_id
        let cob_id = frame.id().as_u32();
        let node_id = (cob_id & 0x7F) as u8;
        let function_code = frame.id().as_u32() & (0x0F << 7);

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

    async fn send_new_nmt_state(&mut self) {

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

        if let Some(var) = self.eds_data.od.get_mut(&input_index)
            .and_then(|vars| vars.get_mut(&input_sub_index)) {

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

                        match var.value {
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
                        match var.value {
                            DataValue::Integer8(_) => var.value = DataValue::Integer8(input_data[4] as i8),
                            DataValue::Integer16(_) => var.value = DataValue::Integer16(i16::from_le_bytes([input_data[4], input_data[5]])),
                            DataValue::Integer32(_) => var.value = DataValue::Integer32(i32::from_le_bytes([input_data[4], input_data[5], input_data[6], input_data[7]])),
                            DataValue::Unsigned8(_) => var.value = DataValue::Unsigned8(input_data[4]),
                            DataValue::Unsigned16(_) => var.value = DataValue::Unsigned16(u16::from_le_bytes([input_data[4], input_data[5]])),
                            DataValue::Unsigned32(_) => var.value = DataValue::Unsigned32(u32::from_le_bytes([input_data[4], input_data[5], input_data[6], input_data[7]])),
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

                data[1..3].copy_from_slice(&input_index.to_le_bytes());

                data[3] = input_sub_index;

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

    async fn parse_rpdo(&mut self, rpdo_number: &u16, input_data: &[u8]) {

        let mut enabled_sub_indices: u8 = 0;
        let mut rpdo_indices: BTreeMap<u8, u32> = BTreeMap::new();

        if let Some(var) = self.eds_data.od.get(&(0x1600 | (*rpdo_number - 1)))
            .and_then(|vars| vars.get(&0)) {
                match var.value {
                    DataValue::Unsigned8(value) => {
                        enabled_sub_indices = value;
                    }
                    _ => {},
                }
            }

        if let Some(vars) = self.eds_data.od.get(&(0x1600 | (*rpdo_number - 1))) {
            for (sub_index, var) in vars.iter() {
                if *sub_index != 0 as u8 {
                    match var.value {
                        DataValue::Unsigned32(value) => {
                            rpdo_indices.insert(*sub_index, value);
                        }
                        _ => {},
                    }
                }
            }
        }

        let mut data = input_data;

        for i in 0..enabled_sub_indices {

            if let Some(rpdo_index_value) = rpdo_indices.get(&(i + 1)) {

                let index_to_set = (rpdo_index_value >> 16) as u16;
                let sub_index_to_set = ((rpdo_index_value >> 8) & 0xFF) as u8;
                let data_type = (rpdo_index_value & 0xFF) as u8;

                if let Some(vars) = self.eds_data.od.get_mut(&index_to_set) {

                    for (sub_index, var) in vars.iter_mut() {

                        if sub_index == &sub_index_to_set {

                            // Break loop when there is no data left
                            if data == &[] {
                                break;
                            }

                            match (data_type, &var.value) {
                                (0x08, DataValue::Unsigned8(_)) => {
                                    var.value = DataValue::Unsigned8(data[0]);
                                    data = drop_front(data, 1);
                                }
                                (0x08, DataValue::Integer8(_)) => {
                                    var.value = DataValue::Integer8(data[0] as i8);
                                    data = drop_front(data, 1);
                                }
                                (0x10, DataValue::Unsigned16(_)) => {
                                  var.value = DataValue::Unsigned16(u16::from_le_bytes([data[0], data[1]]));
                                    data = drop_front(data, 2);
                                }
                                (0x10, DataValue::Integer16(_)) => {
                                    var.value = DataValue::Integer16(i16::from_le_bytes([data[0], data[1]]));
                                    data = drop_front(data, 2);
                                }
                                (0x20, DataValue::Unsigned32(_)) => {
                                    var.value = DataValue::Unsigned32(u32::from_le_bytes([data[0], data[1], data[2], data[3]]));
                                    data = drop_front(data, 4);
                                }
                                (0x20, DataValue::Integer32(_)) => {
                                    var.value = DataValue::Integer32(i32::from_le_bytes([data[0], data[1], data[2], data[3]]));
                                    data = drop_front(data, 4);
                                }
                                _ => log::error!("Data type not implemented. Data type: 0x{:X}, data value: {:?}", data_type, var.value)
                            };
                        }
                    }
                }
            }
        }
    }  

    async fn parse_sync(&self) {
        let mut tpdos_enabled = BTreeMap::new();
        let mut tpdos_sync_type = BTreeMap::new();
        let mut number_of_entries = BTreeMap::new();
        let mut tpdo_objects = BTreeMap::new();
    
        for i in 0..8 {
            self.extract_tpdo_data(i, &mut tpdos_enabled, &mut tpdos_sync_type);
            self.extract_entry_count(i, &mut number_of_entries);
        }
    
        for (&tpdo_number, &tpdo_enabled) in tpdos_enabled.iter() {
            if let Some(&sync_type) = tpdos_sync_type.get(&tpdo_number) {
                if tpdo_enabled && sync_type == 255 {
                    self.collect_tpdo_objects(tpdo_number, number_of_entries.get(&tpdo_number), &mut tpdo_objects);
                }
            }
        }
    
        for (&tpdo_number, tpdo_sub_indices) in tpdo_objects.iter() {
            self.process_and_send_tpdo(tpdo_number, tpdo_sub_indices).await;
        }
    }
    
    fn extract_tpdo_data(&self, index: u16, tpdos_enabled: &mut BTreeMap<u16, bool>, tpdos_sync_type: &mut BTreeMap<u16, u8>) {
        if let Some(vars) = self.eds_data.od.get(&(0x1800 + index)) {
            for (&sub_index, var) in vars.iter() {
                match (sub_index, &var.value) {
                    (1, DataValue::Unsigned32(value)) => {
                        tpdos_enabled.insert(index, (value & (1 << 31)) == 0);
                    }
                    (2, DataValue::Unsigned8(value)) => {
                        tpdos_sync_type.insert(index, *value);
                    }
                    _ => {}
                }
            }
        }
    }
    
    fn extract_entry_count(&self, index: u16, number_of_entries: &mut BTreeMap<u16, u8>) {
        if let Some(vars) = self.eds_data.od.get(&(0x1A00 + index)) {
            for (&sub_index, var) in vars.iter() {
                if sub_index == 0 {
                    if let DataValue::Unsigned8(value) = var.value {
                        number_of_entries.insert(index, value);
                    }
                }
            }
        }
    }
    
    fn collect_tpdo_objects(
        &self,
        tpdo_number: u16,
        entry_count: Option<&u8>,
        tpdo_objects: &mut BTreeMap<u16, BTreeMap<u8, u32>>,
    ) {
        if let Some(&entries) = entry_count {
            if let Some(vars) = self.eds_data.od.get(&(0x1A00 + tpdo_number)) {
                for i in 1..=entries {
                    if let Some(var) = vars.get(&i) {
                        if let DataValue::Unsigned32(value) = var.value {
                            tpdo_objects.entry(tpdo_number).or_default().insert(i, value);
                        }
                    }
                }
            }
        }
    }
    
    async fn process_and_send_tpdo(&self, tpdo_number: u16, tpdo_sub_indices: &BTreeMap<u8, u32>) {
        let mut data_to_send = Vec::new();
    
        for (&sub_index, &tpdo_content) in tpdo_sub_indices {
            if let Some(data) = self.extract_tpdo_content(tpdo_content) {
                data_to_send.extend(data);
            }
        }
    
        let functions_code = u16::from_str_radix(&format!("{:X}80", tpdo_number + 1), 16).unwrap();
        let cob_id = CanId::new_base(functions_code | self.node_id as u16).unwrap();
    
        if let Err(_) = self.socket.send(&CanFrame::new(cob_id, &data_to_send, None).unwrap()).await {
            log::error!("Error sending frame");
        }
    }
    
    fn extract_tpdo_content(&self, tpdo_content: u32) -> Option<Vec<u8>> {
        let index_to_find = (tpdo_content >> 16) as u16;
        let sub_index_to_find = ((tpdo_content >> 8) & 0xFF) as u8;
        let data_type = (tpdo_content & 0xFF) as u8;
    
        if let Some(vars) = self.eds_data.od.get(&index_to_find) {
            if let Some(var) = vars.get(&sub_index_to_find) {
                return match (data_type, &var.value) {
                    (0x08, DataValue::Unsigned8(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x08, DataValue::Integer8(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x10, DataValue::Unsigned16(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x10, DataValue::Integer16(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x20, DataValue::Unsigned32(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x20, DataValue::Integer32(value)) => Some(value.to_le_bytes().to_vec()),
                    _ => None,
                };
            }
        }
        None
    }

    async fn parse_emcy(&mut self) {

        println!("Emcy");

    }

    async fn send_to_controller(&self, msg: Message) {
        if let Err(e) = self.tx.send(msg).await {
            log::error!("Failed sending data, with error: {e}")
        }
    }

    async fn update_od(&mut self, data: Message) {

        if data.msg_type == MessagaType::Set {
            if let Some(var) = self.eds_data.od.get_mut(&data.index)
            .and_then(|vars| vars.get_mut(&data.sub_index)) {
                var.value = data.value
            }
        } else if data.msg_type == MessagaType::Get {
            if let Some(var) = self.eds_data.od.get_mut(&data.index)
            .and_then(|vars| vars.get_mut(&data.sub_index)) {
                let message = Message {
                    msg_type: MessagaType::Set,
                    index: data.index,
                    sub_index: data.sub_index,
                    value: var.value.clone(),
                };
                self.send_to_controller(message).await;
            }
        }
    }

}

fn drop_front(slice: &[u8], count: usize) -> &[u8] {
    if count > slice.len() {
        &[]
    } else {
        &slice[count..]
    }
}