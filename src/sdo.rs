use can_socket::{CanFrame, CanId};

use crate::eds::{DataValue, EDSData, Var};

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

}

#[derive(Debug)]
pub enum ClientCommand {

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
    pub fn client_command(value: u8) -> ClientCommand {
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


pub fn sdo_response(node_id: u8, eds_data: &mut EDSData, data: &[u8]) -> Result<CanFrame, ()> {

    if data.len() > 8 {
        log::error!("Data length too long")
    };

    let ccs = (data[0] >> 5) & 0b111;

    match ClientCommand::client_command(ccs) {
        ClientCommand::InitiateUpload => {
            return Ok(create_sdo_response_frame(node_id, sdo_upload(eds_data, data)));
        }

        ClientCommand::InitiateDownload => {
            return Ok(create_sdo_response_frame(node_id, sdo_download(eds_data, data)));
        }

        _ => Err(())
    }

}

fn sdo_upload(eds_data: &mut EDSData, input_data: &[u8]) -> [u8; 8] {

    let index_to_get = get_index(&input_data);
    let sub_index_to_get = get_sub_index(&input_data);

    let mut data: [u8; 8] = [0; 8];
    let mut n = 0;

    if let Some(var) = get_var(index_to_get, sub_index_to_get, eds_data) {
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
    } else {
        log::error!("Variable not found");
    }

    let s = 1;
    let e = 1;
    let scs = ServerCommand::InitiateUploadResponse;

    data[0] = data[0] | (scs as u8 & 0b111) << 5;
    data[0] = data[0] | (n & 0b11) << 2;
    data[0] = data[0] | e << 1;
    data[0] = data[0] | s << 0;

    data[1..3].copy_from_slice(&index_to_get.to_le_bytes());

    data[3] = sub_index_to_get;

    data

}

fn sdo_download(eds_data: &mut EDSData, input_data: &[u8]) -> [u8; 8] {

    let index_to_set = get_index(&input_data);
    let sub_index_to_set = get_sub_index(&input_data);

    if let Some(var) = get_var(index_to_set, sub_index_to_set, eds_data) {
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
    } else {
        log::error!("Variable not found");
    }

    // Construct response data
    let scs = ServerCommand::InitiateDownloadResponse;

    let mut data: [u8; 8] = [0; 8];

    data[0] = data[0] | (scs as u8 & 0b111) << 5;

    data[1..3].copy_from_slice(&index_to_set.to_le_bytes());

    data[3] = sub_index_to_set;

    data

}

fn get_var(index: u16, sub_index: u8, eds_data: &mut EDSData) -> Option<&mut Var> {

    if let Some(var) = eds_data.od.get_mut(&index).and_then(|vars| vars.get_mut(&sub_index)) {
        Some(var)
    } else {
        None
    }

}

fn get_index(data: &[u8]) -> u16 {
    u16::from_le_bytes([data[1], data[2]])
}

fn get_sub_index(data: &[u8]) -> u8 {
    data[3]
}

fn create_sdo_response_frame(node_id: u8, data: [u8; 8]) -> CanFrame {

    let cob = u16::from_str_radix("580", 16).unwrap();
    let cob_id = CanId::new_base(cob | node_id as u16).unwrap();

    CanFrame::new(
        cob_id,
        &data,
        None,
    )
    .unwrap()
}