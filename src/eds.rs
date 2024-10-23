use ini::Ini;
use std::fs;

#[derive(Debug)]
pub struct FileInfo {
    pub file_name: String,
    pub file_version: u32,
    pub file_revision: u32,
    pub eds_version: f32,
    pub description: String,
    pub created_by: String,
}

#[derive(Debug)]
pub struct DeviceInfo {
    pub vendor_name: String,
    pub vendor_number: u32,
    pub product_name: String,
    pub product_number: u32,
}

#[derive(Debug)]
struct Object {
    index: u16,
    sub_index: u8,
    parameter_name: String,
    object_type: u8,
    data_type: u32,
    access_type: String,
    default_value: String,
    pdo_mapping: bool,
    sub_number: u8,
}

#[derive(Debug)]
pub struct EDSData {
    pub file_info: FileInfo,
    pub device_info: DeviceInfo,
    od: Vec<Object>,
}

pub fn parse_eds() -> Result<EDSData, Box<dyn std::error::Error>> {
    // Load the EDS file
    let eds_content = fs::read_to_string("CPB3-1-2.eds")?;
    
    // Parse the INI content
    let ini = Ini::load_from_str(&eds_content)?;

    // Extract DeviceInfo
    let file_info_section = ini.section(Some("FileInfo")).expect("Missing FileInfo section");
    let file_info = FileInfo {
        file_name: file_info_section.get("FileName").unwrap_or_default().to_string(),
        file_version: file_info_section.get("FileVersion").unwrap_or("0").parse().unwrap_or(0),
        file_revision: file_info_section.get("FileRevision").unwrap_or("0").parse().unwrap_or(0),
        eds_version: file_info_section.get("EDSVersion").unwrap_or("0.0").parse().unwrap_or(0.0),
        description: file_info_section.get("Description").unwrap_or_default().to_string(),
        created_by: file_info_section.get("CreatedBy").unwrap_or_default().to_string(),
    };

    // Extract DeviceInfo
    let device_info_section = ini.section(Some("DeviceInfo")).expect("Missing DeviceInfo section");
    let device_info = DeviceInfo {
        vendor_name: device_info_section.get("VendorName").unwrap_or_default().to_string(),
        vendor_number: device_info_section.get("VendorNumber").unwrap_or("0").parse().unwrap_or(0),
        product_name: device_info_section.get("ProductName").unwrap_or_default().to_string(),
        product_number: device_info_section.get("ProductNumber").unwrap_or("0").parse().unwrap_or(0),
    };

    // Extact Objects
    let mut od = Vec::new();

    for section in ini.sections().flatten() {

        let (index, sub_index) = parse_section(section);

        let object = Object {
            index,
            sub_index,
            parameter_name: ini.section(Some(section)).unwrap().get("ParameterName").unwrap_or_default().to_string(),
            object_type: parse_str_to_u8(ini.section(Some(section)).unwrap().get("ObjectType").unwrap_or("0")),
            data_type: parse_str_to_u32(ini.section(Some(section)).unwrap().get("DataType").unwrap_or("0")),
            access_type: ini.section(Some(section)).unwrap().get("AccessType").unwrap_or_default().to_string(),
            default_value: ini.section(Some(section)).unwrap().get("DefaultValue").unwrap_or_default().to_string(),
            pdo_mapping: parse_str_to_bool(ini.section(Some(section)).unwrap().get("PDOMapping").unwrap_or_default()),
            sub_number: parse_str_to_u8(ini.section(Some(section)).unwrap().get("SubNumber").unwrap_or("0")),
        };
        od.push(object);

    }

    // Create EDSData struct
    let eds_data = EDSData {
        file_info,
        device_info,
        od
    };

    println!("Test: {}", eds_data.od[0].index);

    Ok(eds_data)
}


fn parse_section(section: &str) -> (u16, u8) {

    // Split the input into the prefix and suffix
    let (prefix, suffix) = section.split_at(4);

    // Check if the prefix is exactly 4 hex digits
    if let Ok(index) = u16::from_str_radix(prefix, 16) {
        if let Some(suffix) = suffix.strip_prefix("sub") {
            // Try to parse the sub_index from the suffix
            if let Ok(sub_index) = u8::from_str_radix(suffix, 16) {
                return (index, sub_index);
            }
        }
        // If no "sub" prefix or parsing sub_index fails, return MainObject
        return (index, 0);
    } else {
        return (0, 0)
    }
}

fn parse_str_to_bool(hex_str: &str) -> bool {

    match hex_str {
        "0" => false,
        "1" => true,
        _ => false,
    }
}

fn parse_str_to_u8(hex_str: &str) -> u8 {

    // Remove the "0x" prefix if it exists
    let trimmed_hex = hex_str.trim_start_matches("0x");

    // Convert the trimmed string to a u8 using base 16
    u8::from_str_radix(trimmed_hex, 16).unwrap()
}

fn parse_str_to_u32(hex_str: &str) -> u32 {

    // Remove the "0x" prefix if it exists
    let trimmed_hex = hex_str.trim_start_matches("0x");

    // Convert the trimmed string to a u8 using base 16
    u32::from_str_radix(trimmed_hex, 16).unwrap()
}