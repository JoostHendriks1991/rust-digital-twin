use ini::Ini;
use std::fs;
use std::collections::BTreeMap;

#[derive(Debug)]
pub struct Var {
    pub value: DataValue,
}

#[derive(Debug)]
pub struct EDSData {
    pub od: BTreeMap<u16, BTreeMap<u8, Var>>,
}

#[derive(Debug, Clone)]
pub enum DataType {
    Unknown,
    Boolean,
    Integer8,
    Integer16,
    Integer32,
    Unsigned8,
    Unsigned16,
    Unsigned32,
    Real32,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub enum DataValue {
    Unknown(i32),
    Boolean(bool),
    Integer8(i8),
    Integer16(i16),
    Integer32(i32),
    Unsigned8(u8),
    Unsigned16(u16),
    Unsigned32(u32),
    Real32(f32),
}

fn get_data_type(data_type: &u32) -> DataType {
    match data_type {
        0x0001 => DataType::Boolean,
        0x0002 => DataType::Integer8,
        0x0003 => DataType::Integer16,
        0x0004 => DataType::Integer32,
        0x0005 => DataType::Unsigned8,
        0x0006 => DataType::Unsigned16,
        0x0007 => DataType::Unsigned32,
        0x0008 => DataType::Real32,
        _ => DataType::Unknown,
    }
}

// Function to parse the default value into a typed DataValue
fn parse_default_value(node_id: u8, data_type: DataType, default_value: &str) -> Result<DataValue, String> {

    let default_value = if default_value.is_empty() { "0" } else { default_value };

    match data_type {
        DataType::Unknown => {
            Ok(DataValue::Unknown(0))
        }
        DataType::Boolean => {
            let val = default_value.parse::<bool>().map_err(|_| "Invalid Boolean value")?;
            Ok(DataValue::Boolean(val))
        }
        DataType::Integer8 => {
            if default_value.contains("0x") {
                let val = i8::from_str_radix(default_value.trim_start_matches("0x"), 16).map_err(|_| "Invalid i8 value")?;
                Ok(DataValue::Integer8(val))
            } else {
                let val = default_value.parse::<i8>().map_err(|_| "Invalid i8 value")?;
                Ok(DataValue::Integer8(val))
            }
        }
        DataType::Integer16 => {
            if default_value.contains("0x") {
                let val = i16::from_str_radix(default_value.trim_start_matches("0x"), 16).map_err(|_| "Invalid i16 value")?;
                Ok(DataValue::Integer16(val))
            } else {
                let val = default_value.parse::<i16>().map_err(|_| "Invalid i16 value")?;
                Ok(DataValue::Integer16(val))
            }
        }
        DataType::Integer32 => {
            if default_value.contains("0x") {
                let val = i32::from_str_radix(default_value.trim_start_matches("0x"), 16).map_err(|_| "Invalid i32 value")?;
                Ok(DataValue::Integer32(val))
            } else {
                let val = default_value.parse::<i32>().map_err(|_| "Invalid i32 value")?;
                Ok(DataValue::Integer32(val))
            }
        }
        DataType::Unsigned8 => {
            if default_value.contains("0x") {
                let val = u8::from_str_radix(default_value.trim_start_matches("0x"), 16).map_err(|_| "Invalid u8 value")?;
                Ok(DataValue::Unsigned8(val))
            } else {
                let val = default_value.parse::<u8>().map_err(|_| "Invalid u8 value")?;
                Ok(DataValue::Unsigned8(val))
            }
        }
        DataType::Unsigned16 => {
            if default_value.contains("0x") {
                let val = u16::from_str_radix(default_value.trim_start_matches("0x"), 16).map_err(|_| "Invalid u16 value")?;
                Ok(DataValue::Unsigned16(val))
            } else if default_value.contains("$NODEID") {
                let val = u16::from_str_radix(default_value.trim_start_matches("$NODEID+0x"), 16).map_err(|_| "Invalid u16 value")? | node_id as u16;
                Ok(DataValue::Unsigned16(val))
            } else {
                let val = default_value.parse::<u16>().map_err(|_| "Invalid u16 value")?;
                Ok(DataValue::Unsigned16(val))
            }
        }
        DataType::Unsigned32 => {
            if default_value.contains("$NODEID") {
                let val = (u16::from_str_radix(default_value.trim_start_matches("$NODEID+0x"), 16).map_err(|_| "Invalid u32 value")? | node_id as u16) as u32;
                Ok(DataValue::Unsigned32(val))
            } else if default_value.contains("0x") {
                    let val = u32::from_str_radix(default_value.trim_start_matches("0x"), 16).map_err(|_| "Invalid u32 value")?;
                    Ok(DataValue::Unsigned32(val))
            } else {
                let val = default_value.parse::<u32>().map_err(|_| "Invalid u32 value")?;
                Ok(DataValue::Unsigned32(val))
            }
        }
        DataType::Real32 => {

            let val = default_value.parse::<f32>().map_err(|_| "Invalid f32 value")?;
            Ok(DataValue::Real32(val))

        }
    }
}


pub fn parse_eds(node_id: &u8, eds_file: &String) -> Result<EDSData, Box<dyn std::error::Error>> {
    
    // Load the EDS file
    let eds_content = fs::read_to_string(eds_file)?;
    
    // Parse the INI content
    let ini = Ini::load_from_str(&eds_content)?;

    // Extact Objects
    let mut od = BTreeMap::new();

    for section in ini.sections().flatten() {

        let (index, sub_index) = parse_section(section);
        let object_type = parse_str_to_u8(ini.section(Some(section)).unwrap().get("ObjectType").unwrap_or("0"));
        let data_type = get_data_type(&parse_str_to_u32(ini.section(Some(section)).unwrap().get("DataType").unwrap_or("0")));
        let default_value = ini.section(Some(section)).unwrap().get("DefaultValue").unwrap_or_default().to_string();

        if object_type == 0x7 {

            let var = Var {
                value: parse_default_value(*node_id, data_type.clone(), default_value.clone().as_str()).unwrap(),
            };

            log::debug!("Adding object with index: 0x{:X}, Sub Index: {}, Object type: {:?}, Default value: {}", index, sub_index, object_type, default_value);

            od.entry(index)
                .or_insert_with(BTreeMap::new)
                .insert(sub_index, var);

        }

    }

    // Create EDSData struct
    let eds_data = EDSData {
        od
    };

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
        // If no "sub" prefix or parsing sub_index fails, return None
        return (index, 0);
    } else {
        return (0, 0)
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