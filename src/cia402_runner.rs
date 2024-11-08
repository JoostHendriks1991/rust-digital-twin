use crate::cia301::Node;
use crate::eds::{DataValue, ObjectType};

/// Controlword
#[derive(Default)]
pub enum Command {
    #[default]
    None,
	Shutdown,
	SwitchOn,
	DisableVoltage,
	QuickStop,
	DisableOperation,
	EnableOperation,
	EnableOperationAfterQuickStop,
	FaultReset
}

/// Statusword
#[derive(Default, Debug, PartialEq)]
pub enum State {
    #[default]
	NotReadyToSwitchOn,
	SwitchedOnDisabled,
	ReadyToSwitchOn,
	SwitchedOn,
	OperationEnabled,
	QuickStopActive,
	FaultReactionActive,
	Fault
}

impl Node {

    pub async fn update_controller(&mut self) {

        let mut command = Command::None;

        // Check objects and adjust motor controller status
        for object in self.eds_data.od.iter() {

            match object {

                ObjectType::Var(content) => {

                    if content.index == 0x6060 && content.sub_index == 0 {
                        match content.value {
                            DataValue::Integer8(value) => {
                                self.motor_controller.mode_of_operation = value;
                            }
                            _ => {},
                        }
                    }

                    if content.index == 0x6040 && content.sub_index == 0 {
                        match content.value {
                            DataValue::Unsigned16(value) => {
                                command = get_command(&value);
                            }
                            _ => {},
                        }
                    }

                }
                _ => {},
            }
        }

        // Do logic based on input
        self.motor_controller.state = self.update_state(&command);

        // Adjust eds according to motor controller status
        for object in self.eds_data.od.iter_mut() {

            match object {

                ObjectType::Var(content) => {

                    if content.index == 0x6061 && content.sub_index == 0 {

                        match content.value {
                            DataValue::Integer8(_) => content.value = DataValue::Integer8(self.motor_controller.mode_of_operation),
                            _ => {},
                        }
            
                    }

                    if content.index == 0x6041 && content.sub_index == 0 {
                        
                        match content.value {
                            DataValue::Unsigned16(_) => content.value = DataValue::Unsigned16(construct_statusword(&self.motor_controller.state)),
                            _ => {},
                        }

                    }
                }
                _ => {},
            }
        }

    }

    fn update_state(&mut self, command: &Command) -> State {

        match self.motor_controller.state {
            State::NotReadyToSwitchOn => State::SwitchedOnDisabled,
            State::SwitchedOnDisabled => match command {
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOnDisabled,
            }
            State::ReadyToSwitchOn => match command {
                Command::SwitchOn => State::SwitchedOn,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::ReadyToSwitchOn,
            }
            State::SwitchedOn => match command {
                Command::EnableOperation => State::OperationEnabled,
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOn,
            }
            State::OperationEnabled => match command {
                Command::QuickStop => State::QuickStopActive,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::OperationEnabled,
            }
            State::QuickStopActive => match command {
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::EnableOperationAfterQuickStop => State::OperationEnabled,
                _ => State::QuickStopActive,
            }
            State::FaultReactionActive => State::Fault,
            State::Fault => match command {
                Command::FaultReset => State::SwitchedOnDisabled,
                _ => State::Fault,
            }

        }
    }

}

fn get_command(controlword: &u16) -> Command {
    const BIT_INDICES: [usize; 5] = [0, 1, 2, 3, 7];
    
    let bits: Vec<bool> = BIT_INDICES.iter().map(|&i| get_bit_16(&controlword, i)).collect();

    match (bits[4], bits[3], bits[2], bits[1], bits[0]) {
        (false, false, false, false, false) => Command::None,
        (false, _, true, true, false) => Command::Shutdown,
        (false, false, true, true, true) => Command::SwitchOn,
        (false, _, _, false, _) => Command::DisableVoltage,
        (false, _, false, true, _) => Command::QuickStop,
        (false, true, true, true, true) => Command::EnableOperation,
        (true, _, _, _, _) => Command::FaultReset,
    }
}

fn get_bit_16(u16_value: &u16, index: usize) -> bool {
    let mask = 1 << index;
    (u16_value & mask) != 0
}

fn construct_statusword(state: &State) -> u16 {

    let mut statusword: u16 = 0;

    match state {
        State::NotReadyToSwitchOn => {
            statusword = set_bit_16(&statusword, 0, false);
            statusword = set_bit_16(&statusword, 1, false);
            statusword = set_bit_16(&statusword, 2, false);
            statusword = set_bit_16(&statusword, 3, false);
            statusword = set_bit_16(&statusword, 5, false);
            statusword = set_bit_16(&statusword, 6, false);
        }
        State::SwitchedOnDisabled => {
            statusword = set_bit_16(&statusword, 0, false);
            statusword = set_bit_16(&statusword, 1, false);
            statusword = set_bit_16(&statusword, 2, false);
            statusword = set_bit_16(&statusword, 3, false);
            statusword = set_bit_16(&statusword, 6, true);
        }
        State::ReadyToSwitchOn => {
            statusword = set_bit_16(&statusword, 0, true);
            statusword = set_bit_16(&statusword, 1, false);
            statusword = set_bit_16(&statusword, 2, false);
            statusword = set_bit_16(&statusword, 3, false);
            statusword = set_bit_16(&statusword, 5, true);
            statusword = set_bit_16(&statusword, 6, false);
        }
        State::SwitchedOn => {
            statusword = set_bit_16(&statusword, 0, true);
            statusword = set_bit_16(&statusword, 1, true);
            statusword = set_bit_16(&statusword, 2, false);
            statusword = set_bit_16(&statusword, 3, false);
            statusword = set_bit_16(&statusword, 5, true);
            statusword = set_bit_16(&statusword, 6, false);
        }
        State::OperationEnabled => {
            statusword = set_bit_16(&statusword, 0, true);
            statusword = set_bit_16(&statusword, 1, true);
            statusword = set_bit_16(&statusword, 2, true);
            statusword = set_bit_16(&statusword, 3, false);
            statusword = set_bit_16(&statusword, 5, true);
            statusword = set_bit_16(&statusword, 6, false);
        }
        State::QuickStopActive => {
            statusword = set_bit_16(&statusword, 0, true);
            statusword = set_bit_16(&statusword, 1, true);
            statusword = set_bit_16(&statusword, 2, true);
            statusword = set_bit_16(&statusword, 3, false);
            statusword = set_bit_16(&statusword, 5, false);
            statusword = set_bit_16(&statusword, 6, false);
        }
        State::FaultReactionActive => {
            statusword = set_bit_16(&statusword, 0, true);
            statusword = set_bit_16(&statusword, 1, true);
            statusword = set_bit_16(&statusword, 2, true);
            statusword = set_bit_16(&statusword, 3, true);
            statusword = set_bit_16(&statusword, 6, false);
        }
        State::Fault => {
            statusword = set_bit_16(&statusword, 0, false);
            statusword = set_bit_16(&statusword, 1, false);
            statusword = set_bit_16(&statusword, 2, false);
            statusword = set_bit_16(&statusword, 3, true);
            statusword = set_bit_16(&statusword, 6, false);
        }
    }

    statusword
}

fn set_bit_16(u16_value: &u16, bit_position: usize, value: bool) -> u16 {
    let mask = 1 << bit_position;
    if value {
        u16_value | mask
    } else {
        u16_value & !mask
    }
}