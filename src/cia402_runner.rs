use std::time::{Instant, Duration};
use std::collections::HashMap;

use crate::cia301::Node;
use crate::eds::{DataValue, ObjectType};

/// Operation mode
#[derive(Default, Debug, PartialEq, Clone)]
pub enum ModeOfOperation {
    #[default]
	NoMode = 0,
	ProfilePosition = 1,
	ProfileVelocity = 3,
	Homing = 6,
}

/// Controlword
#[derive(Default, PartialEq)]
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
#[derive(Default, Debug, PartialEq, Hash, Eq)]
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

/// Homing status
#[derive(Default, Debug)]
pub enum ProfilePositionStatus {
    #[default]
    SetpointAcknownlegde,
    Moving,
}

/// Homing status
#[derive(Default, Debug)]
pub enum HomeStatus {
    #[default]
    WaitingForStart,
    Homing,
}

impl ModeOfOperation {
    fn mode_of_operation(value: i8) -> ModeOfOperation {
        match value {
            0 => ModeOfOperation::NoMode,
            1 => ModeOfOperation::ProfilePosition,
            3 => ModeOfOperation::ProfileVelocity,
            6 => ModeOfOperation::Homing,
            _ => panic!("Mode of operation not implemented")
        }
    }
}

impl Node {

    pub async fn update_controller(&mut self) {

        // Check objects and adjust motor controller status
        for object in self.eds_data.od.iter() {

            match object {

                ObjectType::Var(content) => {

                    if content.index == 0x6060 && content.sub_index == 0 {
                        match content.value {
                            DataValue::Integer8(value) => {
                                self.motor_controller.mode_of_operation = ModeOfOperation::mode_of_operation(value);
                            }
                            _ => {},
                        }
                    }

                    if content.index == 0x6040 && content.sub_index == 0 {
                        match content.value {
                            DataValue::Unsigned16(value) => {
                                self.motor_controller.controlword = value;
                            }
                            _ => {},
                        }
                    }

                }
                _ => {},
            }
        }

        // Do logic based on input
        self.parse_controlword();
        self.update_state();

        match (&self.motor_controller.mode_of_operation, &self.motor_controller.state) {

            (ModeOfOperation::ProfilePosition, State::OperationEnabled) => {

                match &self.motor_controller.profile_position_status {

                    ProfilePositionStatus::SetpointAcknownlegde => {

                        self.motor_controller.status_oms1 = true;
                        self.motor_controller.target_reached = true;

                        if self.motor_controller.control_oms1[0] && !self.motor_controller.control_oms1[1] {

                            self.motor_controller.timer = Some(Instant::now());
                            self.motor_controller.profile_position_status = ProfilePositionStatus::Moving

                        }
                    }

                    ProfilePositionStatus::Moving => {

                        self.motor_controller.status_oms1 = false;
                        self.motor_controller.target_reached = false;

                        if self.motor_controller.timer.unwrap().elapsed() > Duration::from_millis(100) {
                            self.motor_controller.profile_position_status = ProfilePositionStatus::SetpointAcknownlegde
                        }

                    }

                }

            }

            (ModeOfOperation::Homing, State::OperationEnabled) => {

                match &self.motor_controller.home_status {

                    HomeStatus::WaitingForStart => {

                        self.motor_controller.target_reached = true;
                        self.motor_controller.status_oms2 = false;

                        if self.motor_controller.control_oms1[0] && !self.motor_controller.control_oms1[1] {
                            self.motor_controller.timer = Some(Instant::now());
                            self.motor_controller.home_status = HomeStatus::Homing
                        }
                    }
                    HomeStatus::Homing => {

                        self.motor_controller.target_reached = false;
                        self.motor_controller.status_oms1 = false;
                        self.motor_controller.status_oms2 = false;

                        if self.motor_controller.timer.unwrap().elapsed() > Duration::from_millis(100) {

                            self.motor_controller.target_reached = true;
                            self.motor_controller.status_oms1 = true;
                            self.motor_controller.status_oms2 = false;

                            self.motor_controller.home_status = HomeStatus::WaitingForStart
                        }
                    }

                }
            }

            _ => {},
        }
        
        self.set_statusword();

        // Adjust eds according to motor controller status
        for object in self.eds_data.od.iter_mut() {

            match object {

                ObjectType::Var(content) => {

                    if content.index == 0x6061 && content.sub_index == 0 {

                        match content.value {
                            DataValue::Integer8(_) => content.value = DataValue::Integer8(self.motor_controller.mode_of_operation.clone() as i8),
                            _ => {},
                        }
            
                    }

                    if content.index == 0x6041 && content.sub_index == 0 {
                        
                        match content.value {
                            DataValue::Unsigned16(_) => content.value = DataValue::Unsigned16(self.motor_controller.statusword),
                            _ => {},
                        }

                    }
                }
                _ => {},
            }
        }

    }

    fn parse_controlword(&mut self) {

        const BIT_INDICES: [usize; 5] = [0, 1, 2, 3, 7];
        
        let bits: Vec<bool> = BIT_INDICES.iter().map(|&i| get_bit_16(&self.motor_controller.controlword, i)).collect();
    
        self.motor_controller.command = match (bits[4], bits[3], bits[2], bits[1], bits[0]) {
            (false, _, true, true, false) => Command::Shutdown,
            (false, false, true, true, true) => Command::SwitchOn,
            (false, _, _, false, _) => Command::DisableVoltage,
            (false, _, false, true, _) => Command::QuickStop,
            (false, true, true, true, true) => Command::EnableOperation,
            (true, _, _, _, _) => Command::FaultReset,
        };

        self.motor_controller.control_oms1.push_front(get_bit_16(&self.motor_controller.controlword, 4));
        self.motor_controller.control_oms1.pop_back();
    }

    fn update_state(&mut self) {

        self.motor_controller.state = match self.motor_controller.state {
            State::NotReadyToSwitchOn => State::SwitchedOnDisabled,
            State::SwitchedOnDisabled => match &self.motor_controller.command {
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOnDisabled,
            }
            State::ReadyToSwitchOn => match &self.motor_controller.command {
                Command::SwitchOn => State::SwitchedOn,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::ReadyToSwitchOn,
            }
            State::SwitchedOn => match &self.motor_controller.command {
                Command::EnableOperation => State::OperationEnabled,
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOn,
            }
            State::OperationEnabled => match &self.motor_controller.command {
                Command::QuickStop => State::QuickStopActive,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::OperationEnabled,
            }
            State::QuickStopActive => match &self.motor_controller.command {
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::EnableOperationAfterQuickStop => State::OperationEnabled,
                _ => State::QuickStopActive,
            }
            State::FaultReactionActive => State::Fault,
            State::Fault => match &self.motor_controller.command {
                Command::FaultReset => State::SwitchedOnDisabled,
                _ => State::Fault,
            }

        };

    }
    
    fn set_statusword(&mut self) {
        let bit_configs: HashMap<State, Vec<(usize, bool)>> = HashMap::from([
            (State::NotReadyToSwitchOn, vec![(0, false), (1, false), (2, false), (3, false), (5, false), (6, false)]),
            (State::SwitchedOnDisabled, vec![(0, false), (1, false), (2, false), (3, false), (6, true)]),
            (State::ReadyToSwitchOn, vec![(0, true), (1, false), (2, false), (3, false), (5, true), (6, false)]),
            (State::SwitchedOn, vec![(0, true), (1, true), (2, false), (3, false), (5, true), (6, false)]),
            (State::OperationEnabled, vec![(0, true), (1, true), (2, true), (3, false), (5, true), (6, false)]),
            (State::QuickStopActive, vec![(0, true), (1, true), (2, true), (3, false), (5, false), (6, false)]),
            (State::FaultReactionActive, vec![(0, true), (1, true), (2, true), (3, true), (6, false)]),
            (State::Fault, vec![(0, false), (1, false), (2, false), (3, true), (6, false)]),
        ]);
    
        if let Some(bits) = bit_configs.get(&self.motor_controller.state) {
            set_bits(&mut self.motor_controller.statusword, bits);
        }
    
        self.motor_controller.statusword = set_bit_16(&self.motor_controller.statusword, 10, self.motor_controller.target_reached);
        self.motor_controller.statusword = set_bit_16(&self.motor_controller.statusword, 12, self.motor_controller.status_oms1);
        self.motor_controller.statusword = set_bit_16(&self.motor_controller.statusword, 13, self.motor_controller.status_oms2);
    }

}

fn get_bit_16(u16_value: &u16, index: usize) -> bool {
    let mask = 1 << index;
    (u16_value & mask) != 0
}

fn set_bit_16(u16_value: &u16, bit_position: usize, value: bool) -> u16 {
    let mask = 1 << bit_position;
    if value {
        u16_value | mask
    } else {
        u16_value & !mask
    }
}

fn set_bits(statusword: &mut u16, bits: &[(usize, bool)]) {
    for &(bit, value) in bits {
        *statusword = set_bit_16(statusword, bit, value);
    }
}