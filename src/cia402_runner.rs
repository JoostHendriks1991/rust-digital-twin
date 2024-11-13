use std::time::{Instant, Duration};
use std::collections::HashMap;
use std::collections::VecDeque;
use tokio::sync::mpsc;

use crate::eds::DataValue;

pub struct MotorController {
    rx: mpsc::Receiver<(u16, u8, DataValue)>,
    tx: mpsc::Sender<(u16, u8, DataValue)>,
    pub mode_of_operation: ModeOfOperation,
    pub controlword: u16,
    pub command: Command,
    pub statusword: u16,
    pub state: State,
    pub profile_position_status: ProfilePositionStatus,
    pub profile_velocity_status: ProfileVelocityStatus,
    pub halt: bool,
    pub control_oms1: VecDeque<bool>,
    pub home_status: HomeStatus,
    pub target_reached: bool,
    pub status_oms1: bool,
    pub status_oms2: bool,
    pub timer: Instant,
}

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
#[derive(Default, Debug, PartialEq)]
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
pub enum ProfileVelocityStatus {
    #[default]
    WaitingForStart,
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

impl MotorController {

    /// Initialize the motor controller.
    pub async fn initialize(
        rx: mpsc::Receiver<(u16, u8, DataValue)>,
        tx: mpsc::Sender<(u16, u8, DataValue)>
    ) -> Result<Self, ()> {
        let controller = Self {
            rx,
            tx,
            mode_of_operation: Default::default(),
            controlword: Default::default(),
            command: Default::default(),
            statusword: Default::default(),
            state: Default::default(),
            profile_position_status: Default::default(),
            profile_velocity_status: Default::default(),
            halt: Default::default(),
            control_oms1: Default::default(),
            home_status: Default::default(),
            target_reached: Default::default(),
            status_oms1: Default::default(),
            status_oms2: Default::default(),
            timer: Instant::now(),
        };
        Ok(controller)
    }

    pub async fn run(&mut self) {

        loop {

            let data = self.rx.recv().await;
            self.update_controller(data.unwrap()).await;
            
        }

    }

    pub async fn update_controller(&mut self, data: (u16, u8, DataValue)) {

        let index = data.0;
        let sub_index = data.1;
        let value = data.2;


        match index {
            0x6040 => self.parse_controlword(value),
            0x6060 => self.parse_mode_of_operation(value),
            _ => {},
        }

        self.update_state();

        match (&self.mode_of_operation, &self.state) {

            (ModeOfOperation::ProfilePosition, State::OperationEnabled) => {

                match &self.profile_position_status {

                    ProfilePositionStatus::SetpointAcknownlegde => {

                        self.status_oms1 = true;
                        self.target_reached = true;

                        if self.control_oms1[0] && !self.control_oms1[1] {

                            self.timer = Instant::now();
                            self.profile_position_status = ProfilePositionStatus::Moving

                        }
                    }

                    ProfilePositionStatus::Moving => {

                        self.status_oms1 = false;
                        self.target_reached = false;

                        if self.timer.elapsed() > Duration::from_millis(100) {
                            self.profile_position_status = ProfilePositionStatus::SetpointAcknownlegde
                        }

                    }

                }

            }

            (ModeOfOperation::ProfileVelocity, State::OperationEnabled) => {

                match &self.profile_velocity_status {

                    ProfileVelocityStatus::WaitingForStart => {

                        self.target_reached = false;

                        if !&self.halt {

                            self.timer = Instant::now();
                            self.profile_velocity_status = ProfileVelocityStatus::Moving

                        }
                    }

                    ProfileVelocityStatus::Moving => {

                        self.target_reached = true;

                        if self.timer.elapsed() > Duration::from_millis(100) {
                            self.profile_velocity_status = ProfileVelocityStatus::WaitingForStart
                        }

                    }

                }

            }

            (ModeOfOperation::Homing, State::OperationEnabled) => {

                match &self.home_status {

                    HomeStatus::WaitingForStart => {

                        self.target_reached = true;
                        self.status_oms2 = false;

                        if self.control_oms1[0] && !self.control_oms1[1] {
                            self.timer = Instant::now();
                            self.home_status = HomeStatus::Homing
                        }
                    }
                    HomeStatus::Homing => {

                        self.target_reached = false;
                        self.status_oms1 = false;
                        self.status_oms2 = false;

                        if self.timer.elapsed() > Duration::from_millis(100) {

                            self.target_reached = true;
                            self.status_oms1 = true;
                            self.status_oms2 = false;

                            self.home_status = HomeStatus::WaitingForStart
                        }
                    }

                }
            }

            _ => {},
        }
        
        self.set_statusword();

        // Adjust eds according to motor controller status


    }

    fn parse_controlword(&mut self, value: DataValue) {

        match value {
            DataValue::Unsigned16(value) => self.controlword = value,
            _ => {},
        };

        const BIT_INDICES: [usize; 5] = [0, 1, 2, 3, 7];
        
        let bits: Vec<bool> = BIT_INDICES.iter().map(|&i| get_bit_16(&self.controlword, i)).collect();
    
        self.command = match (bits[4], bits[3], bits[2], bits[1], bits[0]) {
            (false, _, true, true, false) => Command::Shutdown,
            (false, false, true, true, true) => Command::SwitchOn,
            (false, _, _, false, _) => Command::DisableVoltage,
            (false, _, false, true, _) => Command::QuickStop,
            (false, true, true, true, true) => Command::EnableOperation,
            (true, _, _, _, _) => Command::FaultReset,
        };

        self.control_oms1.push_front(get_bit_16(&self.controlword, 4));
        self.control_oms1.pop_back();

        self.halt = get_bit_16(&self.controlword, 8)
    }

    fn parse_mode_of_operation(&mut self, value: DataValue) {

        match value {
            DataValue::Integer8(value) => self.mode_of_operation = ModeOfOperation::mode_of_operation(value),
            _ => {},
        };
        println!("{:?}", self.mode_of_operation)
    }

    fn update_state(&mut self) {


        self.state = match self.state {
            State::NotReadyToSwitchOn => State::SwitchedOnDisabled,
            State::SwitchedOnDisabled => match &self.command {
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOnDisabled,
            }
            State::ReadyToSwitchOn => match &self.command {
                Command::SwitchOn => State::SwitchedOn,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::ReadyToSwitchOn,
            }
            State::SwitchedOn => match &self.command {
                Command::EnableOperation => State::OperationEnabled,
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOn,
            }
            State::OperationEnabled => match &self.command {
                Command::QuickStop => State::QuickStopActive,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::SwitchOn => State::SwitchedOn,
                _ => State::OperationEnabled,
            }
            State::QuickStopActive => match &self.command {
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::EnableOperationAfterQuickStop => State::OperationEnabled,
                _ => State::QuickStopActive,
            }
            State::FaultReactionActive => State::Fault,
            State::Fault => match &self.command {
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
    
        if let Some(bits) = bit_configs.get(&self.state) {
            set_bits(&mut self.statusword, bits);
        }
    
        self.statusword = set_bit_16(&self.statusword, 10, self.target_reached);
        self.statusword = set_bit_16(&self.statusword, 12, self.status_oms1);
        self.statusword = set_bit_16(&self.statusword, 13, self.status_oms2);
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