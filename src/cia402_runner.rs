use std::time::{Instant, Duration};
use std::collections::{BTreeMap, HashMap};
use std::collections::VecDeque;
use tokio::sync::mpsc;
use s_curve::*;

use crate::eds::DataValue;

pub struct MotorController {
    rx: mpsc::Receiver<Message>,
    tx: mpsc::Sender<Message>,
    node_id: u8,
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
    pub acceleration: f64,
    pub profile_velocity: f64,
    pub current_position: f64,
    pub destination_point: f64,
    pub motion_map: BTreeMap<usize, f64>,
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
#[allow(dead_code)]
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

pub struct Message {
    pub msg_type: MessagaType,
    pub index: u16,
    pub sub_index: u8,
    pub value: DataValue,
}

#[derive(PartialEq)]
pub enum MessagaType {
    Get,
    Set,
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
    pub fn initialize(
        node_id: u8,
        rx: mpsc::Receiver<Message>,
        tx: mpsc::Sender<Message>,
    ) -> Result<Self, ()> {
        let mut controller = Self {
            rx,
            tx,
            node_id,
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
            acceleration: 0.0,
            profile_velocity: 0.0,
            current_position: 0.0,
            destination_point: 0.0,
            motion_map: BTreeMap::new(),
        };
        controller.control_oms1 = VecDeque::from(vec![false; 2]);
        Ok(controller)
    }

    pub async fn run(&mut self, speed_factor: &f64) {

        loop {

            // Synchronize
            let sync_time = Instant::now();
            let sync_cycle = Duration::from_millis(1);
            let sync_end = sync_time + sync_cycle;

            self.update_parameters().await;
            self.update_controller(speed_factor).await;            
            self.send_status().await;

            // println!("Sync time: {:?}", sync_time.elapsed());

            tokio::time::sleep_until(sync_end.into()).await;
            
        }

    }

    async fn update_parameters(&mut self) {

        let mut parameters_to_get: Vec<(MessagaType, u16, u8, DataValue)> = Vec::new();

        parameters_to_get.push((MessagaType::Get, 0x6040, 0, DataValue::Unknown(0)));
        parameters_to_get.push((MessagaType::Get, 0x6060, 0, DataValue::Unknown(0)));
        parameters_to_get.push((MessagaType::Get, 0x6083, 0, DataValue::Unknown(0)));
        parameters_to_get.push((MessagaType::Get, 0x6081, 0, DataValue::Unknown(0)));
        parameters_to_get.push((MessagaType::Get, 0x607A, 0, DataValue::Unknown(0)));

        let messages = messages_to_send(parameters_to_get);

        self.get_values(messages).await;

    }

    async fn get_values(&mut self, messages: Vec<Message>) {

        for message in messages {
            if let Err(e) = self.tx.send(message).await {
                log::error!("Error sending message, with error {e}")
            }
    
            if let Some(data) = self.rx.recv().await {
                self.update_variable(data)
            }

            // match self.rx.try_recv() {
            //     Ok(data) => self.update_variable(data),
            //     Err(_) => {},
            // }
        }

    }

    async fn update_controller(&mut self, speed_factor: &f64) {

        self.update_state();
        self.update_operation(speed_factor);
        self.set_statusword();
    
    }

    fn update_variable(&mut self, data: Message) {

        match (data.msg_type, data.index, data.sub_index) {
            (MessagaType::Set, 0x6040, 0) => self.parse_controlword(data.value),
            (MessagaType::Set, 0x6060, 0) => self.parse_mode_of_operation(data.value),
            (MessagaType::Set, 0x6083, 0) => self.parse_acceleration(data.value),
            (MessagaType::Set, 0x6081, 0) => self.parse_velocity(data.value),
            (MessagaType::Set, 0x607A, 0) => self.parse_destination_point(data.value),
            _ => {},
        }

    }

    fn parse_acceleration(&mut self, value: DataValue) {
        match value {
            DataValue::Unsigned32(value) => self.acceleration = value as f64,
            _ => {},
        };
    }

    fn parse_velocity(&mut self, value: DataValue) {
        match value {
            DataValue::Unsigned32(value) => {
                self.profile_velocity = value as f64;
            }
            _ => {},
        };
    }

    fn parse_destination_point(&mut self, value: DataValue) {
        match value {
            DataValue::Integer32(value) => self.destination_point = value as f64,
            _ => {},
        };
    }

    fn update_operation(&mut self, speed_factor: &f64) {

        match (&self.mode_of_operation, &self.state) {

            (ModeOfOperation::ProfilePosition, State::OperationEnabled) => {

                match &self.profile_position_status {

                    ProfilePositionStatus::SetpointAcknownlegde => {

                        self.target_reached = true;
                        self.status_oms1 = false;

                        if self.control_oms1[0] && !self.control_oms1[1] {

                            log::debug!("Trying to move node {} with: acc: {}, vel: {}, curr: {}, dest: {}", self.node_id, self.acceleration, self.profile_velocity, self.current_position, self.destination_point);

                            match construct_motion_map(self.node_id, &self.acceleration, &self.profile_velocity, &self.current_position, &self.destination_point, speed_factor) {
                                Ok(motion_map) => {
                                    self.motion_map = motion_map;
                                    self.status_oms1 = true;
                                    self.timer = Instant::now();
                                    self.profile_position_status = ProfilePositionStatus::Moving
                                }
                                Err(_) => log::warn!("Invalid move node id: {}, acc: {}, vel: {}, dest: {}", self.node_id, self.acceleration, self.profile_velocity, self.destination_point)
                            }
                        }

                    }

                    ProfilePositionStatus::Moving => {

                        self.target_reached = false;

                        let elapsed_time = self.timer.elapsed().as_millis() as usize;
                        if let Some(just_passed_point) = self.motion_map.range(..=elapsed_time).next_back().map(|(&key, _)| key) {
                            if let Some(current_positoin) = self.motion_map.get(&just_passed_point) {
                                self.current_position = *current_positoin;
                                log::info!("Current position node {}: {}", self.node_id, self.current_position);
                            }
                        }

                        let (end_time, _end_position) = self.motion_map.last_key_value().unwrap();

                        if self.timer.elapsed() > Duration::from_millis(*end_time as u64) {
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

    async fn send_status(&self) {

        let mut parameters_to_send: Vec<(MessagaType, u16, u8, DataValue)> = Vec::new();

        parameters_to_send.push((MessagaType::Set, 0x6041, 0, DataValue::Unsigned16(self.statusword)));
        parameters_to_send.push((MessagaType::Set, 0x6061, 0, DataValue::Unsigned8(self.mode_of_operation.clone() as u8)));

        let messages = messages_to_send(parameters_to_send);

        for message in messages {
            if let Err(e) = self.tx.send(message).await {
                log::error!("Failed sending data, with error: {e}")
            }
        }
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

fn construct_motion_map(node_id: u8, profile_acceleration: &f64, profile_velocity: &f64, current_position: &f64, destination_point: &f64, speed_factor: &f64) -> Result<BTreeMap<usize, f64>, String> {

    if *profile_acceleration == 0. || *profile_velocity == 0. {
        return Err(format!("Destination point invalid for node: {}", node_id));
    }

    const RPM_TO_RPS: f64 = 1./60.;
    const INC_PER_ROT: f64 = 3600.;
    const SEC_TO_MSEC: f64 = 1000.;

    let constraints = SCurveConstraints {
        max_jerk: 10.,
        max_acceleration: profile_acceleration * RPM_TO_RPS,
        max_velocity: profile_velocity * RPM_TO_RPS,
    };

    let travel_distance = destination_point - current_position;

    let mut motion_map: BTreeMap<usize, f64> = BTreeMap::new();

    if travel_distance == 0. {
        motion_map.insert(0, 0.);
        return Ok(motion_map);
    }

    let  start_conditions = SCurveStartConditions {
        q0: current_position / INC_PER_ROT, // start position
        q1: destination_point / INC_PER_ROT, // end position
        v0: 0., // start velocity
        v1: 0. // end velocity
    };

    let input  =  SCurveInput{constraints, start_conditions};
    let (params, s_curve) = s_curve_generator(&input, Derivative::Position);

    let points = (travel_distance.abs() / 10.) as u64;

    for point in 0..(points + 1) {
        let time = (point as f64 * ((params.time_intervals.total_duration() * SEC_TO_MSEC) / speed_factor) / points as f64) as usize;
        let position = s_curve(point as f64 * params.time_intervals.total_duration() / points as f64) * INC_PER_ROT;
        motion_map.insert(time, position);
    }

    let total_duration = params.time_intervals.total_duration();

    log::debug!("Duration move: {}", total_duration);

    Ok(motion_map)
}

fn messages_to_send(parameters: Vec<(MessagaType, u16, u8, DataValue)>) -> Vec<Message> {
    let mut messages: Vec<Message> = Vec::new();

    for parameter in parameters {
        let message = Message {
            msg_type: parameter.0,
            index: parameter.1,
            sub_index: parameter.2,
            value: parameter.3
        };
        messages.push(message);
    }
    messages

}