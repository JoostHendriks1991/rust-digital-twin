use canopen_tokio::CanOpenSocket;

use crate::eds::EDSData;

pub struct MotorController {
    pub eds_data: EDSData,
}


impl MotorController {
    /// Initialize the motor controller.
    pub async fn initialize(
        bus: &mut CanOpenSocket,
        eds_data: EDSData,
    ) -> Result<Self, ()> {
        let controller = Self {
            eds_data
        };

        Ok(controller)
    }

}