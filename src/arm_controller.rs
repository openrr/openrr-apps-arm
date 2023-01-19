use arci::{Error, JointTrajectoryClient, WaitFuture};

pub struct ArmController<J>
where
    J: JointTrajectoryClient,
{
    client: J,
}

impl<J> ArmController<J>
where
    J: JointTrajectoryClient,
{
    pub fn new(client: J) -> Self {
        Self { client }
    }

    pub fn run(&self, positions: &[f64]) -> Result<WaitFuture, Error> {
        let mut target = vec![];

        for pose in positions {
            target.push(*pose);
        }

        self.client
            .send_joint_positions(target, std::time::Duration::from_millis(100))
    }

    pub fn get_feedback(&self) -> Vec<f64> {
        self.client.current_joint_positions().unwrap()
    }
}
