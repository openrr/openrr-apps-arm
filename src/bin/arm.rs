#[cfg(feature = "ros")]
use arci_ros::*;
#[cfg(feature = "ros2")]
use arci_ros2::*;
#[cfg(not(any(feature = "ros", feature = "ros2")))]
use arci_urdf_viz::UrdfVizWebClient;
use clap::Parser;
use openrr_apps_arm::{argument::CmdArgument, urdf_viz_planner::UrdfVizPlanner};

#[tokio::main]
async fn main() {
    let arg = CmdArgument::parse();

    let client;

    #[cfg(feature = "ros")]
    {
        arci_ros::init("lite6");
        client = RosControlClient::new(
            vec![
                "joint1".to_string(),
                "joint2".to_string(),
                "joint3".to_string(),
                "joint4".to_string(),
                "joint5".to_string(),
                "joint6".to_string(),
            ],
            "/ufactory/lite6_traj_controller",
            false,
        );
    }
    #[cfg(feature = "ros2")]
    {
        let ctx = r2r::Context::create().unwrap();
        let action_name = "/lite6_traj_controller";
        client = Ros2ControlClient::new(ctx, action_name);
    }
    #[cfg(not(any(feature = "ros", feature = "ros2")))]
    {
        client = UrdfVizWebClient::default();
        client.run_send_joint_positions_thread();
    }

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0x0100);
    });

    let mut uviz_planner = UrdfVizPlanner::new(client, &arg.urdf_path, &arg.end_link_name);

    uviz_planner.run();
}
