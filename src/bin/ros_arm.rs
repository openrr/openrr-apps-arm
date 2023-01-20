#[cfg(feature = "ros")]
#[tokio::main]
async fn main() {
    use arci_ros::*;
    use clap::Parser;
    use openrr_apps_arm::{argument::CmdArgument, target_set_ui::TargetSetUI};

    let arg = CmdArgument::parse();

    arci_ros::init("lite6");

    let ros_client = RosControlClient::new(
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

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0x0100);
    });

    let mut ui = TargetSetUI::new(ros_client, &arg.urdf_path, &arg.end_link_name);

    ui.run();
}

#[cfg(not(feature = "ros"))]
fn main() {
    println!("ROS is not featured.");
}
