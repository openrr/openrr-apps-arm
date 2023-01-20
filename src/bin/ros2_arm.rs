#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() {
    use arci_ros2::*;
    use clap::Parser;
    use openrr_apps_arm::{argument::CmdArgument, target_set_ui::TargetSetUI};

    let arg = CmdArgument::parse();

    let ctx = r2r::Context::create().unwrap();
    let action_name = "/lite6_traj_controller";
    let ros2_client = Ros2ControlClient::new(ctx, action_name);

    let mut ui = TargetSetUI::new(ros2_client, &arg.urdf_path, &arg.end_link_name);

    ui.run();
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("ROS2 is not featured.");
}
