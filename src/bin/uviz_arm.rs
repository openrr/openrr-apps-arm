use arci_urdf_viz::UrdfVizWebClient;
use clap::Parser;
use openrr_apps_arm::{argument::CmdArgument, urdf_viz_planner::UrdfVizPlanner};

#[tokio::main]
async fn main() {
    let arg = CmdArgument::parse();

    let urdf_viz_client = UrdfVizWebClient::default();
    urdf_viz_client.run_send_joint_positions_thread();

    let mut uviz_planner = UrdfVizPlanner::new(urdf_viz_client, &arg.urdf_path, &arg.end_link_name);

    uviz_planner.run();
}
