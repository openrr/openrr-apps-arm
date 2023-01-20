use arci_urdf_viz::UrdfVizWebClient;
use clap::Parser;
use openrr_apps_arm::{argument::CmdArgument, target_set_ui::TargetSetUI};

#[tokio::main]
async fn main() {
    let arg = CmdArgument::parse();

    let urdf_viz_client = UrdfVizWebClient::default();
    urdf_viz_client.run_send_joint_positions_thread();

    let mut ui = TargetSetUI::new(urdf_viz_client, &arg.urdf_path, &arg.end_link_name);

    ui.run();
}
