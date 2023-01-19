use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[clap(name = "openrr_apps_arm")]
pub struct CmdArgument {
    #[clap(short = 'p', long = "urdf-path")]
    pub urdf_path: PathBuf,
    #[clap(short = 'l', long = "end-link-name")]
    pub end_link_name: String,
}
