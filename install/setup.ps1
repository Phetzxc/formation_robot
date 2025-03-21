# generated from colcon_powershell/shell/template/prefix_chain.ps1.em

# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# function to source another script with conditional trace output
# first argument: the path of the script
function _colcon_prefix_chain_powershell_source_script {
  param (
    $_colcon_prefix_chain_powershell_source_script_param
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_prefix_chain_powershell_source_script_param) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_prefix_chain_powershell_source_script_param'"
    }
    . "$_colcon_prefix_chain_powershell_source_script_param"
  } else {
    Write-Error "not found: '$_colcon_prefix_chain_powershell_source_script_param'"
  }
}

# source chained prefixes
_colcon_prefix_chain_powershell_source_script "/opt/ros/humble\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/articubot_one/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/vertical_farm_robot/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/path_tracking_sim_ros2/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/Exam1_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/ros2_ws/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/robot_modelling/src/mycobot_ros2/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/robot_modelling/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/exam1_turtle/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/fun4_38/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/phet/uros_ws/install\local_setup.ps1"

# source this prefix
$env:COLCON_CURRENT_PREFIX=(Split-Path $PSCommandPath -Parent)
_colcon_prefix_chain_powershell_source_script "$env:COLCON_CURRENT_PREFIX\local_setup.ps1"
