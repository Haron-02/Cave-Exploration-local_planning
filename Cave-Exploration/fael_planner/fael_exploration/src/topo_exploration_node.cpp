//
// Created by hjl on 2021/12/9.
//

#include <explorer/explorer_fsm.h>
#include <unordered_map>
#include <control_planner_interface/pci_vehicle.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topo_exploration_node");

    ros::NodeHandle nh_private("~");

    std::shared_ptr<interface::PCIManager> pci_manager = std::make_shared<interface::PCIVehicle>(nh_private);
    std::shared_ptr<interface::ControlPlannerInterface> interface = std::make_shared<interface::ControlPlannerInterface>(
        nh_private, pci_manager);

    explorer::Explorer_FSM explorer(nh_private, interface);

    ros::spin();

    return 0;
}
