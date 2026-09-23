#pragma once 
#include <memory>
#include <string>
#include <vector>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

#include "rclcpp/rclcpp.hpp"

#include "ctrlx_axis_itf.hpp"
#include "ctrlx_status_itf.hpp"

namespace ros2_ctrlx_motion{

// Owns the Data Layer system and everything built on it.
//
// Clients: one per axis for actions (CtrlxActionItf), plus one shared for status
// (CtrlxStatusItf). The status client is a subscription, so it costs one
// connection no matter how many axes exist.
class CtrlxMotionNode : public rclcpp::Node{
    public:
        CtrlxMotionNode(); 
        ~CtrlxMotionNode(); 

        // Connect, discover axes, build the handlers. Separate from the
        // constructor so a failure can be reported without throwing out of it.
        bool init(); 

    private: 
        bool browseAxes(std::vector<std::string> &out_axNames); 

        // DECLARATION ORDER MATTERS. Members are destroyed in reverse, so the
        // system must be declared first: the handlers hold IClient3 pointers
        // created from its factory and must be gone before it is stopped.
        comm::datalayer::DatalayerSystem m_datalayerSystem; 
        rclcpp::CallbackGroup::SharedPtr m_action_group; 
        std::vector<std::unique_ptr<CtrlxActionItf>> m_axes; 
        std::unique_ptr<CtrlxStatusItf> m_status; 
        };
}
