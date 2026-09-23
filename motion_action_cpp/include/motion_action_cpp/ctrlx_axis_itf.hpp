#pragma once 
#include <string>
#include <atomic>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "axis_status_msg/msg/axis_status.hpp"
#include "ctrlx_motion_action/action/move_position.hpp"
#include "ctrlx_motion_action/action/power_on.hpp"
#include "ctrlx_motion_action/action/reset_fault.hpp"


namespace ros2_ctrlx_motion{
class CtrlxActionItf{
    public:
        using AxisStatus = axis_status_msg::msg::AxisStatus;
        using MovePosition = ctrlx_motion_action::action::MovePosition;
        using PowerOn = ctrlx_motion_action::action::PowerOn;
        using ResetFault = ctrlx_motion_action::action::ResetFault;
        CtrlxActionItf(rclcpp::Node *rosNode,
                       std::string axName, 
                       comm::datalayer::DatalayerSystem &datalayerSystem, 
                       rclcpp::CallbackGroup::SharedPtr actions); 
        ~CtrlxActionItf(); 
    
    private: 
        rclcpp_action::GoalResponse acceptPowerOn([[maybe_unused]] const rclcpp_action::GoalUUID & uuid, [[maybe_unused]] std::shared_ptr<const PowerOn::Goal> goal); 
        rclcpp_action::CancelResponse cancelPowerOn([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<PowerOn>> handle); 
        void executePowerOn(std::shared_ptr<rclcpp_action::ServerGoalHandle<PowerOn>> handle);  
        rclcpp_action::GoalResponse acceptReset([[maybe_unused]] const rclcpp_action::GoalUUID & uuid, [[maybe_unused]] std::shared_ptr<const ResetFault::Goal> goal);
        rclcpp_action::CancelResponse cancelReset([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault>> handle);  
        void executeReset(std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault>> handle); 
        rclcpp_action::GoalResponse acceptMove([[maybe_unused]] const rclcpp_action::GoalUUID & uuid, [[maybe_unused]] std::shared_ptr<const MovePosition::Goal> goal);
        rclcpp_action::CancelResponse cancelMove([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePosition>> handle);  
        void executeMove(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePosition>> handle); 
        bool takeAction(); 
        void releaseAction(); 
        bool isDone(const comm::datalayer::Variant &payload, bool &out_isValid);

        comm::datalayer::IClient3* m_client; 
        rclcpp_action::Server<PowerOn>::SharedPtr m_power_server;
        rclcpp_action::Server<ResetFault>::SharedPtr m_reset_server;
        rclcpp_action::Server<MovePosition>::SharedPtr m_move_server;
        std::atomic<bool> m_busy{false}; 
        rclcpp::Node *m_node; 
        std::string m_axName;  
        std::string m_power_addr; 
        std::string m_reset_addr; 
        std::string m_cmd_status_addr; 
        std::string m_cmd_posabs_addr; 
        std::string m_cmd_abort_addr; 
        };
}