#include  "../include/motion_action_cpp/ctrlx_axis_itf.hpp"
#include  "../include/motion_action_cpp/dl_helper.hpp"
#include <thread>
#include "../include/motion_action_cpp/fbs/motion/core/axsCmdPosData_generated.h"
#include "../include/motion_action_cpp/fbs/motion/core/dynamicLimits_generated.h"
#include "../include/motion_action_cpp/fbs/motion/core/axsCmdAbortData_generated.h"
#include <cstring>

ros2_ctrlx_motion::CtrlxActionItf::CtrlxActionItf(rclcpp::Node *rosNode, std::string axName,
                                                  comm::datalayer::DatalayerSystem &datalayerSystem,
                                                  rclcpp::CallbackGroup::SharedPtr actions):m_node(rosNode),
                                                                 m_axName(std::move(axName))
                                                 
{
    auto conStr = dl_helper::getConnectionString(); 
    m_client = datalayerSystem.factory()->createClient3(conStr); 
    m_power_server = rclcpp_action::create_server<PowerOn>(rosNode, 
                                                           m_axName+ "_power",
                                                           [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PowerOn::Goal> goal) 
                                                           {return acceptPowerOn(uuid, goal);}, 
                                                           [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<PowerOn>> handle)
                                                           {return cancelPowerOn(handle);}, 
                                                           [this]( const std::shared_ptr<rclcpp_action::ServerGoalHandle<PowerOn>> handle)
                                                           {
                                                            std::thread([this,handle]{
                                                                executePowerOn(handle);
                                                            }).detach();
                                                           },
                                                           rcl_action_server_get_default_options(), actions); 

    m_reset_server = rclcpp_action::create_server<ResetFault>(rosNode, 
                                                              m_axName + "_reset",
                                                              [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ResetFault::Goal> goal) 
                                                                {return acceptReset(uuid, goal);}, 
                                                              [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault>> handle)
                                                                {return cancelReset(handle);}, 
                                                              [this]( const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault>> handle)
                                                                {
                                                                    std::thread([this,handle]{
                                                                    executeReset(handle);
                                                                    }).detach();
                                                                },
                                                              rcl_action_server_get_default_options(), actions);
    
    m_move_server = rclcpp_action::create_server<MovePosition>(rosNode, 
                                                              m_axName + "_move_position",
                                                              [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MovePosition::Goal> goal) 
                                                                {return acceptMove(uuid, goal);}, 
                                                              [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePosition>> handle)
                                                                {return cancelMove(handle);}, 
                                                              [this]( const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePosition>> handle)
                                                                {
                                                                    std::thread([this,handle]{
                                                                    executeMove(handle);
                                                                    }).detach();
                                                                },
                                                              rcl_action_server_get_default_options(), actions);

                                      
    m_power_addr = "motion/axs/"+m_axName+"/cmd/power"; 
    m_cmd_status_addr = "motion/axs/"+m_axName+"/state/cmd-state/"; 
    m_reset_addr = "motion/axs/" +m_axName+"/cmd/reset"; 
    m_cmd_posabs_addr = "motion/axs/"+m_axName+"/cmd/pos-abs"; 
    m_cmd_abort_addr = "motion/axs/"+m_axName+"/cmd/abort"; 

}

rclcpp_action::GoalResponse ros2_ctrlx_motion::CtrlxActionItf::acceptPowerOn([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, [[maybe_unused]] std::shared_ptr<const PowerOn::Goal> goal)
{
    if (takeAction()){
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ros2_ctrlx_motion::CtrlxActionItf::cancelPowerOn([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<PowerOn>> handle)
{
    
    return rclcpp_action::CancelResponse::ACCEPT; 
}

rclcpp_action::GoalResponse ros2_ctrlx_motion::CtrlxActionItf::acceptReset([[maybe_unused]] const rclcpp_action::GoalUUID & uuid, [[maybe_unused]] std::shared_ptr<const ResetFault::Goal> goal)
{
    if (takeAction()){
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ros2_ctrlx_motion::CtrlxActionItf::cancelReset([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault>> handle)
{
  return rclcpp_action::CancelResponse::ACCEPT; 
}

rclcpp_action::GoalResponse ros2_ctrlx_motion::CtrlxActionItf::acceptMove([[maybe_unused]] const rclcpp_action::GoalUUID & uuid, [[maybe_unused]] std::shared_ptr<const MovePosition::Goal> goal)
{
  if (takeAction()){
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }
    return rclcpp_action::GoalResponse::REJECT;
}
rclcpp_action::CancelResponse ros2_ctrlx_motion::CtrlxActionItf::cancelMove([[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePosition>> handle)
{
  return rclcpp_action::CancelResponse::ACCEPT; 
}

void ros2_ctrlx_motion::CtrlxActionItf::executePowerOn(std::shared_ptr<rclcpp_action::ServerGoalHandle<PowerOn>> handle)
{
  rclcpp::WallRate t_update_rate(5.0); 
  bool t_command = handle->get_goal()->power_on; 
  comm::datalayer::Variant t_payload; 
  t_payload.setValue(t_command); 
  auto t_ros_result = std::make_shared<PowerOn::Result>(); 

  auto t_result = m_client->createSync(m_power_addr, &t_payload);
  if (t_result != DL_OK)
  {
    t_ros_result->powered_on = false; 
    handle->abort(t_ros_result);  
    RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Power On - %s",
        m_axName.c_str(), "DL_Create Fail");
        releaseAction(); 
    return; 
  }
  u_int64_t id = u_int64_t(t_payload);
  std::string t_cmd_status_addr = m_cmd_status_addr + std::to_string(id); 
  bool done = false; 
  while(!done && rclcpp::ok()){
    t_update_rate.sleep(); 
    if(handle->is_canceling()){
      t_ros_result->powered_on = false; 
      handle->canceled(t_ros_result); 
      break; 
    }
    t_result = m_client->readSync(t_cmd_status_addr,&t_payload); 
    if (t_result != DL_OK)
    {
       t_ros_result->powered_on = false; 
       handle->abort(t_ros_result);  
       RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Power On - %s",
        m_axName.c_str(), "Command Status Read Fail");
      break; 
    }
    bool t_isValid; 
    if (isDone(t_payload, t_isValid))
    {
      t_ros_result->powered_on = t_command; 
      handle->succeed(t_ros_result); 
      done = true; 
    }
    else if(!t_isValid){
       t_ros_result->powered_on= false; 
       handle->abort(t_ros_result);  
      RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Power On - %s",
        m_axName.c_str(), "Cmd Response Invalid");
      break;
    }
  }
  releaseAction(); 
}

void ros2_ctrlx_motion::CtrlxActionItf::executeReset(std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault>> handle)
{
  rclcpp::WallRate  t_update_rate(5.0);  
  comm::datalayer::Variant t_payload; 
  auto t_ros_result = std::make_shared<ResetFault::Result>(); 
  bool done = false; 
  auto t_result = m_client->createSync(m_reset_addr,&t_payload); 
  if (t_result != DL_OK)
  {
    t_ros_result->done = false; 
    RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Reset - %s",
        m_axName.c_str(), "DL_Create Fail");

    handle->abort(t_ros_result); 
    releaseAction(); 
    return; 
  }
  u_int64_t id = u_int64_t(t_payload);
  std::string t_cmd_status_addr = m_cmd_status_addr + std::to_string(id); 
  while(!done && rclcpp::ok()){
    t_update_rate.sleep();
     if(handle->is_canceling()){
      t_ros_result->done = false; 
      handle->canceled(t_ros_result); 
      break; 
    }
    t_result = m_client->readSync(t_cmd_status_addr,&t_payload); 
    if (t_result != DL_OK)
    {
       t_ros_result->done= false; 
       handle->abort(t_ros_result);  
       RCLCPP_ERROR(
         m_node->get_logger(), "Axis '%s': Failed to Reset - %s",
         m_axName.c_str(), "Cmd Status Read Failed");
      break; 
    }
    bool t_isValid;
    if (isDone(t_payload, t_isValid))
    {
      t_ros_result->done = true; 
      handle->succeed(t_ros_result); 
      done = true; 
    }
    else if(!t_isValid)
    {
       t_ros_result->done= false; 
       handle->abort(t_ros_result);  
       RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Reset - %s",
        m_axName.c_str(), "Command Response Invalid");
      break; 
    }
  }
  releaseAction();
}

void ros2_ctrlx_motion::CtrlxActionItf::executeMove(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePosition>> handle)
{
  flatbuffers::FlatBufferBuilder builder(512); 
  auto t_dyn = motion::core::fbtypes::CreateDynamicLimits(builder,
                                             handle->get_goal()->vel, 
                                             handle->get_goal()->acc, 
                                             handle->get_goal()->dcc, 
                                             0,
                                             0); 
  auto t_cmd = motion::core::fbtypes::CreateAxsCmdPosData(builder, 
                                                        handle->get_goal()->pos, 
                                                        false, 
                                                        t_dyn, 
                                                        motion::core::fbtypes::CmdPosAbsDir_SHORTEST_WAY); 
  builder.Finish(t_cmd);
  comm::datalayer::Variant t_payload; 
  auto t_ros_result = std::make_shared<MovePosition::Result>(); 
  t_payload.copyFlatbuffers(builder); 
  auto t_result = m_client->createSync(m_cmd_posabs_addr,&t_payload); 

  if (t_result != DL_OK){
    RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Move Abs - %s",
        m_axName.c_str(), "DL_CREATE Failure");
    t_ros_result->done = false; 
    handle->abort(t_ros_result); 
    releaseAction(); 
    return; 
  } 
  auto t_ros_feedback = std::make_shared<MovePosition::Feedback>(); 
  u_int64_t id = u_int64_t(t_payload);
  std::string t_cmd_status_addr = m_cmd_status_addr + std::to_string(id); 
  bool done = false; 
  rclcpp::WallRate t_update_rate(5.0);  
  while(!done && rclcpp::ok()){
    t_update_rate.sleep();
    if(handle->is_canceling()){
      break; 
    }
    t_result = m_client->readSync(t_cmd_status_addr,&t_payload); 
    if (t_result != DL_OK)
    {
       t_ros_result->done= false; 
       handle->abort(t_ros_result);  
       RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Move Abs - %s",
        m_axName.c_str(), "Cmd Status Read Fail");
      break; 
    }
    bool t_isValid;
    if (isDone(t_payload,t_isValid))
    {
      t_ros_result->done = true; 
      handle->succeed(t_ros_result); 
      done = true; 
    }
    else if(!t_isValid){
      t_ros_result->done= false; 
      handle->abort(t_ros_result);  
      RCLCPP_ERROR(
        m_node->get_logger(), "Axis '%s': Failed to Move Abs - %s",
        m_axName.c_str(), "command response invalid");
        break; 
    }
    else{
      t_ros_feedback->dist_to_target = -1; //not implemented in Motion App yet
      handle->publish_feedback(t_ros_feedback); 
    }
  }
  if (handle->is_canceling()){
    done = false; 
    builder.Clear(); 
    auto t_abort_data = motion::core::fbtypes::CreateAxsCmdAbortData(builder, 
                                                                      handle->get_goal()->dcc, 
                                                                      0); 
    builder.Finish(t_abort_data); 
    t_payload.copyFlatbuffers(builder); 
    t_result = m_client->createSync(m_cmd_abort_addr,&t_payload); 
    if (t_result != DL_OK){
          RCLCPP_ERROR(
            m_node->get_logger(), "Axis '%s': Failed to cancel Move Abs - %s",
            m_axName.c_str(), "DL_CREATE Failure");
            t_ros_result->done = false; 
            handle->abort(t_ros_result); 
            releaseAction(); 
            return; 
    }   
    id = u_int64_t(t_payload);
    t_cmd_status_addr = m_cmd_status_addr + std::to_string(id); 
    while(!done && rclcpp::ok())
    {
      t_update_rate.sleep();
      t_result = m_client->readSync(t_cmd_status_addr,&t_payload); 
      if(t_result != DL_OK){
          t_ros_result->done= false; 
          handle->abort(t_ros_result);  
          RCLCPP_ERROR(
          m_node->get_logger(), "Axis '%s': Failed to cancel Move Abs - %s",
          m_axName.c_str(), "Cmd Status Read Fail");
          break; 
          }
      bool t_isValid; 
      if(isDone(t_payload,t_isValid))
      {
        t_ros_result->done = false; 
        handle->canceled(t_ros_result);
        done = true;  
      }
      else if(!t_isValid){
          t_ros_result->done= false; 
          handle->abort(t_ros_result);  
          RCLCPP_ERROR(
          m_node->get_logger(), "Axis '%s': Failed to cancel Move Abs - %s",
          m_axName.c_str(), "command response invalid");
          break; 
      }
    }

  }
  releaseAction();
  


}

bool ros2_ctrlx_motion::CtrlxActionItf::takeAction()
{
  bool expected = false; 
  if (m_busy.compare_exchange_strong(expected,true,
                                    std::memory_order_acquire,
                                    std::memory_order_relaxed)){
    return true;
  }
  return false; 
}

void ros2_ctrlx_motion::CtrlxActionItf::releaseAction()
{
    m_busy.store(false, std::memory_order_release); 
}

bool ros2_ctrlx_motion::CtrlxActionItf::isDone(const comm::datalayer::Variant &payload, bool &out_isValid)
{
  const char* t_done = "DONE";
  out_isValid = true; 
  if (payload.getType() == comm::datalayer::VariantType::STRING)
  {
    return strcmp(t_done, payload)==0; 
  }
  out_isValid = false; 
  return false; 
}

ros2_ctrlx_motion::CtrlxActionItf::~CtrlxActionItf()
{
  delete m_client; 
}