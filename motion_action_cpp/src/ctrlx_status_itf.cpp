#include  "../include/motion_action_cpp/ctrlx_status_itf.hpp"
#include  "../include/motion_action_cpp/dl_helper.hpp"
#include  "../include/motion_action_cpp/fbs/motion/core/axsIpoValues_generated.h"

#include "comm/datalayer/notify_info_generated.h"
#include "comm/datalayer/sub_properties_generated.h"

#include <chrono>
#include <cstring>
#include <set>

ros2_ctrlx_motion::CtrlxStatusItf::CtrlxStatusItf(rclcpp::Node *rosNode,
                                                  const std::vector<std::string> &axNames,
                                                  comm::datalayer::DatalayerSystem &datalayerSystem,
                                                  uint32_t rosPublishIntervalMs,
                                                  uint32_t publishIntervalMs,
                                                  uint64_t samplingIntervalUs):m_node(rosNode)
{
    auto conStr = dl_helper::getConnectionString(); 
    m_client = datalayerSystem.factory()->createClient3(conStr); 

    m_sub_id = std::string(rosNode->get_name()) + "_axis_status"; 

    m_axes.reserve(axNames.size()); 
    for (size_t i = 0; i < axNames.size(); i++)
    {
        AxisEntry t_entry; 
        t_entry.axName = axNames[i]; 
        // One topic per axis, named to match the action servers
        // (<axis>_power, <axis>_reset, <axis>_move_position, <axis>_status).
        // Shallow depth on purpose: a late subscriber wants the current state of
        // the machine, not a backlog of stale samples.
        t_entry.pub = rosNode->create_publisher<AxisStatus>(axNames[i] + "_status",
                                                            rclcpp::QoS(10)); 
        m_axes.push_back(t_entry); 

        // TODO verify these two addresses.
        // Interpolator values, not actual values: AxsActualValues documents
        // actualVel/actualAcc as "currently not supported for real drives", so
        // act_vel/act_acc would publish as zero on hardware.
        m_routes["motion/axs/"+axNames[i]+"/state/values/ipo"] = {i, NodeKind::IpoValues}; 
        m_routes["motion/axs/"+axNames[i]+"/state/opstate/plcopen"]    = {i, NodeKind::OpState}; 
    }

    m_write.resize(axNames.size()); 
    m_read.resize(axNames.size()); 

    // Both buffers are sized before the subscription exists, so the callback
    // never races the constructor.
    auto t_result = createSubscription(publishIntervalMs, samplingIntervalUs); 
    if (t_result != DL_OK)
    {
        RCLCPP_ERROR(
            m_node->get_logger(), "Status: subscription failed - %s",
            std::string(t_result.toString()).c_str()); 
    }

    // Its own group so publishing is never serialised behind an action callback.
    m_timer_group = rosNode->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
    m_timer = rosNode->create_wall_timer(std::chrono::milliseconds(rosPublishIntervalMs),
                                         [this]{ onTimer(); },
                                         m_timer_group); 
}

ros2_ctrlx_motion::CtrlxStatusItf::~CtrlxStatusItf()
{
    // Stop publishing first, then drop the client - which drops the
    // subscription, and with it any further callbacks into this object.
    if (m_timer != nullptr)
    {
        m_timer->cancel(); 
    }
    delete m_client; 
    m_client = nullptr; 
}

comm::datalayer::DlResult ros2_ctrlx_motion::CtrlxStatusItf::createSubscription(uint32_t publishIntervalMs,
                                                                               uint64_t samplingIntervalUs)
{
    if (m_client == nullptr || !m_client->isConnected())
    {
        return DL_CLIENT_NOT_CONNECTED; 
    }

    flatbuffers::FlatBufferBuilder builder; 
    auto t_sampling = comm::datalayer::CreateSampling(builder, samplingIntervalUs); 
    auto t_rule = comm::datalayer::CreateProperty(builder,
                                                  comm::datalayer::Properties::Properties_Sampling,
                                                  t_sampling.Union()); 
    auto t_rules = builder.CreateVector(&t_rule, 1); 
    auto t_id = builder.CreateString(m_sub_id); 
    builder.Finish(comm::datalayer::CreateSubscriptionProperties(builder,
                                                                 t_id,
                                                                 60000,
                                                                 publishIntervalMs,
                                                                 t_rules)); 
    comm::datalayer::Variant t_properties; 
    t_properties.copyFlatbuffers(builder); 

    auto t_result = m_client->createSubscriptionSync(t_properties,
                                                     [this](comm::datalayer::DlResult result,
                                                            const std::vector<comm::datalayer::NotifyItem> &items)
                                                     {
                                                        onPublish(result, items); 
                                                     }); 
    if (t_result != DL_OK)
    {
        return t_result; 
    }

    // Every node of every axis added to the one subscription in a single call.
    std::set<std::string> t_addresses; 
    for (const auto &t_route : m_routes)
    {
        t_addresses.insert(t_route.first); 
    }
    return m_client->subscribeSync(m_sub_id, t_addresses); 
}

// Subscription thread. Decodes the whole burst first, then takes the lock only
// to assign the results. No DDS here, and no flatbuffer work under the mutex.
void ros2_ctrlx_motion::CtrlxStatusItf::onPublish(comm::datalayer::DlResult result,
                                                  const std::vector<comm::datalayer::NotifyItem> &items)
{
    if (result != DL_OK)
    {
        RCLCPP_ERROR_THROTTLE(
            m_node->get_logger(), *m_node->get_clock(), 5000,
            "Status: subscription reported - %s", std::string(result.toString()).c_str()); 
        return; 
    }

    // ---- decode, unlocked ----
    std::vector<Update> t_updates; 
    t_updates.reserve(items.size()); 

    for (const auto &t_item : items)
    {
        std::string t_address; 
        if (!nodeAddress(t_item.info, t_address))
        {
            continue; 
        }

        auto t_route = m_routes.find(t_address); 
        if (t_route == m_routes.end())
        {
            continue; 
        }

        Update t_update{}; 
        t_update.axIndex = t_route->second.axIndex; 
        t_update.kind = t_route->second.kind; 

        bool t_decoded = false; 
        switch (t_route->second.kind)
        {
            case NodeKind::IpoValues: t_decoded = decodeIpoValues(t_item.data, t_update); break; 
            case NodeKind::OpState:   t_decoded = decodeOpState(t_item.data, t_update);   break; 
        }

        if (t_decoded)
        {
            t_updates.push_back(t_update); 
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                m_node->get_logger(), *m_node->get_clock(), 5000,
                "Axis '%s': unusable status data from %s (type %s)",
                m_axes[t_route->second.axIndex].axName.c_str(), t_address.c_str(),
                t_item.data.typeAsString().c_str()); 
        }
    }

    if (t_updates.empty())
    {
        return; 
    }

    // ---- apply, locked ----
    std::lock_guard<std::mutex> lock(m_buffer_mutex); 
    for (const auto &t_update : t_updates)
    {
        AxisSample &t_sample = m_write[t_update.axIndex]; 
        switch (t_update.kind)
        {
            case NodeKind::IpoValues:
                t_sample.msg.act_pos = t_update.pos; 
                t_sample.msg.act_vel = t_update.vel; 
                t_sample.msg.act_acc = t_update.acc; 
                break; 
            case NodeKind::OpState:
                t_sample.msg.powered_on = t_update.powered_on; 
                t_sample.msg.faulted = t_update.faulted; 
                break; 
        }
        t_sample.seeded = true; 
    }
}

// ROS thread. Lock, copy, release - then publish.
void ros2_ctrlx_motion::CtrlxStatusItf::onTimer()
{
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex); 
        m_read = m_write; 
    }

    // Every axis every tick, from cache. The subscription only sends nodes that
    // changed, so an idle axis would otherwise go silent; republishing the last
    // known state gives every topic a steady rate.
    for (size_t i = 0; i < m_read.size(); i++)
    {
        if (m_read[i].seeded)
        {
            m_axes[i].pub->publish(m_read[i].msg); 
        }
    }
}

bool ros2_ctrlx_motion::CtrlxStatusItf::nodeAddress(const comm::datalayer::Variant &info,
                                                    std::string &out_address)
{
    if (info.verifyFlatbuffers(comm::datalayer::VerifyNotifyInfoBuffer) != DL_OK)
    {
        return false; 
    }
    const auto *t_info = comm::datalayer::GetNotifyInfo(info.getData()); 
    if (t_info == nullptr || t_info->node() == nullptr)
    {
        return false; 
    }
    out_address = t_info->node()->str(); 
    return true; 
}

bool ros2_ctrlx_motion::CtrlxStatusItf::decodeIpoValues(const comm::datalayer::Variant &data,
                                                        Update &out_update)
{
    if (data.verifyFlatbuffers(motion::core::fbtypes::VerifyAxsIpoValuesBuffer) != DL_OK)
    {
        return false; 
    }
    const auto *t_ipo = motion::core::fbtypes::GetAxsIpoValues(data.getData()); 
    if (t_ipo == nullptr)
    {
        return false; 
    }
    out_update.pos = t_ipo->ipoPos(); 
    out_update.vel = t_ipo->ipoVel(); 
    out_update.acc = t_ipo->ipoAcc(); 
    return true; 
}

bool ros2_ctrlx_motion::CtrlxStatusItf::decodeOpState(const comm::datalayer::Variant &data,
                                                      Update &out_update)
{
    if (data.getType() != comm::datalayer::VariantType::STRING)
    {
        return false; 
    }
    const char *t_state = data; 
    if (t_state == nullptr)
    {
        return false; 
    }

    // TODO verify these state names against the firmware. An unrecognised name
    // returns false, so nothing is assigned and the cached flags keep their
    // previous value rather than publishing a wrong false - silently "not
    // faulted" is the dangerous way to be wrong.
    if (strcmp(t_state, "ERROR") == 0 || strcmp(t_state, "ERROR_STOP") == 0)
    {
        out_update.powered_on = false; 
        out_update.faulted = true; 
        return true; 
    }
    if (strcmp(t_state, "DISABLED") == 0 || strcmp(t_state, "STOPPED") == 0)
    {
        out_update.powered_on = false; 
        out_update.faulted = false; 
        return true; 
    }
    if (strcmp(t_state, "STANDSTILL") == 0 || strcmp(t_state, "IN_POSITION") == 0 ||
        strcmp(t_state, "IN_MOTION") == 0  || strcmp(t_state, "HOMING") == 0)
    {
        out_update.powered_on = true; 
        out_update.faulted = false; 
        return true; 
    }
    return false; 
}
