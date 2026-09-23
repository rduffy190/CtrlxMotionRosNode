#pragma once 
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

#include "rclcpp/rclcpp.hpp"
#include "axis_status_msg/msg/axis_status.hpp"

namespace ros2_ctrlx_motion{

// Status for every axis: ONE Data Layer client with ONE subscription, and one
// ROS topic per axis.
//
// The single subscription keeps the Data Layer side cheap - one connection and
// one ruleset regardless of axis count, pushed by the motion app rather than
// polled. The fan-out happens on the ROS side, one topic per axis.
//
// Double buffered, and the lock is held for assignments only:
//
//   subscription thread : decode the burst into local Updates (flatbuffer
//                         verify, field extraction, string compares) OUTSIDE
//                         the lock, then take it once to apply them
//   ROS timer thread    : take the lock, copy m_write into m_read, release,
//                         then publish from m_read
//
// Neither side does real work under the mutex. That matters because the Data
// Layer calls its callback from client context and the SDK is explicit that
// blocking there stops all other callbacks, outgoing requests and broker
// responses for this client.
//
// The buffers are copied rather than swapped on purpose. The subscription
// delivers only the nodes that CHANGED, so m_write has to stay a complete
// authoritative cache; a plain swap would hand the writer back a
// one-generation-old buffer and any field that had not changed recently would
// be republished as zero.
class CtrlxStatusItf{
    public:
        using AxisStatus = axis_status_msg::msg::AxisStatus;

        CtrlxStatusItf(rclcpp::Node *rosNode, 
                       const std::vector<std::string> &axNames, 
                       comm::datalayer::DatalayerSystem &datalayerSystem, 
                       uint32_t rosPublishIntervalMs = 100, 
                       uint32_t publishIntervalMs = 100, 
                       uint64_t samplingIntervalUs = 50000); 
        ~CtrlxStatusItf(); 

    private: 
        // Which part of AxisStatus a subscribed address feeds.
        enum class NodeKind{ IpoValues, OpState }; 

        struct NodeRoute{
            size_t axIndex; 
            NodeKind kind; 
        }; 

        // Static per axis - set up in the constructor, read by the timer only.
        struct AxisEntry{
            std::string axName; 
            rclcpp::Publisher<AxisStatus>::SharedPtr pub; 
        }; 

        // The buffered part.
        struct AxisSample{
            AxisStatus msg; 
            // Set once this axis has had at least one node delivered. Until then
            // msg is all zeros, and publishing that would look like a real
            // reading of pos 0 / not powered / not faulted.
            bool seeded = false; 
        }; 

        // One decoded node, ready to be assigned into the write buffer.
        struct Update{
            size_t axIndex; 
            NodeKind kind; 
            double pos; 
            double vel; 
            double acc; 
            bool powered_on; 
            bool faulted; 
        }; 
        

        comm::datalayer::DlResult createSubscription(uint32_t publishIntervalMs, 
                                                     uint64_t samplingIntervalUs); 
        void onPublish(comm::datalayer::DlResult result, 
                       const std::vector<comm::datalayer::NotifyItem> &items); 
        void onTimer(); 
        bool nodeAddress(const comm::datalayer::Variant &info, std::string &out_address); 
        bool decodeIpoValues(const comm::datalayer::Variant &data, Update &out_update); 
        bool decodeOpState(const comm::datalayer::Variant &data, Update &out_update); 

        comm::datalayer::IClient3* m_client; 
        rclcpp::Node *m_node; 
        rclcpp::TimerBase::SharedPtr m_timer; 
        rclcpp::CallbackGroup::SharedPtr m_timer_group; 

        std::string m_sub_id; 
        std::vector<AxisEntry> m_axes; 
        // Full node address -> which axis, which field. Written in the
        // constructor, read-only afterwards. The callback delivers items in
        // undefined order and may omit or repeat nodes, so each item is routed
        // by the address in its notify_info, never by position.
        std::unordered_map<std::string, NodeRoute> m_routes; 

        std::vector<AxisSample> m_write;   // subscription callback writes here
        std::vector<AxisSample> m_read;    // timer publishes from here
        std::mutex m_buffer_mutex;         // assignments and the copy, nothing else
        };
}
