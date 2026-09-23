#include  "../include/motion_action_cpp/ctrlx_motion_node.hpp"
#include  "../include/motion_action_cpp/dl_helper.hpp"
#include <chrono>
#include <thread>

ros2_ctrlx_motion::CtrlxMotionNode::CtrlxMotionNode():rclcpp::Node("ctrlx_motion_node")
{
    declare_parameter<std::string>("axes_root", "motion/axs"); 
    declare_parameter<std::vector<std::string>>("axes", std::vector<std::string>{}); 
    declare_parameter<int>("status_ros_publish_interval_ms", 100); 
    declare_parameter<int>("status_publish_interval_ms", 100); 
    declare_parameter<int>("status_sampling_interval_us", 50000); 
}

ros2_ctrlx_motion::CtrlxMotionNode::~CtrlxMotionNode()
{
    // Explicit, and in this order: both hold clients created from the system's
    // factory, so they must be released before the system stops. Declaration
    // order already guarantees this, but a running action is on a detached
    // thread, so tear the handlers down before anything else goes away.
    m_status.reset(); 
    m_axes.clear(); 
    m_datalayerSystem.stop(); 
}

bool ros2_ctrlx_motion::CtrlxMotionNode::init()
{
    // false: we are a pure client, no broker.
    m_datalayerSystem.start(false); 

    std::vector<std::string> t_axNames = get_parameter("axes").as_string_array(); 
    if (t_axNames.empty() && !browseAxes(t_axNames))
    {
        return false; 
    }
    if (t_axNames.empty())
    {
        RCLCPP_ERROR(
            get_logger(), "No axes found under '%s' - none configured, or wrong axes_root",
            get_parameter("axes_root").as_string().c_str()); 
        return false; 
    }

    // Reentrant: goal and cancel callbacks for different axes must be able to
    // run at the same time. The executes are on their own detached threads.
    m_action_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant); 

    m_axes.reserve(t_axNames.size()); 
    for (const auto &t_axName : t_axNames)
    {
        RCLCPP_INFO(get_logger(), "Axis '%s': creating action servers", t_axName.c_str()); 
        m_axes.push_back(std::make_unique<CtrlxActionItf>(this,
                                                          t_axName,
                                                          m_datalayerSystem,
                                                          m_action_group)); 
    }

    m_status = std::make_unique<CtrlxStatusItf>(this,
                                                t_axNames,
                                                m_datalayerSystem,
                                                uint32_t(get_parameter("status_ros_publish_interval_ms").as_int()),
                                                uint32_t(get_parameter("status_publish_interval_ms").as_int()),
                                                uint64_t(get_parameter("status_sampling_interval_us").as_int())); 

    RCLCPP_INFO(get_logger(), "Serving %zu axes", t_axNames.size()); 
    return true; 
}

bool ros2_ctrlx_motion::CtrlxMotionNode::browseAxes(std::vector<std::string> &out_axNames)
{
    auto conStr = dl_helper::getConnectionString(); 
    auto *t_client = m_datalayerSystem.factory()->createClient3(conStr); 
    if (t_client == nullptr || !t_client->isConnected())
    {
        RCLCPP_ERROR(get_logger(), "Could not connect to the Data Layer to browse axes"); 
        delete t_client; 
        return false; 
    }

    const std::string t_root = get_parameter("axes_root").as_string(); 
    comm::datalayer::Variant t_payload; 
    auto t_result = t_client->browseSync(t_root, &t_payload); 
    if (t_result != DL_OK)
    {
        RCLCPP_ERROR(
            get_logger(), "Browse of '%s' failed - %s",
            t_root.c_str(), std::string(t_result.toString()).c_str()); 
        delete t_client; 
        return false; 
    }

    if (t_payload.getType() != comm::datalayer::VariantType::ARRAY_OF_STRING)
    {
        RCLCPP_ERROR(
            get_logger(), "Browse of '%s' returned %s, expected an array of strings",
            t_root.c_str(), t_payload.typeAsString().c_str()); 
        delete t_client; 
        return false; 
    }

    const char **t_entries = t_payload; 
    for (size_t i = 0; t_entries != nullptr && i < t_payload.getCount(); i++)
    {
        if (t_entries[i] != nullptr)
        {
            // TODO filter here if the axis root also lists non-axis children.
            out_axNames.push_back(t_entries[i]); 
            RCLCPP_INFO(get_logger(), "Discovered axis '%s'", t_entries[i]); 
        }
    }

    // Browsing is a one-off; the handlers open their own clients.
    delete t_client; 
    return true; 
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); 

    auto t_node = std::make_shared<ros2_ctrlx_motion::CtrlxMotionNode>(); 
    if (!t_node->init())
    {
        RCLCPP_FATAL(t_node->get_logger(), "Initialisation failed"); 
        rclcpp::shutdown(); 
        return 1; 
    }

    // MultiThreaded so goal and cancel callbacks for different axes are not
    // serialised behind each other.
    rclcpp::executors::MultiThreadedExecutor t_executor; 
    t_executor.add_node(t_node); 
    t_executor.spin(); 

    // Detached action threads exit on rclcpp::ok() going false; give them a
    // moment before the handlers (and their clients) are destroyed.
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); 

    rclcpp::shutdown(); 
    return 0; 
}
