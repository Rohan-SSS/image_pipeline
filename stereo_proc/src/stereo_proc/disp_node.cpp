#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/stereo_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include <stereo_proc/stereo_proc.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <opencv2/calib3d/calib3d.hpp>

namespace stereo_proc{

class DisparityNode : public rclcpp::Node{
public:
    explicit DisparityNode(const rclcpp::NodeOptions & options);

private:
    enum StereoAlgorithm{
        ELAS_MATCHING = 0
    };

    // Subscriptions
    image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_l_info_, sub_r_info_;
    using ExactPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;
    using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;
    using ExactSync = message_filters::Synchronizer<ExactPolicy>;
    using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
    std::shared_ptr<ExactSync> exact_sync_;
    std::shared_ptr<ApproximateSync> approximate_sync_;

    // Publications
    std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> pub_disparity_;

    // Handle to parameters callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // Processing state (note: only safe because we're single-threaded!)
    image_geometry::StereoCameraModel model_;

    // contains scratch buffers for block matching
    stereo_proc::StereoProcessor elas_matcher_;

    void connectCb();

    void imageCb(
        const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg);

    rcl_interfaces::msg::SetParametersResult parameterSetCb(
        const std::vector<rclcpp::Parameter> & parameters);
};

// Some helper functions for adding a parameter to a collection
static void add_param_to_map(
    std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
    const std::string & name,
    const std::string & description,
    const int default_value 
){
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    parameters[name] = std::make_pair(default_value, descriptor);
}

static void add_param_to_map(
    std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
    const std::string & name,
    const std::string & description,
    const double default_value
){
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    parameters[name] = std::make_pair(default_value, descriptor);
}

DisparityNode::DisparityNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("disparity_node", options){
    using namespace std::placeholders;

    // Declare/read parameters
    int queue_size = this->declare_parameter("queue_size", 5);
    bool approx = this->declare_parameter("approximate_sync", false);
    this->declare_parameter("use_system_default_qos", false);

    // Synchronize callbacks
    if (approx) {
        approximate_sync_.reset(
            new ApproximateSync(
                ApproximatePolicy(queue_size),
                sub_l_image_, sub_l_info_,
                sub_r_image_, sub_r_info_));
        approximate_sync_->registerCallback(
          std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
      } else {
        exact_sync_.reset(
            new ExactSync(
                ExactPolicy(queue_size),
                sub_l_image_, sub_l_info_,
                sub_r_image_, sub_r_info_));
        exact_sync_->registerCallback(
          std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
    }

    // Register a callback for when parameters are set
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DisparityNode::parameterSetCb, this, _1));
    
    // Describe int parameters
    std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> int_params;
    add_param_to_map(
        int_params,
        "stereo_algorithm",
        "Stereo algorithm: ELAS Matching (0)",
        0);
    add_param_to_map(
        int_params,
        "disp_min",
        "Minimum disparity value to consider in the search range",
        0);
    add_param_to_map(
        int_params,
        "disp_max",
        "Maximum disparity value to consider in the search range",
        255);
    add_param_to_map(
        int_params,
        "support_texture",
        "Minimum texture intensity for a point to be considered a support point",
        10);
    add_param_to_map(
        int_params,
        "candidate_stepsize",
        "Grid step size for selecting support point candidates",
        5);
    add_param_to_map(
        int_params,
        "incon_window_size",
        "Window size for checking inconsistent support points",
        5);
    add_param_to_map(
        int_params,
        "incon_threshold",
        "Disparity similarity threshold for support point consistency",
        5);
    add_param_to_map(
        int_params,
        "incon_min_support",
        "Minimum number of consistent support points required",
        5);
    add_param_to_map(
        int_params,
        "grid_size",
        "Neighborhood size for extrapolating additional support points",
        20);
    add_param_to_map(
        int_params,
        "match_texture",
        "Minimum texture for dense matching (avoids low-texture areas)",
        1);
    add_param_to_map(
        int_params,
        "lr_threshold",
        "Threshold for left-right disparity consistency check",
        2);
    add_param_to_map(
        int_params,
        "speckle_size",
        "Maximum size (in pixels) of speckle regions to be removed",
        200);
    add_param_to_map(
        int_params,
        "ipol_gap_width",
        "Maximum gap width (in pixels) to interpolate missing disparities",
        3);

    // describe float params
    std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> double_params;
    add_param_to_map(
        double_params,
        "support_threshold",
        "Uniqueness ratio threshold for support point matches",
        0.85);
    add_param_to_map(
        double_params,
        "beta",
        "Weight parameter for image likelihood in cost computation.",
        0.02);
    add_param_to_map(
        double_params,
        "gamma",
        "Prior constant for smoothness regularization",
        3.0);
    add_param_to_map(
        double_params,
        "sigma",
        "Prior sigma (smoothness) for disparity gradients",
        1.0);
    add_param_to_map(
        double_params,
        "sradius",
        "Radius for prior sigma smoothing",
        2.0);
    add_param_to_map(
        double_params,
        "speckle_sim_threshold",
        "Similarity threshold for speckle region segmentation",
        1.0);

    // describe bool params
    std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> int_bool_params;
    add_param_to_map(
        int_bool_params,
        "add_corners",
        "Flag to add support points at image corners using nearest-neighbor disparities",
        0);
    add_param_to_map(
        int_bool_params,
        "filter_median",
        "Flag to enable median filtering for postprocessing",
        0);
    add_param_to_map(
        int_bool_params,
        "filter_adaptive_mean",
        "Flag to enable adaptive mean filtering for postprocessing",
        1);
    add_param_to_map(
        int_bool_params,
        "postprocess_only_left",
        "Flag to skip right image postprocessing for speed",
        1);
    add_param_to_map(
        int_bool_params,
        "subsampling",
        "Flag to compute disparities at half-resolution for speed",
        0);

    // declare parameters, triggers callback
    this->declare_parameters("", int_params);
    this->declare_parameters("", double_params);
    this->declare_parameters("", int_bool_params);

    // Update the publisher options to allow reconfigurable qos settings.
    rclcpp::PublisherOptions pub_opts;
    pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1, pub_opts);

    connectCb();
}

void DisparityNode::connectCb(){

    image_transport::TransportHints hints(this, "raw");

    const bool use_system_default_qos = this->get_parameter("use_system_default_qos").as_bool();
    rclcpp::QoS image_sub_qos = rclcpp::SensorDataQoS();
    if (use_system_default_qos) {
      image_sub_qos = rclcpp::SystemDefaultsQoS();
    }

    const auto image_sub_rmw_qos = image_sub_qos.get_rmw_qos_profile();
    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    sub_l_image_.subscribe(
      this, "left/image_rect", hints.getTransport(), image_sub_rmw_qos, sub_opts);
    sub_l_info_.subscribe(this, "left/camera_info", image_sub_rmw_qos, sub_opts);
    sub_r_image_.subscribe(
      this, "right/image_rect", hints.getTransport(), image_sub_rmw_qos, sub_opts);
    sub_r_info_.subscribe(this, "right/camera_info", image_sub_rmw_qos, sub_opts);
}

void DisparityNode::imageCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg)
{
    // If there are no subscriptions for the disparity image, do nothing
    if (pub_disparity_->get_subscription_count() == 0u) {
        return;
    }

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Allocate new disparity image message
    auto disp_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
    disp_msg->header = l_info_msg->header;
    disp_msg->image.header = l_info_msg->header;

    const cv::Mat_<uint8_t> l_image =
        cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
    const cv::Mat_<uint8_t> r_image =
        cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

    // Perform ELAS matching to find the disparities
    elas_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);
    pub_disparity_->publish(*disp_msg);
}

rcl_interfaces::msg::SetParametersResult DisparityNode::parameterSetCb(
    const std::vector<rclcpp::Parameter> & parameters
){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for(const auto & param: parameters) {
        const std::string param_name = param.get_name();
        if ("stereo_algorithm" == param_name){
            const int stereo_algorithm_value = param.as_int();
            if (ELAS_MATCHING == stereo_algorithm_value){
                elas_matcher_.setStereoType(StereoProcessor::EM);
            } else {
                result.successful = false;
                std::ostringstream oss;
                oss << "Unknown stereo algorithm type '" << stereo_algorithm_value << "'";
                result.reason = oss.str();
            }
        } else if ("disp_min" == param_name){
            elas_matcher_.setMinDisparity(param.as_int());
        } else if ("disp_max" == param_name){
            elas_matcher_.setDisparityRange(param.as_int());
        } else if ("support_threshold" == param_name){
            elas_matcher_.setSupportThreshold(param.as_double());
        } else if ("support_texture" == param_name){
            elas_matcher_.setSupportThreshold(param.as_int());
        } else if ("candidate_stepsize" == param_name){
            elas_matcher_.setCandidateStepsize(param.as_int());
        } else if ("incon_window_size" == param_name){
            elas_matcher_.setInconsistencyWindowSize(param.as_int());
        } else if ("incon_threshold" == param_name){
            elas_matcher_.setInconsistencyThreshold(param.as_int());
        } else if ("incon_min_support" == param_name){
            elas_matcher_.setInconsistencyMinimumSupport(param.as_int());
        } else if ("add_corners" == param_name){
            elas_matcher_.setAddCorners(param.as_int());
        } else if ("grid_size" == param_name){
            elas_matcher_.setGridSize(param.as_int());
        } else if ("beta" == param_name){
            elas_matcher_.setBeta(param.as_double());
        } else if ("gamma" == param_name){
            elas_matcher_.setGamma(param.as_double());
        } else if ("sigma" == param_name){
            elas_matcher_.setSigma(param.as_double());
        } else if ("sradius" == param_name){
            elas_matcher_.setSigmaRadius(param.as_double());
        } else if ("match_texture" == param_name){
            elas_matcher_.setMatchTexture(param.as_int());
        } else if ("lr_threshold" == param_name){
            elas_matcher_.setLRDThreshold(param.as_int());
        } else if ("speckle_sim_threshold" == param_name){
            elas_matcher_.setSpeckleSimilarityThreshold(param.as_double());
        } else if ("speckle_size" == param_name){
            elas_matcher_.setSpeckleWindowSize(param.as_int());
        } else if ("ipol_gap_width" == param_name){
            elas_matcher_.setInterpolateGapWidth(param.as_int());
        } else if ("filter_median" == param_name){
            elas_matcher_.setFilterMedian(param.as_int());
        } else if ("filter_adaptive_mean" == param_name){
            elas_matcher_.setFilterAdaptiveMean(param.as_int());
        } else if ("postprocess_only_left" == param_name){
            elas_matcher_.setPostProcessOnlyLeft(param.as_int());
        } else if ("subsampling" == param_name){
            elas_matcher_.setSubSampling(param.as_int());        
        }
    }
    return result;
}
    
} // stereo_proc

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_proc::DisparityNode)