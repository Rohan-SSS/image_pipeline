#include <stereo_proc/stereo_proc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace stereo_proc{

bool StereoProcessor::process(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_raw,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_raw,
    const image_geometry::StereoCameraModel & model,
    StereoImageSet & output,
    int flags) const 
{
    // Do monocular processing on left and right images
    int left_flags = flags & LEFT_ALL;
    int right_flags = flags & RIGHT_ALL;

    if (flags & STEREO_ALL) {
        // Need the rectified images for stereo processing
        left_flags |= LEFT_RECT;
        right_flags |= RIGHT_RECT;
    }

    if (!mono_processor_.process(left_raw, model.left(), output.left, left_flags)) {
        return false;
    }
    if (!mono_processor_.process(right_raw, model.right(), output.right, right_flags >> 4)) {
        return false;
    }

    // Do elas matching to produce the disparity image
    if (flags & DISPARITY) {
        processDisparity(output.left.rect, output.right.rect, model, output.disparity);
    }
};

void StereoProcessor::processDisparity(
    const cv::Mat & left_rect,
    const cv::Mat & right_rect,
    const image_geometry::StereoCameraModel & model,
    stereo_msgs::msg::DisparityImage & disparity) const 
{
    // DPP = 1 as EM use 32-bit Float 
    static const int DPP = 1;
    static const double inv_dpp = 1.0 / DPP;

    // current_stereo_algorithm_ == EM throws error
    // if (current_stereo_algorithm_ == EM){
        elas_matcher_->compute(left_rect, right_rect, disparity32_);
    // }

    // disparity msg
    sensor_msgs::msg::Image & dimage = disparity.image;
    dimage.height = disparity32_.rows;
    dimage.width = disparity32_.cols;
    dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    dimage.step = dimage.width * sizeof(float);
    dimage.data.resize(dimage.step * dimage.height);

    // disparity matrix for priciple focus adjustment
    cv::Mat_<float> dmat(
        dimage.height, dimage.width, 
        reinterpret_cast<float*>(&dimage.data[0]), 
        dimage.step);

    // inv_dpp = 1.0 | (true_disparity = raw_disparity - (cx_l - cx_r))
    disparity32_.convertTo(dmat, dmat.type(), inv_dpp, -(model.left().cx() - model.right().cx()));

    // focal and baseline
    disparity.f = model.right().fx();
    disparity.t = model.baseline();

    disparity.min_disparity = getMinDisparity();
    disparity.max_disparity = getMinDisparity() + getDisparityRange() - 1;
    disparity.delta_d = inv_dpp;
}

} // namespace stereo_proc