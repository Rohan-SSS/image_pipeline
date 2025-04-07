#ifndef STEREO_PROC__STEREO_PROC_HPP_
#define STEREO_PROC__STEREO_PROC_HPP_

#include <string>
#include <stereo_proc/elas_matching.hpp>
#include "image_geometry/stereo_camera_model.h"
#include <image_proc/processor.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

namespace stereo_proc{

struct StereoImageSet{
    image_proc::ImageSet left;
    image_proc::ImageSet right;
    stereo_msgs::msg::DisparityImage disparity;
    // stereo_msgs::msg::PointCloud points;
    // stereo_msgs::msg::PointCloud2 points2;
};

class StereoProcessor{
public:
    StereoProcessor(){
        elas_matcher_ = cv::StereoEM::create();
    }

    enum StereoType{
        EM
    };

    enum
    {
        LEFT_MONO        = 1 << 0,
        LEFT_RECT        = 1 << 1,
        LEFT_COLOR       = 1 << 2,
        LEFT_RECT_COLOR  = 1 << 3,
        RIGHT_MONO       = 1 << 4,
        RIGHT_RECT       = 1 << 5,
        RIGHT_COLOR      = 1 << 6,
        RIGHT_RECT_COLOR = 1 << 7,
        DISPARITY        = 1 << 8,
        POINT_CLOUD      = 1 << 9,
        POINT_CLOUD2     = 1 << 10,
        LEFT_ALL = LEFT_MONO | LEFT_RECT | LEFT_COLOR | LEFT_RECT_COLOR,
        RIGHT_ALL = RIGHT_MONO | RIGHT_RECT | RIGHT_COLOR | RIGHT_RECT_COLOR,
        STEREO_ALL = DISPARITY | POINT_CLOUD | POINT_CLOUD2,
        ALL = LEFT_ALL | RIGHT_ALL | STEREO_ALL
    };

    inline StereoType getStereoType() const{
        return current_stereo_algorithm_;
    }

    inline void setStereoType(StereoType type){
        current_stereo_algorithm_ = type;
    }

    // INTERP_LINEAR 
    inline int getInterpolation() const
    {
      return mono_processor_.interpolation_;
    }
  
    inline void setInterpolation(int interp)
    {
      mono_processor_.interpolation_ = interp;
    }

    // StereoMatcher  
    inline int getMinDisparity() const {
        return elas_matcher_->getMinDisparity();
    }

    inline void setMinDisparity(int minDisparity){
        elas_matcher_->setMinDisparity(minDisparity);
    }

    inline int getDisparityRange() const {
        return elas_matcher_->getNumDisparities();
    }

    inline void setDisparityRange(int numDisparities){
        elas_matcher_->setNumDisparities(numDisparities);
    }

    inline int getSpeckleWindowSize() const {
        return elas_matcher_->getSpeckleWindowSize();
    }

    inline void setSpeckleWindowSize(int speckleWindowSize){
        elas_matcher_->setSpeckleWindowSize(speckleWindowSize);
    }

    // StereoEM
    inline float getSupportThreshold() const {
        return elas_matcher_->getSupportThreshold();
    }

    inline void setSupportThreshold(float supportThreshold){
        elas_matcher_->setSupportThreshold(supportThreshold);
    }

    inline int getSupportTexture() const {
        return elas_matcher_->getSupportTexture();
    }

    inline void setSupportTexture(int supportTexture){
        elas_matcher_->setSupportTexture(supportTexture);
    }

    inline int getCandidateStepsize() const {
        return elas_matcher_->getCandidateStepsize();
    }

    inline void setCandidateStepsize(int candidateStepsize){
        elas_matcher_->setCandidateStepsize(candidateStepsize);
    }

    inline int getInconsistencyWindowSize() const {
        return elas_matcher_->getInconsistencyWindowSize();
    }
    
    inline void setInconsistencyWindowSize(int inconWindowSize){
        elas_matcher_->setInconsistencyWindowSize(inconWindowSize);
    }

    inline int getInconsistencyThreshold() const {
        return elas_matcher_->getInconsistencyThreshold();
    }
    
    inline void setInconsistencyThreshold(int inconThreshold){
        elas_matcher_->setInconsistencyThreshold(inconThreshold);
    }

    inline int getInconsistencyMinimumSupport() const {
        return elas_matcher_->getInconsistencyMinimumSupport();
    }

    inline void setInconsistencyMinimumSupport(int inconMinSupport){
        elas_matcher_->setInconsistencyMinimumSupport(inconMinSupport);
    }

    inline int getAddCorners() const {
        return elas_matcher_->getAddCorners();
    }

    inline void setAddCorners(int addCorners){
        elas_matcher_->setAddCorners(addCorners);
    }

    inline int getGridSize() const {
        return elas_matcher_->getGridSize();
    }

    inline void setGridSize(int gridSize){
        elas_matcher_->setGridSize(gridSize);
    }

    inline float getBeta() const {
        return elas_matcher_->getBeta();
    }
    
    inline void setBeta(float beta){
        elas_matcher_->setBeta(beta);
    }

    inline float getGamma() const {
        return elas_matcher_->getGamma();
    }

    inline void setGamma(float gamma){
        elas_matcher_->setGamma(gamma);
    }

    inline float getSigma() const {
        return elas_matcher_->getSigma();
    }

    inline void setSigma(float sigma){
        elas_matcher_->setSigma(sigma);
    }

    inline float getSigmaRadius() const {
        return elas_matcher_->getSigmaRadius();
    }
    inline void setSigmaRadius(float sigmaRadius){
        elas_matcher_->setSigmaRadius(sigmaRadius);
    }

    inline int getMatchTexture() const {
        return elas_matcher_->getMatchTexture();
    }

    inline void setMatchTexture(int matchTexture){
        elas_matcher_->setMatchTexture(matchTexture);
    }

    inline int getLRDThreshold() const {
        return elas_matcher_->getLRDThreshold();
    }
    
    inline void setLRDThreshold(int lrdThreshold){
        elas_matcher_->setLRDThreshold(lrdThreshold);
    }

    inline float getSpeckleSimilarityThreshold() const {
        return elas_matcher_->getSpeckleSimilarityThreshold();
    }

    inline void setSpeckleSimilarityThreshold(float speckleSimilarityThreshold){
        elas_matcher_->setSpeckleSimilarityThreshold(speckleSimilarityThreshold);
    }

    inline int getInterpolateGapWidth() const {
        return elas_matcher_->getInterpolateGapWidth();
    }

    inline void setInterpolateGapWidth(int interpolateGapWidth){
        elas_matcher_->setInterpolateGapWidth(interpolateGapWidth);
    }

    inline int getFilterMedian() const {
        return elas_matcher_->getFilterMedian();
    }

    inline void setFilterMedian(int filterMedian){
        elas_matcher_->setFilterMedian(filterMedian);
    }  

    inline int getFilterAdaptiveMean() const {
        return elas_matcher_->getFilterAdaptiveMean();
    }

    inline void setFilterAdaptiveMean(int filterAdaptiveMedian){
        elas_matcher_->setFilterAdaptiveMean(filterAdaptiveMedian);
    }

    inline int getPostProcessOnlyLeft() const {
        return elas_matcher_->getPostProcessOnlyLeft();
    }

    inline void setPostProcessOnlyLeft(int postProcessOnlyLeft){
        elas_matcher_->setPostProcessOnlyLeft(postProcessOnlyLeft);
    }

    inline int getSubSampling() const {
        return elas_matcher_->getSubSampling();
    }

    inline void setSubSampling(int subSampling){
        elas_matcher_->setSubSampling(subSampling);
    }

    // 
    bool process(
        const sensor_msgs::msg::Image::ConstSharedPtr & letf_raw,
        const sensor_msgs::msg::Image::ConstSharedPtr & right_raw,
        const image_geometry::StereoCameraModel & model,
        StereoImageSet & output,
        int flags) const;

    void processDisparity(
        const cv::Mat & left_rect,
        const cv::Mat & right_rect,
        const image_geometry::StereoCameraModel & model,
        stereo_msgs::msg::DisparityImage & disparity_msg) const;

    void processPoints() const;

    void processPoints2() const;

private:
    image_proc::Processor mono_processor_;
    /// Scratch buffer for 16-bit signed disparity image
    mutable cv::Mat_<float> disparity32_;
    /// Contains scratch buffers for block matching.
    mutable cv::Ptr<cv::StereoEM> elas_matcher_;
    StereoType current_stereo_algorithm_;
};
} // namespace stereo_proc

#endif