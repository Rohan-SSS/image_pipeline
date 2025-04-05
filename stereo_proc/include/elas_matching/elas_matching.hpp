#ifndef ELAS_MATCHING__ELAS_MATCHING_HPP_
#define ELAS_MATCHING__ELAS_MATCHING_HPP_

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace cv
{

class CV_EXPORTS_W StereoEM : public cv::StereoMatcher {
public:

    // float   support_threshold;      // max. uniqueness ratio (best vs. second best support match)
    CV_WRAP virtual float getSupportThreshold() const = 0;
    CV_WRAP virtual void setSupportThreshold(float supportThreshold) = 0;

    // int32_t support_texture;        // min texture for support points
    CV_WRAP virtual int getSupportTexture() const = 0;
    CV_WRAP virtual void setSupportTexture(int supportTexture) = 0;

    // int32_t candidate_stepsize;     // step size of regular grid on which support points are matched
    CV_WRAP virtual int getCandidateStepsize() const = 0;
    CV_WRAP virtual void setCandidateStepsize(int candidateStepsize) = 0;

    // int32_t incon_window_size;      // window size of inconsistent support point check
    CV_WRAP virtual int getInconsistencyWindowSize() const = 0;
    CV_WRAP virtual void setInconsistencyWindowSize(int inconWindowSize) = 0;
    
    // int32_t incon_threshold;        // disparity similarity threshold for support point to be considered consistent
    CV_WRAP virtual int getInconsistencyThreshold() const = 0;
    CV_WRAP virtual void setInconsistencyThreshold(int inconThreshold) = 0;
    
    // int32_t incon_min_support;      // minimum number of consistent support points
    CV_WRAP virtual int getInconsistencyMinimumSupport() const = 0;
    CV_WRAP virtual void setInconsistencyMinimumSupport(int inconMinSupport) = 0;

    // bool    add_corners;            // add support points at image corners with nearest neighbor disparities
    CV_WRAP virtual int getAddCorners() const = 0;
    CV_WRAP virtual void setAddCorners(int addCorners) = 0;

    // int32_t grid_size;              // sfize of neighborhood for additional support point extrapolation
    CV_WRAP virtual int getGridSize() const = 0;
    CV_WRAP virtual void setGridSize(int gridSize) = 0;

    // float   beta;                   // image likelihood parameter
    CV_WRAP virtual float getBeta() const = 0;
    CV_WRAP virtual void setBeta(float beta) = 0;
    
    // float   gamma;                  // prior constant
    CV_WRAP virtual float getGamma() const = 0;
    CV_WRAP virtual void setGamma(float gamma) = 0;
    
    // float   sigma;                  // prior sigma
    CV_WRAP virtual float getSigma() const = 0;
    CV_WRAP virtual void setSigma(float sigma) = 0;
    
    // float   sradius;                // prior sigma radius
    CV_WRAP virtual float getSigmaRadius() const = 0;
    CV_WRAP virtual void setSigmaRadius(float sigmaRadius) = 0;

    // int32_t match_texture;          // min texture for dense matching
    CV_WRAP virtual int getMatchTexture() const = 0;
    CV_WRAP virtual void setMatchTexture(int matchTexture) = 0;

    // int32_t lr_threshold;           // disparity threshold for left/right consistency check
    CV_WRAP virtual int getLRDThreshold() const = 0;
    CV_WRAP virtual void setLRDThreshold(int lrdThreshold) = 0;

    // float   speckle_sim_threshold;  // similarity threshold for speckle segmentation
    CV_WRAP virtual float getSpeckleSimilarityThreshold() const = 0;
    CV_WRAP virtual void setSpeckleSimilarityThreshold(float speckleSimilarityThreshold) = 0;

    // int32_t ipol_gap_width;         // interpolate small gaps (left<->right, top<->bottom)
    CV_WRAP virtual int getInterpolateGapWidth() const = 0;
    CV_WRAP virtual void setInterpolateGapWidth(int interpolateGapWidth) = 0;

    // bool    filter_median;          // optional median filter (approximated)
    CV_WRAP virtual int getFilterMedian() const = 0;
    CV_WRAP virtual void setFilterMedian(int filterMedian) = 0;

    // bool    filter_adaptive_mean;   // optional adaptive mean filter (approximated)
    CV_WRAP virtual int getFilterAdaptiveMean() const = 0;
    CV_WRAP virtual void setFilterAdaptiveMean(int filterAdaptiveMedian) = 0;

    // bool    postprocess_only_left;  // saves time by not postprocessing the right image
    CV_WRAP virtual int getPostProcessOnlyLeft() const = 0;
    CV_WRAP virtual void setPostProcessOnlyLeft(int postProcessOnlyLeft) = 0;
    
    // bool    subsampling;            // saves time by only computing disparities for each 2nd pixel
    CV_WRAP virtual int getSubSampling() const = 0;
    CV_WRAP virtual void setSubSampling(int subSampling) = 0;

    // overriding opencv StereoMatcher::compute method (empty impl)
    CV_WRAP void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity) override {};
    
    CV_WRAP static cv::Ptr<StereoEM> create();
};

} // cv

#endif