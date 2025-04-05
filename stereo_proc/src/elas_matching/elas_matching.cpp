#include "elas_matching/elas_matching.hpp"
#include "elas.h"
#include <opencv2/imgproc.hpp>

namespace cv
{

class StereoEMImpl : public StereoEM {
public:
    // class constructor loads the ROBOTICS default parameters preset 
    StereoEMImpl() : elas(Elas::ROBOTICS) {}

    void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity) override {

        if (left.empty() || right.empty()) {
            CV_Error(cv::Error::StsBadArg, "Both left and right images must not be empty");
        }

        if (left.size() != right.size()){
            CV_Error(cv::Error::StsUnmatchedSizes, "All the images must have the same size");
        }

        if (left.type() != CV_8UC1 || right.type() != CV_8UC1){
            CV_Error(cv::Error::StsUnsupportedFormat, "Both input images must have CV_8UC1");
        }

        if (disparity.fixedType() && disparity.type() != CV_32FC1) {
            CV_Error(cv::Error::StsUnsupportedFormat, "Disparity image must have CV_32FC1 format");
        }

        // pre allocating cv::OutputArray disparity
        disparity.create(left.size(), CV_32FC1);
        cv::Mat left_disp = disparity.getMat();

        // memory for right disparity (required but not used) 
        cv::Mat right_disp(right.size(), CV_32F);

        cv::Mat left_mat = left.getMat();
        cv::Mat right_mat = right.getMat();
        // dimensions array for elas.process()
        const int32_t dims[3] = {
            left_mat.cols,
            left_mat.rows,
            static_cast<int32_t>(left_mat.step[0])
        };
    
        // elas.process(uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims);
        elas.process(
            left_mat.data,
            right_mat.data,
            reinterpret_cast<float*> (left_disp.data),
            reinterpret_cast<float*> (right_disp.data),
            dims
        );
    }

    // Empty implementations, not used in ELAS, but required by cv::SteroMatcher
    int getBlockSize() const override {return 0;} 
    void setBlockSize(int blockSize) override {}    

    int getSpeckleRange() const override {return 0;}  
    void setSpeckleRange(int speckleRange) override {}

    int getDisp12MaxDiff() const override {return 0;}  
    void setDisp12MaxDiff(int disp12MaxDiff) override {}

    // getters and setters
    // StereoMatcher params
    int getMinDisparity() const override {return elas.param.disp_min;}
    void setMinDisparity(int minDisparity) override {elas.param.disp_min = minDisparity;}

    int getNumDisparities() const override {return elas.param.disp_max;}
    void setNumDisparities(int numDisparities) override {elas.param.disp_max = numDisparities;}

    int getSpeckleWindowSize() const override {return elas.param.speckle_size;}
    void setSpeckleWindowSize(int speckleWindowSize) override {elas.param.speckle_size = speckleWindowSize;}

    // StereoEM params
    // int params
    int getSupportTexture() const {return elas.param.support_texture;}
    void setSupportTexture(int supportTexture){elas.param.support_texture = supportTexture;}

    int getCandidateStepsize() const {return elas.param.candidate_stepsize;}
    void setCandidateStepsize(int candidateStepsize){elas.param.candidate_stepsize = candidateStepsize;}

    int getInconsistencyWindowSize() const {return elas.param.incon_window_size;}
    void setInconsistencyWindowSize(int inconWindowSize){elas.param.incon_window_size = inconWindowSize;}

    int getInconsistencyThreshold() const {return elas.param.incon_threshold;}
    void setInconsistencyThreshold(int inconThreshold){elas.param.incon_threshold = inconThreshold;}

    int getInconsistencyMinimumSupport() const {return elas.param.incon_min_support;}
    void setInconsistencyMinimumSupport(int inconMinSupport){elas.param.incon_min_support = inconMinSupport;}

    int getGridSize() const {return elas.param.grid_size;}
    void setGridSize(int gridSize){elas.param.grid_size = gridSize;}

    int getMatchTexture() const {return elas.param.match_texture;}
    void setMatchTexture(int matchTexture){elas.param.match_texture = matchTexture;}

    int getLRDThreshold() const {return elas.param.lr_threshold;}
    void setLRDThreshold(int lrdThreshold){elas.param.lr_threshold = lrdThreshold;}

    int getInterpolateGapWidth() const {return elas.param.ipol_gap_width;}
    void setInterpolateGapWidth(int interpolateGapWidth){elas.param.ipol_gap_width = interpolateGapWidth;}

    // float params
    float getSupportThreshold() const {return elas.param.support_threshold;}
    void setSupportThreshold(float supportThreshold){elas.param.support_threshold = supportThreshold;}
    
    float getBeta() const {return elas.param.beta;}
    void setBeta(float beta){elas.param.beta = beta;}

    float getGamma() const {return elas.param.gamma;}
    void setGamma(float gamma){elas.param.gamma = gamma;}

    float getSigma() const {return elas.param.sigma;}
    void setSigma(float sigma){elas.param.sigma = sigma;}
    
    float getSigmaRadius() const {return elas.param.sradius;}
    void setSigmaRadius(float sigmaRadius){elas.param.sradius = sigmaRadius;}

    float getSpeckleSimilarityThreshold() const {return elas.param.speckle_sim_threshold;}
    void setSpeckleSimilarityThreshold(float speckleSimilarityThreshold){elas.param.speckle_sim_threshold = speckleSimilarityThreshold;}

    // int bool params
    int getAddCorners() const {return elas.param.add_corners;}
    void setAddCorners(int addCorners){elas.param.add_corners = addCorners;}

    int getFilterMedian() const {return elas.param.filter_median;}
    void setFilterMedian(int filterMedian) { elas.param.filter_median = filterMedian; }

    int getFilterAdaptiveMean() const {return elas.param.filter_adaptive_mean;}
    void setFilterAdaptiveMean(int filterAdaptiveMean){elas.param.filter_adaptive_mean = filterAdaptiveMean;}

    int getPostProcessOnlyLeft() const {return elas.param.postprocess_only_left;}
    void setPostProcessOnlyLeft(int postProcessOnlyLeft){elas.param.postprocess_only_left = postProcessOnlyLeft;}
    
    int getSubSampling() const {return elas.param.subsampling;}
    void setSubSampling(int subSampling){elas.param.subsampling = subSampling;}

private:
    Elas elas;
    static const char* name_;
};

const char* StereoEMImpl::name_ = "StereoMatcher.EM";

CV_WRAP Ptr<StereoEM> StereoEM::create() {
    return cv::makePtr<StereoEMImpl>(); 
}
} //cvf