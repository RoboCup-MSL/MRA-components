#ifndef _MRA_FALCONS_LOCALIZATION_VISION_FLOOR_HPP
#define _MRA_FALCONS_LOCALIZATION_VISION_FLOOR_HPP

#include <opencv2/opencv.hpp>
#include "FalconsLocalizationVision_datatypes.hpp"


namespace MRA::FalconsLocalizationVision
{

class Floor
{
public:
    Floor();
    ~Floor();

    void configure(Params const &config);

    cv::Mat createMat() const;
    void letterModelToShapes(StandardLetterModel const &model, std::vector<MRA::Datatypes::Shape> &shapes) const;
    void shapesToCvMat(std::vector<MRA::Datatypes::Shape> const &shapes, cv::Mat &m) const;
    cv::Mat applyBlur(const cv::Mat &image, float blurFactor, int blurMaxDepth, uchar blurMinValue) const;
    cv::Point pointFcsToPixel(MRA::Datatypes::Point const &p) const;

    // diagnostics-specific
    void addGridLines(cv::Mat &m, float step, cv::Scalar color) const;

private:
    Params settings;
    float _ppm = 1;
    float _sizeX = 0.0;
    float _sizeY = 0.0;
    float _originX = 0.0;
    float _originY = 0.0;
    int _numPixelsX = 0;
    int _numPixelsY = 0;

    // blur helpers
    int blurSinglePass(cv::Mat &image, std::vector<cv::Point> &whitePixels, float blurFactor, uchar blurMinValue) const;

}; // class Floor

} // namespace MRA::FalconsLocalizationVision

#endif

