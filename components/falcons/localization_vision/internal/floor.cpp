#include "floor.hpp"
#include "geometry.hpp" // from MRA library geometry
#include "logging.hpp" // from MRA library logging

using namespace MRA::FalconsLocalizationVision;

Floor::Floor()
{
    MRA_TRACE_FUNCTION();
}

Floor::~Floor()
{
    MRA_TRACE_FUNCTION();
}

void Floor::configure(Params const &config)
{
    MRA_TRACE_FUNCTION_INPUTS(config);
    settings.CopyFrom(config);
    _ppm = settings.solver().pixelspermeter();
    // determine floor size
    _sizeX = settings.model().b() + 2.0 * settings.solver().floorborder();
    _sizeY = settings.model().a() + 2.0 * settings.solver().floorborder();
    // origin is the FCS point at pixel (0,0)
    _originX = -0.5 * _sizeX;
    _originY = -0.5 * _sizeY;
    // number of pixels (rotated)
    _numPixelsY = int(_sizeY * _ppm);
    _numPixelsX = int(_sizeX * _ppm);
    MRA_TRACE_FUNCTION_OUTPUTS(_numPixelsX, _numPixelsY);
}

cv::Mat Floor::createMat() const
{
    MRA_TRACE_FUNCTION_INPUTS(_numPixelsX, _numPixelsY);
    if (_numPixelsX * _numPixelsY == 0) {
        throw std::runtime_error("Floor::createMat: invalid shape (" + std::to_string(_numPixelsX) + "," + std::to_string(_numPixelsY) + ")");
    }
    return cv::Mat::zeros(_numPixelsX, _numPixelsY, CV_8UC1);
}

void Floor::letterModelToShapes(StandardLetterModel const &model, std::vector<MRA::Datatypes::Shape> &shapes) const
{
    MRA_TRACE_FUNCTION_INPUTS(model);
    // some elements can be omitted from the letter model, but not the main dimensions (A,B) and linewidth K

    // usability: any existing custom shapes without specified linewidth get configured model.K value
    for (auto &existing_shape: shapes)
    {
        if (existing_shape.linewidth() == 0)
        {
            existing_shape.set_linewidth(model.k());
        }
    }

    // all coordinates are in FCS - conversion to pixels (and potential 90degree rotation) happens at to cv::Mat operations
    MRA::Datatypes::Shape s;
    s.set_linewidth(model.k());
    // field outer boundary rectangle
    s.mutable_rectangle()->mutable_size()->set_x(model.b() - model.k());
    s.mutable_rectangle()->mutable_size()->set_y(model.a() - model.k());
    shapes.push_back(s);
    // field middle line
    s.mutable_line()->mutable_from()->set_x(-0.5 * (model.b() - model.k()));
    s.mutable_line()->mutable_to()  ->set_x( 0.5 * (model.b() - model.k()));
    shapes.push_back(s);
    // field middle circle
    if (model.h() > 0.1)
    {
        s.mutable_circle()->set_radius( 0.5 * (model.h() - model.k()));
        shapes.push_back(s);
    }
    // penalty and goal areas
    for (int sign = -1; sign <= 1; sign += 2)
    {
        if (model.c() > 0.0 && model.e() > 0.0)
        {
            s.mutable_rectangle()->mutable_center()->set_x(0.0);
            s.mutable_rectangle()->mutable_center()->set_y(sign * 0.5 * (model.a() - model.e()));
            s.mutable_rectangle()->mutable_size()->set_x(model.c() - model.k());
            s.mutable_rectangle()->mutable_size()->set_y(model.e() - model.k());
            shapes.push_back(s);
        }
        if (model.d() > 0.0 && model.f() > 0.0)
        {
            s.mutable_rectangle()->mutable_center()->set_x(0.0);
            s.mutable_rectangle()->mutable_center()->set_y(sign * 0.5 * (model.a() - model.f()));
            s.mutable_rectangle()->mutable_size()->set_x(model.d() - model.k());
            s.mutable_rectangle()->mutable_size()->set_y(model.f() - model.k());
            shapes.push_back(s);
        }
    }
    // corner circles (arcs)
    // angles on protobuf are in radians, however opencv ellipse() takes degrees
    if (model.g() > 0.0)
    {
        float deg2rad = M_PI / 180.0;
        for (int signX = -1; signX <= 1; signX += 2)
        {
            for (int signY = -1; signY <= 1; signY += 2)
            {
                s.mutable_arc()->mutable_center()->set_x(signX * 0.5 * model.b());
                s.mutable_arc()->mutable_center()->set_y(signY * 0.5 * model.a());
                s.mutable_arc()->mutable_size()->set_x(model.g() - 0.5 * model.k());
                s.mutable_arc()->mutable_size()->set_y(model.g() - 0.5 * model.k());
                // which quadrant?
                float a_quadrant = deg2rad * (signX>0 ? (signY>0 ? 180 : 270) : (signY>0 ? 90 : 0));
                s.mutable_arc()->set_angle(a_quadrant);
                // reduce 90degree arc angles a bit, to prevent painting pixels outside of the field boundary lines
                // (TODO: make this robust for linewidth K)
                s.mutable_arc()->set_startangle(5 * deg2rad);
                s.mutable_arc()->set_endangle(85 * deg2rad);
                shapes.push_back(s);
            }
        }
    }
    // center and penalty spots
    s.mutable_circle()->set_radius(0.0);
    if (model.j() > 0.0)
    {
        s.set_linewidth(2.0 * model.j());
        shapes.push_back(s);
    }
    if (model.i() > 0.0)
    {
        for (int signY = -1; signY <= 1; signY += 2)
        {
            s.mutable_circle()->mutable_center()->set_y(signY * (0.5 * model.a() - model.i()));
            shapes.push_back(s);
        }
    }
#ifdef MRA_LOGGING_ENABLED
    int numShapes = shapes.size();
    MRA_TRACE_FUNCTION_OUTPUT(numShapes);
#endif
}

cv::Point Floor::pointFcsToPixel(MRA::Datatypes::Point const &p) const
{
    // flip x,y so that the image better fits on a typical computer screen
    return cv::Point((p.y() - _originY) * _ppm, (p.x() - _originX) * _ppm);
}

cv::Mat Floor::applyBlur(const cv::Mat &image, float blurFactor, int blurMaxDepth, uchar blurMinValue) const
{
    int nx = image.cols;
    int ny = image.rows;
    MRA_TRACE_FUNCTION_INPUTS(nx, ny, blurFactor);

    // First create a list with all current white inner pixels
    std::vector<cv::Point> whitePixels;
    for (int x = 1; x < nx-1; x++)
    {
        for (int y = 1; y < ny-1; y++)
        {
            if (image.at<uchar>(y, x) > 0)  // Assuming white pixels have non-zero intensity
            {
                whitePixels.push_back(cv::Point(x, y));
            }
        }
    }

    // The list of whitePixels grows with each iteration
    // Iterate until no change
    cv::Mat imageOut = image.clone();
    bool done = false;
    int iteration = 0;
    while (!done and iteration < blurMaxDepth)
    {
        done = (0 == blurSinglePass(imageOut, whitePixels, blurFactor, blurMinValue));
    }
    return imageOut;
}

int Floor::blurSinglePass(cv::Mat &image, std::vector<cv::Point> &whitePixels, float blurFactor, uchar blurMinValue) const
{
    MRA_TRACE_FUNCTION_INPUTS(blurFactor);

    int nx = image.cols;
    int ny = image.rows;
    int numCurrent = whitePixels.size();
    int numAdded = 0;

    // Assign new pixels, count them
    std::vector<cv::Point> whitePixelsCurrent = whitePixels;
    for (const cv::Point& whitePixel : whitePixelsCurrent)
    {
        int x = whitePixel.x;
        int y = whitePixel.y;

        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                int neighborX = x + i;
                int neighborY = y + j;

                if (image.at<uchar>(neighborY, neighborX) == 0)  // Assuming black pixels have intensity 0
                {
                    // Set neighboring black pixel to reduced pixel value
                    uchar newValue = static_cast<uchar>(blurFactor * static_cast<float>(image.at<uchar>(y, x)));
                    if (newValue > blurMinValue)
                    {
                        if (neighborX > 0 and neighborX < nx-1 and neighborY > 0 and neighborY < ny-1)
                        {
                            image.at<uchar>(neighborY, neighborX) = newValue;
                            whitePixels.push_back(cv::Point(neighborX, neighborY));
                            ++numAdded;
                        }
                    }
                }
            }
        }
    }

    // Drop current set of white pixels, these do not need to be reconsidered anymore
    whitePixels.erase(whitePixels.begin(), whitePixels.begin() + numCurrent);

    MRA_TRACE_FUNCTION_OUTPUTS(numCurrent, numAdded);
    return numAdded;
}

void Floor::shapesToCvMat(std::vector<MRA::Datatypes::Shape> const &shapes, cv::Mat &m) const
{
#ifdef MRA_LOGGING_ENABLED
    int numShapes = shapes.size();
    MRA_TRACE_FUNCTION_INPUTS(numShapes);
#endif
    cv::Scalar color(255, 255, 255); // white
    for (auto const &s: shapes)
    {
        if (s.has_line())
        {
            int lw = s.linewidth() * _ppm;
            auto pfrom = pointFcsToPixel(s.line().from());
            auto pto = pointFcsToPixel(s.line().to());
            cv::line(m, pfrom, pto, color, lw);
        }
        else if (s.has_circle())
        {
            int lw = s.linewidth() * _ppm;
            int radius = s.circle().radius() * _ppm;
            auto p = pointFcsToPixel(s.circle().center());
            cv::circle(m, p, radius, color, lw);
        }
        else if (s.has_arc())
        {
            int lw = s.linewidth() * _ppm;
            // angles on protobuf are in radians, however opencv ellipse() takes degrees
            float rad2deg = 180.0 / M_PI;
            float a = s.arc().angle() * rad2deg;
            float as = s.arc().startangle() * rad2deg;
            float ae = s.arc().endangle() * rad2deg;
            auto p = pointFcsToPixel(s.arc().center());
            cv::Size2f sz(s.arc().size().x() * _ppm, s.arc().size().y() * _ppm);
            cv::ellipse(m, p, sz, a, as, ae, color, lw);
        }
        else if (s.has_rectangle())
        {
            int lw = s.linewidth() * _ppm;
            MRA::Geometry::Point pc(s.rectangle().center()); // provides more operators
            MRA::Geometry::Point ps(s.rectangle().size()); // provides more operators
            auto p1 = pointFcsToPixel(pc - ps * 0.5);
            auto p2 = pointFcsToPixel(pc + ps * 0.5);
            cv::rectangle(m, p1, p2, color, lw);
        }
    }
}

void Floor::addGridLines(cv::Mat &m, float step, cv::Scalar color) const
{
    MRA_TRACE_FUNCTION_INPUTS(step);
    int nx = ceil(_sizeX / step);
    int ny = ceil(_sizeY / step);
    float ox = nx * step;
    float oy = ny * step;
    for (int ix = -nx; ix <= nx; ++ix)
    {
        auto pfrom = pointFcsToPixel(MRA::Geometry::Point(ix * step, -oy));
        auto pto = pointFcsToPixel(MRA::Geometry::Point(ix * step, oy));
        cv::line(m, pfrom, pto, color, 1);
    }
    for (int iy = -ny; iy <= ny; ++iy)
    {
        auto pfrom = pointFcsToPixel(MRA::Geometry::Point(-ox, iy * step));
        auto pto = pointFcsToPixel(MRA::Geometry::Point(ox, iy * step));
        cv::line(m, pfrom, pto, color, 1);
    }
}

