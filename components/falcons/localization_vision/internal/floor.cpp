#include "floor.hpp"
#include "geometry.hpp" // from MRA library geometry

using namespace MRA::FalconsLocalizationVision;

Floor::Floor()
{
}

Floor::~Floor()
{
}

void Floor::configure(Params const &config)
{
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
}

cv::Mat Floor::createMat() const
{
    return cv::Mat::zeros(_numPixelsX, _numPixelsY, CV_8UC1);
}

void Floor::letterModelToShapes(StandardLetterModel const &model, std::vector<MRA::Datatypes::Shape> &shapes) const
{
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
}

cv::Point Floor::pointFcsToPixel(MRA::Datatypes::Point const &p) const
{
    // flip x,y so that the image better fits on a typical computer screen
    return cv::Point((p.y() - _originY) * _ppm, (p.x() - _originX) * _ppm);
}

// convert the floor into a blurred floor
// this is done by calculating for each pixel the values of the surrounding pixels
// because the surrounding also have surrounding, the calculation is run iterative
// this calculation is not part of the main calculation loop, so optimization is not
// that important for this function
void Floor::applyBlur(cv::Mat &image, int np) const
{
    for (int ii = 0; ii < np; ii++)
    {
        float blurValue = cos( 2.0f*M_PI*ii/(4*np) ); // use only first 45 degrees of cos curve
        image = blurFloor(image, blurValue);
    }
}

// copy each pixel to a new floor and copy also the 8 Neighbours to the new floor
// the 8 Neighbours should have a lower value then the center pixel
cv::Mat Floor::blurFloor(cv::Mat &imageIn, float blurValue) const
{
    int nx = imageIn.cols;
    int ny = imageIn.rows;
    cv::Mat imageOut = cv::Mat::zeros(ny, nx, CV_8UC1);
    for (int x = 1; x < nx-1; x++)
    {
        for (int y = 1; y < ny-1; y++)
        {
            int pixelVal = imageIn.at<uchar>(y,x);
            if (imageOut.at<uchar>(y  ,x  ) < pixelVal) { imageOut.at<uchar>(y  ,x  ) = pixelVal; } // center pixel
            pixelVal = (int)(blurValue * pixelVal + 0.5f);
            if (imageOut.at<uchar>(y-1,x-1) < pixelVal) { imageOut.at<uchar>(y-1,x-1) = pixelVal; } // bottom left
            if (imageOut.at<uchar>(y-1,x  ) < pixelVal) { imageOut.at<uchar>(y-1,x  ) = pixelVal; } // bottom
            if (imageOut.at<uchar>(y-1,x+1) < pixelVal) { imageOut.at<uchar>(y-1,x+1) = pixelVal; } // bottom right
            if (imageOut.at<uchar>(y  ,x+1) < pixelVal) { imageOut.at<uchar>(y  ,x+1) = pixelVal; } // right
            if (imageOut.at<uchar>(y+1,x+1) < pixelVal) { imageOut.at<uchar>(y+1,x+1) = pixelVal; } // top right
            if (imageOut.at<uchar>(y+1,x  ) < pixelVal) { imageOut.at<uchar>(y+1,x  ) = pixelVal; } // top
            if (imageOut.at<uchar>(y+1,x-1) < pixelVal) { imageOut.at<uchar>(y+1,x-1) = pixelVal; } // top left
            if (imageOut.at<uchar>(y  ,x-1) < pixelVal) { imageOut.at<uchar>(y  ,x-1) = pixelVal; } // left
        }
    }
    return imageOut;
}

void Floor::shapesToCvMat(std::vector<MRA::Datatypes::Shape> const &shapes, float blurFactor, cv::Mat &m) const
{
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
    // apply blur
    int blurKernelSize = (int)_ppm * blurFactor;
    if (blurKernelSize)
    {
        applyBlur(m, blurKernelSize);
    }
}

void Floor::linePointsToCvMat(std::vector<Landmark> const &linePoints, cv::Mat &m, float overruleRadius) const
{
    // for every linepoint, create a circle
    std::vector<MRA::Datatypes::Shape> shapes;
    MRA::Datatypes::Shape s;
    for (auto const &p: linePoints)
    {
        float r = overruleRadius;
        if (overruleRadius == 0)
        {
            r = settings.solver().linepoints().fit().radiusconstant();
            float sf = settings.solver().linepoints().fit().radiusscalefactor();
            float rmin = settings.solver().linepoints().fit().radiusminimum();
            if (sf)
            {
                float distance = sqrt(p.x() * p.x() + p.y() * p.y());
                r += sf * distance;
            }
            r = std::max(rmin, r); // clip, opencv can't handle negative circle radius
        }
        s.mutable_circle()->mutable_center()->set_x(p.x());
        s.mutable_circle()->mutable_center()->set_y(p.y());
        s.mutable_circle()->set_radius(r); // in meters, not pixels (anymore)
        s.set_linewidth(-1); // fill
        shapes.push_back(s);
    }
    // make use of shapesToCvMat
    shapesToCvMat(shapes, 0.0, m);
}

void Floor::addGridLines(cv::Mat &m, float step, cv::Scalar color) const
{
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
