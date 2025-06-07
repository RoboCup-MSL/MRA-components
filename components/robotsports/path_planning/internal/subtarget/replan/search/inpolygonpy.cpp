#include <vector>
#include <algorithm> // For std::min and std::max
#include <limits>    // For std::numeric_limits

// This is a simple ray-casting algorithm to check if a point is inside a polygon.
// It assumes the polygon is simple (non-self-intersecting).
// It also checks for points on the boundary.
std::vector<bool> inpolygon(const std::vector<double>& x_points, const std::vector<double>& y_points,
                            const std::vector<double>& x_vertices, const std::vector<double>& y_vertices) {
    
    std::vector<bool> results;
    int num_points = x_points.size();
    int num_vertices = x_vertices.size();

    if (num_vertices < 3) {
        // A polygon must have at least 3 vertices
        results.resize(num_points, false);
        return results;
    }

    // Convert vertices to a more convenient format
    std::vector<std::pair<double, double>> polygon_vertices(num_vertices);
    for (int i = 0; i < num_vertices; ++i) {
        polygon_vertices[i] = {x_vertices[i], y_vertices[i]};
    }

    for (int p_idx = 0; p_idx < num_points; ++p_idx) {
        double px = x_points[p_idx];
        double py = y_points[p_idx];
        bool inside = false;

        for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
            double xi = polygon_vertices[i].first;
            double yi = polygon_vertices[i].second;
            double xj = polygon_vertices[j].first;
            double yj = polygon_vertices[j].second;

            // Check if point is on the boundary segment
            // This is a simplified check for collinearity and being within segment bounds.
            // A more robust check would involve cross products.
            double dist_cross = (px - xi) * (yj - yi) - (py - yi) * (xj - xi);
            if (std::abs(dist_cross) < std::numeric_limits<double>::epsilon()) { // Check for collinearity
                if (std::min(xi, xj) <= px && px <= std::max(xi, xj) &&
                    std::min(yi, yj) <= py && py <= std::max(yi, yj)) {
                    inside = true; // Point is on the boundary
                    break;
                }
            }

            // Ray-casting algorithm: check for intersections
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        results.push_back(inside);
    }
    return results;
}