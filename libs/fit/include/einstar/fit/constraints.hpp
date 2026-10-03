#pragma once

// Fitting all features of a part together under geometric constraints.
//
// Each feature is a surface with its support (scan points of its region). The solve minimises every
// feature's robust point-to-surface distances while holding the constraints exactly: equality-constrained
// Gauss-Newton, each step solving the KKT system of the linearised constraints, so a perpendicularity or a
// measured diameter holds to machine precision, and the fit to the scan takes up the rest.
//
// Datums are coordinate frames solved with the features. Aligning faces to a datum's axes ("the box faces
// are square to each other") replaces pairwise perpendicular and parallel constraints and gives the
// exported model its origin and orientation.
//
// The report says, per constraint, whether it holds, repeats others (redundant) or contradicts them
// (conflict), and what it costs: how far it moved the surfaces off where the scan alone puts them.

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "einstar/fit/surface.hpp"

namespace einstar::fit {

struct Feature {
    Surface surface;            // starting estimate (e.g. the region's free fit)
    std::vector<Vec3> points;   // support
    std::vector<double> weights;  // optional, per point
    bool fixed = false;         // keep the surface as given
};

struct Datum {
    SE3 frame = SE3::Identity();  // axes are the columns of frame.linear(), origin frame.translation()
    bool fixed = false;
};

// Constraints refer to features and datums by index. "Direction" is a plane's normal or the axis of a
// cylinder, cone or torus; "position" is a point on the plane or the axis.
struct Aligned {  // the feature's direction along datum axis 0, 1 or 2 (either sense)
    int feature, datum, axis;
};
struct Parallel {  // directions parallel
    int a, b;
};
struct Perpendicular {  // directions perpendicular
    int a, b;
};
struct Angle {  // between the directions, degrees in [0, 90]
    int a, b;
    double degrees;
};
struct Coplanar {  // two planes on one plane
    int a, b;
};
struct Coaxial {  // two axes on one line
    int a, b;
};
struct Radius {  // cylinder, sphere, cone (none), torus: the tube radius (a fillet)
    int feature;
    double value;
};
struct Diameter {
    int feature;
    double value;
};
struct Distance {  // between two parallel planes (implies parallel)
    int a, b;
    double value;
};
struct Offset {  // the feature's position along datum axis k, from the datum origin
    int feature, datum, axis;
    double value;
};

struct AxisDistance {  // two parallel axes this far apart (implies parallel): a hole pitch, a bolt circle's radius
    int a, b;
    double value;
};

using Constraint = std::variant<Aligned, Parallel, Perpendicular, Angle, Coplanar, Coaxial, Radius, Diameter, Distance, Offset, AxisDistance>;

[[nodiscard]] std::string describe(const Constraint& c);

struct SolveOptions {
    int max_iterations = 50;
    double huber = 2.0;          // in units of each feature's sigma
    double min_sigma = 0.01;     // mm, floor of each feature's noise level
    std::size_t max_points = 3000;  // per feature (evenly subsampled)
    double tolerance = 1e-12;    // on the constraint violation
    bool analyse_costs = true;   // re-solve without each constraint to measure what it costs
};

enum class ConstraintStatus { satisfied, redundant, conflict, invalid };
[[nodiscard]] std::string_view status_name(ConstraintStatus s);

struct ConstraintReport {
    ConstraintStatus status = ConstraintStatus::satisfied;
    std::string message;
    double violation = 0;     // after the solve (rad or mm)
    // What the constraint costs (when analysed): the change of the involved features' rms, and the
    // largest movement of any of their support points' surface distance, against the solve without it.
    double delta_rms_mm = 0;
    double max_move_mm = 0;
};

struct FeatureReport {
    double rms = 0;     // of the support's distances to the solved surface (inliers within 3 sigma)
    double sigma = 0;   // the feature's noise level (from its starting fit)
    // One standard deviation of the main dimension (radius of cylinders/spheres, tube radius of tori), from
    // the solve's covariance under the constraints; 0 for planes and fixed values.
    double radius_sd = 0;
    double direction_sd_deg = 0;  // of the direction (planes, axes)
    double max_move_mm = 0;  // largest change of a support point's distance against the starting surface
};

struct SolveResult {
    std::vector<Surface> surfaces;
    std::vector<SE3> datums;
    std::vector<ConstraintReport> constraints;
    std::vector<FeatureReport> features;
    bool converged = false;
    int iterations = 0;
};

struct SolveInput {
    std::vector<Feature> features;
    std::vector<Datum> datums;
    std::vector<Constraint> constraints;
};

[[nodiscard]] SolveResult solve(const SolveInput& input, const SolveOptions& options = {});

// A datum frame from two features' directions: axis `first_axis` along a's direction, the next axis as
// close to b's direction as possible (e.g. z from the top face, x from a side), origin at a's position.
[[nodiscard]] SE3 datum_from(const Surface& a, int first_axis, const Surface& b);

// Direction (unit) of a plane, cylinder, cone or torus; nullopt for a sphere.
[[nodiscard]] std::optional<Vec3> direction_of(const Surface& s);
// A point on the plane or axis (a sphere's or torus' centre, a cone's apex).
[[nodiscard]] Vec3 position_of(const Surface& s);

}  // namespace einstar::fit
