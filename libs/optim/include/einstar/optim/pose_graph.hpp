#pragma once

// Pose graph over scan fragments: relative-pose edges (odometry between consecutive fragments,
// verified loop closures between revisits) and optional marker landmarks shared between fragments
// (fixed when they come from a global-marker map). Solved with Ceres; loop closures that disagree
// with the rest of the graph are pruned and the graph re-solved.

#include <map>
#include <set>
#include <vector>

#include "einstar/core/se3.hpp"

namespace einstar::optim {

// Measured relative pose T_i_j = T_world_i^-1 * T_world_j (maps node j's frame into node i's).
// The error is the right perturbation of T_i_j: exp([w; v]) = T_i_j^-1 * T_world_i^-1 * T_world_j,
// ordered [rotation; translation]; `information` is its inverse covariance in that ordering.
struct PoseEdge {
    int i = -1, j = -1;
    SE3 T_i_j = SE3::Identity();
    Mat6 information = Mat6::Identity();
    bool loop = false;  // loop closures may be pruned; odometry edges are kept
};

struct LandmarkObservation {
    int node = -1;
    int landmark = -1;
    Vec3 p_node;          // landmark in the node's frame (mm)
    double sigma_mm = 0.05;
};

// Unary prior on a node: error exp([w; v]) = T_prior^-1 * T_world_node (right perturbation).
struct PosePrior {
    int node = -1;
    SE3 T_prior = SE3::Identity();
    Mat6 information = Mat6::Identity();
};

struct PoseGraph {
    std::map<int, SE3> nodes;   // T_world_node
    std::vector<PoseEdge> edges;
    std::vector<PosePrior> priors;
    std::map<int, Vec3> landmarks;
    std::set<int> fixed_landmarks;
    std::vector<LandmarkObservation> observations;
};

struct PoseGraphParams {
    int max_iterations = 100;
    // After a solve, loop edges whose whitened error (chi = sqrt(e^T I e)) exceeds this are removed
    // and the graph re-solved, until none do.
    double prune_chi = 5.0;
    int max_prune_rounds = 20;
    int fixed_node = -1;  // gauge (when no fixed landmarks); -1 = lowest node id
};

struct PoseGraphReport {
    double cost_before = 0, cost_after = 0;
    int loop_edges = 0;
    int pruned_edges = 0;
    int solves = 0;
    bool converged = false;
};

PoseGraphReport optimize(PoseGraph& graph, const PoseGraphParams& params = {});

// Error vector of an edge at the current node poses (see PoseEdge).
[[nodiscard]] Vec6 edge_error(const PoseGraph& graph, const PoseEdge& e);

}  // namespace einstar::optim
