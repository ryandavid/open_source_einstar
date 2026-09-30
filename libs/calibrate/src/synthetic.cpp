#include "einstar/calibrate/synthetic.hpp"

#include "einstar/synth/demo.hpp"
#include "einstar/synth/speckle_scene.hpp"

namespace einstar::calibrate {

std::pair<ImageU8, ImageU8> render_board_pair(const RigCalibration& rig, const SE3& T_left_board, const BoardSpec& board,
                                              const SyntheticBoardParams& params) {
    // World = board coordinates; the board's dots face -z (towards the cameras), its body lies behind.
    synth::Scene scene;
    synth::Box body;
    body.T_world_box.translation() = board.centre() + Vec3(0, 0, 1.5);
    body.half_extent = Vec3(0.5 * (board.cols + 1) * board.pitch_mm, 0.5 * (board.rows + 1) * board.pitch_mm, 1.5);
    scene.primitives.emplace_back(body);
    auto dot = [&](const Vec2& g, bool large) {
        synth::Marker m;
        m.center = board.point(g);
        m.normal = -Vec3::UnitZ();
        m.diameter = large ? 9.0 : 5.0;
        m.ring_diameter = large ? 14.0 : 9.0;
        scene.markers.push_back(m);
    };
    for (int gy = 0; gy < board.rows; ++gy)
        for (int gx = 0; gx < board.cols; ++gx) {
            const Vec2 g(gx, gy);
            dot(g, std::ranges::find(board.large, g) != board.large.end());
        }
    dot(board.large[3], true);  // the lone one, off the grid

    synth::Projector light = synth::speckle_projector();  // off: the ring light is the ambient term
    light.power = 0;
    synth::RenderParams rp;
    rp.ambient = params.board_grey;
    rp.noise_sigma = params.noise_sigma;
    rp.supersample = params.supersample;
    rp.seed = params.seed;
    const SE3 T_world_left = T_left_board.inverse();
    const SE3 T_world_right = T_world_left * rig.T_right_left.inverse();
    auto left = synth::render_view(scene, light, rig.left, T_world_left, rp).image;
    rp.seed = params.seed * 2654435761u + 1;
    auto right = synth::render_view(scene, light, rig.right, T_world_right, rp).image;
    return {std::move(left), std::move(right)};
}

}  // namespace einstar::calibrate
