#include "poisson.hpp"

#include <PreProcessor.h>
#include <Reconstructors.h>

#include <exception>
#include <mutex>

namespace einstar::recon::detail {
namespace {

using namespace PoissonRecon;

struct Samples : Reconstructor::InputOrientedSampleStream<float, 3> {
    const PoissonInput& in;
    std::size_t next = 0;
    explicit Samples(const PoissonInput& i) : in(i) {}
    void reset() override { next = 0; }
    bool read(Point<float, 3>& p, Point<float, 3>& n) override {
        if (next >= in.count) return false;
        for (unsigned d = 0; d < 3; ++d) p[d] = in.points[3 * next + d], n[d] = in.normals[3 * next + d];
        ++next;
        return true;
    }
};

struct Vertices : Reconstructor::OutputLevelSetVertexStream<float, 3> {
    std::vector<float>& out;
    explicit Vertices(std::vector<float>& o) : out(o) {}
    size_t size() const override { return out.size() / 3; }
    size_t write(const Point<float, 3>& p, const Point<float, 3>&, const float&) override {
        for (unsigned d = 0; d < 3; ++d) out.push_back(p[d]);
        return out.size() / 3 - 1;
    }
};

struct Faces : Reconstructor::OutputFaceStream<2> {
    std::vector<std::uint32_t>& out;
    explicit Faces(std::vector<std::uint32_t>& o) : out(o) {}
    size_t size() const override { return out.size() / 3; }
    size_t write(const std::vector<node_index_type>& polygon) override {
        // Triangles (polygonMesh is off); anything larger is fanned.
        for (std::size_t k = 1; k + 1 < polygon.size(); ++k)
            out.insert(out.end(), {static_cast<std::uint32_t>(polygon[0]), static_cast<std::uint32_t>(polygon[k]),
                                   static_cast<std::uint32_t>(polygon[k + 1])});
        return out.size() / 3 - 1;
    }
};

std::mutex poisson_mutex;  // PoissonRecon keeps global state (thread pool settings, scratch)

}  // namespace

PoissonOutput screened_poisson(const PoissonInput& in) {
    PoissonOutput out;
    if (in.count == 0) return out;
    std::lock_guard lock(poisson_mutex);
    try {
        using Poisson = Reconstructor::Poisson;
        static constexpr unsigned int kSig = FEMDegreeAndBType<Poisson::DefaultFEMDegree, Poisson::DefaultFEMBoundary>::Signature;
        using Sigs = IsotropicUIntPack<3, kSig>;
        Poisson::SolutionParameters<float> sp;
        sp.width = static_cast<float>(in.cell_mm);  // depth from the finest cell
        sp.depth = 0;
        sp.scale = 2.0f;  // room for the surface it closes over open borders (bulges past the samples)
        sp.pointWeight = static_cast<float>(in.point_weight);
        sp.samplesPerNode = static_cast<float>(in.samples_per_node);
        // Its multi-threaded level-set extraction fails to close loops at times, and threads did not
        // make the solve faster: single-threaded throughout.
        ThreadPool::ParallelizationType = ThreadPool::ParallelType::NONE;
        Samples samples(in);
        auto* implicit = Poisson::Solver<float, 3, Sigs>::Solve(samples, sp);
        Vertices vs(out.vertices);
        Faces fs(out.triangles);
        Reconstructor::LevelSetExtractionParameters ep;
        implicit->extractLevelSet(vs, fs, ep);
        delete implicit;
    } catch (const std::exception& e) {
        out = {};
        out.error = e.what();
    }
    return out;
}

}  // namespace einstar::recon::detail
