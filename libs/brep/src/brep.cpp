#include "einstar/brep/brep.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <tuple>

#include <BOPAlgo_CellsBuilder.hxx>
#include <BOPAlgo_MakerVolume.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_Printer.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <Standard_Failure.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <gp_Ax3.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>

namespace einstar::brep {

struct Shape {
    TopoDS_Shape shape;
};

namespace {

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

// Scan points of a face lying on one piece of it, and one of them (to look at the piece's side there).
struct Support {
    int count = 0;
    Vec3 sample = Vec3::Zero();
};

gp_Pnt to_pnt(const Vec3& v) { return {v.x(), v.y(), v.z()}; }
gp_Dir to_dir(const Vec3& v) { return {v.x(), v.y(), v.z()}; }
Vec3 to_vec(const gp_Pnt& p) { return {p.X(), p.Y(), p.Z()}; }
Vec3 to_vec(const gp_Vec& p) { return {p.X(), p.Y(), p.Z()}; }

// A frame with its z along `axis`, origin at `origin`.
gp_Ax3 frame(const Vec3& origin, const Vec3& axis) {
    const Vec3 x = fit::any_perpendicular(axis);
    return {to_pnt(origin), to_dir(axis), to_dir(x)};
}

// The input surface as a face reaching `reach` past the part's centre in every direction.
TopoDS_Face extended_face(const fit::Surface& s, const Vec3& center, double reach) {
    return std::visit(Overloaded{
                          [&](const fit::Plane& p) -> TopoDS_Face {
                              const gp_Pln pln(frame(fit::project(s, center), p.normal));
                              return BRepBuilderAPI_MakeFace(pln, -reach, reach, -reach, reach).Face();
                          },
                          [&](const fit::Cylinder& c) -> TopoDS_Face {
                              const Vec3 o = c.point + (center - c.point).dot(c.axis) * c.axis;
                              const gp_Cylinder cyl(frame(o, c.axis), c.radius);
                              return BRepBuilderAPI_MakeFace(cyl, 0, 2 * std::numbers::pi, -reach, reach).Face();
                          },
                          [&](const fit::Cone& c) -> TopoDS_Face {
                              const gp_Cone cone(frame(c.apex, c.axis), c.half_angle, 0.0);
                              return BRepBuilderAPI_MakeFace(cone, 0, 2 * std::numbers::pi, 0, 2 * reach / std::cos(c.half_angle)).Face();
                          },
                          [&](const fit::Sphere& sp) -> TopoDS_Face { return BRepBuilderAPI_MakeFace(gp_Sphere(frame(sp.center, Vec3::UnitZ()), sp.radius)).Face(); },
                          [&](const fit::Torus& t) -> TopoDS_Face {
                              return BRepBuilderAPI_MakeFace(gp_Torus(frame(t.center, t.axis), t.major, t.minor)).Face();
                          },
                          [&](const fit::Freeform& f) -> TopoDS_Face {
                              // The same spline: uniform knots, poles at the Greville points (fit::Freeform). Its
                              // extent is the fitted one (the fit already reaches past the data).
                              TColgp_Array2OfPnt poles(1, f.nu, 1, f.nv);
                              for (int i = 0; i < f.nu; ++i)
                                  for (int j = 0; j < f.nv; ++j)
                                      poles(i + 1, j + 1) = to_pnt(f.frame * Vec3(f.u0 + (i - 1) * f.du, f.v0 + (j - 1) * f.dv,
                                                                                (*f.heights)[static_cast<std::size_t>(i * f.nv + j)]));
                              TColStd_Array1OfReal uk(1, f.nu + 4), vk(1, f.nv + 4);
                              TColStd_Array1OfInteger um(1, f.nu + 4), vm(1, f.nv + 4);
                              for (int k = 0; k < f.nu + 4; ++k) {
                                  uk(k + 1) = f.u0 + (k - 3) * f.du;
                                  um(k + 1) = 1;
                              }
                              for (int k = 0; k < f.nv + 4; ++k) {
                                  vk(k + 1) = f.v0 + (k - 3) * f.dv;
                                  vm(k + 1) = 1;
                              }
                              const Handle(Geom_BSplineSurface) surf = new Geom_BSplineSurface(poles, uk, vk, um, vm, 3, 3);
                              return BRepBuilderAPI_MakeFace(surf, 1e-7).Face();
                          },
                      },
                      s);
}

// Unit normal of a face at a point near it, pointing out of the solid the face bounds (its orientation).
std::optional<Vec3> face_normal(const TopoDS_Face& f, const Vec3& p) {
    const Handle(Geom_Surface) surf = BRep_Tool::Surface(f);
    if (surf.IsNull()) return std::nullopt;
    ShapeAnalysis_Surface sas(surf);
    const gp_Pnt2d uv = sas.ValueOfUV(to_pnt(p), 1e-7);
    BRepAdaptor_Surface ad(f, false);
    gp_Pnt pt;
    gp_Vec du, dv;
    ad.D1(uv.X(), uv.Y(), pt, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-12) return std::nullopt;
    n.Normalize();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    return to_vec(n);
}

bool face_contains(const TopoDS_Face& f, const Vec3& p) {
    const Handle(Geom_Surface) surf = BRep_Tool::Surface(f);
    ShapeAnalysis_Surface sas(surf);
    const gp_Pnt2d uv = sas.ValueOfUV(to_pnt(p), 1e-7);
    BRepClass_FaceClassifier fc(f, uv, 1e-6);
    return fc.State() == TopAbs_IN || fc.State() == TopAbs_ON;
}

// A point inside a face: the middle of its parameter range if that is on it, otherwise the first point of
// finer and finer grids over the range (a face with a large opening, like a flange around a body).
std::optional<Vec3> point_on(const TopoDS_Face& f) {
    double u0, u1, v0, v1;
    BRepTools::UVBounds(f, u0, u1, v0, v1);
    BRepAdaptor_Surface ad(f);
    for (const int n : {1, 3, 7, 15, 31}) {
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                const gp_Pnt2d uv(u0 + (i + 0.5) / n * (u1 - u0), v0 + (j + 0.5) / n * (v1 - v0));
                BRepClass_FaceClassifier fc(f, uv, 1e-7);
                if (fc.State() == TopAbs_IN) return to_vec(ad.Value(uv.X(), uv.Y()));
            }
    }
    return std::nullopt;
}

// OpenCASCADE reports every STEP transfer on stdout; only failures are of interest.
void quiet_occt() {
    for (const auto& printer : Message::DefaultMessenger()->Printers()) printer->SetTraceLevel(Message_Fail);
}

int count_of(const TopoDS_Shape& s, TopAbs_ShapeEnum kind) {
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, kind, m);
    return m.Extent();
}

std::string kind_name(GeomAbs_SurfaceType t) {
    if (t == GeomAbs_Plane) return "plane";
    if (t == GeomAbs_Cylinder) return "cylinder";
    if (t == GeomAbs_Cone) return "cone";
    if (t == GeomAbs_Sphere) return "sphere";
    if (t == GeomAbs_Torus) return "torus";
    return "other";
}

// Which input a face of the result lies on: input faces (by distance to their surface), then holes (wall
// or floor), then fillets (within their radius of both faces, on a surface of that radius).
std::string name_face(const TopoDS_Face& f, const BuildInput& in) {
    const auto p = point_on(f);
    if (!p) return {};
    constexpr double kTol = 1e-4;
    for (const auto& fi : in.faces)
        if (std::abs(fit::signed_distance(fi.surface, *p)) < kTol) return fi.name;
    for (const auto& h : in.holes) {
        const Vec3 v = *p - h.entry;
        const double depth = v.dot(h.axis);
        const double rho = (v - depth * h.axis).norm();
        const double r = 0.5 * h.diameter;
        if (h.counterbore_diameter && h.counterbore_depth) {
            const double rc = 0.5 * *h.counterbore_diameter;
            if (std::abs(rho - rc) < kTol && depth < *h.counterbore_depth + kTol) return h.name + " counterbore";
            if (std::abs(depth - *h.counterbore_depth) < kTol && rho > r - kTol && rho < rc + kTol) return h.name + " counterbore floor";
        }
        if (h.countersink_diameter) {
            const double t = std::tan(0.5 * h.countersink_angle_deg.value_or(90) * std::numbers::pi / 180);
            if (std::abs(rho - (0.5 * *h.countersink_diameter - depth * t)) < kTol && rho > r - kTol) return h.name + " countersink";
        }
        if (std::abs(rho - r) < kTol) return h.name;
        if (h.depth && h.point_angle_deg) {
            const double t = std::tan(0.5 * *h.point_angle_deg * std::numbers::pi / 180);
            if (depth > *h.depth - kTol && std::abs(rho - (*h.depth + r / t - depth) * t) < kTol) return h.name + " point";
        } else if (h.depth && std::abs(depth - *h.depth) < kTol && rho < r + kTol) {
            return h.name + " floor";
        }
    }
    BRepAdaptor_Surface ad(f);
    for (const auto& fl : in.fillets) {
        const auto fa = static_cast<std::size_t>(fl.face_a), fb = static_cast<std::size_t>(fl.face_b);
        if (fa >= in.faces.size() || fb >= in.faces.size()) continue;
        const double da = std::abs(fit::signed_distance(in.faces[fa].surface, *p));
        const double db = std::abs(fit::signed_distance(in.faces[fb].surface, *p));
        const bool radius_ok = ad.GetType() != GeomAbs_Cylinder || std::abs(ad.Cylinder().Radius() - fl.radius) < 1e-3;
        if (da <= fl.radius + kTol && db <= fl.radius + kTol && radius_ok) return fl.name;
    }
    return {};
}

// Area of every piece (indexed as piece_index).
std::vector<double> piece_area(const std::vector<std::vector<TopoDS_Face>>& pieces, const TopTools_DataMapOfShapeInteger& index, std::size_t count) {
    std::vector<double> area(count, 0);
    for (const auto& list : pieces)
        for (const auto& pc : list) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(pc, props);
            area[static_cast<std::size_t>(index.Find(pc))] = props.Mass();
        }
    return area;
}

// Pieces of faces the scan did not see but the user gave (e.g. the bottom): bounding the solid there is
// intended, not a guess.
std::vector<bool> given_piece(const BuildInput& in, const std::vector<std::vector<TopoDS_Face>>& pieces, const TopTools_DataMapOfShapeInteger& index,
                              std::size_t count) {
    std::vector<bool> given(count, false);
    for (std::size_t i = 0; i < in.faces.size(); ++i)
        if (in.faces[i].points.empty())
            for (const auto& pc : pieces[i]) given[static_cast<std::size_t>(index.Find(pc))] = true;
    return given;
}

// Minimum s-t cut (Edmonds-Karp) on a small graph; returns the nodes on the source side.
std::vector<bool> min_cut_source_side(std::size_t nodes, std::size_t s, std::size_t t,
                                      const std::vector<std::tuple<std::size_t, std::size_t, double>>& edges) {
    struct Arc {
        std::size_t to, rev;
        double cap;
    };
    std::vector<std::vector<Arc>> g(nodes);
    for (const auto& [a, b, c] : edges) {
        g[a].push_back({b, g[b].size(), c});
        g[b].push_back({a, g[a].size() - 1, 0.0});
    }
    std::vector<std::pair<std::size_t, std::size_t>> parent(nodes);
    for (;;) {
        std::vector<bool> seen(nodes, false);
        std::vector<std::size_t> queue{s};
        seen[s] = true;
        for (std::size_t q = 0; q < queue.size() && !seen[t]; ++q)
            for (std::size_t k = 0; k < g[queue[q]].size(); ++k) {
                const Arc& a = g[queue[q]][k];
                if (a.cap > 1e-12 && !seen[a.to]) {
                    seen[a.to] = true;
                    parent[a.to] = {queue[q], k};
                    queue.push_back(a.to);
                }
            }
        if (!seen[t]) return seen;  // the source side
        double flow = 1e300;
        for (std::size_t v = t; v != s; v = parent[v].first) flow = std::min(flow, g[parent[v].first][parent[v].second].cap);
        for (std::size_t v = t; v != s; v = parent[v].first) {
            Arc& a = g[parent[v].first][parent[v].second];
            a.cap -= flow;
            g[a.to][a.rev].cap += flow;
        }
    }
}

// Material or air per cell: a two-label graph cut. Each cell's vote is the evidence (scan points on its
// boundary, on the side the scan says material is). Making a boundary through a piece the scan did not see
// costs that piece's area at about half the scan's point density: a cell hidden inside the part (the flange
// under the body), which no point can vote for, goes with its neighbours rather than becoming a void, and
// the solid does not grow faces where the scan saw none.
std::vector<bool> choose_material(const std::vector<std::vector<int>>& cell_pieces, const std::vector<long>& votes,
                                  const std::vector<Support>& support, const std::vector<double>& area, const std::vector<bool>& given) {
    const std::size_t nc = cell_pieces.size();
    double points = 0, supported_area = 0;
    for (std::size_t pc = 0; pc < support.size(); ++pc)
        if (support[pc].count > 0) {
            points += support[pc].count;
            supported_area += area[pc];
        }
    const double density = supported_area > 0 ? points / supported_area : 0;
    const double lambda = 0.5 * density;
    std::vector<std::vector<std::size_t>> cells_of(support.size());
    for (std::size_t c = 0; c < nc; ++c)
        for (const int pc : cell_pieces[c]) cells_of[static_cast<std::size_t>(pc)].push_back(c);
    const std::size_t s = nc, t = nc + 1;
    std::vector<std::tuple<std::size_t, std::size_t, double>> edges;
    for (std::size_t c = 0; c < nc; ++c) {
        if (votes[c] > 0) edges.emplace_back(s, c, static_cast<double>(votes[c]));
        if (votes[c] < 0) edges.emplace_back(c, t, static_cast<double>(-votes[c]));
    }
    for (std::size_t pc = 0; pc < support.size(); ++pc) {
        if (given[pc]) continue;
        // The unseen share of the piece (a piece with some support is partly seen).
        const double unseen = std::max(0.0, area[pc] - (density > 0 ? support[pc].count / density : 0.0));
        const double w = lambda * unseen;
        if (w <= 0) continue;
        const auto& cs = cells_of[pc];
        if (cs.size() == 2) {
            edges.emplace_back(cs[0], cs[1], w);
            edges.emplace_back(cs[1], cs[0], w);
        } else if (cs.size() == 1) {
            edges.emplace_back(cs[0], t, w);  // outside the arrangement is air
        }
    }
    const std::vector<bool> side = min_cut_source_side(nc + 2, s, t, edges);
    return {side.begin(), side.begin() + static_cast<std::ptrdiff_t>(nc)};
}

}  // namespace

static BuildResult build_unguarded(const BuildInput& in) {
    BuildResult out;
    if (in.faces.empty()) {
        out.log.push_back("no faces");
        return out;
    }
    Eigen::AlignedBox3d box;
    for (const auto& f : in.faces)
        for (const auto& p : f.points) box.extend(p);
    if (box.isEmpty()) {
        out.log.push_back("no scan points: the part's extent is unknown");
        return out;
    }
    const Vec3 center = box.center();
    const double reach = 0.5 * box.diagonal().norm() + in.margin_mm;

    // 1. The arrangement of the extended faces.
    std::vector<TopoDS_Face> inputs;
    TopTools_ListOfShape args;
    for (const auto& f : in.faces) {
        inputs.push_back(extended_face(f.surface, center, reach));
        args.Append(inputs.back());
    }
    BOPAlgo_MakerVolume mv;
    mv.SetArguments(args);
    mv.SetIntersect(true);
    mv.SetRunParallel(true);
    mv.SetFuzzyValue(1e-5);
    mv.Perform();
    if (mv.HasErrors()) {
        out.log.push_back("the faces could not be intersected into cells");
        return out;
    }
    const TopoDS_Shape cells = mv.Shape();

    // Pieces of each input face, and how many of its scan points each piece holds.
    TopTools_DataMapOfShapeInteger piece_owner;  // piece -> input face
    std::vector<std::vector<TopoDS_Face>> pieces(in.faces.size());
    for (std::size_t i = 0; i < inputs.size(); ++i) {
        const TopTools_ListOfShape& images = mv.Modified(inputs[i]);
        if (images.IsEmpty()) pieces[i].push_back(inputs[i]);
        for (const auto& img : images) pieces[i].push_back(TopoDS::Face(img));
        for (const auto& pc : pieces[i]) piece_owner.Bind(pc, static_cast<int>(i));
    }
    TopTools_DataMapOfShapeInteger piece_index;
    std::vector<Support> support;
    std::vector<int> outward(in.faces.size(), 0);  // material side: +1 along the surface normal, -1 against
    for (std::size_t i = 0; i < in.faces.size(); ++i) {
        const FaceInput& f = in.faces[i];
        double side = 0;
        for (std::size_t k = 0; k < f.points.size() && k < f.normals.size(); ++k) side += fit::normal_at(f.surface, f.points[k]).dot(f.normals[k]);
        outward[i] = side > 0 ? 1 : side < 0 ? -1 : 0;
        for (const auto& pc : pieces[i]) {
            piece_index.Bind(pc, static_cast<int>(support.size()));
            support.push_back({});
        }
        if (outward[i] == 0) continue;
        const std::size_t stride = f.points.size() > in.max_vote_points ? (f.points.size() + in.max_vote_points - 1) / in.max_vote_points : 1;
        for (std::size_t k = 0; k < f.points.size(); k += stride) {
            const Vec3 p = fit::project(f.surface, f.points[k]);
            for (const auto& pc : pieces[i])
                if (face_contains(pc, p)) {
                    Support& s = support[static_cast<std::size_t>(piece_index.Find(pc))];
                    if (s.count++ == 0) s.sample = p;
                    break;
                }
        }
    }

    // 2. Material cells: each supported piece votes with its points, for the cell on its material side.
    std::vector<TopoDS_Shape> cell_shapes;
    std::vector<std::vector<int>> cell_pieces;
    std::vector<bool> is_material;
    std::vector<long> votes;
    for (TopExp_Explorer ex(cells, TopAbs_SOLID); ex.More(); ex.Next()) {
        long vote = 0;
        std::vector<int> pcs;
        for (TopExp_Explorer fx(ex.Current(), TopAbs_FACE); fx.More(); fx.Next()) {
            const TopoDS_Face& face = TopoDS::Face(fx.Current());
            if (!piece_index.IsBound(face)) continue;
            pcs.push_back(piece_index.Find(face));
            const Support& s = support[static_cast<std::size_t>(pcs.back())];
            if (s.count == 0) continue;
            const auto i = static_cast<std::size_t>(piece_owner.Find(face));
            const auto n = face_normal(face, s.sample);
            if (!n) continue;
            const Vec3 material_out = outward[i] * fit::normal_at(in.faces[i].surface, s.sample);
            vote += n->dot(material_out) > 0 ? s.count : -s.count;
        }
        cell_shapes.push_back(ex.Current());
        cell_pieces.push_back(std::move(pcs));
        votes.push_back(vote);
    }
    is_material = choose_material(cell_pieces, votes, support, piece_area(pieces, piece_index, support.size()),
                                  given_piece(in, pieces, piece_index, support.size()));
    TopTools_ListOfShape material;
    for (std::size_t c = 0; c < cell_shapes.size(); ++c)
        if (is_material[c]) material.Append(cell_shapes[c]);
    out.log.push_back(std::format("{} faces split space into {} cells, {} of them material", in.faces.size(), cell_shapes.size(), material.Extent()));

    // How much of the scan ends up on the solid's boundary (pieces with material on exactly one side). Where
    // the scan is open (a bottom never seen) the faces enclose nothing there, and the scanned faces around
    // the opening bound no cell.
    {
        std::vector<int> material_sides(support.size(), 0);
        for (std::size_t c = 0; c < cell_shapes.size(); ++c)
            if (is_material[c])
                for (const int pc : cell_pieces[c]) ++material_sides[static_cast<std::size_t>(pc)];
        long total = 0, bounding = 0;
        for (std::size_t pc = 0; pc < support.size(); ++pc) {
            total += support[pc].count;
            if (material_sides[pc] == 1) bounding += support[pc].count;
        }
        out.scan_coverage = total > 0 ? static_cast<double>(bounding) / static_cast<double>(total) : 0;
        if (out.scan_coverage < 0.9)
            out.log.push_back(std::format("only {:.0f}% of the scanned faces bound the solid: the faces do not enclose the part. "
                                          "Add a face where the scan is open (e.g. the bottom it stood on).",
                                          100 * out.scan_coverage));
    }
    if (material.IsEmpty()) {
        out.log.push_back("no cell is on the material side of the scan");
        return out;
    }

    TopoDS_Shape shape;
    if (material.Extent() == 1) {
        shape = material.First();
    } else {
        BOPAlgo_CellsBuilder cb;
        cb.SetArguments(material);
        cb.SetRunParallel(true);
        cb.Perform();
        if (cb.HasErrors()) {
            out.log.push_back("the material cells could not be joined");
            return out;
        }
        cb.AddAllToResult(1, false);
        cb.RemoveInternalBoundaries();
        shape = cb.Shape();
    }
    {
        ShapeUpgrade_UnifySameDomain unify(shape, true, true, false);
        unify.Build();
        shape = unify.Shape();
    }

    // 3. Fillets on the edges between their two faces.
    if (!in.fillets.empty()) {
        TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
        const auto owner = [&](const TopoDS_Shape& f) -> int {
            const auto p = point_on(TopoDS::Face(f));
            if (!p) return -1;
            for (std::size_t i = 0; i < in.faces.size(); ++i)
                if (std::abs(fit::signed_distance(in.faces[i].surface, *p)) < 1e-4) return static_cast<int>(i);
            return -1;
        };
        std::vector<std::vector<TopoDS_Edge>> fillet_edges(in.fillets.size());
        for (int e = 1; e <= edge_faces.Extent(); ++e) {
            const TopTools_ListOfShape& faces = edge_faces(e);
            if (faces.Extent() != 2) continue;
            const int a = owner(faces.First()), b = owner(faces.Last());
            for (std::size_t k = 0; k < in.fillets.size(); ++k)
                if ((in.fillets[k].face_a == a && in.fillets[k].face_b == b) || (in.fillets[k].face_a == b && in.fillets[k].face_b == a))
                    fillet_edges[k].push_back(TopoDS::Edge(edge_faces.FindKey(e)));
        }
        // All at once with the radii given; then with near-equal radii made equal (fillets meeting at a corner
        // need consistent radii, and measured ones differ slightly); then one at a time, keeping those that work.
        const auto try_all = [&](const std::vector<double>& radii) -> bool {
            try {
                BRepFilletAPI_MakeFillet mk(shape);
                int added = 0;
                for (std::size_t k = 0; k < in.fillets.size(); ++k)
                    for (const auto& e : fillet_edges[k]) {
                        mk.Add(radii[k], e);
                        ++added;
                    }
                if (added == 0) return true;
                mk.Build();
                if (!mk.IsDone() || !BRepCheck_Analyzer(mk.Shape()).IsValid()) return false;
                shape = mk.Shape();
                out.log.push_back(std::format("{} fillets on {} edges", in.fillets.size(), added));
                return true;
            } catch (const Standard_Failure&) {
                return false;
            }
        };
        std::vector<double> radii;
        for (const auto& f : in.fillets) radii.push_back(f.radius);
        bool done = try_all(radii);
        if (!done) {
            std::vector<double> equal = radii;
            std::vector<double> sorted = radii;
            std::ranges::sort(sorted);
            for (std::size_t k = 0; k < equal.size(); ++k) {
                std::vector<double> near;
                for (const double r : sorted)
                    if (std::abs(r - radii[k]) <= 0.1 * radii[k]) near.push_back(r);
                equal[k] = near[near.size() / 2];
            }
            if (equal != radii && try_all(equal)) {
                done = true;
                out.log.push_back("fillets of nearly equal radius were made equal (where they meet, they must agree): set the radii to make this exact");
            }
        }
        if (!done) {
            out.log.push_back("the fillets failed together; trying them one at a time");
            for (std::size_t k = 0; k < in.fillets.size(); ++k) {
                const FilletInput& fl = in.fillets[k];
                try {
                    TopTools_IndexedDataMapOfShapeListOfShape ef;
                    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ef);
                    BRepFilletAPI_MakeFillet one(shape);
                    int n = 0;
                    for (int e = 1; e <= ef.Extent(); ++e) {
                        const TopTools_ListOfShape& faces = ef(e);
                        if (faces.Extent() != 2) continue;
                        const int a = owner(faces.First()), b = owner(faces.Last());
                        if ((fl.face_a == a && fl.face_b == b) || (fl.face_a == b && fl.face_b == a)) {
                            one.Add(fl.radius, TopoDS::Edge(ef.FindKey(e)));
                            ++n;
                        }
                    }
                    if (n == 0) {
                        out.log.push_back(std::format("fillet '{}': its faces share no edge", fl.name));
                        continue;
                    }
                    one.Build();
                    if (one.IsDone() && BRepCheck_Analyzer(one.Shape()).IsValid()) shape = one.Shape();
                    else out.log.push_back(std::format("fillet '{}' (R{:.3f}) could not be made", fl.name, fl.radius));
                } catch (const Standard_Failure& e) {
                    out.log.push_back(std::format("fillet '{}' (R{:.3f}) could not be made: {}", fl.name, fl.radius, e.GetMessageString()));
                }
            }
        }
    }

    // 4. Holes.
    const double through = 2 * reach + 10;
    for (const auto& h : in.holes) {
        constexpr double kLead = 1.0;  // the cutters start outside the entry face
        const Vec3 start = h.entry - kLead * h.axis;
        const double r = 0.5 * h.diameter;
        std::vector<std::pair<TopoDS_Shape, std::string>> tools;
        tools.emplace_back(BRepPrimAPI_MakeCylinder(gp_Ax2(to_pnt(start), to_dir(h.axis)), r, h.depth ? *h.depth + kLead : through).Shape(), "");
        if (h.counterbore_diameter && h.counterbore_depth && *h.counterbore_diameter > h.diameter)
            tools.emplace_back(BRepPrimAPI_MakeCylinder(gp_Ax2(to_pnt(start), to_dir(h.axis)), 0.5 * *h.counterbore_diameter, *h.counterbore_depth + kLead).Shape(),
                               "counterbore");
        if (h.countersink_diameter && *h.countersink_diameter > h.diameter) {
            const double t = std::tan(0.5 * h.countersink_angle_deg.value_or(90) * std::numbers::pi / 180);
            const double r_top = 0.5 * *h.countersink_diameter + kLead * t, r_end = 0.5 * r;  // ends inside the bore
            tools.emplace_back(BRepPrimAPI_MakeCone(gp_Ax2(to_pnt(start), to_dir(h.axis)), r_top, r_end, (r_top - r_end) / t).Shape(), "countersink");
        }
        if (h.depth && h.point_angle_deg) {
            const double t = std::tan(0.5 * *h.point_angle_deg * std::numbers::pi / 180);
            tools.emplace_back(BRepPrimAPI_MakeCone(gp_Ax2(to_pnt(h.entry + *h.depth * h.axis), to_dir(h.axis)), r, 0.0, r / t).Shape(), "point");
        }
        for (const auto& [tool, part] : tools) {
            BRepAlgoAPI_Cut cut(shape, tool);
            cut.SetRunParallel(true);
            cut.Build();
            if (cut.IsDone() && !cut.HasErrors()) shape = cut.Shape();
            else out.log.push_back(std::format("hole '{}'{} could not be cut", h.name, part.empty() ? "" : " " + part));
        }
    }

    // 5. Check and describe.
    out.shape = std::make_shared<Shape>(Shape{shape});
    out.ok = BRepCheck_Analyzer(shape).IsValid();
    if (!out.ok) out.log.push_back("the result is not a valid shape");
    out.solids = count_of(shape, TopAbs_SOLID);
    out.faces = count_of(shape, TopAbs_FACE);
    out.edges = count_of(shape, TopAbs_EDGE);
    out.closed = out.solids > 0;
    if (out.closed) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape, props);
        out.volume = props.Mass();
    }
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) out.face_names.push_back(name_face(TopoDS::Face(fx.Current()), in));
    return out;
}

// OpenCASCADE reports failures by throwing (Standard_Failure); none of them may reach the caller.
BuildResult build(const BuildInput& in) {
    try {
        return build_unguarded(in);
    } catch (const Standard_Failure& e) {
        BuildResult out;
        out.log.push_back(std::format("the geometry kernel failed: {} ({})", e.GetMessageString(), e.DynamicType()->Name()));
        return out;
    }
}

Result<void> write_step(const BuildResult& result, const std::filesystem::path& path, const std::string& part_name) {
    if (!result.shape) return make_error(Errc::invalid_argument, "nothing to export");
    const TopoDS_Shape& shape = result.shape->shape;
    Handle(TDocStd_Document) doc;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", doc);
    const Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    const TDF_Label label = tool->AddShape(shape, false);
    TDataStd_Name::Set(label, TCollection_ExtendedString(part_name.c_str()));
    std::size_t i = 0;
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next(), ++i) {
        if (i >= result.face_names.size() || result.face_names[i].empty()) continue;
        const TDF_Label sub = tool->AddSubShape(label, fx.Current());
        if (!sub.IsNull()) TDataStd_Name::Set(sub, TCollection_ExtendedString(result.face_names[i].c_str()));
    }
    quiet_occt();
    STEPCAFControl_Writer writer;  // (defines the STEP parameters, which are set after it)
    Interface_Static::SetCVal("write.step.schema", "AP242DIS");
    Interface_Static::SetCVal("write.step.unit", "MM");
    Interface_Static::SetIVal("write.stepcaf.subshapes.name", 1);
    writer.SetNameMode(true);
    if (!writer.Transfer(doc, STEPControl_AsIs)) return make_error(Errc::io, "the shape could not be translated to STEP");
    if (writer.Write(path.string().c_str()) != IFSelect_RetDone) return make_error(Errc::io, "could not write " + path.string());
    return {};
}

Tessellation tessellate(const BuildResult& result, double deflection_mm) {
    Tessellation out;
    if (!result.shape) return out;
    const TopoDS_Shape& shape = result.shape->shape;
    BRepMesh_IncrementalMesh mesher(shape, deflection_mm, false, 0.3, true);
    int face_index = 0;
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next(), ++face_index) {
        const TopoDS_Face& face = TopoDS::Face(fx.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        const auto base = static_cast<std::uint32_t>(out.mesh.vertices.size());
        const gp_Trsf T = loc.Transformation();
        for (int n = 1; n <= tri->NbNodes(); ++n) out.mesh.vertices.push_back(to_vec(tri->Node(n).Transformed(T)).cast<float>());
        const bool reversed = face.Orientation() == TopAbs_REVERSED;
        for (int t = 1; t <= tri->NbTriangles(); ++t) {
            int a, b, c;
            tri->Triangle(t).Get(a, b, c);
            if (reversed) std::swap(b, c);
            out.mesh.triangles.push_back({base + static_cast<std::uint32_t>(a - 1), base + static_cast<std::uint32_t>(b - 1), base + static_cast<std::uint32_t>(c - 1)});
            out.triangle_face.push_back(face_index);
        }
    }
    out.mesh.compute_normals();
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    for (int e = 1; e <= edges.Extent(); ++e) {
        const TopoDS_Edge& edge = TopoDS::Edge(edges(e));
        if (BRep_Tool::Degenerated(edge)) continue;
        BRepAdaptor_Curve curve(edge);
        GCPnts_TangentialDeflection pts(curve, 0.1, deflection_mm);
        for (int i = 1; i < pts.NbPoints(); ++i) out.edges.push_back({to_vec(pts.Value(i)).cast<float>(), to_vec(pts.Value(i + 1)).cast<float>()});
    }
    return out;
}

StepSummary read_step(const std::filesystem::path& path) {
    StepSummary s;
    quiet_occt();
    STEPCAFControl_Reader reader;
    Interface_Static::SetIVal("read.stepcaf.subshapes.name", 1);
    reader.SetNameMode(true);
    if (reader.ReadFile(path.string().c_str()) != IFSelect_RetDone) {
        s.error = "could not read " + path.string();
        return s;
    }
    Handle(TDocStd_Document) doc;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", doc);
    if (!reader.Transfer(doc)) {
        s.error = "could not translate the file";
        return s;
    }
    const Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence free;
    tool->GetFreeShapes(free);
    TopoDS_Compound all;
    BRep_Builder builder;
    builder.MakeCompound(all);
    for (int i = 1; i <= free.Length(); ++i) {
        builder.Add(all, XCAFDoc_ShapeTool::GetShape(free.Value(i)));
        TDF_LabelSequence subs;
        XCAFDoc_ShapeTool::GetSubShapes(free.Value(i), subs);
        for (int k = 1; k <= subs.Length(); ++k) {
            Handle(TDataStd_Name) name;
            if (subs.Value(k).FindAttribute(TDataStd_Name::GetID(), name)) {
                const TCollection_AsciiString ascii(name->Get());
                s.face_names.emplace_back(ascii.ToCString());
            }
        }
    }
    s.solids = count_of(all, TopAbs_SOLID);
    s.shells = count_of(all, TopAbs_SHELL);
    s.faces = count_of(all, TopAbs_FACE);
    s.edges = count_of(all, TopAbs_EDGE);
    s.valid = BRepCheck_Analyzer(all).IsValid();
    if (s.solids > 0) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(all, props);
        s.volume = props.Mass();
    }
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(all, TopAbs_FACE, faces);
    for (int i = 1; i <= faces.Extent(); ++i) {
        BRepAdaptor_Surface ad(TopoDS::Face(faces(i)));
        ++s.surface_kinds[kind_name(ad.GetType())];
        if (ad.GetType() == GeomAbs_Cylinder) s.cylinder_radii.push_back(ad.Cylinder().Radius());
    }
    std::ranges::sort(s.cylinder_radii);
    s.cylinder_radii.erase(std::unique(s.cylinder_radii.begin(), s.cylinder_radii.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }),
                           s.cylinder_radii.end());
    s.ok = true;
    return s;
}

}  // namespace einstar::brep
