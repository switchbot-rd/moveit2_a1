// Offline tool only; generated collision meshes have no CGAL runtime dependency.
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedral_envelope.h>
#include <CGAL/boost/graph/IO/polygon_mesh_io.h>
#include <CGAL/IO/polygon_soup_io.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Polyhedral_envelope_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace SMS = CGAL::Surface_mesh_simplification;
using Kernel = CGAL::Simple_cartesian<double>;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;

int main(int argc, char** argv)
{
  if (argc != 4) return 2;
  const double tolerance = std::stod(argv[3]);
  if (!std::isfinite(tolerance) || tolerance <= 0.) return 2;
  std::vector<Kernel::Point_3> source_points;
  std::vector<std::vector<std::size_t>> source_faces;
  if (!CGAL::IO::read_polygon_soup(argv[1], source_points, source_faces)) return 1;
  std::vector<std::vector<std::size_t>> surface_faces;
  for (const auto& face : source_faces) {
    if (face.size() != 3) return 1;
    if (!CGAL::collinear(source_points[face[0]], source_points[face[1]], source_points[face[2]]))
      surface_faces.push_back(face);
  }
  Mesh original;
  // CAD STL assemblies can share coincident edges. Repair connectivity and
  // orientation, but validate against the untouched original triangle soup.
  if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(argv[1], original)
      || !CGAL::is_triangle_mesh(original)) {
    std::cerr << "Input cannot be represented as a triangle mesh; retain original CAD\n";
    return 1;
  }
  const auto name = std::filesystem::path(argv[1]).filename().string();
  const bool gripper = std::string(argv[1]).find("woan_picker") != std::string::npos;
  const double pad_x = name == "Link_L3.STL" ? -.0252 : .0252;
  const bool keep_pad = name == "Link_L3.STL" || name == "Link_R3.STL";
  CGAL::Polyhedral_envelope<Kernel> original_envelope(source_points, surface_faces, tolerance);
  // Refine rather than accepting an output whose reverse surface error is too large.
  for (double factor : {1., .8, .4, .2}) {
    Mesh mesh = original;
    auto constrained = mesh.add_property_map<Mesh::Edge_index, bool>("e:fixed", false).first;
    // Moving finger mechanisms retain their sharp boundaries. The rigid
    // external picker housing uses the same bounded reduction as the arm shells.
    if (gripper && name != "base_link.STL")
      CGAL::Polygon_mesh_processing::detect_sharp_edges(mesh, 45., constrained);
    if (keep_pad) {
      for (auto edge : mesh.edges()) {
        auto h = halfedge(edge, mesh);
        const auto a = mesh.point(source(h, mesh)), b = mesh.point(target(h, mesh));
        // These are the inner pad planes used by PickerGeometry. Keep all
        // their edges and vertices, including the contact boundary, unchanged.
        if (std::abs(a.x()-pad_x) <= .0001 || std::abs(b.x()-pad_x) <= .0001)
          constrained[edge] = true;
      }
    }
    using Placement = SMS::Constrained_placement<SMS::LindstromTurk_placement<Mesh>, decltype(constrained)>;
    using Filter = SMS::Polyhedral_envelope_filter<Kernel, SMS::Bounded_normal_change_filter<>>;
    SMS::edge_collapse(mesh, SMS::Edge_count_stop_predicate<Mesh>(0),
        CGAL::parameters::edge_is_constrained_map(constrained)
            .get_cost(SMS::LindstromTurk_cost<Mesh>()).get_placement(Placement(constrained))
            .filter(Filter(tolerance*factor)));
    mesh.collect_garbage();
    if (mesh.number_of_faces() >= original.number_of_faces()) continue;
    if (!CGAL::IO::write_polygon_mesh(argv[2], mesh, CGAL::parameters::use_binary_mode(true))) return 1;
    // Verify the exported float32 STL, not just the double-precision working mesh.
    Mesh exported;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(argv[2], exported)) return 1;
    CGAL::Polyhedral_envelope<Kernel> simplified_envelope(exported, tolerance);
    if (original_envelope(exported) && simplified_envelope(source_points, source_faces)) {
      std::cout << "{\"triangles_before\":" << source_faces.size()
                << ",\"triangles_after\":" << exported.number_of_faces()
                << ",\"max_surface_error_m\":" << tolerance
                << ",\"bidirectional_envelope_verified\":true}" << std::endl;
      return 0;
    }
  }
  std::filesystem::remove(argv[2]);
  std::cerr << "No smaller mesh passed the bidirectional surface envelope\n";
  return 1;
}
