#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

// Reconstruct surface with vertex and normal infromation
void surfaceReconstruct(Eigen::MatrixXd &vertices, Eigen::MatrixXd &normals) {
  // Init point cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
  cloud->points.resize(vertices.rows());
  for (int i = 0; i < vertices.rows(); i++) {
    cloud->points[i].x = vertices(i, 0);
    cloud->points[i].y = vertices(i, 1);
    cloud->points[i].z = vertices(i, 2);
    cloud->points[i].normal_x = normals(i, 0);
    cloud->points[i].normal_y = normals(i, 1);
    cloud->points[i].normal_z = normals(i, 2);
  }

  // Initialize poisson surface reconstruction
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(7);
  poisson.setInputCloud(cloud);

  // Reconstruct surfacd
  pcl::PolygonMesh mesh;
  poisson.performReconstruction(mesh);

  // Extract vertex information
  pcl::PointCloud<pcl::PointXYZ>::Ptr meshVertices(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *meshVertices);
  Eigen::MatrixXd meshV(meshVertices->size(), 3);

  for (size_t i = 0; i < meshVertices->size(); i++) {
    meshV.row(i) << 
      meshVertices->points[i].x,
      meshVertices->points[i].y,
      meshVertices->points[i].z;

    // std::cout
    //   << meshVertices->points[i].x << " "
    //   << meshVertices->points[i].y << " "
    //   << meshVertices->points[i].z << " "
    //   << std::endl;
  }

  // Extract face information
  std::vector<pcl::Vertices> meshFaces = mesh.polygons;
  Eigen::MatrixXi meshF(meshFaces.size(), 3);

  for (size_t i = 0; i < meshFaces.size(); i++) {
    pcl::Vertices face = meshFaces[i];
    assert(face.vertices.size() == 3);
    for (size_t j = 0; j < face.vertices.size(); j++) {
      meshF(i, j) = face.vertices[j];
    }

    // for (size_t j = 0; j < face.vertices.size(); j++) {
    //   std::cout << face.vertices[j] << " ";
    // }
    // std::cout << std::endl;
  }

  // Output results
  std::cout << "\nFinished surface reconstruction" << std::endl;
  std::cout << "Vertex num:\t" << meshVertices->points.size() << std::endl;
  std::cout << "Face num:\t" << mesh.polygons.size() << std::endl;

  // Register mesh
  polyscope::SurfaceMesh *surfaceMesh = polyscope::registerSurfaceMesh("Surface Mesh", meshV, meshF);
  surfaceMesh->setSurfaceColor({ 0.155, 0.186, 0.790 });
  surfaceMesh->setMaterial("normal");
}
