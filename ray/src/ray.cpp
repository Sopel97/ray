#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include <ray/material/Material.h>
#include <ray/material/MaterialPtrStorage.h>
#include <ray/material/MaterialDatabase.h>
#include <ray/material/Patterns.h>
#include <ray/material/TextureDatabase.h>

#include <ray/math/Angle2.h>
#include <ray/math/Raycast.h>
#include <ray/math/Transform3.h>
#include <ray/math/Vec3.h>

#include <ray/scene/StaticScene.h>
#include <ray/scene/bvh/StaticBvh.h>
#include <ray/scene/bvh/StaticBvhObjectMedianPartitioner.h>
#include <ray/scene/bvh/StaticBvhObjectMeanPartitioner.h>
#include <ray/scene/object/SceneObject.h>
#include <ray/scene/object/SceneObjectBlob.h>

#include <ray/sampler/AdaptiveMultisampler.h>
#include <ray/sampler/JitteredMultisampler.h>
#include <ray/sampler/QuincunxMultisampler.h>
#include <ray/sampler/UniformGridMultisampler.h>
#include <ray/sampler/Sampler.h>

#include <ray/shape/Box3.h>
#include <ray/shape/ClosedTriangleMesh.h>
#include <ray/shape/Capsule.h>
#include <ray/shape/Cylinder.h>
#include <ray/shape/Disc3.h>
#include <ray/shape/OrientedBox3.h>
#include <ray/shape/Triangle3.h>
#include <ray/shape/Plane.h>
#include <ray/shape/Shapes.h>
#include <ray/shape/Sphere.h>

#include <ray/Camera.h>
#include <ray/Image.h>
#include <ray/Raytracer.h>

#include <chrono>
#include <iostream>
#include <string>
#include <random>

using namespace ray;

struct Index3
{
    Index3(int i, int j, int k) :
        i(i),
        j(j),
        k(k)
    {
    }

    int i, j, k;
};

struct RawTriangleMesh
{
    std::vector<Point3f> vertices;
    std::vector<Index3> faces;
};

RawTriangleMesh createRawIcosahedron(const Vec3f& offset, float radius)
{
    RawTriangleMesh mesh;

    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f * radius;

    mesh.vertices.emplace_back(-radius, t, 0.0f);
    mesh.vertices.emplace_back(radius, t, 0.0f);
    mesh.vertices.emplace_back(-radius, -t, 0.0f);
    mesh.vertices.emplace_back(radius, -t, 0.0f);

    mesh.vertices.emplace_back(0.0f, -radius, t);
    mesh.vertices.emplace_back(0.0f, radius, t);
    mesh.vertices.emplace_back(0.0f, -radius, -t);
    mesh.vertices.emplace_back(0.0f, radius, -t);

    mesh.vertices.emplace_back(t, 0.0f, -radius);
    mesh.vertices.emplace_back(t, 0.0f, radius);
    mesh.vertices.emplace_back(-t, 0.0f, -radius);
    mesh.vertices.emplace_back(-t, 0.0f, radius);

    // 5 faces around point 0
    mesh.faces.emplace_back(0, 11, 5);
    mesh.faces.emplace_back(0, 5, 1);
    mesh.faces.emplace_back(0, 1, 7);
    mesh.faces.emplace_back(0, 7, 10);
    mesh.faces.emplace_back(0, 10, 11);

    // 5 adjacent faces
    mesh.faces.emplace_back(1, 5, 9);
    mesh.faces.emplace_back(5, 11, 4);
    mesh.faces.emplace_back(11, 10, 2);
    mesh.faces.emplace_back(10, 7, 6);
    mesh.faces.emplace_back(7, 1, 8);

    // 5 faces around point 3
    mesh.faces.emplace_back(3, 9, 4);
    mesh.faces.emplace_back(3, 4, 2);
    mesh.faces.emplace_back(3, 2, 6);
    mesh.faces.emplace_back(3, 6, 8);
    mesh.faces.emplace_back(3, 8, 9);

    // 5 adjacent faces
    mesh.faces.emplace_back(4, 9, 5);
    mesh.faces.emplace_back(2, 4, 11);
    mesh.faces.emplace_back(6, 2, 10);
    mesh.faces.emplace_back(8, 6, 7);
    mesh.faces.emplace_back(9, 8, 1);

    return mesh;
}

ClosedTriangleMesh createIcosahedron(const Vec3f& offset, float radius, const SurfaceMaterial* surface, const MediumMaterial* medium)
{
    RawTriangleMesh basicMesh = createRawIcosahedron(offset, radius);

    ClosedTriangleMesh mesh(medium);

    for (const Index3& face : basicMesh.faces)
    {
        Vec3f centerOffset = (Vec3f(basicMesh.vertices[face.i]) + Vec3f(basicMesh.vertices[face.j]) + Vec3f(basicMesh.vertices[face.k])) * 0.333333333333f;
        Normal3f normal = centerOffset.normalized();
        mesh.addVertex(ClosedTriangleMeshVertex{ basicMesh.vertices[face.i] + offset, normal, {} });
        mesh.addVertex(ClosedTriangleMeshVertex{ basicMesh.vertices[face.j] + offset, normal, {} });
        mesh.addVertex(ClosedTriangleMeshVertex{ basicMesh.vertices[face.k] + offset, normal, {} });
    }

    for (int i = 0; i < basicMesh.faces.size(); ++i)
    {
        mesh.addFace(3*i, 3*i+1, 3*i+2, surface);
    }

    return mesh;
}

ClosedTriangleMesh createSmoothIcosahedron(const Vec3f& offset, float radius, const SurfaceMaterial* surface, const MediumMaterial* medium)
{
    RawTriangleMesh basicMesh = createRawIcosahedron(offset, radius);

    ClosedTriangleMesh mesh(medium);

    for (const Point3f& vertex : basicMesh.vertices)
    {
        Normal3f normal = Vec3f(vertex).normalized();
        mesh.addVertex(ClosedTriangleMeshVertex{ vertex + offset, normal, {} });
    }

    for (const Index3& face : basicMesh.faces)
    {
        mesh.addFace(face.i, face.j, face.k, surface);
    }

    return mesh;
}

int main()
{
    constexpr int width = 1920;
    constexpr int height = 1080;

    sf::RenderWindow window(sf::VideoMode(width, height), "ray");

    TextureDatabase texDb;
    texDb.emplace<SquarePattern>("square-pattern", ColorRGBf(0.8f, 0.8f, 0.8f), ColorRGBf(0.6f, 0.6f, 0.6f), 0.25f);
    auto& pat = texDb.get("square-pattern");
    texDb.emplace<SquarePattern>("square-pattern2", ColorRGBf(0.8f, 0.8f, 0.8f), ColorRGBf(0.6f, 0.6f, 0.6f), 2.0f);
    auto& pat2 = texDb.get("square-pattern2");

    MaterialDatabase matDb;
    auto& m1s = matDb.emplaceSurface("mat1", ColorRGBf(0.2, 0.2, 0.2), ColorRGBf(0, 0, 0), 0.0f, 0.3f, 0.4f, &pat);
    auto& m11s = matDb.emplaceSurface("mat11", ColorRGBf(0.6, 0.6, 0.6), ColorRGBf(0, 0, 0), 0.0f, 0.1f, 0.7f, &pat2);
    auto& m2s = matDb.emplaceSurface("mat2", ColorRGBf(1.00, 0.32, 0.36), ColorRGBf(0, 0, 0), 0.5f, 0.4f, 0.0f);
    auto& m3s = matDb.emplaceSurface("mat3", ColorRGBf(0.90, 0.76, 0.46), ColorRGBf(0, 0, 0), 0.9f, 0.1f, 0.0f);
    auto& m4s = matDb.emplaceSurface("mat4", ColorRGBf(0.65, 0.77, 0.97), ColorRGBf(0, 0, 0), 0.1f, 0.8f, 0.0f);
    auto& m5s = matDb.emplaceSurface("mat5", ColorRGBf(0.90, 0.90, 0.90), ColorRGBf(0, 0, 0), 0.9f, 0.1f, 0.0f);
    auto& m6s = matDb.emplaceSurface("mat6", ColorRGBf(0.00, 0.00, 0.00), ColorRGBf(3, 3, 3), 0.0f, 0.0f, 0.0f);
    auto& m7s = matDb.emplaceSurface("mat7", ColorRGBf(1, 1, 1), ColorRGBf(0, 0, 0), 0.95f, 0.05f, 0.0f);
    auto& m7as = matDb.emplaceSurface("mat7a", ColorRGBf(1, 1, 1), ColorRGBf(0, 0, 0), 1.0f, 0.0f, 0.0f);
    auto& m8s = matDb.emplaceSurface("mat8", ColorRGBf(0.9f, 0.9f, 0.9f), ColorRGBf(0, 0, 0), 0.0f, 1.0f, 0.0f);
    auto& m9s = matDb.emplaceSurface("mat9", ColorRGBf(1.00, 0.00, 0.00), ColorRGBf(0, 0, 0), 0.33f, 0.0f, 0.0f);
    auto& m10s = matDb.emplaceSurface("mat10", ColorRGBf(0.00, 1.00, 0.00), ColorRGBf(0, 0, 0), 0.33f, 0.0f, 0.0f);

    auto& m1m = matDb.emplaceMedium("mat1", ColorRGBf(0, 0, 0), 1.1f);
    auto& m11m = matDb.emplaceMedium("mat11", ColorRGBf(0, 0, 0), 1.1f);
    auto& m2m = matDb.emplaceMedium("mat2", ColorRGBf(0, 0, 0), 1.1f);
    auto& m3m = matDb.emplaceMedium("mat3", ColorRGBf(0, 0, 0), 1.1f);
    auto& m4m = matDb.emplaceMedium("mat4", ColorRGBf(0, 0, 0), 1.1f);
    auto& m5m = matDb.emplaceMedium("mat5", ColorRGBf(0, 0, 0), 1.1f);
    auto& m6m = matDb.emplaceMedium("mat6", ColorRGBf(0, 0, 0), 1.1f);
    auto& m7m = matDb.emplaceMedium("mat7", ColorRGBf(0.5f, 0.5f, 0.2f), 1.13f);
    auto& m7am = matDb.emplaceMedium("mat7a", ColorRGBf(0.5f, 0.5f, 0.2f), 1.0f);
    auto& m8m = matDb.emplaceMedium("mat8", ColorRGBf(0, 0, 0), 1.13f);
    auto& m9m = matDb.emplaceMedium("mat9", ColorRGBf(0, 0, 0), 1.0f);
    auto& m10m = matDb.emplaceMedium("mat10", ColorRGBf(0, 0, 0), 1.0f);

    //*
    using ShapeT = Sphere;
    std::vector<SceneObject<ShapeT>> spheres;
    std::vector<SceneObject<Plane>> planes;
    std::vector<SceneObject<Box3>> boxes;
    std::vector<SceneObject<Triangle3>> tris;
    std::vector<SceneObject<ClosedTriangleMeshFace>> closedTris;
    std::vector<SceneObject<CsgShape>> csgs;
    std::vector<SceneObject<Disc3>> discs;
    std::vector<SceneObject<Cylinder>> cylinders;
    std::vector<SceneObject<Capsule>> capsules;
    std::vector<SceneObject<OrientedBox3>> obbs;

    //ClosedTriangleMesh mesh = createSmoothIcosahedron(Vec3f(0, 0, -7), 3.5f/2.0f, &m7);
    ClosedTriangleMesh mesh = createIcosahedron(Vec3f(0, 0, -7), 3.5f / 2.0f, &m7s, &m7m);

    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    //planes.emplace_back(SceneObject<Plane>(Plane(Normal3f(0.0, -1.0, 0.0), 4), { &m1 }));
    discs.emplace_back(SceneObject<Disc3>(Disc3(Point3f(0, -4, 0), Normal3f(0.0, -1.0, 0.0), 100), { { &m1s } }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 0, -20), 3.5), { { &m2s }, { &m2m } }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(5.0, -1, -15), 2), { { &m3s }, { &m3m } })); // this sphere looks weird, is it right?
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(5.0, 0, -25), 3), { { &m4s }, { &m4m } }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(-5.5, 0, -15), 3), { { &m5s }, { &m5m } })); // this sphere looks weird, is it right?
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 20, -30), 3), { { &m6s }, { &m6m } }));
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 0, -7), 3.5), { { &m7s }, { &m7m } }));
    for (int i = 0; i < mesh.numFaces(); ++i)
    {
        //closedTris.emplace_back(SceneObject<ClosedTriangleMeshFace>(mesh.face(i), mesh.material(i)));
    }
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 0, -7), 1.5), { &m1 }));
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(-3.0, 0, -7), 1.5), { &m1 }));
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(3.0, 0, -7), 1.5), { &m1 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(-30, 20, -20), 20), { { &m8s }, { &m8m } }));
    //boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, -2, -20), Point3f(20, 8, -10)), { &m11 }));

    /*
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, -2+0.01f, -20), Point3f(20, -1, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, -1 + 0.01f, -20), Point3f(20, 0, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 0+0.01f, -20), Point3f(20, 1, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 1+0.01f, -20), Point3f(20, 2, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 2+0.01f, -20), Point3f(20, 3, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 3+0.01f, -20), Point3f(20, 4, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 4+0.01f, -20), Point3f(20, 5, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 5+0.01f, -20), Point3f(20, 6, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 6+0.01f, -20), Point3f(20, 7, -10)), { &m11 }));
    boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, 7+0.01f, -20), Point3f(20, 8, -10)), { &m11 }));
    */


    /*
    cylinders.emplace_back(SceneObject<Cylinder>(Cylinder(Point3f(10, 9 - 10, -10), Point3f(20, 9 - 10, -10), 1.0f), { { &m7s, &m4s }, { &m7m } }));
    cylinders.emplace_back(SceneObject<Cylinder>(Cylinder(Point3f(10, 9 - 7, -10), Point3f(20, 9 - 7, -13), 1.0f), { { &m7s, &m4s }, { &m7m } }));
    cylinders.emplace_back(SceneObject<Cylinder>(Cylinder(Point3f(10, 9 - 4, -10), Point3f(20, 9 - 4, -16), 1.0f), { { &m7s, &m4s }, { &m7m } }));
    */
    
    capsules.emplace_back(SceneObject<Capsule>(Capsule(Point3f(10, 9 - 10, -10), Point3f(20, 9 - 10, -10), 1.0f), { { &m7s, &m4s }, { &m7m } }));
    capsules.emplace_back(SceneObject<Capsule>(Capsule(Point3f(10, 9 - 7, -10), Point3f(20, 9 - 7, -13), 1.0f), { { &m7s, &m4s }, { &m7m } }));
    capsules.emplace_back(SceneObject<Capsule>(Capsule(Point3f(10, 9 - 4, -10), Point3f(20, 9 - 4, -16), 1.0f), { { &m7s, &m4s }, { &m7m } }));

    /*
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 9 - 10, -10), Point3f(20, 9 - 10, -10), Point3f(10, 9 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 10 - 10, -10), Point3f(20, 10 - 10, -10), Point3f(10, 10 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 11 - 10, -10), Point3f(20, 11 - 10, -10), Point3f(10, 11 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 12 - 10, -10), Point3f(20, 12 - 10, -10), Point3f(10, 12 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 13 - 10, -10), Point3f(20, 13 - 10, -10), Point3f(10, 13 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 14 - 10, -10), Point3f(20, 14 - 10, -10), Point3f(10, 14 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 15 - 10, -10), Point3f(20, 15 - 10, -10), Point3f(10, 15 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 16 - 10, -10), Point3f(20, 16 - 10, -10), Point3f(10, 16 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 17 - 10, -10), Point3f(20, 17 - 10, -10), Point3f(10, 17 - 10, -20)), { &m11 }));
    tris.emplace_back(SceneObject<Triangle3>(Triangle3(Point3f(10, 18 - 10, -10), Point3f(20, 18 - 10, -10), Point3f(10, 18 - 10, -20)), { &m11 }));
    */

    Normal3f n1(-1.0f, 0.5f, 0.0f);
    Normal3f n2(1.0f, 2.0f, 1.0f);
    auto obb = OrientedBox3{
        Point3f(0, 0, -7),
        Vec3f(1, 2, 3),
        OrthonormalBasis3f(n1, n2, Handedness3::Right)
        };
    //obbs.emplace_back(SceneObject<OrientedBox3>(obb, { &m11 }));

    auto sumPart1 = SceneObject<CsgShape>(Sphere(Point3f(-1.5, 0, -7), 3.5), { { &m7s }, { &m7m } });
    auto sumPart2 = SceneObject<CsgShape>(Sphere(Point3f(1.5, 0, -7), 3.5), { { &m7s }, { &m7m } });
    auto subPart3 = SceneObject<CsgShape>(Box3(Point3f(-1.5, -2, -7), Point3f(1.5, 2, -3.5)), { { &m7s }, { &m7m } });
    auto subPart4 = SceneObject<CsgShape>(Box3(Point3f(-0.5, -1, -7), Point3f(0.5, 1, -5)), { { &m11s }, { &m11m } });
    auto mulPart5 = SceneObject<CsgShape>(Sphere(Point3f(0, -7.5, -7), 8.5), { { &m7s }, { &m7m } });
    auto sumPart6 = SceneObject<CsgShape>(Sphere(Point3f(-2, 0, -7), 1.5), { { &m4s }, { &m4m } });
    auto sumPart7 = SceneObject<CsgShape>(Sphere(Point3f(2, 0, -7), 1.5), { { &m4s }, { &m4m } });

    auto subPart8 = SceneObject<CsgShape>(Sphere(Point3f(0, 2.4, -7), 2.5), { { &m7s }, { &m7m } });
    auto subPart9 = SceneObject<CsgShape>(Sphere(Point3f(0, 2.4, -7), 2.4), { { &m7s }, { &m7m } });

    auto subPart10 = SceneObject<CsgShape>(Sphere(Point3f(-1.8, -0.9, -4.1), 0.8), { { &m7s }, { &m7m } });
    auto subPart11 = SceneObject<CsgShape>(Sphere(Point3f(-1.5, 0, -3.5), 0.5), { { &m7s }, { &m7m } });
    auto subPart12 = SceneObject<CsgShape>(Sphere(Point3f(1.8, -0.9, -4.1), 0.8), { { &m7s }, { &m7m } });
    auto subPart13 = SceneObject<CsgShape>(Sphere(Point3f(1.5, 0, -3.5), 0.5), { { &m7s }, { &m7m } });

    auto diffPart14 = SceneObject<CsgShape>(obb, { { &m11s }, { &m11m } });

    //auto diffPart15 = SceneObject<CsgShape>(Cylinder(obb.min(), obb.max(), 2.0f), { &m7, &m4 });
    auto diffPart15 = SceneObject<CsgShape>(Capsule(obb.min(), obb.max(), 2.0f), { { &m7s, &m4s }, { &m7m } });

    
    csgs.emplace_back(
        (
            (
                (
                    (
                        (sumPart1 | sumPart2) 
                        - (subPart3 - subPart4)
                    ) 
                    & mulPart5
                ) 
                | (sumPart6 | sumPart7)
                | (subPart8 - subPart9)
            )
            - (subPart10 | subPart11 | subPart12 | subPart13)
        ) 
         - diffPart14
        // - diffPart15
    );
    
    //auto lensPart1 = SceneObject<CsgShape>(Sphere(Point3f(0, 0, -4 + 3), 3.5), { &m7a });
    //auto lensPart2 = SceneObject<CsgShape>(Sphere(Point3f(0, 0, -4- 3), 3.5), { &m7a });
    //csgs.emplace_back(lensPart1 | lensPart2);

    using ShapesT = Shapes<ShapeT, Plane, Box3, Triangle3, ClosedTriangleMeshFace, CsgShape, Disc3, Cylinder, Capsule, OrientedBox3>;
    using PartitionerType = StaticBvhObjectMeanPartitioner;
    using BvhParamsType = BvhParams<ShapesT, Box3, PackedSceneObjectStorageProvider>;
    RawSceneObjectBlob<ShapesT> shapes(std::move(obbs), std::move(spheres), std::move(planes), std::move(boxes), std::move(tris), std::move(closedTris), std::move(csgs), std::move(discs), std::move(cylinders), std::move(capsules));
    StaticScene<StaticBvh<BvhParamsType, PartitionerType>> scene(shapes, 3);
    //StaticScene<PackedSceneObjectBlob<ShapesT>> scene(shapes);
    //*/

    /*
    std::vector<SceneObject<BoundedSharedAnyShape>> spheres;
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(0.0, 0, -20), 3.5), { &m2 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(5.0, -1, -15), 2), { &m3 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(5.0, 0, -25), 3), { &m4 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(-5.5, 0, -15), 3), { &m5 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(0.0, 20, -30), 3), { &m6 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(0.0, 0, -7), 3.5), { &m7 }));
    spheres.emplace_back(SceneObject<BoundedSharedAnyShape>(Sphere(Point3f(-30, 20, -20), 20), { &m8 }));
    using ShapesT = Shapes<BoundedSharedAnyShape>;
    using PartitionerType = StaticBvhObjectMedianPartitioner;
    using BvhParamsType = BvhParams<ShapesT, Box3, PackedSceneObjectStorageProvider>;
    RawSceneObjectBlob<ShapesT> shapes(std::move(spheres));
    StaticScene<StaticBvh<BvhParamsType, PartitionerType>> scene(shapes);
    //StaticScene<SceneObjectBlob<Shapes<Sphere>>> scene(shapes);
    //StaticBlobScene<Shapes<Sphere>> scene(shapes);
    //*/

    scene.setBackgroundColor(ColorRGBf(0.57f, 0.88f, 0.98f));

    /*
    BlobScene scene;
    scene.setBackgroundColor(ColorRGBf(0.57f, 0.88f, 0.98f));
    Material m1{ ColorRGBf(0.2, 0.2, 0.2), ColorRGBf(0, 0, 0), 0.0f, 1.1f, 0.0f, 0.7f };
    Material m2{ ColorRGBf(1.00, 1, 1), ColorRGBf(0, 0, 0), 0.9f, 1.1f, 0.1f, 0.0f };
    Material m6{ ColorRGBf(0.00, 0.00, 0.00), ColorRGBf(3, 3, 3), 0.0f, 1.1f, 0.0f, 0.0f };
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, -9995, -1000), 10000), { &m1 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, -1, -7), 3.5), { &m2 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(20, 30, 10), 3), { &m6 }));
    */

    /*
    BlobScene scene;
    scene.setBackgroundColor(ColorRGBf(2.0f, 2.0f, 2.0f)*0.2f);
    Material mat{ ColorRGBf(1.0f, 0.0f, 0.0f), ColorRGBf(0.0f, 0.0f, 0.0f), 0.2f, 1.458f, 0.07f, 0.0f };
    Material mat2{ ColorRGBf(0.0f, 1.0f, 0.0f), ColorRGBf(0.0f, 0.0f, 0.0f), 0.2f, 1.458f, 0.07f, 0.0f };
    Material lightMat{ ColorRGBf(0.0f, 0.0f, 0.0f), ColorRGBf(3.0f, 3.0f, 3.0f), 1.0f, 1.0f, 0.0f, 0.0f };
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0, 0, -2), 1.0f), { &mat }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0, 1.0f, -1.2f), 0.5f), { &mat2 }));
    scene.add(SceneObject<Sphere>(Sphere({ 0.0f, 7.0f, 1.0f }, 0.5f), { &lightMat }));
    */

#if !defined(RAY_GATHER_PERF_STATS)
    auto t0 = std::chrono::high_resolution_clock().now();
#endif
    Raytracer raytracer(scene);
    auto camera = Camera({ 0, 0.5f, 0 }, Normal3f(0, 0, -1), Normal3f(0, 1, 0), width, height, Angle2f::degrees(45));
    //auto sampler = UniformGridMultisampler(1);
    //auto sampler = JitteredMultisampler(1, 256, 0.66f);
    //auto sampler = QuincunxMultisampler();
    //auto sampler = AdaptiveMultisampler(0.05f, JitteredMultisampler(3, 256, 0.66f));
    //auto sampler = AdaptiveMultisampler(0.05f, QuincunxMultisampler());
    //auto sampler = AdaptiveMultisampler(0.05f, UniformGridMultisampler(3));
    auto sampler = Sampler{};
    Image img = raytracer.capture(camera, sampler);
    //Image img = raytracer.capture(camera);

#if defined(RAY_GATHER_PERF_STATS)
    perf::gGlobalPerfStats.collect(); // threads are in a pool, may not have ended
    std::cout << perf::gGlobalPerfStats.summary();
#else
    auto t1 = std::chrono::high_resolution_clock().now();
    auto diff = t1 - t0;
    std::cout << std::to_string(static_cast<double>(diff.count()) / 1e9) << "s\n";
#endif
    sf::Image sfImg = img.toSfImage();
    sf::Texture texture;
    texture.loadFromImage(sfImg);
    sf::Sprite sprite;
    sprite.setTexture(texture, true);

    /*
    Camera camera(Image(7, 5), {}, Normal3f(0, 0, -1), Normal3f(0, 1, 0), Angle2f::degrees(45));
    camera.forEachPixelRay([](auto r) {std::cout << r.direction().x << '\n'; });
    */

    /*
    BlobScene s;
    Material m;
    s.add(SceneObject<Sphere>(Sphere{}, { &m }));
    */

    /*
    //expected 2 - sqrt(3)/2 = ~1.133
    const Point3f O(0, 0.5f, 0);
    std::cout << raycast(Ray(O, Normal3f(0, 0, -1)), Sphere(O, 6.0f)).value().point.z;
    */

    for (;;)
    {
        sf::Event e;
        while (window.pollEvent(e));

        window.clear();
        window.draw(sprite);
        window.display();
    }
}