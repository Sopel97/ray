#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#if defined(RAY_GATHER_PERF_STATS)
#include <ray/perf/PerformanceStats.h>
#endif

#include <ray/shape/Plane.h>
#include <ray/shape/Shapes.h>
#include <ray/shape/Sphere.h>

#include <ray/material/MaterialDatabase.h>
#include <ray/material/Patterns.h>
#include <ray/material/TextureDatabase.h>

#include <ray/math/Angle.h>
#include <ray/math/Raycast.h>
#include <ray/math/Vec3.h>

#include <ray/scene/StaticScene.h>
#include <ray/scene/bvh/StaticBvh.h>
#include <ray/scene/bvh/StaticBvhObjectMedianPartitioner.h>
#include <ray/scene/object/SceneObject.h>
#include <ray/scene/object/SceneObjectBlob.h>

#include <ray/sampler/AdaptiveMultisampler.h>
#include <ray/sampler/JitteredMultisampler.h>
#include <ray/sampler/QuincunxMultisampler.h>
#include <ray/sampler/UniformGridMultisampler.h>
#include <ray/sampler/Sampler.h>

#include <ray/Camera.h>
#include <ray/Image.h>
#include <ray/Raytracer.h>

#include <chrono>
#include <iostream>
#include <string>
#include <random>

using namespace ray;

int main() 
{
    constexpr int width = 1920;
    constexpr int height = 1080;

    sf::RenderWindow window(sf::VideoMode(width, height), "ray");

    /*
        ColorRGBf surfaceColor;
        ColorRGBf emissionColor;
        float transparency;
        float refractiveIndex;
        float reflectivity;
        float diffuse;
    */

    TextureDatabase texDb;
    texDb.emplace<SquarePattern>("square-pattern", ColorRGBf(0.8f, 0.8f, 0.8f), ColorRGBf(0.6f, 0.6f, 0.6f), 0.25f);
    auto& pat = texDb.get("square-pattern");
    texDb.emplace<SquarePattern>("square-pattern2", ColorRGBf(0.8f, 0.8f, 0.8f), ColorRGBf(0.6f, 0.6f, 0.6f), 2.0f);
    auto& pat2 = texDb.get("square-pattern2");

    MaterialDatabase matDb;
    auto& m1 = matDb.emplace("mat1", ColorRGBf(0.2, 0.2, 0.2), ColorRGBf(0, 0, 0), 0.0f, 1.1f, 0.3f, 0.4f, ColorRGBf(0, 0, 0), &pat);
    auto& m11 = matDb.emplace("mat11", ColorRGBf(0.6, 0.6, 0.6), ColorRGBf(0, 0, 0), 0.0f, 1.1f, 0.1f, 0.7f, ColorRGBf(0, 0, 0), &pat2);
    auto& m2 = matDb.emplace("mat2", ColorRGBf(1.00, 0.32, 0.36), ColorRGBf(0, 0, 0), 0.5f, 1.1f, 0.4f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m3 = matDb.emplace("mat3", ColorRGBf(0.90, 0.76, 0.46), ColorRGBf(0, 0, 0), 0.9f, 1.1f, 0.1f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m4 = matDb.emplace("mat4", ColorRGBf(0.65, 0.77, 0.97), ColorRGBf(0, 0, 0), 0.1f, 1.1f, 0.8f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m5 = matDb.emplace("mat5", ColorRGBf(0.90, 0.90, 0.90), ColorRGBf(0, 0, 0), 0.9f, 1.1f, 0.1f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m6 = matDb.emplace("mat6", ColorRGBf(0.00, 0.00, 0.00), ColorRGBf(3, 3, 3), 0.0f, 1.1f, 0.0f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m7 = matDb.emplace("mat7", ColorRGBf(1, 1, 1), ColorRGBf(0, 0, 0), 0.95f, 1.13f, 0.05f, 0.0f, ColorRGBf(0.5f, 0.5f, 0.2f));
    auto& m8 = matDb.emplace("mat8", ColorRGBf(0.9f, 0.9f, 0.9f), ColorRGBf(0, 0, 0), 0.0f, 1.13f, 1.0f, 0.0f, ColorRGBf(0, 0, 0));

    //*
    using ShapeT = Sphere;
    std::vector<SceneObject<ShapeT>> spheres;
    std::vector<SceneObject<Plane>> planes;
    std::vector<SceneObject<Box3>> boxes;
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    planes.emplace_back(SceneObject<Plane>(Plane(Normal3f(0.0, -1.0, 0.0), 4), { &m1 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 0, -20), 3.5), { &m2 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(5.0, -1, -15), 2), { &m3 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(5.0, 0, -25), 3), { &m4 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(-5.5, 0, -15), 3), { &m5 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 20, -30), 3), { &m6 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 0, -7), 3.5), { &m7 }));
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(0.0, 0, -7), 1.5), { &m1 }));
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(-3.0, 0, -7), 1.5), { &m1 }));
    //spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(3.0, 0, -7), 1.5), { &m1 }));
    spheres.emplace_back(SceneObject<ShapeT>(Sphere(Point3f(-30, 20, -20), 20), { &m8 }));
    //boxes.emplace_back(SceneObject<Box3>(Box3(Point3f(10, -2, -20), Point3f(20, 8, -10)), { &m11 }));
    
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
    
    using ShapesT = Shapes<ShapeT, Plane, Box3>;
    using PartitionerType = StaticBvhObjectMedianPartitioner;
    using BvhParamsType = BvhParams<ShapesT, Box3, PackedSceneObjectStorageProvider>;
    RawSceneObjectBlob<ShapesT> shapes(std::move(spheres), std::move(planes), std::move(boxes));
    //StaticScene<StaticBvh<BvhParamsType, PartitionerType>> scene(shapes);
    StaticScene<PackedSceneObjectBlob<ShapesT>> scene(shapes);
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
    auto camera = Camera({ 0, 0.5f, 0 }, Normal3f(0, 0, -1), Normal3f(0, 1, 0), width, height, Angle::degrees(45));
    //auto sampler = UniformGridMultisampler(1);
    //auto sampler = JitteredMultisampler(1, 256, 0.66f);
    //auto sampler = QuincunxMultisampler();
    //auto sampler = AdaptiveMultisampler(0.05f, JitteredMultisampler(3, 256, 0.66f));
    //auto sampler = AdaptiveMultisampler(0.05f, QuincunxMultisampler());
    auto sampler = AdaptiveMultisampler(0.05f, UniformGridMultisampler(3));
    Image img = raytracer.capture(camera, sampler);
    //Image img = raytracer.capture(camera);

#if defined(RAY_GATHER_PERF_STATS)
    std::cout << perf::gPerfStats.summary();
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
    Camera camera(Image(7, 5), {}, Normal3f(0, 0, -1), Normal3f(0, 1, 0), Angle::degrees(45));
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