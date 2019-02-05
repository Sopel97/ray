#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "src/Angle.h"
#include "src/AnyShape.h"
#include "src/BlobScene.h"
#include "src/Camera.h"
#include "src/Image.h"
#include "src/MaterialDatabase.h"
#include "src/Patterns.h"
#include "src/Raycast.h"
#include "src/Raytracer.h"
#include "src/SceneObject.h"
#include "src/SceneObjectBlob.h"
#include "src/Sphere.h"
#include "src/StaticDeferredStorageScene.h"
#include "src/TextureDatabase.h"
#include "src/Vec3.h"

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
    texDb.emplace<SquarePattern>("square-pattern", ColorRGBf(0.8f, 0.8f, 0.8f), ColorRGBf(0.6f, 0.6f, 0.6f), 30.0f);
    auto& pat = texDb.get("square-pattern");

    MaterialDatabase matDb;
    auto& m1 = matDb.emplace("mat1", ColorRGBf(0.2, 0.2, 0.2), ColorRGBf(0, 0, 0), 0.0f, 1.1f, 0.3f, 0.4f, ColorRGBf(0, 0, 0), &pat);
    auto& m2 = matDb.emplace("mat2", ColorRGBf(1.00, 0.32, 0.36), ColorRGBf(0, 0, 0), 0.5f, 1.1f, 0.4f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m3 = matDb.emplace("mat3", ColorRGBf(0.90, 0.76, 0.46), ColorRGBf(0, 0, 0), 0.9f, 1.1f, 0.1f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m4 = matDb.emplace("mat4", ColorRGBf(0.65, 0.77, 0.97), ColorRGBf(0, 0, 0), 0.1f, 1.1f, 0.8f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m5 = matDb.emplace("mat5", ColorRGBf(0.90, 0.90, 0.90), ColorRGBf(0, 0, 0), 0.9f, 1.1f, 0.1f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m6 = matDb.emplace("mat6", ColorRGBf(0.00, 0.00, 0.00), ColorRGBf(3, 3, 3), 0.0f, 1.1f, 0.0f, 0.0f, ColorRGBf(0, 0, 0));
    auto& m7 = matDb.emplace("mat7", ColorRGBf(1, 1, 1), ColorRGBf(0, 0, 0), 0.95f, 1.13f, 0.05f, 0.0f, ColorRGBf(0.5f, 0.5f, 0.2f));
    auto& m8 = matDb.emplace("mat8", ColorRGBf(0.9f, 0.9f, 0.9f), ColorRGBf(0, 0, 0), 0.0f, 1.13f, 1.0f, 0.0f, ColorRGBf(0, 0, 0));

    /*
    BlobScene<Sphere> scene;
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, 0, -20), 3.5), { &m2 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(5.0, -1, -15), 2), { &m3 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(5.0, 0, -25), 3), { &m4 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(-5.5, 0, -15), 3), { &m5 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, 20, -30), 3), { &m6 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, 0, -7), 3.5), { &m7 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(-30, 20, -20), 20), { &m8 }));
    */

    /*
    std::vector<SceneObject<Sphere>> spheres;
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(0.0, 0, -20), 3.5), { &m2 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(5.0, -1, -15), 2), { &m3 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(5.0, 0, -25), 3), { &m4 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(-5.5, 0, -15), 3), { &m5 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(0.0, 20, -30), 3), { &m6 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(0.0, 0, -7), 3.5), { &m7 }));
    spheres.emplace_back(SceneObject<Sphere>(Sphere(Point3f(-30, 20, -20), 20), { &m8 }));
    StaticDeferredStorageScene<SceneObjectBlob<Sphere>> scene(std::move(spheres));
    */

    BlobScene<AnyShape> scene;
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(0.0, 0, -20), 3.5), { &m2 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(5.0, -1, -15), 2), { &m3 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(5.0, 0, -25), 3), { &m4 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(-5.5, 0, -15), 3), { &m5 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(0.0, 20, -30), 3), { &m6 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(0.0, 0, -7), 3.5), { &m7 }));
    scene.add(SceneObject<AnyShape>(Sphere(Point3f(-30, 20, -20), 20), { &m8 }));


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

    Raytracer raytracer(scene);
    Image img = raytracer.capture(Camera({0, 0.5f, 0}, Normal3f(0, 0, -1), Normal3f(0, 1, 0), width, height, Angle::degrees(45)));
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