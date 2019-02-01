#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "src/Angle.h"
#include "src/Camera.h"
#include "src/Image.h"
#include "src/Raycast.h"
#include "src/Raytracer.h"
#include "src/Scene.h"
#include "src/SceneObject.h"
#include "src/Sphere.h"
#include "src/Vec3.h"

using namespace ray;

int main() 
{
    constexpr int width = 800;
    constexpr int height = 600;

    sf::RenderWindow window(sf::VideoMode(width, height), "ray");

    /*
        ColorRGBf surfaceColor;
        ColorRGBf emissionColor;
        float opacity;
        float refractiveIndex;
        float reflectivity;
        float diffuse;
    */
    Scene scene;
    scene.setBackgroundColor(ColorRGBf(2.0f, 2.0f, 2.0f));
    Material m1{ColorRGBf(0.2, 0.2, 0.2), ColorRGBf(0, 0, 0), 1.0f, 1.1f, 0.0f, 1.0f};
    Material m2{ ColorRGBf(1.00, 0.32, 0.36), ColorRGBf(0, 0, 0), 0.5, 1.1f, 0.5f, 0.0f };
    Material m3{ ColorRGBf(0.90, 0.76, 0.46), ColorRGBf(0, 0, 0), 1.0f, 1.1f, 1.0f, 0.0f };
    Material m4{ ColorRGBf(0.65, 0.77, 0.97), ColorRGBf(0, 0, 0), 1.0f, 1.1f, 1.0f, 0.0f };
    Material m5{ ColorRGBf(0.90, 0.90, 0.90), ColorRGBf(0, 0, 0), 1.0f, 1.1f, 1.0f, 0.0f };
    Material m6{ ColorRGBf(0.00, 0.00, 0.00), ColorRGBf(3, 3, 3), 1.0f, 1.1f, 1.0f, 0.0f };
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, -10004, -20), 10000), { &m1 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, 0, -20), 4), { &m2 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(5.0, -1, -15), 2), { &m3 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(5.0, 0, -25), 3), { &m4 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(-5.5, 0, -15), 3), { &m5 }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0.0, 20, -30), 3), { &m6 }));

    /*
    Scene scene;
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
    Scene s;
    Material m;
    s.add(SceneObject<Sphere>(Sphere{}, { &m }));
    */

    /*
    //expected 2 - sqrt(3)/2 = ~1.133
    const Point3f O(0, 0.5f, 0);
    std::cout << raycastOutside(Ray(O, Normal3f(0, 0, -1)), Sphere(O, 6.0f)).value().point.z;
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