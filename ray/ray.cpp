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

    Scene scene;
    scene.setLightPosition({ 0.0f, 7.0f, 1.0f });
    Material mat{ ColorRGBf(1.0f, 0.0f, 0.0f) };
    Material mat2{ ColorRGBf(0.0f, 1.0f, 0.0f) };
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0, 0, -2), 1.0f), { &mat }));
    scene.add(SceneObject<Sphere>(Sphere(Point3f(0, 1.0f, -1.2f), 0.5f), { &mat2 }));


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
    std::cout << distance(raycast(Ray(O, Normal3f(0, 0, -1)), Sphere(Point3f(0, 0, -2), 1.0f)).value().point, O);
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