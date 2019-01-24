#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "src/Angle.h"
#include "src/Camera.h"
#include "src/Image.h"
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

    Image img(width, height);
    sf::Image sfImg = img.toSfImage();
    sf::Texture texture;
    texture.loadFromImage(sfImg);
    sf::Sprite sprite;
    sprite.setTexture(texture, true);

    //Camera camera(Image(7, 5), {}, Vec3f(0, 0, -1), Vec3f(0, 1, 0), Angle::degrees(45));
    //camera.forEachPixelRay([](auto r) {std::cout << r.direction().x << '\n'; });

    /*
    Scene s;
    Material m;
    s.add(SceneObject<Sphere>(Sphere{}, { &m }));
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