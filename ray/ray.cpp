#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "src/Image.h"

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

    for (;;)
    {
        sf::Event e;
        while (window.pollEvent(e));

        window.clear();
        window.draw(sprite);
        window.display();
    }
}