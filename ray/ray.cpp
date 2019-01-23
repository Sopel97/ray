#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

int main() 
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "ray");

    for (;;)
    {
        sf::Event e;
        while (window.pollEvent(e));

        window.clear();
        window.display();
    }
}