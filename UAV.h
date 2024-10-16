#pragma once

#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

class UAV {
public:
    UAV(const std::string& window_title = "Визуализация полета БПЛА");
    ~UAV();

    // Основные методы обновления и отрисовки
    void update(const Eigen::Vector3d& position, 
                const Eigen::Vector3d& velocity,
                double altitude);
    void render();
    bool isRunning() const;

    // Загрузка маршрута
    void loadFlightPath(const std::string& filename);

    // Управление визуализацией
    void togglePause();
    void handleEvents();

private:
    // Окно и отображение
    sf::RenderWindow window;
    sf::View mainView;
    bool isPaused;

    // Шрифт и текст
    sf::Font font;
    sf::Text altitudeText;
    sf::Text velocityText;
    sf::Text statusText;

    // Элементы визуализации
    sf::CircleShape uavMarker;
    std::vector<sf::CircleShape> pathPoints;
    std::vector<sf::RectangleShape> pathLines;
    std::vector<sf::RectangleShape> terrain;

    // Данные о маршруте
    std::vector<Eigen::Vector3d> flightPath;
    size_t currentPathIndex;

    // Вспомогательные методы
    void initializeWindow();
    void initializeText();
    void initializeMarkers();
    void createPathVisuals();
    void updateTexts(const Eigen::Vector3d& velocity, double altitude);
    void drawTerrain();
    void drawPath();
    void drawUAV();
    void drawInterface();

    // Преобразование координат
    sf::Vector2f worldToScreen(const Eigen::Vector3d& worldPos) const;
    static constexpr float SCALE_FACTOR = 10.0f;  // Масштабный коэффициент для отображения
    static constexpr float WINDOW_WIDTH = 1200.0f;
    static constexpr float WINDOW_HEIGHT = 800.0f;
};