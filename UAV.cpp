#include "UAV.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

UAV::UAV(const std::string& window_title) 
    : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), window_title),
      isPaused(false),
      currentPathIndex(0)
{
    initializeWindow();
    initializeText();
    initializeMarkers();
}

UAV::~UAV() {
    window.close();
}

void UAV::initializeWindow() {
    window.setFramerateLimit(60);
    mainView = window.getDefaultView();
    
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Error loading font" << std::endl;
    }
}

void UAV::initializeText() {
    // Инициализация текстовых элементов
    altitudeText.setFont(font);
    velocityText.setFont(font);
    statusText.setFont(font);

    altitudeText.setCharacterSize(20);
    velocityText.setCharacterSize(20);
    statusText.setCharacterSize(20);

    altitudeText.setPosition(10, 10);
    velocityText.setPosition(10, 40);
    statusText.setPosition(10, 70);

    altitudeText.setFillColor(sf::Color::White);
    velocityText.setFillColor(sf::Color::White);
    statusText.setFillColor(sf::Color::White);
}

void UAV::initializeMarkers() {
    // Инициализация маркера БПЛА
    uavMarker.setRadius(5.0f);
    uavMarker.setFillColor(sf::Color::Red);
    uavMarker.setOrigin(5.0f, 5.0f);
}

void UAV::loadFlightPath(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    
    flightPath.clear();
    pathPoints.clear();
    pathLines.clear();

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double x, y, z;
        if (iss >> x >> y >> z) {
            flightPath.push_back(Eigen::Vector3d(x, y, z));
        }
    }

    createPathVisuals();
}

void UAV::createPathVisuals() {
    // Создание точек маршрута
    for (const auto& point : flightPath) {
        sf::CircleShape pathPoint(3.0f);
        pathPoint.setFillColor(sf::Color::Yellow);
        pathPoint.setPosition(worldToScreen(point));
        pathPoint.setOrigin(3.0f, 3.0f);
        pathPoints.push_back(pathPoint);
    }

    // Создание линий маршрута
    for (size_t i = 0; i < flightPath.size() - 1; ++i) {
        sf::Vector2f start = worldToScreen(flightPath[i]);
        sf::Vector2f end = worldToScreen(flightPath[i + 1]);
        
        sf::RectangleShape line;
        float length = std::sqrt(
            std::pow(end.x - start.x, 2) + 
            std::pow(end.y - start.y, 2)
        );
        float angle = std::atan2(end.y - start.y, end.x - start.x);

        line.setSize(sf::Vector2f(length, 2.0f));
        line.setPosition(start);
        line.setRotation(angle * 180 / M_PI);
        line.setFillColor(sf::Color(255, 255, 0, 128));
        
        pathLines.push_back(line);
    }
}

void UAV::update(const Eigen::Vector3d& position, 
                 const Eigen::Vector3d& velocity,
                 double altitude) {
    if (!isPaused) {
        // Обновление позиции маркера БПЛА
        sf::Vector2f screenPos = worldToScreen(position);
        uavMarker.setPosition(screenPos);

        // Обновление текстовой информации
        updateTexts(velocity, altitude);
    }
}

void UAV::updateTexts(const Eigen::Vector3d& velocity, double altitude) {
    float speed = velocity.norm();
    
    std::stringstream alt, vel, status;
    alt << "Высота: " << std::fixed << std::setprecision(1) << altitude << " м";
    vel << "Скорость: " << std::fixed << std::setprecision(1) << speed << " м/с";
    status << "Статус: " << (isPaused ? "Пауза" : "Полет");

    altitudeText.setString(alt.str());
    velocityText.setString(vel.str());
    statusText.setString(status.str());
}

void UAV::render() {
    window.clear(sf::Color(30, 30, 30));

    drawTerrain();
    drawPath();
    drawUAV();
    drawInterface();

    window.display();
}

void UAV::drawTerrain() {
    // Отрисовка сетки координат
    for (int i = 0; i < 20; ++i) {
        sf::RectangleShape horizontalLine(sf::Vector2f(WINDOW_WIDTH, 1.0f));
        sf::RectangleShape verticalLine(sf::Vector2f(1.0f, WINDOW_HEIGHT));
        
        horizontalLine.setPosition(0, i * WINDOW_HEIGHT / 20);
        verticalLine.setPosition(i * WINDOW_WIDTH / 20, 0);
        
        horizontalLine.setFillColor(sf::Color(100, 100, 100));
        verticalLine.setFillColor(sf::Color(100, 100, 100));
        
        window.draw(horizontalLine);
        window.draw(verticalLine);
    }
}

void UAV::drawPath() {
    // Отрисовка линий маршрута
    for (const auto& line : pathLines) {
        window.draw(line);
    }
    
    // Отрисовка точек маршрута
    for (const auto& point : pathPoints) {
        window.draw(point);
    }
}

void UAV::drawUAV() {
    window.draw(uavMarker);
}

void UAV::drawInterface() {
    window.draw(altitudeText);
    window.draw(velocityText);
    window.draw(statusText);
}

void UAV::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
        else if (event.type == sf::Event::KeyPressed) {
            if (event.key.code == sf::Keyboard::Space) {
                togglePause();
            }
        }
    }
}

void UAV::togglePause() {
    isPaused = !isPaused;
}

bool UAV::isRunning() const {
    return window.isOpen();
}

sf::Vector2f UAV::worldToScreen(const Eigen::Vector3d& worldPos) const {
    return sf::Vector2f(
        worldPos.x() * SCALE_FACTOR + WINDOW_WIDTH / 2,
        WINDOW_HEIGHT - (worldPos.y() * SCALE_FACTOR + WINDOW_HEIGHT / 2)
    );
}