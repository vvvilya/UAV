#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <queue>
#include <thread>
#include <mutex>
#include <random>
#include <iostream>
#include <sstream>
#include "UAV.h"

// Улучшенные реализации для GPS и INS
class GPS {
public:
    Eigen::Vector3d get_position() {
        static std::default_random_engine generator;
        static std::normal_distribution<double> distribution(0.0, 1.0);
        return Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
    }

    Eigen::Vector3d get_velocity() {
        static std::default_random_engine generator;
        static std::normal_distribution<double> distribution(0.0, 0.1);
        return Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
    }

    double get_time() {
        static auto start = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - start).count();
    }
};

class INS {
public:
    Eigen::Vector3d get_acceleration() {
        static std::default_random_engine generator;
        static std::normal_distribution<double> distribution(0.0, 0.01);
        return Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
    }

    Eigen::Vector3d get_angular_velocity() {
        static std::default_random_engine generator;
        static std::normal_distribution<double> distribution(0.0, 0.001);
        return Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
    }
};

class Barometer {
public:
    double get_altitude() {
        static std::default_random_engine generator;
        static std::normal_distribution<double> distribution(100.0, 0.1);
        return distribution(generator);
    }
};

class IntegrationSystem {
protected:
    GPS* gps;
    INS* ins;
    Barometer* barometer;
    
    Eigen::VectorXd x; // Вектор состояния: [x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
    Eigen::MatrixXd P; // Ковариационная матрица
    Eigen::MatrixXd Q; // Ковариационная матрица шума процесса
    Eigen::MatrixXd R; // Ковариационная матрица шума измерений

    std::queue<Eigen::Vector3d> flight_path;
    std::mutex flight_path_mutex;
    std::thread flight_path_thread;
    bool is_running;

    std::ofstream log_file;

    // Параметры БПЛА
    double mass; // масса БПЛА
    double drag_coefficient; // коэффициент лобового сопротивления
    double front_area; // площадь поперечного сечения

    // Атмосферная модель
    std::function<double(double)> atmosphere_model;

    virtual Eigen::VectorXd predict_state(const Eigen::VectorXd& state, double dt) = 0;
    virtual Eigen::VectorXd predict_measurement(const Eigen::VectorXd& state) = 0;

    void unscented_kalman_filter(const Eigen::VectorXd& measurement, double dt) {
        int n = x.size();
        double lambda = 3 - n;
        
        // Генерация сигма-точек
        Eigen::MatrixXd X(n, 2*n+1);
        X.col(0) = x;
        Eigen::MatrixXd P_sqrt = P.llt().matrixL();
        for (int i = 0; i < n; ++i) {
            X.col(i+1) = x + std::sqrt(n+lambda) * P_sqrt.col(i);
            X.col(n+i+1) = x - std::sqrt(n+lambda) * P_sqrt.col(i);
        }
        
        // Прогноз
        Eigen::MatrixXd X_pred(n, 2*n+1);
        for (int i = 0; i < 2*n+1; ++i) {
            X_pred.col(i) = predict_state(X.col(i), dt);
        }
        
        Eigen::VectorXd x_pred = X_pred * Eigen::VectorXd::Constant(2*n+1, 1.0/(2*(n+lambda)));
        x_pred += lambda/(n+lambda) * (X_pred.col(0) - x_pred);
        
        Eigen::MatrixXd P_pred = Q;
        for (int i = 0; i < 2*n+1; ++i) {
            Eigen::VectorXd diff = X_pred.col(i) - x_pred;
            P_pred += (i == 0 ? lambda/(n+lambda) : 1.0/(2*(n+lambda))) * diff * diff.transpose();
        }
        
        // Обновление
        Eigen::MatrixXd Z_pred(measurement.size(), 2*n+1);
        for (int i = 0; i < 2*n+1; ++i) {
            Z_pred.col(i) = predict_measurement(X_pred.col(i));
        }
        
        Eigen::VectorXd z_pred = Z_pred * Eigen::VectorXd::Constant(2*n+1, 1.0/(2*(n+lambda)));
        z_pred += lambda/(n+lambda) * (Z_pred.col(0) - z_pred);
        
        Eigen::MatrixXd S = R;
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, measurement.size());
        for (int i = 0; i < 2*n+1; ++i) {
            Eigen::VectorXd diff_z = Z_pred.col(i) - z_pred;
            Eigen::VectorXd diff_x = X_pred.col(i) - x_pred;
            double weight = (i == 0) ? lambda/(n+lambda) : 1.0/(2*(n+lambda));
            S += weight * diff_z * diff_z.transpose();
            C += weight * diff_x * diff_z.transpose();
        }
        
        Eigen::MatrixXd K = C * S.inverse();
        x = x_pred + K * (measurement - z_pred);
        P = P_pred - K * S * K.transpose();
        
        log("UKF update completed");
    }

    void load_flight_path() {
        std::ifstream file("terrain.txt");
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double x, y, z;
            if (iss >> x >> y >> z) {
                std::lock_guard<std::mutex> lock(flight_path_mutex);
                flight_path.push(Eigen::Vector3d(x, y, z));
            }
        }
        log("Flight path loaded from terrain.txt");
    }

    void process_flight_path() {
        while (is_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz update rate
            Eigen::Vector3d next_point;
            {
                std::lock_guard<std::mutex> lock(flight_path_mutex);
                if (!flight_path.empty()) {
                    next_point = flight_path.front();
                    flight_path.pop();
                } else {
                    continue;
                }
            }
            
            // Обновление целевой точки в системе управления БПЛА
            update_target_position(next_point);
            log("Processed next flight path point: " + std::to_string(next_point.x()) + ", " 
                + std::to_string(next_point.y()) + ", " + std::to_string(next_point.z()));
        }
    }

    void update_target_position(const Eigen::Vector3d& target) {
        Eigen::Vector3d current_position = x.segment<3>(0);
        Eigen::Vector3d error = target - current_position;
        
        // Простой П-регулятор для демонстрации
        double Kp = 0.1;
        Eigen::Vector3d control = Kp * error;
        
        // Применение управляющего воздействия к модели БПЛА
        x.segment<3>(3) += control;
        
        log("Updated target position. Error: " + std::to_string(error.norm()));
    }

    void log(const std::string& message) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        log_file << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X") << " - " << message << std::endl;
    }

public:
     Eigen::Vector3d get_gps_position() const {
        return gps->get_position();
    }
    IntegrationSystem(GPS* gps, INS* ins, Barometer* barometer)
        : gps(gps), ins(ins), barometer(barometer), is_running(true) {
        x = Eigen::VectorXd::Zero(15);
        P = Eigen::MatrixXd::Identity(15, 15);
        Q = Eigen::MatrixXd::Identity(15, 15) * 0.01;
        R = Eigen::MatrixXd::Identity(6, 6) * 0.1;

        mass = 10.0; // кг
        drag_coefficient = 0.5;
        front_area = 0.2; // м^2

        atmosphere_model = [](double altitude) {
            const double p0 = 101325; // давление на уровне моря, Па
            const double T0 = 288.15; // температура на уровне моря, К
            const double L = 0.0065; // температурный градиент, К/м
            const double g = 9.80665; // ускорение свободного падения, м/с^2
            const double R = 287.05; // газовая постоянная для воздуха, Дж/(кг·К)
            double pressure = p0 * std::pow((1 - L * altitude / T0), (g / (R * L)));
            return pressure / (R * T0 * std::pow((1 - L * altitude / T0), (g / (R * L) - 1)));
        };

        log_file.open("integration_system_log.txt", std::ios::app);
        log("IntegrationSystem initialized");

        load_flight_path();
        flight_path_thread = std::thread(&IntegrationSystem::process_flight_path, this);
    }

    virtual ~IntegrationSystem() {
        is_running = false;
        if (flight_path_thread.joinable()) {
            flight_path_thread.join();
        }
        log_file.close();
    }

    virtual void update() = 0;

    Eigen::Vector3d get_position() const {
        return x.segment<3>(0);
    }

    Eigen::Vector3d get_velocity() const {
        return x.segment<3>(3);
    }

    Eigen::Vector3d get_acceleration() const {
        return x.segment<3>(6);
    }

    Eigen::Vector3d get_orientation() const {
        return x.segment<3>(9);
    }

    Eigen::Vector3d get_gyro_bias() const {
        return x.segment<3>(12);
    }
};

class LooselyIntegratedSystem : public IntegrationSystem {
private:
    Eigen::VectorXd predict_state(const Eigen::VectorXd& state, double dt) override {
        Eigen::VectorXd new_state = state;
        
        // Обновление позиции и скорости
        new_state.segment<3>(0) += state.segment<3>(3) * dt + 0.5 * state.segment<3>(6) * dt * dt;
        new_state.segment<3>(3) += state.segment<3>(6) * dt;
        
        // Обновление ускорения с учетом гравитации и сопротивления воздуха
        Eigen::Vector3d velocity = state.segment<3>(3);
        Eigen::Vector3d gravity(0, 0, -9.81);
        double altitude = state(2);
        double air_density = atmosphere_model(altitude);
        double velocity_magnitude = velocity.norm();
        Eigen::Vector3d drag = -0.5 * air_density * velocity_magnitude * drag_coefficient * front_area * velocity.normalized() / mass;
        
        new_state.segment<3>(6) = gravity + drag;
        
        // Обновление ориентации (упрощенная модель)
        Eigen::Vector3d angular_velocity = ins->get_angular_velocity() - state.segment<3>(12);
        new_state.segment<3>(9) += angular_velocity * dt;
        
        // Обновление смещения гироскопа (предполагаем, что оно меняется медленно)
        new_state.segment<3>(12) += Eigen::Vector3d::Random() * 0.0001 * dt;
        
        return new_state;
    }

    Eigen::VectorXd predict_measurement(const Eigen::VectorXd& state) override {
        // Измерения: [x, y, z, vx, vy, vz] (GPS)
        Eigen::VectorXd measurement(6);
        measurement << state.segment<3>(0), state.segment<3>(3);
        return measurement;
    }

public:
    LooselyIntegratedSystem(GPS* gps, INS* ins, Barometer* barometer)
        : IntegrationSystem(gps, ins, barometer) {
        log("LooselyIntegratedSystem initialized");
    }

    void update() override {
        // Получение измерений
        Eigen::Vector3d gps_position = gps->get_position();
        Eigen::Vector3d gps_velocity = gps->get_velocity();
        Eigen::Vector3d ins_acceleration = ins->get_acceleration();
        double baro_altitude = barometer->get_altitude();

        Eigen::VectorXd measurement(6);
        measurement << gps_position, gps_velocity;

        // Вычисление dt
        static auto last_update = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_update).count();
        last_update = current_time;

        // Обновление UKF
        unscented_kalman_filter(measurement, dt);

        // Обновление состояния ИНС
        x.segment<3>(6) = ins_acceleration;

        log("LooselyIntegratedSystem state updated: " + state_to_string());
    }

private:
    std::string state_to_string() const {
        std::ostringstream oss;
        oss << "position=[" << x.segment<3>(0).transpose() << "], "
            << "velocity=[" << x.segment<3>(3).transpose() << "], "
            << "acceleration=[" << x.segment<3>(6).transpose() << "], "
            << "orientation=[" << x.segment<3>(9).transpose() << "], "
            << "gyro_bias=[" << x.segment<3>(12).transpose() << "]";
        return oss.str();
    }
};

class TightlyIntegratedSystem : public IntegrationSystem {
private:
    Eigen::VectorXd predict_state(const Eigen::VectorXd& state, double dt) override {
        Eigen::VectorXd new_state = state;
        
        // Обновление позиции и скорости
        new_state.segment<3>(0) += state.segment<3>(3) * dt + 0.5 * state.segment<3>(6) * dt * dt;
        new_state.segment<3>(3) += state.segment<3>(6) * dt;
        
        // Обновление ускорения с учетом гравитации и сопротивления воздуха
        Eigen::Vector3d velocity = state.segment<3>(3);
        Eigen::Vector3d gravity(0, 0, -9.81);
        double altitude = state(2);
        double air_density = atmosphere_model(altitude);
        double velocity_magnitude = velocity.norm();
        Eigen::Vector3d drag = -0.5 * air_density * velocity_magnitude * drag_coefficient * front_area * velocity.normalized() / mass;
        
        new_state.segment<3>(6) = gravity + drag;
        
        // Обновление ориентации (упрощенная модель)
        Eigen::Vector3d angular_velocity = ins->get_angular_velocity() - state.segment<3>(12);
        new_state.segment<3>(9) += angular_velocity * dt;
        
        // Обновление смещения гироскопа (предполагаем, что оно меняется медленно)
        new_state.segment<3>(12) += Eigen::Vector3d::Random() * 0.0001 * dt;
        
        return new_state;
    }

    Eigen::VectorXd predict_measurement(const Eigen::VectorXd& state) override {
        // Измерения: [x, y, z, vx, vy, vz, ax, ay, az] (GPS + INS)
        Eigen::VectorXd measurement(9);
        measurement << state.segment<3>(0), state.segment<3>(3), state.segment<3>(6);
        return measurement;
    }

public:
    TightlyIntegratedSystem(GPS* gps, INS* ins, Barometer* barometer)
        : IntegrationSystem(gps, ins, barometer) {
        // Обновляем размерность R для тесно связанной интеграции
        R = Eigen::MatrixXd::Identity(9, 9) * 0.1;
        log("TightlyIntegratedSystem initialized");
    }

    void update() override {
        // Получение измерений
        Eigen::Vector3d gps_position = gps->get_position();
        Eigen::Vector3d gps_velocity = gps->get_velocity();
        Eigen::Vector3d ins_acceleration = ins->get_acceleration();
        double baro_altitude = barometer->get_altitude();

        Eigen::VectorXd measurement(9);
        measurement << gps_position, gps_velocity, ins_acceleration;

        // Вычисление dt
        static auto last_update = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_update).count();
        last_update = current_time;

        // Обновление UKF
        unscented_kalman_filter(measurement, dt);

        log("TightlyIntegratedSystem state updated: " + state_to_string());
    }

private:
    std::string state_to_string() const {
        std::ostringstream oss;
        oss << "position=[" << x.segment<3>(0).transpose() << "], "
            << "velocity=[" << x.segment<3>(3).transpose() << "], "
            << "acceleration=[" << x.segment<3>(6).transpose() << "], "
            << "orientation=[" << x.segment<3>(9).transpose() << "], "
            << "gyro_bias=[" << x.segment<3>(12).transpose() << "]";
        return oss.str();
    }
};

class AdaptiveIntegrationSystem {
private:
    std::unique_ptr<LooselyIntegratedSystem> loosely_integrated;
    std::unique_ptr<TightlyIntegratedSystem> tightly_integrated;
    bool use_tightly_integrated;
    
    std::ofstream performance_log;

public:
    AdaptiveIntegrationSystem(GPS* gps, INS* ins, Barometer* barometer)
        : loosely_integrated(std::make_unique<LooselyIntegratedSystem>(gps, ins, barometer)),
          tightly_integrated(std::make_unique<TightlyIntegratedSystem>(gps, ins, barometer)),
          use_tightly_integrated(false) {
        performance_log.open("adaptive_integration_performance.csv");
        performance_log << "Time,IntegrationType,PositionError,VelocityError,AccelerationError" << std::endl;
    }

    ~AdaptiveIntegrationSystem() {
        performance_log.close();
    }

    void update() {
        static auto start_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();

        loosely_integrated->update();
        tightly_integrated->update();

        // Оценка ошибок
        Eigen::Vector3d true_position = loosely_integrated->get_gps_position(); // Предполагаем, что GPS дает "истинное" положение
        Eigen::Vector3d loosely_position_error = (loosely_integrated->get_position() - true_position).cwiseAbs();
        Eigen::Vector3d tightly_position_error = (tightly_integrated->get_position() - true_position).cwiseAbs();

        // Выбор предпочтительной модели
        if (tightly_position_error.norm() < loosely_position_error.norm()) {
            use_tightly_integrated = true;
        } else {
            use_tightly_integrated = false;
        }

        // Логирование производительности
        performance_log << elapsed_time << ","
                        << (use_tightly_integrated ? "Tightly" : "Loosely") << ","
                        << (use_tightly_integrated ? tightly_position_error : loosely_position_error).norm() << ","
                        << (use_tightly_integrated ? tightly_integrated->get_velocity() : loosely_integrated->get_velocity()).norm() << ","
                        << (use_tightly_integrated ? tightly_integrated->get_acceleration() : loosely_integrated->get_acceleration()).norm()
                        << std::endl;

        std::cout << "Using " << (use_tightly_integrated ? "tightly" : "loosely") << " integrated system at time " << elapsed_time << "s" << std::endl;
    }

    Eigen::Vector3d get_position() const {
        return use_tightly_integrated ? tightly_integrated->get_position() : loosely_integrated->get_position();
    }

    Eigen::Vector3d get_velocity() const {
        return use_tightly_integrated ? tightly_integrated->get_velocity() : loosely_integrated->get_velocity();
    }

    Eigen::Vector3d get_acceleration() const {
        return use_tightly_integrated ? tightly_integrated->get_acceleration() : loosely_integrated->get_acceleration();
    }

    Eigen::Vector3d get_orientation() const {
        return use_tightly_integrated ? tightly_integrated->get_orientation() : loosely_integrated->get_orientation();
    }
};

int main() {
   try {
        // Инициализация датчиков
        GPS gps;
        INS ins;
        Barometer barometer;
        
        // Создание системы интеграции
        AdaptiveIntegrationSystem ais(&gps, &ins, &barometer);
        
        // Создание визуализации
        UAV visualization("Визуализация полета БПЛА");
        
        // Загрузка маршрута полета
        visualization.loadFlightPath("terrain.txt");
        
        // Файл для записи телеметрии
        std::ofstream telemetry_file("telemetry.csv");
        telemetry_file << "Time,X,Y,Z,Vx,Vy,Vz,Ax,Ay,Az" << std::endl;
        
        // Время начала симуляции
        auto start_time = std::chrono::steady_clock::now();
        
        // Основной цикл симуляции
        while (visualization.isRunning()) {
            // Обработка событий SFML
            visualization.handleEvents();
            
            // Обновление состояния системы
            ais.update();
            
            // Получение текущего состояния
            Eigen::Vector3d position = ais.get_position();
            Eigen::Vector3d velocity = ais.get_velocity();
            Eigen::Vector3d acceleration = ais.get_acceleration();
            
            // Обновление визуализации
            visualization.update(position, velocity, position[2]);
            
            // Отрисовка
            visualization.render();
            
            // Запись телеметрии
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(
                current_time - start_time).count();
            
            telemetry_file << std::fixed << std::setprecision(3)
                          << elapsed_seconds << ","
                          << position.x() << ","
                          << position.y() << ","
                          << position.z() << ","
                          << velocity.x() << ","
                          << velocity.y() << ","
                          << velocity.z() << ","
                          << acceleration.x() << ","
                          << acceleration.y() << ","
                          << acceleration.z() << std::endl;
            
            // Задержка для стабильной частоты обновления
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Закрытие файла телеметрии
        telemetry_file.close();
        
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
}