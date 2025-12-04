#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <queue>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"

using namespace std::chrono_literals;

using RobotSensors = cg_interfaces::msg::RobotSensors;
using MoveCmd      = cg_interfaces::srv::MoveCmd;
using Reset        = cg_interfaces::srv::Reset;

// Codificação do grid interno
constexpr int UNKNOWN = -1;
constexpr int FREE    = 0;
constexpr int WALL    = 1;
constexpr int TARGET  = 2;

class LabyrinthMapper : public rclcpp::Node
{
public:
    LabyrinthMapper()
    : Node("cg_part2_sensor"),
      grid_(GRID_SIZE, std::vector<int>(GRID_SIZE, UNKNOWN)),
      visited_(GRID_SIZE, std::vector<bool>(GRID_SIZE, false)),
      cur_r_(GRID_SIZE / 2),
      cur_c_(GRID_SIZE / 2),
      start_r_(GRID_SIZE / 2),
      start_c_(GRID_SIZE / 2),
      goal_r_(-1),
      goal_c_(-1),
      target_found_(false),
      sensors_ready_(false),
      sensor_seq_(0)
    {
        // Célula inicial do robô no mapa interno
        grid_[cur_r_][cur_c_]    = FREE;
        visited_[cur_r_][cur_c_] = true;


        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        // IMPORTANTE: tópico de sensores com o namespace correto
        sensors_sub_ = this->create_subscription<RobotSensors>(
            "/culling_games/robot_sensors",   // <<< confere com `ros2 topic list`
            qos_profile,
            std::bind(&LabyrinthMapper::sensorCallback, this, std::placeholders::_1));

        move_client_  = create_client<MoveCmd>("/move_command");
        reset_client_ = create_client<Reset>("/reset");

        RCLCPP_INFO(get_logger(),
                    "Parte 2 (mapeamento online) iniciada. Grid interno %dx%d.",
                    GRID_SIZE, GRID_SIZE);
    }

    void run()
    {
        // 1) Espera primeira leitura de sensores (robô já dentro do maze)
        wait_first_sensor();

        // 2) Explora o labirinto andando de verdade
        explore();

        if (!target_found_) {
            RCLCPP_WARN(get_logger(),
                        "Exploração terminou sem encontrar o alvo. Encerrando.");
            return;
        }

        RCLCPP_INFO(get_logger(),
                    "Exploração concluída. TARGET em (%d,%d) no grid interno.",
                    goal_r_, goal_c_);

        // 3) Reset do mundo físico (mesmo mapa, robô volta pra origem)
        RCLCPP_INFO(get_logger(), "Resetando o labirinto antes de executar BFS final...");
        if (!reset_world()) {
            RCLCPP_ERROR(get_logger(),
                         "Falha ao resetar. Não será possível executar o caminho.");
            return;
        }

        // Espera sensores refletirem o novo estado pós-reset
        int before = sensor_seq_;
        wait_sensor_update(before, 2000);
        std::this_thread::sleep_for(200ms);

        // Robô volta pra origem lógica no mapa interno
        cur_r_ = start_r_;
        cur_c_ = start_c_;

        // 4) Planeja caminho ótimo usando BFS no mapa descoberto e executa
        execute_best_path();
    }

private:
    static constexpr int GRID_SIZE = 100;  // grade "infinita" centrada na origem lógica

    // Comunicação ROS
    rclcpp::Subscription<RobotSensors>::SharedPtr sensors_sub_;
    rclcpp::Client<MoveCmd>::SharedPtr            move_client_;
    rclcpp::Client<Reset>::SharedPtr              reset_client_;

    // Estado interno
    std::vector<std::vector<int>>  grid_;    // UNKNOWN / FREE / WALL / TARGET
    std::vector<std::vector<bool>> visited_; // visitado na exploração
    int cur_r_, cur_c_;                      // posição lógica atual do robô
    int start_r_, start_c_;                  // origem lógica
    int goal_r_, goal_c_;                    // alvo encontrado
    bool target_found_;
    bool sensors_ready_;
    int  sensor_seq_;

    // ========================= SENSORES =========================

    void wait_first_sensor()
    {
        RCLCPP_INFO(get_logger(),
                    "Aguardando primeira mensagem de /culling_games/robot_sensors...");
        while (rclcpp::ok() && !sensors_ready_) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(50ms);
        }
        RCLCPP_INFO(get_logger(), "Primeira leitura de sensores recebida.");
    }

    void wait_sensor_update(int prev_seq, int max_ms = 1000)
    {
        int waited = 0;
        while (rclcpp::ok() && sensor_seq_ <= prev_seq && waited < max_ms) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(20ms);
            waited += 20;
        }
    }

    void sensorCallback(const RobotSensors::SharedPtr msg)
    {
        sensors_ready_ = true;
        ++sensor_seq_;

        RCLCPP_INFO(get_logger(),
                "Sensores: up=%s down=%s left=%s right=%s",
                msg->up.c_str(), msg->down.c_str(),
                msg->left.c_str(), msg->right.c_str());

        // célula atual é sempre livre
        grid_[cur_r_][cur_c_] = FREE;

        auto mark = [&](int r, int c, const std::string &val) {
            if (r < 0 || c < 0 || r >= GRID_SIZE || c >= GRID_SIZE) return;

            int old = grid_[r][c];
            char ch = val.empty() ? 'f' : val[0];  // pega primeiro caractere

            if (ch == 'b' || ch == 'B') {
                grid_[r][c] = WALL;
            } else if (ch == 't' || ch == 'T') {
                grid_[r][c] = TARGET;
                goal_r_ = r;
                goal_c_ = c;
                target_found_ = true;
            } else if (ch == 'f' || ch == 'F' || ch == 'r' || ch == 'R') {
                if (old != WALL) grid_[r][c] = FREE;
            } else {
                if (old != WALL) grid_[r][c] = FREE;
            }
        };

        // visinhos cardinais
        mark(cur_r_ - 1, cur_c_    , msg->up);
        mark(cur_r_ + 1, cur_c_    , msg->down);
        mark(cur_r_    , cur_c_ - 1, msg->left);
        mark(cur_r_    , cur_c_ + 1, msg->right);

        // diagonais (bom pra achar alvo cedo)
        mark(cur_r_ - 1, cur_c_ - 1, msg->up_left);
        mark(cur_r_ - 1, cur_c_ + 1, msg->up_right);
        mark(cur_r_ + 1, cur_c_ - 1, msg->down_left);
        mark(cur_r_ + 1, cur_c_ + 1, msg->down_right);
    }

    // ========================= MOVIMENTO =========================

    bool call_move(const std::string &dir)
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(shared_from_this());

        for (int i = 0; i < 30 && rclcpp::ok(); ++i) {
            if (move_client_->wait_for_service(1s)) break;
            RCLCPP_WARN(get_logger(),
                        "Aguardando /move_command... tentativa %d/30", i + 1);
        }

        if (!move_client_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "Serviço /move_command indisponível.");
            return false;
        }

        auto req = std::make_shared<MoveCmd::Request>();
        req->direction = dir;

        auto future = move_client_->async_send_request(req);
        auto status = exec.spin_until_future_complete(future, 10s);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Timeout ao chamar /move_command.");
            return false;
        }

        auto resp = future.get();
        if (!resp->success) {
            RCLCPP_WARN(get_logger(),
                        "Movimento '%s' falhou (success=false).", dir.c_str());
            return false;
        }

        return true;
    }

    // ========================= EXPLORAÇÃO (DFS ONLINE) =========================

    void explore()
    {
        RCLCPP_INFO(get_logger(), "Iniciando exploração (DFS online com backtracking).");

        // pilha de posições já percorridas (para voltar quando travar)
        std::vector<std::pair<int,int>> stack;
        stack.emplace_back(cur_r_, cur_c_);

        const int dr[4]   = {-1, 1, 0, 0};
        const int dc[4]   = {0, 0, -1, 1};
        const std::string dirs[4] = {"up", "down", "left", "right"};

        while (rclcpp::ok()) {
            // atualiza sensores
            rclcpp::spin_some(shared_from_this());

            bool moved = false;

            // tenta ir para algum vizinho livre ainda não visitado
            for (int i = 0; i < 4; ++i) {
                int nr = cur_r_ + dr[i];
                int nc = cur_c_ + dc[i];

                if (nr < 0 || nc < 0 || nr >= GRID_SIZE || nc >= GRID_SIZE) continue;

                // não entra em parede nem no alvo durante exploração
                if (grid_[nr][nc] == WALL || grid_[nr][nc] == TARGET) continue;

                if (!visited_[nr][nc]) {
                    int before = sensor_seq_;
                    if (call_move(dirs[i])) {
                        cur_r_ = nr;
                        cur_c_ = nc;
                        visited_[cur_r_][cur_c_] = true;
                        stack.emplace_back(cur_r_, cur_c_);

                        // espera sensores atualizarem
                        wait_sensor_update(before);
                        std::this_thread::sleep_for(50ms);

                        moved = true;
                        break;
                    } else {
                        // se não conseguiu andar (parede), marca como visitado pra não tentar sempre
                        visited_[nr][nc] = true;
                    }
                }
            }

            if (moved) {
                continue;  // explora a partir da nova posição
            }

            // não tem vizinho novo direto. Verifica se ainda existe fronteira em qualquer lugar
            if (!exists_frontier() || stack.size() <= 1) {
                RCLCPP_INFO(get_logger(),
                            "Sem novos vizinhos e pilha quase vazia. Exploração encerrada.");
                break;
            }

            // backtracking: volta para a penúltima posição da pilha
            auto prev = stack[stack.size() - 2];
            int pr = prev.first;
            int pc = prev.second;

            std::string back_dir;
            if      (pr == cur_r_ - 1 && pc == cur_c_) back_dir = "up";
            else if (pr == cur_r_ + 1 && pc == cur_c_) back_dir = "down";
            else if (pr == cur_r_     && pc == cur_c_ - 1) back_dir = "left";
            else if (pr == cur_r_     && pc == cur_c_ + 1) back_dir = "right";
            else {
                RCLCPP_ERROR(get_logger(),
                             "Backtracking inconsistente: (%d,%d)->(%d,%d)",
                             cur_r_, cur_c_, pr, pc);
                break;
            }

            if (call_move(back_dir)) {
                cur_r_ = pr;
                cur_c_ = pc;
                stack.pop_back();
                wait_sensor_update(sensor_seq_);
                std::this_thread::sleep_for(50ms);
            } else {
                RCLCPP_WARN(get_logger(),
                            "Falha ao fazer backtracking. Interrompendo exploração.");
                break;
            }
        }
    }

    bool exists_frontier()
    {
        // verifica se existe alguma célula visitada que tenha vizinho livre não visitado
        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        for (int r = 0; r < GRID_SIZE; ++r) {
            for (int c = 0; c < GRID_SIZE; ++c) {
                if (!visited_[r][c]) continue;
                for (int k = 0; k < 4; ++k) {
                    int nr = r + dr[k];
                    int nc = c + dc[k];
                    if (nr < 0 || nc < 0 || nr >= GRID_SIZE || nc >= GRID_SIZE) continue;
                    if (grid_[nr][nc] != WALL && grid_[nr][nc] != TARGET &&
                        !visited_[nr][nc]) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // ========================= RESET DO MUNDO =========================

    bool reset_world()
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(shared_from_this());

        for (int i = 0; i < 30 && rclcpp::ok(); ++i) {
            if (reset_client_->wait_for_service(1s)) break;
            RCLCPP_WARN(get_logger(),
                        "Aguardando /reset... tentativa %d/30", i + 1);
        }

        if (!reset_client_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "Serviço /reset indisponível.");
            return false;
        }

        auto req = std::make_shared<Reset::Request>();
        req->is_random = false;  // mantém o mesmo mapa
        req->map_name  = "";     // vazio -> reseta o mapa atual

        auto future = reset_client_->async_send_request(req);
        auto status = exec.spin_until_future_complete(future, 10s);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Timeout ao chamar /reset.");
            return false;
        }

        auto resp = future.get();
        if (!resp->success) {
            RCLCPP_ERROR(get_logger(),
                         "/reset retornou success=false. Mapa carregado: '%s'",
                         resp->loaded_map_name.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(),
                    "Reset concluído. Mapa carregado: '%s'",
                    resp->loaded_map_name.c_str());
        return true;
    }

    // ========================= BFS NO MAPA DESCOBERTO =========================

    struct Cell { int r, c; };

    std::vector<Cell> bfs_path_on_discovered()
    {
        if (goal_r_ < 0 || goal_c_ < 0) {
            RCLCPP_ERROR(get_logger(),
                         "BFS: alvo não definido no grid interno.");
            return {};
        }

        int rows = GRID_SIZE;
        int cols = GRID_SIZE;

        std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
        std::vector<std::vector<Cell>> parent(rows, std::vector<Cell>(cols, {-1, -1}));
        std::queue<Cell> q;

        auto can_traverse = [&](int r, int c) {
            if (r < 0 || c < 0 || r >= rows || c >= cols) return false;
            if (grid_[r][c] == WALL)    return false;
            if (grid_[r][c] == UNKNOWN) return false;
            return true;
        };

        Cell start{start_r_, start_c_};
        Cell goal{goal_r_, goal_c_};

        q.push(start);
        visited[start.r][start.c] = true;

        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        bool found = false;

        while (!q.empty()) {
            Cell cur = q.front();
            q.pop();

            if (cur.r == goal.r && cur.c == goal.c) {
                found = true;
                break;
            }

            for (int k = 0; k < 4; ++k) {
                int nr = cur.r + dr[k];
                int nc = cur.c + dc[k];

                if (!can_traverse(nr, nc)) continue;
                if (visited[nr][nc]) continue;

                visited[nr][nc] = true;
                parent[nr][nc]  = cur;
                q.push({nr, nc});
            }
        }

        if (!found) {
            RCLCPP_WARN(get_logger(),
                        "BFS no mapa descoberto não encontrou caminho até o alvo.");
            return {};
        }

        std::vector<Cell> path;
        Cell cur = goal;
        while (!(cur.r == start.r && cur.c == start.c)) {
            path.push_back(cur);
            cur = parent[cur.r][cur.c];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    void execute_best_path()
    {
        auto path = bfs_path_on_discovered();
        if (path.empty()) {
            RCLCPP_ERROR(get_logger(),
                         "Nenhum caminho encontrado no mapa descoberto.");
            return;
        }

        RCLCPP_INFO(get_logger(),
                    "Menor caminho no mapa descoberto tem %zu passos.",
                    path.size() - 1);

        for (size_t i = 1; i < path.size(); ++i) {
            auto prev = path[i - 1];
            auto cur  = path[i];

            std::string dir;
            if      (cur.r == prev.r - 1 && cur.c == prev.c) dir = "up";
            else if (cur.r == prev.r + 1 && cur.c == prev.c) dir = "down";
            else if (cur.r == prev.r     && cur.c == prev.c - 1) dir = "left";
            else if (cur.r == prev.r     && cur.c == prev.c + 1) dir = "right";
            else {
                RCLCPP_ERROR(get_logger(),
                             "Passo inválido no caminho BFS: (%d,%d)->(%d,%d)",
                             prev.r, prev.c, cur.r, cur.c);
                break;
            }

            if (!call_move(dir)) {
                RCLCPP_ERROR(get_logger(),
                             "Falha ao mover na direção '%s' durante execução do caminho.",
                             dir.c_str());
                break;
            }

            std::this_thread::sleep_for(50ms);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LabyrinthMapper>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
