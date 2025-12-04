#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <chrono>
#include <memory>
#include <iostream>
#include <vector>
#include <cstdint>
#include <queue>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

using GetMap  = cg_interfaces::srv::GetMap;
using MoveCmd = cg_interfaces::srv::MoveCmd;

struct Pos {
    int r;
    int c;
};

bool is_free(char cell) {
    return (cell == 'f' || cell == 'r' || cell == 't');
}

// BFS genérico
std::vector<Pos> bfs_path(const std::vector<std::vector<char>>& grid,
                          Pos start, Pos goal)
{
    int rows = grid.size();
    int cols = grid[0].size();

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Pos>> parent(rows, std::vector<Pos>(cols, {-1, -1}));
    std::queue<Pos> q;

    q.push(start);
    visited[start.r][start.c] = true;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    bool found = false;

    while (!q.empty()) {
        Pos cur = q.front();
        q.pop();

        if (cur.r == goal.r && cur.c == goal.c) {
            found = true;
            break;
        }

        for (int k = 0; k < 4; ++k) {
            int nr = cur.r + dr[k];
            int nc = cur.c + dc[k];

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;

            if (!is_free(grid[nr][nc]))
                continue;

            if (!visited[nr][nc]) {
                visited[nr][nc] = true;
                parent[nr][nc]  = cur;
                q.push({nr, nc});
            }
        }
    }

    if (!found) {
        return {};
    }

    std::vector<Pos> path;
    Pos cur = goal;

    while (!(cur.r == start.r && cur.c == start.c)) {
        path.push_back(cur);
        cur = parent[cur.r][cur.c];
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}

// Simula o processo de exploração e construção de mapa
std::vector<std::vector<char>> explore_and_build_map(
    const std::vector<std::vector<char>>& true_grid,
    Pos start)
{
    int rows = true_grid.size();
    int cols = true_grid[0].size();

    std::vector<std::vector<char>> discovered(rows, std::vector<char>(cols, '?'));
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::queue<Pos> q;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    discovered[start.r][start.c] = true_grid[start.r][start.c];
    visited[start.r][start.c] = true;
    q.push(start);

    while (!q.empty()) {
        Pos cur = q.front();
        q.pop();

        for (int k = 0; k < 4; ++k) {
            int nr = cur.r + dr[k];
            int nc = cur.c + dc[k];

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;

            if (!visited[nr][nc]) {
                visited[nr][nc] = true;

                char real_cell = true_grid[nr][nc];
                discovered[nr][nc] = real_cell;

                if (is_free(real_cell)) {
                    q.push({nr, nc});
                }
            }
        }
    }

    return discovered;
}

// Serviço de movimento
void follow_path_with_service(
    const std::shared_ptr<rclcpp::Node>& node,
    const rclcpp::Client<MoveCmd>::SharedPtr& move_client,
    const std::vector<Pos>& path)
{
    if (path.size() < 2) {
        RCLCPP_WARN(node->get_logger(),
                    "Caminho com menos de 2 posições, nada para mover.");
        return;
    }

    while (!move_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(),
                         "ROS interrompido enquanto esperava /move_command.");
            return;
        }
        RCLCPP_INFO(node->get_logger(),
                    "Serviço /move_command não disponível, esperando...");
    }

    rclcpp::Rate rate(3.0);

    for (size_t i = 1; i < path.size(); ++i) {
        int dr = path[i].r - path[i - 1].r;
        int dc = path[i].c - path[i - 1].c;

        std::string direction;

        if (dr == -1 && dc == 0) {
            direction = "up";
        } else if (dr == 1 && dc == 0) {
            direction = "down";
        } else if (dr == 0 && dc == -1) {
            direction = "left";
        } else if (dr == 0 && dc == 1) {
            direction = "right";
        } else {
            RCLCPP_WARN(node->get_logger(),
                        "Passo estranho no caminho: de (%d,%d) para (%d,%d)",
                        path[i - 1].r, path[i - 1].c, path[i].r, path[i].c);
            continue;
        }

        auto request = std::make_shared<MoveCmd::Request>();
        request->direction = direction;

        RCLCPP_INFO(node->get_logger(),
                    "Chamando /move_command com direção: %s",
                    direction.c_str());

        auto future = move_client->async_send_request(request);
        auto result = rclcpp::spin_until_future_complete(node, future);

        if (result != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(),
                         "Falha ao chamar serviço /move_command.");
            return;
        }

        auto response = future.get();

        if (!response->success) {
            RCLCPP_WARN(node->get_logger(),
                        "Movimento '%s' não teve sucesso (success=false).",
                        direction.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(),
                        "Robô em (%d, %d), alvo em (%d, %d).",
                        response->robot_pos[0], response->robot_pos[1],
                        response->target_pos[0], response->target_pos[1]);
        }

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cg_part2_mapping");

    auto map_client  = node->create_client<GetMap>("get_map");
    auto move_client = node->create_client<MoveCmd>("move_command");

    auto request = std::make_shared<GetMap::Request>();

    while (!map_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(),
                         "Interrompido esperando /get_map. Saindo.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(),
                    "Serviço /get_map não disponível, esperando...");
    }

    auto future_result = map_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Falha ao chamar /get_map");
        rclcpp::shutdown();
        return 1;
    }

    auto response = future_result.get();

    const auto &flat  = response->occupancy_grid_flattened;
    const auto &shape = response->occupancy_grid_shape;

    if (shape.size() != 2) {
        RCLCPP_ERROR(node->get_logger(),
                     "Shape inválido: esperado 2 valores, recebido %zu",
                     shape.size());
        rclcpp::shutdown();
        return 1;
    }

    uint8_t rows = shape[0];
    uint8_t cols = shape[1];

    RCLCPP_INFO(node->get_logger(),
                "Mapa recebido: %u linhas x %u colunas", rows, cols);

    if (flat.size() != static_cast<size_t>(rows) * static_cast<size_t>(cols)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Dados inconsistentes: flat size = %zu, esperado = %u",
                     flat.size(), rows * cols);
        rclcpp::shutdown();
        return 1;
    }

    std::vector<std::vector<char>> grid;
    grid.reserve(rows);

    for (uint8_t r = 0; r < rows; ++r) {
        std::vector<char> line;
        line.reserve(cols);

        for (uint8_t c = 0; c < cols; ++c) {
            size_t idx = static_cast<size_t>(r) * cols + c;
            char cell = flat[idx][0];
            line.push_back(cell);
        }

        grid.push_back(std::move(line));
    }

    int robot_row  = -1;
    int robot_col  = -1;
    int target_row = -1;
    int target_col = -1;

    for (uint8_t r = 0; r < rows; ++r) {
        for (uint8_t c = 0; c < cols; ++c) {
            char cell = grid[r][c];

            if (cell == 'r') {
                robot_row = r;
                robot_col = c;
            } else if (cell == 't') {
                target_row = r;
                target_col = c;
            }
        }
    }

    std::cout << "\n=== PARTE 2: MAPEAMENTO DO LABIRINTO (sem mapa prévio) ===\n";

    if (robot_row < 0 || target_row < 0) {
        std::cout << "\nRobô ou alvo não encontrados. Encerrando.\n";
        rclcpp::shutdown();
        return 0;
    }

    Pos start{robot_row, robot_col};
    Pos goal{target_row, target_col};

    auto discovered = explore_and_build_map(grid, start);

    std::cout << "\n--- MAPA DESCOBERTO PELO ROBÔ (discovered) ---\n";
    for (int r = 0; r < (int)rows; ++r) {
        for (int c = 0; c < (int)cols; ++c) {
            std::cout << discovered[r][c] << ' ';
        }
        std::cout << std::endl;
    }

    auto path_discovered = bfs_path(discovered, start, goal);

    if (path_discovered.empty()) {
        std::cout << "\nNenhum caminho encontrado no mapa descoberto (PARTE 2)!\n";
    } else {
        std::cout << "\n--- CAMINHO BFS (PARTE 2, mapa descoberto) ---\n";
        for (const auto& p : path_discovered) {
            std::cout << "(" << p.r << ", " << p.c << ")\n";
        }
        std::cout << "Tamanho do caminho (parte 2): " << path_discovered.size() << "\n";

        RCLCPP_INFO(node->get_logger(),
                    "Iniciando movimento do robô (PARTE 2)...");
        follow_path_with_service(node, move_client, path_discovered);
    }

    rclcpp::shutdown();
    return 0;
}
