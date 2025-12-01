#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <vector>
#include <cstdint>
#include <queue>
#include <algorithm>
#include <string>
#include <utility>

using namespace std::chrono_literals;

struct Pos {
    int r;
    int c;
};

bool is_free(char cell) {
    return (cell == 'f' || cell == 'r' || cell == 't');
}

// ==================== BFS (PARTE 1 E PARTE 2) ====================
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

// ============ PARTE 2: EXPLORAR E CONSTRUIR O MAPA ============
// Dado o mapa "verdadeiro" (grid) e a posição inicial,
// simula um robô que não conhece o mapa e vai explorando aos poucos.
// Retorna um "mapa descoberto" (discovered) que o robô construiu.
std::vector<std::vector<char>> explore_and_build_map(
    const std::vector<std::vector<char>>& true_grid,
    Pos start)
{
    int rows = true_grid.size();
    int cols = true_grid[0].size();

    // '?' = célula desconhecida
    std::vector<std::vector<char>> discovered(rows, std::vector<char>(cols, '?'));
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::queue<Pos> q;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    // Robô começa na posição start: ele "sabe" onde está.
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

            // Se ainda não exploramos essa célula
            if (!visited[nr][nc]) {
                visited[nr][nc] = true;

                char real_cell = true_grid[nr][nc];
                // O robô "descobre" o que existe ali
                discovered[nr][nc] = real_cell;

                // Se a célula é transitável, ele pode ir até lá
                if (is_free(real_cell)) {
                    q.push({nr, nc});
                }
            }
        }
    }

    return discovered;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Cria nó do cliente
    auto node = rclcpp::Node::make_shared("get_map_client");

    // Cria o cliente para o serviço /get_map
    auto client = node->create_client<cg_interfaces::srv::GetMap>("get_map");

    // Requisição vazia
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();

    // Espera o serviço ficar disponível
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(),
                    "service /get_map not available, waiting again...");
    }

    // Envia requisição de forma assíncrona
    auto future_result = client->async_send_request(request);

    // Espera resposta
    if (rclcpp::spin_until_future_complete(node, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service /get_map");
        rclcpp::shutdown();
        return 1;
    }

    auto response = future_result.get();

    // Alias para facilitar
    const auto &flat  = response->occupancy_grid_flattened;  // vetor achatado
    const auto &shape = response->occupancy_grid_shape;      // [rows, cols]

    // Verificação básica do shape
    if (shape.size() != 2) {
        RCLCPP_ERROR(node->get_logger(),
                     "Invalid shape: expected 2 values (rows, cols), got %zu",
                     shape.size());
        rclcpp::shutdown();
        return 1;
    }

    uint8_t rows = shape[0];
    uint8_t cols = shape[1];

    RCLCPP_INFO(node->get_logger(),
                "Map received: %u rows x %u cols", rows, cols);

    // Verificação de consistência entre shape e vetor achatado
    if (flat.size() != static_cast<size_t>(rows) * static_cast<size_t>(cols)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Inconsistent data: flat size = %zu, expected = %u",
                     flat.size(), rows * cols);
        rclcpp::shutdown();
        return 1;
    }

    // Reconstruir matriz 2D (mapa completo = "true_grid")
    std::vector<std::vector<char>> grid;
    grid.reserve(rows);

    for (uint8_t r = 0; r < rows; ++r) {
        std::vector<char> line;
        line.reserve(cols);

        for (uint8_t c = 0; c < cols; ++c) {
            size_t idx = static_cast<size_t>(r) * cols + c;  // 0..(rows*cols-1)
            // Cada elemento de flat é algo como "b", "f", "r" ou "t"
            char cell = flat[idx][0];
            line.push_back(cell);
        }

        grid.push_back(std::move(line));
    }

    // Procurar posição do robô (r) e do alvo (t)
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

    // ====================== IMPRESSÃO DO MAPA (PARTE 1) ======================
    std::cout << "\n=== PARTE 1: MAPA COMPLETO (conhecido) ===\n";
    std::cout << "\n--- MAPA A PARTIR DA MATRIZ ---\n";
    for (uint8_t r = 0; r < rows; ++r) {
        for (uint8_t c = 0; c < cols; ++c) {
            std::cout << grid[r][c] << ' ';
        }
        std::cout << std::endl;
    }

    std::cout << "\n--- POSIÇÕES ---\n";
    std::cout << "Robô (r) em: (" << robot_row << ", " << robot_col << ")\n";
    std::cout << "Alvo (t) em: (" << target_row << ", " << target_col << ")\n";

    if (robot_row < 0 || target_row < 0) {
        std::cout << "\nRobô ou alvo não encontrados no mapa. Encerrando.\n";
        rclcpp::shutdown();
        return 0;
    }

    Pos start{robot_row, robot_col};
    Pos goal{target_row, target_col};

    // ====================== PARTE 1: BFS COM MAPA ============================
    auto path_full = bfs_path(grid, start, goal);

    if (path_full.empty()) {
        std::cout << "\nNenhum caminho encontrado no mapa completo (PARTE 1)!\n";
    } else {
        std::cout << "\n--- CAMINHO BFS (PARTE 1, mapa conhecido) ---\n";
        for (const auto& p : path_full) {
            std::cout << "(" << p.r << ", " << p.c << ")\n";
        }
        std::cout << "Tamanho do caminho (parte 1): " << path_full.size() << "\n";
    }

    // ====================== PARTE 2: MAPEAMENTO ==============================
    std::cout << "\n=================================================\n";
    std::cout << "=== PARTE 2: MAPEAMENTO DO LABIRINTO (sem mapa) ===\n";

    // O robô não conhece o mapa; ele vai "descobrir" a partir da posição inicial.
    auto discovered = explore_and_build_map(grid, start);

    std::cout << "\n--- MAPA DESCOBERTO PELO ROBÔ (discovered) ---\n";
    for (int r = 0; r < (int)rows; ++r) {
        for (int c = 0; c < (int)cols; ++c) {
            std::cout << discovered[r][c] << ' ';
        }
        std::cout << std::endl;
    }

    // Agora usamos o MESMO BFS da parte 1, mas em cima do mapa descoberto.
    auto path_discovered = bfs_path(discovered, start, goal);

    if (path_discovered.empty()) {
        std::cout << "\nNenhum caminho encontrado no mapa descoberto (PARTE 2)!\n";
    } else {
        std::cout << "\n--- CAMINHO BFS (PARTE 2, mapa descoberto) ---\n";
        for (const auto& p : path_discovered) {
            std::cout << "(" << p.r << ", " << p.c << ")\n";
        }
        std::cout << "Tamanho do caminho (parte 2): " << path_discovered.size() << "\n";
    }

    // Comparação simples entre as rotas
    if (!path_full.empty() && !path_discovered.empty()) {
        bool same_length = (path_full.size() == path_discovered.size());
        bool same_path = same_length;

        if (same_path) {
            for (size_t i = 0; i < path_full.size(); ++i) {
                if (path_full[i].r != path_discovered[i].r ||
                    path_full[i].c != path_discovered[i].c) {
                    same_path = false;
                    break;
                }
            }
        }

        std::cout << "\n--- COMPARAÇÃO ENTRE PARTE 1 E PARTE 2 ---\n";
        std::cout << "Mesmo tamanho de caminho? " << (same_length ? "Sim" : "Não") << "\n";
        std::cout << "Mesmas coordenadas na sequência? " << (same_path ? "Sim" : "Não") << "\n";
    }

    rclcpp::shutdown();
    return 0;
}
