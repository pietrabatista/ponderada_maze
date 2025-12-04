# Navegação e Mapeamento com ROS 2

Este repositório contém o pacote **`cg_cpp_client`**, desenvolvido para a atividade ponderada do projeto **Culling Games** em ROS 2.
O objetivo do pacote é implementar:

- **Parte 1:** Navegação com mapa conhecido
- **Parte 2:** Mapeamento exploratório com navegação sem mapa prévio

Toda solução foi implementada em **C++ com ROS 2**, utilizando o serviço `/get_map` fornecido pelo pacote original do jogo.

---

# 1. Parte 1 — Navegação com mapa conhecido

Na parte 1, o robô já possui acesso total ao labirinto através do serviço: `/get_map` (srv: cg_interfaces/srv/GetMap)

O programa:

1. Solicita o mapa ao serviço  
2. Reconstrói o grid 2D  
3. Localiza o robô (`r`) e o alvo (`t`)  
4. Executa o algoritmo **BFS (Breadth-First Search)** para encontrar o **menor caminho possível**  
5. Imprime o caminho completo no terminal

O BFS é ideal porque garante **ótima rota em número de passos**, considerando todas as células com o mesmo custo.

---

# 2. Parte 2 — Mapeamento sem conhecimento prévio

Na parte 2 é simulado um cenário realista onde o robô não possui acesso ao mapa global. Ele deve descobrir o ambiente, identificar obstáculos e localizar o alvo utilizando apenas seus sensores locais.

### O Desafio:

- **não conhece o mapa**,  
- **não sabe onde estão paredes, caminhos ou o alvo**,  
- e deve **descobrir o labirinto explorando-o** a partir da sua posição inicial.

### O algoritmo implementado:

- Explora o labirinto usando uma **DFS (Busca em Profundidade)** para garantir movimentação contínua.
- Utiliza Backtracking (pilha) para retornar de becos sem saída.

### A cada célula visitada:

- o robô lê os sensores e descobre se é livre, parede ou alvo
- registra a informação no grid interno (memória)
- segue explorando profundamente os caminhos desconhecidos

### Ao final da exploração:

- Quando o alvo é encontrado ou o mapa esgotado, a exploração para.
- O robô solicita um **Reset** físico para voltar ao início.

### Comprovação (Rota Otimizada)

- Roda o algoritmo **BFS (Busca em Largura)** sobre o mapa descoberto (exatamente como na parte 1).
- Calcula o menor caminho possível usando apenas a memória construída.
- O robô executa fisicamente essa rota.

Isso comprova que o **mapa criado pelo robô é suficiente** para reproduzir a rota ótima de forma autônoma, como exigido.

---

# 3. Como utilizar o pacote

## Pré-requisitos

Você precisa ter o workspace **culling_games** instalado:

```bash
~/culling_games/
├── src/
│ ├── cg 
│ ├── cg_interfaces 
│ ├── cg_teleop 
│ └── cg_cpp_client ← Este Pacote
```
---
# 4. Executando o projeto

A execução sempre envolve **dois terminais**.

## **Terminal 1 — Executar o jogo (maze)**
```bash
cd ~/culling_games
source install/setup.bash
ros2 run cg maze
```
Deixe esse terminal aberto.
Ele fornece o serviço /get_map.

## **Terminal 2 — Rodar o cliente com BFS + Mapeamento**
```bash
cd ~/culling_games
colcon build --packages-select cg_cpp_client
source install/setup.bash
ros2 run cg_cpp_client part1_known_map
ros2 run cg_cpp_client part2_sensor
```
---
# 5. Tecnologias e conceitos utilizados

- **ROS 2 (Jazzy):** Framework de robótica utilizado para comunicação entre nós.
- **rclcpp:** Biblioteca cliente de C++ usada para criação de nós, serviços e tópicos.
- **Comunicação ROS:**
    - **Serviços (`/get_map`, `/move_command`):** Para operações síncronas que exigem confirmação (obter mapa e movimentação passo-a-passo).
    - **Tópicos (`/robot_sensors`):** Para leitura de sensores em tempo real com QoS Best Effort
 
- **Estrutura de dados em C++**
  - `std::vector`: Para alocação dinâmica do mapa (Heap) e gerenciamento da Pilha (Stack).
  - `std::queue`: Para gerenciamento da Fila no algoritmo BFS.
  - `std::shared_ptr`: Para gerenciamento automático de memória (RAII) sem Garbage Collector.
 
- **Algoritmos de Grafos:**
  - **DFS (Busca em Profundidade):** Utilizado na Parte 2 para exploração eficiente do desconhecido (mantendo inércia de movimento).
  - **BFS (Busca em Largura):** Utilizado na Parte 1 e na finalização da Parte 2 para garantir o cálculo da rota mais curta (otimização).
- **Representação Espacial:**
    - Conversão de array linear (Flattened) para Matriz 2D.

