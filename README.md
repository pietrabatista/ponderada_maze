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

Na parte 2 é simulado um cenário onde o robô:

- **não conhece o mapa**,  
- **não sabe onde estão paredes, caminhos ou o alvo**,  
- e deve **descobrir o labirinto explorando-o** a partir da sua posição inicial.

O algoritmo implementado:

### Explora o labirinto usando uma BFS de exploração  
A cada célula visitada:

- o robô “descobre” se é livre ou parede  
- registra no grid `discovered`  
- marca `'?'` para regiões desconhecidas  
- segue explorando apenas células transitáveis

### Constrói um mapa próprio (`mapa descoberto`)  
Ao final da exploração, um mapa é criado **apenas com as informações coletadas durante o deslocamento**.

### Roda o mesmo BFS da parte 1  
É usado exatamente a mesma função `bfs_path()` da parte 1, mas agora no mapa descoberto.

### Compara os resultados  
O código imprime:

- caminho encontrado no mapa descoberto  
- tamanho do caminho  
- se o caminho é idêntico ao da parte 1  
- se o tamanho das rotas coincide  

Isso comprova que o **mapa criado pelo robô é suficiente para reproduzir a rota ótima**, como exigido.

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
ros2 run cg_cpp_client get_map_client
```
---
# 5. Tecnologias e conceitos utilizados

- ROS 2 (Jazzy)
- rclcpp — criação de nós e clientes
- Serviços ROS 2 (/get_map)
- Estrutura de dados em C++
  - vector
  - queue
- Algoritmos de busca
  - BFS para exploração
  - BFS para path planning
- Gerenciamento de grid / matrizes 2D


