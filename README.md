# Culling Games - Ponderada

Projeto desenvolvido em ROS2 para navegação autônoma utilizando dois algoritmos principais: A* (busca de caminho ótimo) e DFS com backtracking (exploração do ambiente).

---
## Execução

Você pode ver o video com a entrega e a execução nesse link: [video](/video/explicacao.mp4)

---

## Estrutura do Projeto

O pacote `ponderada` contém duas partes principais:

- **parte1:**  
  Utiliza o serviço `/get_map` para obter o mapa completo do ambiente e aplica o algoritmo **A*** para descobrir e executar o caminho ideal até o alvo (`t`).

- **parte2:**  
  O robô explora o ambiente do zero, realizando uma busca em profundidade (**DFS**) com **backtracking** para mapear o labirinto.  
  Após completar a exploração, ele envia um reset para o ambiente, executa o **A*** e percorre o caminho ideal até o destino.

---

## Compilação

No diretório raiz do workspace:

```bash
colcon build
```

Após a compilação, carregue as variáveis de ambiente:

```bash
source install/setup.bash
```

---

## Execução

### 1. Iniciar o simulador do labirinto

Em um terminal separado, execute:

```bash
ros2 run cg maze
```

Isso inicia o ambiente gráfico e o servidor dos serviços necessários (`/get_map`, `/move_command`, `/reset`, etc).

---

### 2. Executar a Parte 1 (A*)

Em outro terminal:

```bash
source install/setup.bash
ros2 run ponderada parte1
```

Nesta versão:
- O mapa é obtido via serviço `/get_map`
- O algoritmo A* encontra o caminho ótimo
- O robô executa o trajeto completo automaticamente

---

### 3. Executar a Parte 2 (Exploração + A*)

Em outro terminal:

```bash
source install/setup.bash
ros2 run ponderada parte2
```

Nesta versão:
- O robô inicia sem conhecer o mapa
- Usa DFS com backtracking para explorar completamente o ambiente
- Após o mapeamento, envia um reset para voltar à posição inicial
- Executa o A* sobre o mapa descoberto e segue até o alvo

---

## Resumo dos Algoritmos

| Parte  | Algoritmo Principal    | Objetivo                                                    |
|--------|------------------------|-------------------------------------------------------------|
| parte1 | A* (A-star)            | Encontrar o caminho ótimo a partir de um mapa completo      |
| parte2 | DFS + Backtracking + A* | Explorar o mapa, resetar e executar o caminho ótimo         |

---

## Dicas

- Sempre execute `ros2 run cg maze` antes das partes 1 ou 2.  
- Se ocorrer algum erro de conexão, aguarde alguns segundos até os serviços estarem disponíveis.  
- Para reiniciar o ambiente, feche os terminais e rode novamente os comandos de `source`.