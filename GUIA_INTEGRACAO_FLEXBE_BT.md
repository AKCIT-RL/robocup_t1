# 🤖 Guia Completo: Integração FlexBE + BehaviorTree no RoboCup

## 📚 Índice
1. [Contexto e Motivação](#contexto-e-motivação)
2. [Arquitetura da Solução](#arquitetura-da-solução)
3. [O Que Foi Implementado](#o-que-foi-implementado)
4. [Como Funciona](#como-funciona)
5. [Por Que Esta Abordagem](#por-que-esta-abordagem)
6. [Componentes Criados](#componentes-criados)
7. [Fluxo de Execução](#fluxo-de-execução)
8. [Próximos Passos](#próximos-passos)

---

## 🎯 Contexto e Motivação

### Situação Inicial

Você tinha dois sistemas de controle comportamental no projeto RoboCup:

1. **BehaviorTree.CPP** - Árvores de comportamento em XML
   - Arquivos em `brain/behavior_trees/*.xml`
   - Behaviors como `assist.xml`, `chase.xml`, `game.xml`
   - Executados pelo `flex_bt_server`

2. **FlexBE** - Sistema hierárquico de máquinas de estado
   - WebUI para visualização e edição
   - Estrutura de estados e transições
   - Mas SEM behaviors implementados

### Problema Identificado

Durante os testes, você percebeu:
> *"eu notei uma diferença no comportamento de execução [...] queria ver onde ocorre"*

A investigação revelou que **não existiam behaviors FlexBE implementados**. O FlexBE estava instalado, mas vazio - sem behaviors para executar os arquivos XML que você já tinha.

### Objetivo da Implementação

**Criar uma ponte entre os dois mundos:**
- Usar o FlexBE como camada de orquestração de alto nível
- Executar os BehaviorTrees XML existentes através de estados FlexBE
- Permitir visualização e controle via WebUI
- Adicionar lógica customizada (sons, sensores, game controller) ao redor dos BTs

---

## 🏗️ Arquitetura da Solução

### Componentes Existentes (Base)

Você já tinha o pacote **`flexible_behavior_trees`** no workspace:

```
flexible_behavior_trees/
├── flex_bt_server/          ← Servidor ROS 2 que executa XMLs BT
├── flex_bt_flexbe_states/   ← Estados FlexBE prontos para usar
├── flex_bt_msgs/            ← Mensagens/Actions ROS 2
└── flex_bt_engine/          ← Engine C++ do BehaviorTree.CPP
```

**Este pacote é fundamental!** Ele fornece:

1. **`flex_bt_server`** - Um servidor ROS 2 que:
   - Carrega arquivos XML do BehaviorTree.CPP
   - Executa as árvores de comportamento
   - Expõe actions ROS 2 para controle externo
   - Suporta monitoramento via Groot (portas 1666/1667)

2. **`flex_bt_flexbe_states`** - Estados FlexBE pré-prontos:
   - **`BtExecuteState`** - Executa um BT XML e aguarda conclusão
   - **`BtExecuteGoalState`** - Executa BT com objetivo/goal
   - **`BtLoaderState`** - Carrega dinamicamente um BT XML
   - **`BtGetDataState`** / **`BtSetDataState`** - Transfere dados para/do BT

### Componentes Criados (Nossa Implementação)

Criamos **3 novos pacotes ROS 2** para completar a integração:

```
robocup_t1/src/
├── robocup_flexbe_behaviors/    ← Behaviors FlexBE (orquestração)
├── robocup_flexbe_states/       ← Estados customizados RoboCup
└── robocup_bringup/             ← Launch files e configurações
```

---

## ✨ O Que Foi Implementado

### 1. **robocup_flexbe_behaviors** - Os "Maestros"

Pacote que define **máquinas de estado FlexBE** que orquestram a execução dos BTs.

**4 Behaviors criados:**

#### a) `RobocupAssistSM` (Básico)
```python
# Executa assist.xml quando chamado
State: BtExecuteState
  ├─ Carrega: brain/behavior_trees/assist.xml
  ├─ Executa através do flex_bt_server
  └─ Transições: done→finished, canceled→finished, failed→failed
```

#### b) `RobocupChaseSM` (Básico)
```python
# Executa chase.xml para perseguir a bola
State: BtExecuteState
  ├─ Carrega: brain/behavior_trees/chase.xml
  └─ Mesma estrutura de transições
```

#### c) `RobocupGameSM` (Básico)
```python
# Executa game.xml (lógica completa de jogo)
State: BtExecuteState
  ├─ Carrega: brain/behavior_trees/game.xml
  └─ Inclui integração com game controller
```

#### d) `RobocupChaseWithFeedbackSM` ⭐ (Avançado)
```python
# Behavior complexo com 5 estados sequenciais:

1. Play_Start_Sound
   └─ Toca "chase_start.wav" via sound_play

2. Check_Ball_Location  
   └─ Consulta vision para detectar bola
   
3. [Condicional]
   ├─ Se detectado: Log_Ball_Found
   └─ Se não: Play_Search_Sound
   
4. Execute_Chase_BT
   └─ BtExecuteState executa chase.xml
   
5. Play_Success_Sound
   └─ Toca "chase_complete.wav"
```

**Por que este é importante?**
- Demonstra **composição**: combina custom states + BT execution
- Mostra **feedback ao usuário**: sons indicam progresso
- Usa **sensores**: consulta visão antes de executar
- Modelo para behaviors futuros mais complexos

---

### 2. **robocup_flexbe_states** - Funcionalidades RoboCup

Estados FlexBE customizados para funcionalidades específicas do RoboCup.

#### a) `GetGameStateState` 
**O que faz:**
```python
# Subscreve no tópico do game controller
Subscription: /game_controller/state (std_msgs/String)
Outputs: game_state, game_phase
Outcomes: received | timeout | failed
```

**Como usar:**
```python
GetGameStateState(
    timeout=2.0,      # Espera máxima por mensagem
    topic='/game_controller/state'
)
# Transições:
#   received → próximo estado
#   timeout → lógica alternativa
#   failed → tratamento de erro
```

**Por que existe:**
- Game controller publica estado do jogo (INITIAL, READY, SET, PLAYING, FINISHED)
- Behaviors precisam reagir a mudanças de fase
- Usado no `game.xml` para coordenação com árbitro

#### b) `PlaySoundActionState`
**O que faz:**
```python
# Publica mensagem de áudio
Publisher: /robotsound (std_msgs/String)
Options: blocking (aguarda término) ou non-blocking
Outcomes: done | failed
```

**Como usar:**
```python
PlaySoundActionState(
    sound_file='goal_celebration.wav',
    blocking=True,      # Espera som terminar?
    wait_time=2.0       # Tempo de espera
)
```

**Por que existe:**
- Feedback auditivo para operador e público
- Indicação de eventos importantes (gol, falta, etc)
- Depuração (sons indicam progresso do behavior)

#### c) `GetBallLocationState`
**O que faz:**
```python
# Subscreve em detecções de visão
Subscription: /vision/detections (vision.msg.Detections)
Busca: Objeto com label='ball'
Outputs: ball_detected, ball_position, ball_confidence
Outcomes: detected | not_detected | timeout | failed
```

**Como usar:**
```python
GetBallLocationState(
    ball_label='ball',
    timeout=5.0,
    topic='/vision/detections'
)
# Userdata disponível:
#   ball_position → (x, y, z)
#   ball_confidence → 0.0 a 1.0
```

**Por que existe:**
- Decisões baseadas em visão (bola detectada ou não?)
- Estratégias condicionais (se bola visível → chase, senão → search)
- Graceful degradation: se pacote `vision` indisponível, retorna timeout

**Detalhe técnico importante:**
```python
# Import condicional para desenvolvimento
try:
    from vision.msg import Detections
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    # Estado funciona, mas sempre dá timeout
```

---

### 3. **robocup_bringup** - Startup e Configuração

Pacote de infraestrutura para iniciar o sistema completo.

#### Launch Files

**a) `brain_bt_server.launch.py`**
```python
# Inicia apenas o servidor BehaviorTree
Node: flex_bt_server_node
  ├─ Parameters: robocup_params.yaml
  ├─ BT default: game.xml
  └─ Groot monitoring: portas 1666/1667
```

**b) `flexbe_system.launch.py`**
```python
# Inicia FlexBE completo
Nodes:
  ├─ flexbe_onboard (behavior engine)
  └─ flexbe_webui (web server, porta 8000)
```

**c) `robocup_full.launch.py`** ⭐
```python
# Sistema completo integrado
Includes:
  ├─ brain_bt_server.launch.py
  └─ flexbe_system.launch.py
  
# Futuro:
  └─ game_controller.launch.py (comentado)
```

#### Configurações

**`param/robocup_params.yaml`**
```yaml
bt_server:
  ros__parameters:
    # Performance
    bt_loop_duration: 10        # Hz de execução
    
    # BT padrão
    default_bt_xml_filename: "brain/behavior_trees/game.xml"
    
    # Groot (visualização ao vivo)
    enable_groot_monitoring: true
    publisher_port: 1666
    server_port: 1667
    max_msgs_per_second: 25.0
    
    # Plugins BT (nós customizados C++)
    plugin_lib_names:
      - brain_bt_nodes          # Nós do brain package
      # - robocup_bt_nodes      # Fase 6 (futuro)
```

**`config/flexbe_config.json`**
```json
{
  "default_package": "robocup_flexbe_behaviors",
  "explicit_packages": [
    "robocup_flexbe_behaviors",
    "flexbe_states",
    "flex_bt_flexbe_states"
  ]
}
```

---

## 🔄 Como Funciona

### Fluxo de Execução Completo

Vamos seguir o exemplo do `RobocupChaseWithFeedbackSM`:

```
┌─────────────────────────────────────────────────────────────┐
│ 1. USUÁRIO SELECIONA BEHAVIOR NA WEBUI                      │
│    http://localhost:8000                                     │
│    Escolhe: "RoboCup Chase with Feedback"                   │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. FLEXBE ONBOARD CARREGA O BEHAVIOR                        │
│    /flexbe/start_behavior (action)                          │
│    Instancia: robocup_chase_with_feedback_sm.py             │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. ESTADO 1: Play_Start_Sound                               │
│    PlaySoundActionState("chase_start.wav")                  │
│    ├─ Publica em /robotsound                                │
│    ├─ sound_play node toca o áudio                          │
│    └─ Outcome: done                                         │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. ESTADO 2: Check_Ball_Location                            │
│    GetBallLocationState(timeout=3.0)                        │
│    ├─ Subscreve /vision/detections                          │
│    ├─ Busca objeto com label='ball'                         │
│    ├─ Armazena em userdata.ball_position                    │
│    └─ Outcome: detected ou not_detected                     │
└────────────────┬────────────────────────────────────────────┘
                 │
          ┌──────┴──────┐
          │             │
  detected│             │not_detected
          │             │
          ▼             ▼
    ┌─────────┐   ┌──────────────┐
    │ Log_Ball│   │ Play_Search_ │
    │ _Found  │   │ Sound        │
    └────┬────┘   └──────┬───────┘
          │             │
          └──────┬──────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. ESTADO 4: Execute_Chase_BT                               │
│    BtExecuteState("brain/behavior_trees/chase.xml")         │
│    ├─ Action call: /flex_bt_server/bt_execute              │
│    ├─ flex_bt_server carrega chase.xml                     │
│    ├─ BehaviorTree.CPP executa a árvore:                   │
│    │   RobotFindBall → SimpleChase → PlaySound             │
│    ├─ Publica comandos em /cmd_vel                         │
│    ├─ Monitora via Groot (porta 1666)                      │
│    └─ Outcome: done, canceled ou failed                    │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. ESTADO 5: Play_Success_Sound                             │
│    PlaySoundActionState("chase_complete.wav")              │
│    └─ Outcome: done                                         │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 7. BEHAVIOR FINALIZA                                        │
│    Outcome final: finished                                  │
│    WebUI mostra status de conclusão                         │
└─────────────────────────────────────────────────────────────┘
```

### Comunicação ROS 2

```
┌──────────────────┐
│  FlexBE WebUI    │  (FastAPI server, porta 8000)
│  localhost:8000  │
└────────┬─────────┘
         │ HTTP/WebSocket
         │
┌────────▼─────────┐
│ flexbe_webui     │  (ROS 2 node)
│    (node)        │
└────────┬─────────┘
         │ /flexbe/* topics
         │
┌────────▼─────────┐
│ flexbe_onboard   │  (Behavior Engine)
│    (node)        │  ← Executa os behaviors
└────────┬─────────┘
         │
    ┌────┴────┐
    │         │
    │ Custom  │ BtExecuteState
    │ States  │
    │         │
    ▼         ▼
┌─────────┐ ┌──────────────────┐
│/vision/ │ │/flex_bt_server/  │
│detections│ │bt_execute (action)│
└─────────┘ └────────┬─────────┘
                     │
            ┌────────▼─────────┐
            │ flex_bt_server   │  (BT executor)
            │    (node)        │
            └────────┬─────────┘
                     │
            ┌────────▼─────────┐
            │ BehaviorTree.CPP │  (Engine C++)
            │  chase.xml       │
            └────────┬─────────┘
                     │
            ┌────────▼─────────┐
            │    /cmd_vel      │  (robot movement)
            └──────────────────┘
```

---

## 💡 Por Que Esta Abordagem?

### Vantagens da Integração FlexBE + BehaviorTree

#### 1. **Separação de Responsabilidades**

**BehaviorTree.CPP** (Baixo Nível - Reativo):
- ✅ Controle motor e navegação
- ✅ Loops rápidos (10 Hz)
- ✅ Lógica reativa (se bola próxima → chutar)
- ✅ Subtrees reutilizáveis

**FlexBE** (Alto Nível - Estratégico):
- ✅ Decisões baseadas em sensores externos
- ✅ Coordenação de múltiplos BTs
- ✅ Feedback ao operador (sons, logs)
- ✅ Tratamento de exceções complexas

#### 2. **Melhor Debugabilidade**

**Antes:**
```
❌ Erro no chase.xml - onde exatamente?
❌ Qual estado do game controller?
❌ Robô parou - BT travado ou sensor falhou?
```

**Agora:**
```
✅ WebUI mostra estado atual do FlexBE
✅ Logs indicam: "Check_Ball_Location → not_detected"
✅ Groot monitora execução do BT em tempo real
✅ Sons indicam progresso ("chase_start" → robô começou)
```

#### 3. **Flexibilidade**

**Cenário 1: Teste de Chase Simples**
```python
# Use: RobocupChaseSM (apenas BT)
Execute_Chase_BT → finished
```

**Cenário 2: Demonstração com Feedback**
```python
# Use: RobocupChaseWithFeedbackSM
Sound → Vision → BT → Sound → finished
```

**Cenário 3: Jogo Oficial** (Futuro - Fase 4)
```python
# Use: RobocupFullGameSM (hierárquico)
GameStateMonitor
  ├─ INITIAL → RegisterPlayerState
  ├─ READY → FieldPlayer
  │   ├─ Check Role
  │   ├─ Striker → RobocupChaseSM
  │   └─ Defender → DefendGoalSM
  └─ FINISHED → CelebrationSM
```

#### 4. **Reuso de Código**

Os BTs XML **não foram modificados**! 
- `assist.xml`, `chase.xml`, `game.xml` continuam funcionando
- Podem ser executados diretamente pelo `flex_bt_server`
- OU através do FlexBE (com lógica adicional)

#### 5. **Padrão TurtleBot3 Demo**

Baseado no exemplo oficial:
```
flex_bt_turtlebot3_demo/
├── flex_bt_turtlebot3_demo_flexbe_behaviors/
│   └── patrol_sm.py          ← Nosso equivalente
├── flex_bt_turtlebot3_demo_bt/
│   └── patrol_tree.xml        ← Nossos XMLs
└── flex_bt_turtlebot3_demo_bringup/
    └── launch/                ← Nossa estrutura
```

---

## 📦 Componentes Criados - Detalhamento

### Estrutura de Arquivos

```
robocup_t1/
│
├── README_FLEXBE.md                    ← Docs de uso
├── GUIA_INTEGRACAO_FLEXBE_BT.md       ← Este arquivo
│
├── scripts/
│   └── start_flexbe.sh                 ← ./start_flexbe.sh full
│
└── src/
    │
    ├── robocup_flexbe_behaviors/       📦 PACKAGE 1
    │   ├── package.xml
    │   ├── setup.py
    │   ├── CMakeLists.txt
    │   │
    │   ├── config/
    │   │   └── flexbe_config.json      ← Lista de packages
    │   │
    │   ├── manifest/                    ← Metadados para WebUI
    │   │   ├── robocup_assist.xml
    │   │   ├── robocup_chase.xml
    │   │   ├── robocup_game.xml
    │   │   └── robocup_chase_with_feedback.xml
    │   │
    │   └── robocup_flexbe_behaviors/
    │       ├── __init__.py
    │       ├── robocup_assist_sm.py
    │       ├── robocup_chase_sm.py
    │       ├── robocup_game_sm.py
    │       └── robocup_chase_with_feedback_sm.py
    │
    ├── robocup_flexbe_states/          📦 PACKAGE 2
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   │
    │   └── robocup_flexbe_states/
    │       ├── __init__.py
    │       ├── get_game_state_state.py      ← Game controller
    │       ├── play_sound_action_state.py   ← Audio feedback
    │       └── get_ball_location_state.py   ← Visão
    │
    └── robocup_bringup/                📦 PACKAGE 3
        ├── package.xml
        ├── CMakeLists.txt
        │
        ├── launch/
        │   ├── brain_bt_server.launch.py
        │   ├── flexbe_system.launch.py
        │   └── robocup_full.launch.py
        │
        └── param/
            └── robocup_params.yaml
```

### Dependências

**`robocup_flexbe_behaviors/package.xml`:**
```xml
<depend>flexbe_core</depend>
<depend>flexbe_states</depend>
<depend>flex_bt_flexbe_states</depend>  ← Estados BT prontos
<depend>robocup_flexbe_states</depend>  ← Nossos custom states
```

**`robocup_flexbe_states/package.xml`:**
```xml
<depend>flexbe_core</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<!-- vision opcional - graceful degradation -->
```

**`robocup_bringup/package.xml`:**
```xml
<exec_depend>flex_bt_server</exec_depend>
<exec_depend>flexbe_onboard</exec_depend>
<exec_depend>flexbe_webui</exec_depend>
<exec_depend>robocup_flexbe_behaviors</exec_depend>
```

---

## 🎮 Fluxo de Execução - Casos de Uso

### Caso 1: Desenvolvedor Testando BT Isolado

**Não quer usar FlexBE, só testar chase.xml:**

```bash
# Terminal 1: Inicia apenas BT server
ros2 launch robocup_bringup brain_bt_server.launch.py

# Terminal 2: Groot para visualizar
groot

# Terminal 3: Executar XML
ros2 action send_goal /flex_bt_server/bt_execute \
  flex_bt_msgs/action/BtExecute \
  "{bt_name: 'brain/behavior_trees/chase.xml'}"
```

**Resultado:** BT executa, sem FlexBE envolvido.

---

### Caso 2: Operador Usando WebUI

**Quer controle visual e behaviors prontos:**

```bash
# Terminal 1: Sistema completo
./scripts/start_flexbe.sh full

# Browser: http://localhost:8000
1. Load Behavior → RoboCup Chase with Feedback
2. Click "Start Behavior"
3. Observa na UI:
   - Estado atual: Check_Ball_Location
   - Userdata: ball_detected = True
   - Outcome: detected
4. Escuta sons: chase_start.wav → chase_complete.wav
```

**Resultado:** Execução controlada com feedback visual e auditivo.

---

### Caso 3: Jogo Oficial (Futuro)

**Game controller ativo, estratégia automática:**

```bash
# Sistema completo + game controller
ros2 launch robocup_bringup robocup_full.launch.py

# FlexBE carrega automaticamente: RobocupGameSM
# States executam em loop:
GetGameStateState
  └─ game_phase: PLAYING
       └─ Execute_Game_BT (game.xml)
            └─ Striker strategy
                 └─ FindBall → Chase → Kick
```

**Resultado:** Robô joga autonomamente, responde a comandos do árbitro.

---

## 🚀 Próximos Passos

### Fase 4: Behaviors Hierárquicos (Planejado)

```python
class RobocupFullGameSM:
    """Jogo completo com sub-máquinas de estado"""
    
    states = {
        'GameStateMonitor': GetGameStateState(),
        'RegisterPlayer': RegisterPlayerState(),
        'FieldPlayer': Container([
            'DetermineRole': RoleDecisionState(),
            'Striker': RobocupChaseSM(),        # Reusa!
            'Defender': DefendGoalSM(),
        ]),
        'Goalkeeper': GoalkeeperSM(),
        'ManualOverride': TeleopAssistSM(),
    }
```

### Fase 6: Custom BT Nodes C++ (Planejado)

```cpp
// robocup_bt_nodes/is_game_phase_active.cpp
class IsGamePhaseActive : public BT::ConditionNode {
    // Checa se game_phase == "PLAYING"
    // Permite BTs reagirem ao game controller
};
```

### Fase 7: Game Controller Completo

- `RegisterPlayerState` → subscreve `/game_controller/register`
- Behaviors reagem a penalidades
- Transições automáticas INITIAL → READY → SET → PLAYING

### Fase 8: Visualização

- RViz config com detecções de visão
- Groot ao vivo (já habilitado!)
- Dashboard FlexBE customizado

---

## 🔧 Comandos Úteis

### Desenvolvimento

```bash
# Build apenas os 3 novos packages
colcon build --packages-select \
  robocup_flexbe_behaviors \
  robocup_flexbe_states \
  robocup_bringup

# Testar imports
python3 -c "from robocup_flexbe_states import *; print('OK')"

# Listar behaviors disponíveis
ros2 launch flexbe_onboard behavior_onboard.launch.py
```

### Debugging

```bash
# Ver mensagens do game controller
ros2 topic echo /game_controller/state

# Ver detecções de visão
ros2 topic echo /vision/detections

# Verificar actions disponíveis
ros2 action list | grep bt

# Monitor Groot (visualizar BT ao vivo)
# Conectar em: localhost:1666
```

### Testes

```bash
# Executar behavior via CLI
ros2 action send_goal /flexbe/execute_behavior \
  flexbe_msgs/action/BehaviorExecution \
  "{behavior_name: 'RobocupChaseSM'}"

# Tocar som manualmente
ros2 topic pub /robotsound std_msgs/String \
  "data: 'test_sound.wav'"

# Simular detecção de bola
ros2 topic pub /vision/detections vision/Detections ...
```

---

## 📝 Resumo Executivo

### O Que Foi Feito

✅ **3 novos pacotes ROS 2** integrados ao workspace  
✅ **4 behaviors FlexBE** executando XMLs existentes  
✅ **3 custom states** para RoboCup (game controller, vision, sound)  
✅ **Launch files** para startup facilitado  
✅ **Configurações** otimizadas (Groot, BT server)  
✅ **Documentação** completa e scripts de automação  

### Como Foi Feito

📌 **Usamos `flexible_behavior_trees`** como base  
📌 Estados `BtExecuteState` conectam FlexBE aos XMLs  
📌 Custom states adicionam funcionalidade RoboCup  
📌 Launch files orquestram inicialização  
📌 Padrão baseado no TurtleBot3 demo oficial  

### Por Que Foi Feito

🎯 **Separar lógica estratégica (FlexBE) de reativa (BT)**  
🎯 **Debugar behaviors visualmente (WebUI)**  
🎯 **Adicionar feedback e sensores ao redor dos BTs**  
🎯 **Preparar para jogo oficial com game controller**  
🎯 **Manter compatibilidade com BTs existentes**  

---

## ❓ FAQ

### P: Os arquivos XML foram modificados?
**R:** Não! `assist.xml`, `chase.xml`, `game.xml` estão intactos. Eles são executados "as-is" pelo `flex_bt_server`.

### P: Posso usar BTs sem FlexBE?
**R:** Sim! O `flex_bt_server` funciona independentemente. FlexBE é uma camada opcional de orquestração.

### P: Como debugar um behavior FlexBE?
**R:** 
1. Logs do ROS: `ros2 topic echo /flexbe/log`
2. WebUI: visualiza estado atual e userdata
3. Groot: monitora execução do BT (porta 1666)
4. Sounds: indicam progresso audível

### P: Posso criar novos custom states?
**R:** Sim! Basta:
1. Criar arquivo em `robocup_flexbe_states/`
2. Herdar de `EventState`
3. Implementar `on_enter()`, `execute()`, `on_exit()`
4. Adicionar ao `__init__.py`
5. Rebuild o package

### P: Como adicionar um novo behavior?
**R:**
1. Criar arquivo em `robocup_flexbe_behaviors/`
2. Definir classe herdando de `Behavior`
3. Criar manifest XML em `manifest/`
4. Rebuild e behavior aparece na WebUI

---

**Autor:** GitHub Copilot  
**Data:** Maio 2026  
**Versão:** 1.0  
**Projeto:** RoboCup FlexBE+BT Integration
