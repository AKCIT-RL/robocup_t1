# RoboCup FlexBE + BehaviorTree Integration

Sistema completo de controle de comportamentos para RoboCup usando FlexBE (Flexible Behavior Engine) integrado com BehaviorTree.CPP.

## 📦 Arquitetura

### Pacotes Criados

1. **robocup_flexbe_behaviors** - Behaviors FlexBE principais
   - `robocup_assist_sm.py` - Assistência com IA para teleoperação
   - `robocup_chase_sm.py` - Perseguição de bola
   - `robocup_game_sm.py` - Jogo oficial completo
   - `robocup_chase_with_feedback_sm.py` - Chase com feedback de visão e som

2. **robocup_flexbe_states** - Custom FlexBE states
   - `GetGameStateState` - Lê estado do game controller
   - `PlaySoundActionState` - Toca sons via sound_play
   - `GetBallLocationState` - Retorna posição da bola da visão

3. **robocup_bringup** - Launch files e configurações
   - `brain_bt_server.launch.py` - Inicia BT server
   - `flexbe_system.launch.py` - Inicia FlexBE onboard + WebUI
   - `robocup_full.launch.py` - Sistema completo

## 🚀 Quick Start

### Build do Sistema

```bash
cd /home/lucas_olives/Documents/robocup/robocup_t1
docker build -t robocup_demo:2.0 -f dockerfile .
```

### Iniciar Container

```bash
docker compose up booster_brain
# ou
docker run -it --rm --runtime=nvidia --network=host robocup_demo:2.0
```

### Lançar Sistema Completo

Dentro do container:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Opção 1: Sistema completo (BT server + FlexBE)
ros2 launch robocup_bringup robocup_full.launch.py

# Opção 2: Apenas FlexBE system
ros2 launch robocup_bringup flexbe_system.launch.py

# Opção 3: Apenas BT server
ros2 launch robocup_bringup brain_bt_server.launch.py
```

### Acessar FlexBE WebUI

O FlexBE WebUI roda na porta **8000** dentro do container. Como o container usa `network_mode: host`, acesse:

```
http://localhost:8000
```

No browser do host para ver a interface do operador.

## 🎮 Behaviors Disponíveis

### 1. RoboCup Assist
- **Arquivo**: `robocup_assist_sm.py`
- **BT**: `brain/behavior_trees/assist.xml`
- **Descrição**: Assistência com IA para teleoperação (chase + kick)
- **Estados**: Execute_Assist_BT → finished/failed

### 2. RoboCup Chase
- **Arquivo**: `robocup_chase_sm.py`
- **BT**: `brain/behavior_trees/chase.xml`
- **Descrição**: Perseguição de bola com sons
- **Estados**: Execute_Chase_BT → finished/failed

### 3. RoboCup Game
- **Arquivo**: `robocup_game_sm.py`
- **BT**: `brain/behavior_trees/game.xml`
- **Descrição**: Jogo oficial completo com estratégias
- **Estados**: Execute_Game_BT → finished/failed

### 4. RoboCup Chase with Feedback ⭐ (NOVO)
- **Arquivo**: `robocup_chase_with_feedback_sm.py`
- **Descrição**: Chase com verificação de visão e feedback sonoro
- **Estados**:
  1. Play_Start_Sound - Toca som de início
  2. Check_Ball_Location - Verifica detecção da bola
  3. Log_Ball_Found ou Play_Search_Sound - Feedback
  4. Execute_Chase_BT - Executa chase.xml
  5. Play_Success_Sound - Som de conclusão

## 🔧 Custom States

### GetBallLocationState
```python
from robocup_flexbe_states.get_ball_location_state import GetBallLocationState

# Uso
GetBallLocationState(topic='/vision/detections', timeout=2.0, ball_label='ball')
# Outputs: ball_detected, ball_position, ball_confidence
# Outcomes: detected, not_detected, timeout, failed
```

### PlaySoundActionState
```python
from robocup_flexbe_states.play_sound_action_state import PlaySoundActionState

# Uso
PlaySoundActionState(sound_name='goal', topic='/robotsound', blocking=True, wait_time=2.0)
# Outcomes: done, failed
```

### GetGameStateState
```python
from robocup_flexbe_states.get_game_state_state import GetGameStateState

# Uso
GetGameStateState(topic='/game_controller/state', timeout=2.0)
# Outputs: game_state, game_phase
# Outcomes: received, timeout, failed
```

## ⚙️ Configuração

### robocup_params.yaml
```yaml
bt_server:
  ros__parameters:
    default_bt_xml_filename: "brain/behavior_trees/game.xml"
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
```

### flexbe_config.json
```json
{
  "default_package": "robocup_flexbe_behaviors",
  "explicit_packages": [
    "robocup_flexbe_behaviors", 
    "robocup_flexbe_states",
    "flexbe_states",
    "flex_bt_flexbe_states"
  ]
}
```

## 🔍 Debugging

### Verificar Behaviors Carregados
```bash
ros2 launch flexbe_onboard behavior_onboard.launch.py
# Output deve mostrar: RobocupGameSM, RobocupChaseSM, RobocupAssistSM, RobocupChaseWithFeedbackSM
```

### Verificar Tópicos
```bash
# FlexBE topics
ros2 topic list | grep flexbe

# Visão
ros2 topic echo /vision/detections

# Som
ros2 topic echo /robotsound

# Game Controller
ros2 topic echo /game_controller/state
```

### Groot Monitoring (BehaviorTree Visualization)
```bash
# Instalar Groot
sudo apt install groot

# Conectar ao BT server
groot --zmq_publisher_port 1666 --zmq_server_port 1667
```

## 📁 Estrutura de Arquivos

```
robocup_t1/src/
├── robocup_flexbe_behaviors/
│   ├── manifest/
│   │   ├── robocup_assist.xml
│   │   ├── robocup_chase.xml
│   │   ├── robocup_game.xml
│   │   └── robocup_chase_with_feedback.xml
│   ├── config/
│   │   └── flexbe_config.json
│   └── robocup_flexbe_behaviors/
│       ├── robocup_assist_sm.py
│       ├── robocup_chase_sm.py
│       ├── robocup_game_sm.py
│       └── robocup_chase_with_feedback_sm.py
│
├── robocup_flexbe_states/
│   └── robocup_flexbe_states/
│       ├── get_game_state_state.py
│       ├── play_sound_action_state.py
│       └── get_ball_location_state.py
│
├── robocup_bringup/
│   ├── launch/
│   │   ├── brain_bt_server.launch.py
│   │   ├── flexbe_system.launch.py
│   │   └── robocup_full.launch.py
│   ├── param/
│   │   └── robocup_params.yaml
│   └── rviz/
│       └── (future RViz configs)
│
└── brain/
    └── behavior_trees/
        ├── assist.xml
        ├── chase.xml
        ├── game.xml
        └── subtrees/
```

## 🎯 Próximos Passos

### Fase 4: Behaviors Hierárquicos (Futuro)
- [ ] Criar `RoboCup Full Game` com sub-state machines
- [ ] Adicionar userdata para parametrização
- [ ] Implementar Strategy Selector

### Fase 6: Custom BT Nodes C++ (Futuro)
- [ ] Criar `robocup_bt_nodes` package
- [ ] Implementar `IsGamePhaseActive` condition
- [ ] Implementar `IsBallDetected` condition

### Fase 7: Integração Game Controller (Testagem Futura)
- [ ] Testar GetGameStateState com game controller real
- [ ] Criar RegisterPlayerState
- [ ] Implementar reação a penalidades

## 📝 Notas

- **Network Mode**: Container usa `network_mode: host`, então todas as portas estão acessíveis diretamente
- **WebUI Port**: FlexBE WebUI roda na porta 8000
- **Groot Ports**: ZMQ publisher 1666, server 1667
- **Custom States**: Implementados para funcionar sem game controller real (graceful degradation)

## 🐛 Troubleshooting

### Behaviors não aparecem na WebUI
1. Verificar se flexbe_onboard está rodando
2. Verificar manifests em `install/robocup_flexbe_behaviors/lib/robocup_flexbe_behaviors/manifest/`
3. Verificar flexbe_config.json carregado

### BT Server não encontra XMLs
1. Verificar path em robocup_params.yaml
2. Verificar se brain package foi compilado
3. Verificar `brain/behavior_trees/*.xml` existe

### Custom states com erro de import
1. Verificar se robocup_flexbe_states foi compilado
2. Source `install/setup.bash`
3. Verificar dependências (vision, sound_play)

---

**Implementado em**: Maio 2026  
**Versão**: 1.0  
**Status**: ✅ Fases 1-5 Completas (Launch files, Custom states, Configuração)
