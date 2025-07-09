## Technical Report – LPV Quadrotor Simulation and Control

> **Based on the original code by Mark Misin**
> This project began with the code by Mark Misin presented in the Udemy course
> `Applied Control Systems 3: UAV drone (3D Dynamics & control)`.
> It models the Astec Hummingbird quadrotor with an **LPV-MPC** controller (license
> preserved in the source files).
> The present version adds **dependency injection** for controllers, an **attitude
> PID**, and a new animation class.
> It was developed by **Samuel Lucas** for the course *Quadrotor UAV: Dynamics and
> Control*.

---

### 1.  Overall control-system structure

![planta.png](docs/planta.png)

* **Outer loop** – linearises position, computes total thrust `U1`, and
  reference angles **φ** and **θ**.
* **Inner loop** – controls attitude with MPC *or* PID, updating
  `U2 U3 U4`.
* **Plant** – discrete LPV model (`LPV_cont_discrete`) + RK4 integration.

---

### 2.  Key files and their roles

| File                                | Purpose                                                                                                                                                                                                                             |
| ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`support_files_drone.py`**        | • Defines **constants dict** (mass, inertias, CT/CQ, MPC weights, …)<br>• Generators: trajectory, position controller (`pos_controller`)<br>• Discrete LPV model (`LPV_cont_discrete`)<br>• RK4 integrator (`open_loop_new_states`) |
| **`controllers/pid_controller.py`** | Implements **`PidController`** (diagonal gains, anti-wind-up).                                                                                                                                                                      |
| **`controllers/mpc_controller.py`** | Wraps the original MPC algorithm in a class with the same interface.                                                                                                                                                                |
| **`MAIN_LPV_MPC_drone.py`**         | **`DroneLPV`** class – simulation loop:<br>  1. generate references<br>  2. pick controller (`"pid"` or `"mpc"`) via string<br>  3. log states/inputs.                                                                              |
| **`view/DroneAnimator.py`**         | New **`DroneAnimator`** class: 3-D animation, sub-plots, and “summary plots”.                                                                                                                                                       |
| **`index.py`**                      | Minimal script: creates `DroneLPV` and runs `simulate('pid')`.                                                                                                                                                                      |
| **`utils/mpl_backend.py`**          | Checks if the OS is macOS – if so, rendering is done on the OS backend; otherwise matplotlib chooses automatically.                                                                                                                 |

---

### 3.  Implemented control concepts

| Concept                            | Description / Implementation                                                                                                                                    |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **LPV (Linear Parameter-Varying)** | Linearised about the current state; parameters vary with ω<sub>total</sub>, φ, θ, ψ. Euler discretisation: `Ad = I + Ts·A`.                                     |
| **MPC**                            | Horizon `hz = 4`; compact matrices `Hdb, Fdbt` built in `mpc_simplification`. Solver uses analytic inverse (`np.linalg.inv(Hdb)`) for speed.                    |
| **Attitude PID**                   | Proportional gains equal in roll and pitch, doubled in yaw; derivative on `p,q,r`; integral clamped (±0.5 rad) to prevent wind-up. Runs every sub-step (40 Hz). |
| **Position control**               | Feedback-linearisation: computes `U1 = (v̇z+g)m/∘` and converts position errors into **φ**, **θ** references via poles `p_x, p_y, p_z`.                         |
| **Runge–Kutta 4**                  | Integrates forces, torques, and drag (`drag_switch`) inside `open_loop_new_states`, subdividing `Ts` into `sub_loop = 5` for smoother animation.                |

---

### 4.  Main changes and enhancements

| Area                     | Enhancement                                                                                                      |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| **Dependency injection** | `DroneLPV.simulate(controller: str)` dynamically selects **PID** or **MPC** without touching plant logic.        |
| **Controller classes**   | `PidController` and `MpcController` both expose `control(state, …)`, easing A/B testing.                         |
| **Velocity logging**     | Vector `velocityXYZ_total` now stored at each sub-step for Ẋ Ẏ Ż plots.                                       |
| **Animation API**        | `DroneAnimator` gathers all graphics; supports `play()` (real time) and `plot_summary()` (final plots).          |
| **Directory layout**     | `controllers/`, `view/` folders separate logic, visualisation, and entry point.                                  |
| **Documentation**        | This `.md` report and inline comments clarify the original source (Mark Misin) and new features (PID, refactor). |

---

### 5.  How to run

```bash
pip install -r requirements.txt
python index.py          # uses PID
# or
python - <<'PY'
from MAIN_LPV_MPC_drone import DroneLPV
lpv = DroneLPV()
lpv.simulate('mpc')
PY
```

The simulation opens a 3-D animation; once you close it, static plots generated by
`plot_summary()` appear.

---

### 6.  Final remarks

* **Credits** – The original model, equations, and MPC algorithm belong to
  **Mark Misin**; his license and copyright
  notice have been retained.
* **Current contribution** – Adds the **attitude PID**, dependency injection,
  refactored animation, modular structure, and this report.

---

© 2025 – Study project extending Mark Misin’s code, developed by
Samuel Lucas for the course *Quadrotor UAV: Dynamics and Control*.

---

## Relatório técnico – Simulação e Controle de um Quadricóptero LPV

> **Baseado no código original de Mark Misin**
> Este projeto partiu do código de Mark Misin apresentado no curso
> `Applied Control Systems 3: UAV drone (3D Dynamics & control)` na Udemy
> (licença mantida nos arquivos), que modela o quadricóptero Astec Hummingbird
> com um controlador **LPV-MPC**.
> A presente versão acrescenta **injeção de dependência** para controladores,
> um **PID de atitude** e uma nova classe de animação.
> A presente versão foi desenvolvida por **Samuel Lucas** para a matéria
> *VANT Quadricóptero: Dinâmica e Controle*.

---

### 1 . Estrutura geral do sistema de controle

![planta.png](docs/planta.png)

* **Outer loop**: lineariza posição, calcula empuxo total `U1` e ângulos **phi** e **theta**.
* **Inner loop**: controla atitude com MPC *ou* PID, atualizando `U2 U3 U4`.
* **Planta**: modelo LPV discreto (`LPV_cont_discrete`) + integração RK4.

---

### 2 . Principais arquivos e papéis

| Arquivo                             | Função                                                                                                                                                                                                                                |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`support_files_drone.py`**        | • Define **constant dict** (massa, inercias, CT/CQ, pesos MPC etc.)<br>• Geradores: trajetória, controlador de posição (`pos_controller`)<br>• Modelo LPV discreto (`LPV_cont_discrete`)<br>• Integração RK4 (`open_loop_new_states`) |
| **`controllers/pid_controller.py`** | Implementa **`PidController`** (ganhos diagonalizados, anti-wind-up).                                                                                                                                                                 |
| **`controllers/mpc_controller.py`** | Envolve o algoritmo MPC original numa classe com a mesma interface.                                                                                                                                                                   |
| **`MAIN_LPV_MPC_drone.py`**         | Classe **`DroneLPV`** – laço de simulação:<br>  1. gera referências<br>  2. escolhe controlador (`"pid"` ou `"mpc"`) via string<br>  3. cria log de estados/entradas.                                                                 |
| **`view/DroneAnimator.py`**         | Nova classe **`DroneAnimator`**: animação 3-D, sub-plots e “summary plots”.                                                                                                                                                           |
| **`index.py`**                      | Script mínimo: cria `DroneLPV` e roda `simulate('pid')`.                                                                                                                                                                              |
| **`utils/mpl_backend.py`**          | Faz a checagem se o sistema operacional é macOS; se for, a renderização é feita em cima do SO, caso contrário, o matplotlib escolhe a plataforma.                                                                                     |

---

### 3 . Conceitos de controle implementados

| Conceito                           | Descrição / Implementação                                                                                                                                               |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **LPV (Linear Parameter-Varying)** | Modelo linearizado em torno do estado atual; parâmetros variam com ω<sub>total</sub>, φ, θ e ψ. Discretização por Euler: `Ad = I + Ts·A`.                               |
| **MPC**                            | Horizonte `hz = 4`; matrizes compactas `Hdb, Fdbt` montadas em `mpc_simplification`. O solver usa inversa analítica (`np.linalg.inv(Hdb)`) para velocidade.             |
| **PID de atitude**                 | Ganhos proporcionais iguais em roll e pitch, 2× em yaw; derivada direta de `p,q,r`; integral saturada (±0,5 rad) para evitar wind-up. Executa a cada sub-passo (40 Hz). |
| **Controle de posição**            | Feedback-linearization: calcula `U1 = (v̇z+g)m/∘` e converte erros de posição para referências de **φ** e **θ** via polos `p_x, p_y, p_z`.                              |
| **Runge–Kutta 4**                  | Integra forças, torques e arrasto (`drag_switch`) dentro de `open_loop_new_states`, subdividindo `Ts` em `sub_loop = 5` para suavizar a animação.                       |

---

### 4 . Alterações e aprimoramentos realizados

| Tema                          | Alteração                                                                                                                  |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| **Injeção de dependência**    | `DroneLPV.simulate(controller: str)` escolhe dinamicamente **PID** ou **MPC** sem alterar lógica da planta.                |
| **Classes de controlador**    | `PidController` e `MpcController` obedecem ambos ao método `control(state, …)`, facilitando testes A/B.                    |
| **Registro de velocidades**   | Vetor `velocityXYZ_total` agora armazenado a cada sub-passo para plot de Ẋ Ẏ Ż.                                         |
| **Animation API**             | `DroneAnimator` reúne todo o código gráfico; suporta `play()` (tempo real) e `plot_summary()` (plots finais).              |
| **Arquitetura de diretórios** | Pastas `controllers/`, `view/`, separando lógica, visualização e *entry point*.                                            |
| **Documentação**              | Este relatório `.md` e comentários nos arquivos esclarecem origem (Mark Misin) e novas funcionalidades (PID, refatoração). |

---

### 5 . Como rodar

```bash
pip install -r requirements.txt
python index.py          # usa PID
# ou
python - <<'PY'
from MAIN_LPV_MPC_drone import DroneLPV
lpv = DroneLPV()
lpv.simulate('mpc')
PY
```

A simulação abre a animação 3-D; ao fechar, surgem os gráficos estáticos gerados por `plot_summary()`.

---

### 6 . Observações finais

* **Créditos** – O modelo original, equações e algoritmo MPC pertencem a
  **Mark Misin**; a licença e aviso de copyright
  foram mantidos.
* **Contribuição atual** – Inclui o **PID de atitude**, injeção de dependência,
  refatoração de animação, estrutura modular e este relatório.

---

© 2025 – Projeto de estudo e extensão do código de Mark Misin,
desenvolvido por **Samuel Lucas** para a matéria
*VANT Quadricóptero: Dinâmica e Controle*.
