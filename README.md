# Drone Swarm Simulator v2

Симулятор роя однородных дронов для исследований алгоритмов управления (ArduPilot SITL, MAVLink, Python 3.10+). Результаты экспериментов пишутся в CSV и могут воспроизводиться в ROS/RViz.

## Структура проекта

```
Drone_Swarm_Simulator_v2/
├── launch_simulation.py   # Единая точка входа (одиночный запуск)
├── run_batch.py           # Пакетный запуск серии экспериментов
├── setup_env.py           # Создание venv и установка зависимостей
├── drone_env/             # Виртуальное окружение Python (создаётся setup_env.py, не коммитить)
├── config/                # Параметры ArduPilot (iris.parm и др.)
├── core/                  # Ядро: MAVLink, мониторы, PID, логирование
├── scenarios/             # Сценарии (leader_forward_back, square_pid, ...)
├── replay/                # Воспроизведение логов в ROS/RViz
├── visualizer/            # 2D визуализация (matplotlib): онлайн и replay по CSV
├── experiments/           # Результаты запусков (не коммитить)
└── docs/                  # Документация исследований (локально)
```

## Установка окружения (setup_env.py)

Рекомендуемая установка выполняется на **Linux (Ubuntu)** с правами `sudo`. На Windows скрипт может частично отработать, но окружение для дипломной работы предполагается на Linux.

1. **Клонируйте репозиторий** и перейдите в каталог `Drone_Swarm_Simulator_v2`:

   ```bash
   git clone <url-репозитория>
   cd Drone_Swarm_Simulator_v2
   ```

2. **Запустите скрипт настройки окружения**:

   ```bash
   python3 setup_env.py
   ```

   Скрипт делает следующее:

   - устанавливает системные зависимости через `apt` (только на Ubuntu):
     - `git`, `build-essential`
     - `python<версия>-dev`, `python<версия>-venv`
     - `python3-tk` (tkinter для matplotlib)
     - библиотеки GTK/SDL/картинки/видео для сборки `wxPython`:
       `libgtk-3-dev`, `libglib2.0-dev`, `libsdl2-dev`, `libjpeg-dev`, `libpng-dev`,
       `libtiff-dev`, `libnotify-dev`, `freeglut3-dev`,
       `libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`,
       `libwebkit2gtk-4.1-dev`
   - создаёт виртуальное окружение `drone_env/` **в корне репозитория** `Drone_Swarm_Simulator_v2` (рядом с `launch_simulation.py`, в `.gitignore`)
   - активирует его и устанавливает Python-зависимости через `pip`:
     - `pymavlink`, `pexpect`, `empy==3.3.4`, `dronecan`, `setuptools`, `PyYAML`
     - `numpy`, `matplotlib`, `pyserial`, `future`, `lxml`
     - `wxPython` (для MAVProxy console) — **сборка может занимать 10–20 минут**
     - `opencv-python` (для модуля map в MAVProxy)
   - клонирует `MAVProxy` из GitHub в подкаталог `MAVProxy` рядом с проектом и устанавливает его в режиме editable (`pip install -e`).
   - добавляет `~/.local/bin` в `PATH` в `~/.bashrc` (на Ubuntu), чтобы бинарники `pip` и `MAVProxy` были доступны из командной строки.

3. **Особенности и рекомендации**:

   - На свежих версиях Ubuntu (например, **Ubuntu 25.10**) пакет `wxPython` может отсутствовать в бинарном виде, поэтому `pip` может собирать его **из исходников** — это нормально, просто требует времени и установленного набора dev-библиотек (см. список выше).
   - Каталог `drone_env/` должен быть создан **на той системе, где вы запускаете симуляции**. Не переносите его целиком с Windows на Linux — создайте новое окружение на Linux с помощью `setup_env.py`.
   - На Windows часть шагов (apt, обновление `~/.bashrc`) будет пропущена или ограничена, поэтому рекомендованный и поддерживаемый сценарий — запуск `setup_env.py` на Ubuntu и дальнейшая работа в этом окружении.


## Быстрый старт

```bash
# Окружение (из корня репозитория)
source drone_env/bin/activate

# Одиночный запуск (из корня проекта)
python launch_simulation.py -s -c leader_forward_back -n 3 --duration 60

# Пакетный запуск нескольких экспериментов
python run_batch.py --runs 3 --drones 2 --duration 60 --scenario leader_forward_back
```

Ниже — **расширенные команды** (`--batch-params`, MAVProxy, 2D, Docker+RViz). Обычно все команды выполняются **из корня репозитория** с активированным `drone_env`.

---

## Серия экспериментов: `launch_simulation.py --batch-params` (YAML)

Последовательный запуск комбинаций параметров из YAML (SITL-only, без `--webots`). Результаты: `experiments/<ГГГГ-ММ-ДД_ЧЧ-ММ-СС>/batch_<id>_run_<n>/` (`batch_run.json`, `metadata.json`, `drone_*.csv`).

**Базовый запуск** (путь к YAML по умолчанию — `scenarios/batch_parameters/user_batch_params.yaml`, если флаг без аргумента):

```bash
source drone_env/bin/activate
cd /path/to/Drone_Swarm_Simulator_v2

# Явный путь к YAML (рекомендуется для OFAT)
python launch_simulation.py -s --batch-params scenarios/batch_parameters/ofat_snake_pursuit/stage1_kp.yaml
```

**Полезные параметры лаунчера** (дополняют YAML; часть уходит в каждый дочерний прогон):

| Параметр | Пример | Назначение |
|----------|--------|------------|
| `-s` / `--sitl-only` | обязателен для батча | Только SITL |
| `--batch-params` | `[PATH]` | Файл YAML; без пути — дефолтный `user_batch_params.yaml` |
| `--duration` | `30` | Длительность одного прогона (с); при `0` в CLI берётся из YAML |
| `--param-file` | `config/iris.parm` | Параметры ArduPilot (по умолчанию из лаунчера) |
| `--exchange-hz` | `50` | Частота контура обмена координатами / логирования |
| `--kp`, `--ki`, `--kd`, `--derivative-alpha` | числа | Фиксация PID для OFAT, если соответствующая ось **не** в текущем sweep (`leader_forward_back`, `snake_pursuit`) |
| `--leader-roll-pwm` | `1600` | Только `snake_pursuit` / `snake_distance_ground_follower` |

OFAT-примеры и порядок этапов: **`scenarios/batch_parameters/ofat_snake_pursuit/README.md`**, методика: **`scenarios/batch_parameters/METHODOLOGY_batch_experiments_and_pid_tuning.md`**.

### Графики: `plotter/plotter.py`

Из корня репозитория, с venv (или `./drone_env/bin/python plotter/plotter.py …`).

**Данные:** для **батча** в каждом каталоге прогона нужны **`batch_run.json`**, **`drone_1.csv`**, **`drone_2.csv`** (часто рядом лежит и `metadata.json`). Для **одиночного** запуска без подпапок `batch_*_run_*` достаточно **`metadata.json`** и тех же **`drone_*.csv`** в каталоге вида `experiments/ГГГГ-ММ-ДД_ЧЧ-ММ-СС/`.

**Один прогон** (`--experiment-dir` отключает выбор по `--runs-glob`; график: `plots/<метка>/single_run_time_overlay.png`):

```bash
python plotter/plotter.py \
  --experiment-dir experiments/2026-04-19_17-16-19 \
  --mode time_overlay \
  --out-dir plots
```

**Наложение метрик во времени** для **последней сессии батча** (по умолчанию — последняя папка `experiments/ГГГГ-ММ-ДД_ЧЧ-ММ-СС/` с четырёхзначным годом; выход: `plots/<метка>/batch_time_overlay.png`):

```bash
python plotter/plotter.py --mode time_overlay --out-dir plots
```

**Конкретная сессия** или все сессии:

```bash
python plotter/plotter.py \
  --runs-glob 'experiments/2026-04-17_18-28-25/batch_*_run_*' \
  --mode time_overlay \
  --out-dir plots

python plotter/plotter.py \
  --runs-glob 'experiments/*/batch_*_run_*' \
  --mode time_overlay \
  --out-dir plots
```

**Метрика на time_overlay** (`--time-overlay-metric`):

| Значение | Смысл |
|----------|--------|
| `dxy` (по умолчанию) | Плоская дистанция √(Δx²+Δy²) в NED между **drone_1** и **drone_2** |
| `dx` | Только Δx между **drone_1** и **drone_2** (удобно для движения вдоль X) |
| `chain_mean_dxy` | Среднее по звеньям цепочки **1→2→…→N**: на каждом шаге усредняются расстояния в плоскости XY по соседним дронам; подключаются все подряд идущие **`drone_1.csv` … `drone_N.csv`**. Файл результата: `*_time_overlay_chain.png` (например `single_run_time_overlay_chain.png`). |

Пример `dx`:

```bash
python plotter/plotter.py --mode time_overlay --time-overlay-metric dx --out-dir plots
```

Пример цепочки из нескольких дронов:

```bash
python plotter/plotter.py \
  --experiment-dir experiments/2026-04-19_17-16-19 \
  --time-overlay-metric chain_mean_dxy \
  --out-dir plots
```

Режимы **`bar_metric`** и **`heatmap`** по-прежнему используют RMS **Δx между drone_1 и drone_2** по хвосту записи (`--metric-tail-sec`), не метрику цепочки.

**Столбчатая диаграмма** (RMS по хвосту записи; `--vary` — ключ из `batch_run.json` → `params`, например для OFAT этапа 1):

```bash
python plotter/plotter.py \
  --runs-glob 'experiments/2026-04-17_18-28-25/batch_*_run_*' \
  --mode bar_metric \
  --vary pid.p_gain \
  --metric-tail-sec 15 \
  --out-dir plots
```

Для этапов 2–4 подставьте `pid.d_gain`, `pid.i_gain`, `pid.derivative_alpha`.

**Тепловая карта** (нужны прогоны с двумя параметрами в `params`):

```bash
python plotter/plotter.py \
  --runs-glob 'experiments/<сессия>/batch_*_run_*' \
  --mode heatmap \
  --vary-x pid.p_gain \
  --vary-y pid.d_gain \
  --out-dir plots
```

**Дополнительно:** `--project-root` (если запуск не из корня), `--resample-points 800` (сглаживание по времени; `0` — только сырые точки). Справка:

```bash
python plotter/plotter.py --help
```

Полная таблица аргументов и замечания: **`plotter/README.md`**.

---

## Запуск с MAVProxy: `--with-mavproxy-consoles`

MAVProxy с консолями: сценарии получают MAVLink по **UDP** (как в классической цепочке `sim_vehicle` + MAVProxy). **Несовместимо** с режимом прямого TCP к SITL без MAVProxy.

```bash
source drone_env/bin/activate
cd /path/to/Drone_Swarm_Simulator_v2

# Одиночный сценарий
python launch_simulation.py -s -c snake_pursuit -n 2 --duration 30 --with-mavproxy-consoles

# Батч из YAML (MAVProxy на каждом дочернем прогоне — удобно для стабильных CSV)
python launch_simulation.py -s --batch-params scenarios/batch_parameters/ofat_snake_pursuit/stage1_kp.yaml \
  --with-mavproxy-consoles
```

После завершения прогона (или при прерывании) лаунчер выполняет **`pkill -f mavproxy.py`** — завершаются **все** процессы MAVProxy на машине, совпавшие с шаблоном. Не используйте параллельно другой MAVProxy, который нужно сохранить.

---

## 2D-визуализация в реальном времени: `--with-2d-visualizer`

Matplotlib-окно с позициями дронов (UDP `15551`, см. **visualizer/README.md**).

```bash
source drone_env/bin/activate
cd /path/to/Drone_Swarm_Simulator_v2

python launch_simulation.py -s -c snake_pursuit -n 2 --duration 60 --with-2d-visualizer
```

**Фиксированные оси графика** (опционально):

```bash
python launch_simulation.py -s -c leader_forward_back -n 2 --duration 60 --with-2d-visualizer \
  --viz-fixed-axes --viz-xlim -40 40 --viz-ylim -10 50
```

С батчем 2D-визуализатор можно передать в **родительском** вызове с `--batch-params` (см. справку лаунчера / `_run_batch_from_yaml`).

---

## 3D: воспроизведение в RViz через Docker

Нужны **Docker** и **X11** на хосте. Один раз за сессию (до запуска контейнеров):

```bash
export DISPLAY=:0   # или ваш дисплей, например :1
xhost +local:docker
```

Подставьте в примерах **`/path/to/Drone_Swarm_Simulator_v2`** — абсолютный путь к корню репозитория. Каталог эксперимента — это папка с **`metadata.json`** и **`drone_*.csv`** (например `experiments/2026-04-17_18-28-25/batch_snake_ofat_s1_kp_run_1`).

### Вариант A: один скрипт (replay + RViz)

Из корня репозитория:

```bash
./scripts/replay-with-rviz.sh experiments/<сессия>/batch_<имя>_run_<n> 1.0
# третий аргумент 1 / true / yes — интерактивное управление через ROS-топики (удобно в Docker)
./scripts/replay-with-rviz.sh experiments/<сессия>/batch_<имя>_run_<n> 1.0 1
```

### Вариант B: только replay в Docker (RViz во втором терминале)

**Терминал 1** — roscore + воспроизведение:

```bash
./scripts/replay-docker.sh experiments/<сессия>/batch_<имя>_run_<n> 1.0
# или с интерактивом:
./scripts/replay-docker.sh experiments/<сессия>/batch_<имя>_run_<n> 1.0 1
```

**Терминал 2** — контейнер с RViz (пример монтирования репозитория в `/workspace`):

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/Drone_Swarm_Simulator_v2:/workspace \
  --network host \
  osrf/ros:noetic-desktop \
  bash
```

Внутри контейнера:

```bash
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
```

В RViz: **Fixed Frame** = `world`; добавить **Pose** по топикам `/swarm/drone_1/pose`, `/swarm/drone_2/pose`, …

### Вариант C: ROS не на хосте — replay + RViz + GUI в Docker

```bash
./scripts/replay-with-rviz-and-gui.sh experiments/<сессия>/batch_<имя>_run_<n> 1.0
```

Сборка образа при первом запуске: **`docker/Dockerfile.replay-rviz-gui`**. Подробности, топики `/replay/*` и формат CSV: **`replay/README.md`**.

---

**Опциональная 2D визуализация в реальном времени** (без Docker): см. выше и **visualizer/README.md**.

Подробнее о пакетном запуске через **`run_batch.py`**, формате и путях — раздел ниже и **документацию в `docs/experiments/`**.

## Пакетные эксперименты (run_batch.py)

- **Конфигурация:** только через CLI (отдельного конфиг-файла нет).
- **Результаты без `--batch-id`:** `experiments/exp_1`, `experiments/exp_2`, ...
- **Результаты с `--batch-id`:** `experiments/batch_<id>_run_1`, `experiments/batch_<id>_run_2`, ...  
  Дополнительно создаётся индекс: `experiments/batch_<id>_runs_index.json`.
- В каждой директории запуска: `metadata.json` и `drone_*.csv`.
- Воспроизведение: любой такой каталог можно передать в `replay/replay_rviz.py --experiment <путь>` (см. `replay/README.md`).

Полное описание: **docs/experiments/batch_runs.md** (на русском).

## Воспроизведение в RViz (на хосте с ROS)

Если ROS Noetic установлен локально:

```bash
source drone_env/bin/activate
python replay/replay_rviz.py --experiment experiments/<каталог_прогона> --rate 1.0
```

**Docker и сценарии с GUI** — см. раздел **«3D: воспроизведение в RViz через Docker»** выше. Полное описание CSV, топиков и скриптов: **replay/README.md**.

## Переменные окружения и зависимости

- Python 3.10+, pymavlink, ArduPilot SITL (клон в `../ardupilot` по желанию).
- Для replay: ROS (например Noetic), `rospy`.
- Виртуальное окружение: `drone_env/` в корне репозитория (создаётся `setup_env.py`).
