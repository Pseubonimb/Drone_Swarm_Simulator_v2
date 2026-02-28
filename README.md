# Drone Swarm Simulator v2

Симулятор роя однородных дронов для исследований алгоритмов управления (ArduPilot SITL, MAVLink, Python 3.10+). Результаты экспериментов пишутся в CSV и могут воспроизводиться в ROS/RViz.

## Структура проекта

```
Drone_Swarm_Simulator_v2/
├── launch_simulation.py   # Единая точка входа (одиночный запуск)
├── run_batch.py           # Пакетный запуск серии экспериментов
├── config/                # Параметры ArduPilot (iris.parm и др.)
├── core/                  # Ядро: MAVLink, мониторы, PID, логирование
├── scenarios/             # Сценарии (leader_forward_back, square_pid, ...)
├── replay/                # Воспроизведение логов в ROS/RViz
├── visualizer/            # 2D визуализация (matplotlib): онлайн и replay по CSV
├── experiments/           # Результаты запусков (не коммитить)
└── docs/                  # Документация исследований (локально)
```

## Быстрый старт

```bash
# Окружение
source ../drone_env/bin/activate

# Одиночный запуск (из корня проекта)
python launch_simulation.py -s -c leader_forward_back -n 3 --duration 60

# Пакетный запуск нескольких экспериментов
python run_batch.py --runs 3 --drones 2 --duration 60 --scenario leader_forward_back
```

**Опциональная 2D визуализация в реальном времени:** флаг `--with-2d-visualizer` при запуске лаунчера; подробности — в **visualizer/README.md**.

Подробнее о пакетном запуске, формате конфигурации и расположении результатов см. раздел ниже и **документацию в `docs/experiments/`**.

## Пакетные эксперименты (run_batch.py)

- **Конфигурация:** только через CLI (отдельного конфиг-файла нет).
- **Результаты без `--batch-id`:** `experiments/exp_1`, `experiments/exp_2`, ...
- **Результаты с `--batch-id`:** `experiments/batch_<id>_run_1`, `experiments/batch_<id>_run_2`, ...  
  Дополнительно создаётся индекс: `experiments/batch_<id>_runs_index.json`.
- В каждой директории запуска: `metadata.json` и `drone_*.csv`.
- Воспроизведение: любой такой каталог можно передать в `replay/replay_rviz.py --experiment <путь>` (см. `replay/README.md`).

Полное описание: **docs/experiments/batch_runs.md** (на русском).

## Воспроизведение в RViz

```bash
python replay/replay_rviz.py --experiment experiments/exp_1 --rate 1.0
```

Формат CSV, metadata и топики ROS — в **replay/README.md**.

## Переменные окружения и зависимости

- Python 3.10+, pymavlink, ArduPilot SITL (клон в `../ardupilot` по желанию).
- Для replay: ROS (например Noetic), `rospy`.
- Виртуальное окружение: `../drone_env` (относительно проекта).

## Лицензия и дипломная работа

Документация в `docs/` предназначена для дипломной работы и хранится локально (не публикуется в репозитории).
