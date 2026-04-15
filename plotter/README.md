# Plotter: графики по серии батч-экспериментов

Скрипт `[plotter.py](plotter.py)` строит фигуры по каталогам вида `experiments/<YYYY-MM-DD_HH-MM-SS>/batch_<имя>_run_<n>/` (сессия батча + прогоны), куда лаунчер кладёт `**batch_run.json**` и `**drone_*.csv**` (тот же формат CSV, что в правилах проекта: `t,x,y,z,rx,ry,rz,hasCollision`).

## Окружение и каталог

Работайте из **корня репозитория** (рядом с `launch_simulation.py`), с активированным venv:

```bash
cd /path/to/Drone_Swarm_Simulator_v2
source drone_env/bin/activate
```

Без активации можно так: `./drone_env/bin/python plotter/plotter.py ...`

## Базовая команда: наложение кривых во времени

Сохраняет файл `**plots/<YYYY-MM-DD_HH-MM-SS>/batch_time_overlay.png**` — имя подпапки совпадает с папкой сессии эксперимента под `experiments/` (если прогоны лежат в `experiments/<метка>/batch_*_run_*`). При плоской раскладке `experiments/batch_*_run_*` файл пишется прямо в `**plots/batch_time_overlay.png**`.

```bash
python plotter/plotter.py --mode time_overlay --out-dir plots
```

**По умолчанию** (`--runs-glob` не указывать) плоттер берёт **последнюю по имени** папку сессии под `experiments/`, имя которой совпадает с `YYYY-MM-DD_HH-MM-SS` (четырёхзначный год), и строит график только по `…/batch_*_run_*` внутри неё — например, после свежего батча это будет что-то вроде `experiments/2026-04-08_22-01-21/batch_*_run_*`. В лог пишется строка `Using latest batch session: …`.

Другую сессию или все сессии сразу задайте явным glob:

```bash
python plotter/plotter.py \
  --runs-glob 'experiments/2026-04-07_12-00-00/batch_*_run_*' \
  --mode time_overlay \
  --out-dir plots

# все прогоны во всех сессиях с корректной меткой времени:
python plotter/plotter.py \
  --runs-glob 'experiments/*/batch_*_run_*' \
  --mode time_overlay \
  --out-dir plots
```

Папки сессий вида `26-04-07_…` (двузначный год) **не участвуют** в выборе «последней сессии» по умолчанию; при необходимости укажите путь вручную в `--runs-glob`.

**Что на графике:** для каждого прогона — кривая по времени (лидер = `drone_1.csv`, фоловер = `drone_2.csv`). По умолчанию (`**--time-overlay-metric dxy`**) — **плоская дистанция** √(Δx²+Δy²) в NED из CSV (удобно для змейки по Y, когда x почти не меняется и старый график Δx выглядел «ступеньками»). Режим `**--time-overlay-metric dx`** — только **Δx** (разрыв по северной оси). В легенде — до четырёх пар из `batch_run.json` → `params`.

## Полный список аргументов CLI


| Аргумент                | По умолчанию                 | Назначение                                                                                                                                               |
| ----------------------- | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--project-root`        | родитель каталога `plotter/` | Корень репозитория; glob и `--out-dir` считаются от него, если путь относительный.                                                                       |
| `--runs-glob`           | последняя `experiments/YYYY-MM-DD_HH-MM-SS/batch_*_run_*` | Без флага — новейшая сессия (имя каталога = метка времени с **4**-значным годом). Иначе glob относительно `--project-root`.                                |
| `--out-dir`             | `plots`                      | Куда писать PNG (относительно `--project-root`, если не абсолютный путь).                                                                                |
| `--mode`                | `time_overlay`               | `time_overlay` | `bar_metric` | `heatmap`.                                                                                                               |
| `--metric-tail-sec`     | `10`                         | Длина хвоста (с конца записи) для RMS(Δx) в режимах `bar_metric` и `heatmap`.                                                                            |
| `--vary`                | нет                          | Для `bar_metric`: ключ из `params` для подписи оси X (например `pid.p_gain`); иначе по оси X идёт `run_index`.                                           |
| `--vary-x`, `--vary-y`  | нет                          | Для `heatmap`: два ключа из `params`; нужны прогоны, где оба ключа есть.                                                                                 |
| `--resample-points`     | `800`                        | Равномерная сетка по времени в зоне пересечения логов; линейная интерполяция координат → гладкий `time_overlay` / RMS. `0` — только сырые метки времени. |
| `--time-overlay-metric` | `dxy`                        | `dxy` = √(Δx²+Δy²); `dx` = только x_follower−x_leader (для leader_forward_back по оси X).                                                                |


Имена выходных файлов (при одной сессии `experiments/<stamp>/…` подставляется `<out-dir>/<stamp>/`):

- `time_overlay` → `<out-dir>/<stamp>/batch_time_overlay.png` (или `<out-dir>/batch_time_overlay.png` без подпапки, если метку сессии определить нельзя)
- `bar_metric` → `…/batch_bar_rms_dx.png`
- `heatmap` → `…/batch_heatmap_rms_dx.png`

Справка по всем флагам:

```bash
python plotter/plotter.py --help
```

## Типичный цикл: YAML-батч → график

1. Прогнать серию (из корня репозитория):
  ```bash
   python launch_simulation.py -s --batch-params scenarios/batch_parameters/snake_pursuit_p_gain_batch.yaml
  ```
   Либо интерактивно: `python launch_simulation.py` → ввести путь к YAML.
2. Построить overlay:
  ```bash
   python plotter/plotter.py --mode time_overlay --out-dir plots
  ```
3. Открыть `plots/batch_time_overlay.png` в просмотрщике или вставить в отчёт/диплом.

## Другие режимы (кратко)

**Столбцы RMS по хвосту** (сравнение прогонов одной метрикой):

```bash
python plotter/plotter.py --mode bar_metric --vary pid.p_gain --metric-tail-sec 15 --out-dir plots
```

**Тепловая карта** по двум параметрам из `batch_run.json` (нужна сетка комбинаций в данных):

```bash
python plotter/plotter.py --mode heatmap --vary-x pid.p_gain --vary-y sitl.ANGLE_MAX --out-dir plots
```

## Условия, при которых прогон попадает на график

- В каталоге есть `**batch_run.json**`.
- Есть `**drone_1.csv**` и `**drone_2.csv**`.  
Прогоны только с одним дроном или без второго CSV будут пропущены (в логе предупреждение).

## Замечания

- Если CSV пишется **только при смене позы/аттитуда** (`--log-hz 0`), точек мало — без пересэмплинга кривые выглядят «ломанными». По умолчанию plotter строит **800** равномерных отсчётов на пересечении времён обоих `drone_*.csv`.
- `**sitl.ANGLE_MAX`** в батч-YAML задаётся в **сантидолях градуса** (как в ArduPilot), см. `config/iris.parm`.
- Число комбинаций в `parameter_sweeps` растёт как произведение длин осей — разумно начинать с одной оси, затем небольшие 2D-сетки.
- Matplotlib использует backend **Agg** (файл на диск, без окна) — удобно на SSH и в CI.

