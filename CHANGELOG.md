# Changelog

Краткие сводки изменений для ревью и для разбиения ветки на коммиты (в т.ч. скилл `.cursor/skills/rework-commits`). Не заменяет `git log`.

## Unreleased

- `task_assignment_decentralized` и лончер: **`--target-reach-radius-m` по умолчанию 0.5 м**; batch-ключ **`task.target_reach_radius_m`** (`core/batch/param_cli.py`).
- `task_assignment_decentralized`: восстановлен CLI-флаг **`--allow-converged-without-reaching-targets`** (пропал из `ArgumentParser` при правках) — искал `AttributeError` (`scenarios/task_assignment_decentralized.py`). `default_local_to_common_ned(..., east_spacing_m=FORMATION_D_STAR_M)`; логи `R`, `ε`, лимит и дистанции по дронам; флаг `--target-reach-epsilon-m` для узкого R в SITL (`scenarios/task_assignment_decentralized.py`, `launch_simulation.py`).
- `task_assignment_decentralized`: «на цели» для миссии — **финальная** горизонтальная дистанция после окна полёта (не первое пересечение круга), иначе перелёт при высоком Kp давал ложный `converged`; в метриках `per_drone_final_horiz_dist_m` (`scenarios/task_assignment_decentralized.py`).
- `task_assignment_decentralized`: выход PID на RC не использует «голый» `int()` — мёртвая зона устранена через округление и минимальный шаг ±1 PWM, иначе при узком `--target-reach-radius-m` дрон не дотягивал последние дециметры (`scenarios/task_assignment_decentralized.py`).
- `task_assignment_decentralized`: **`converged`** в `assignment_metrics.json` после прогона = сходимость решателя **и** (по умолчанию) физическое попадание всех назначенных дронов в горизонтальный круг **R** вокруг своей цели в common NED (XY); метрики `time_to_all_targets_sec`, `time_solver_to_all_targets_sec`, `per_drone_time_to_target_from_start_sec`; `viz_converged.png` только при успехе; флаги `--target-reach-radius-m`, `--allow-converged-without-reaching-targets` (paths: `scenarios/task_assignment_decentralized.py`, `launch_simulation.py`).
- `visualizer/ned_xy_snapshot.py`: опция **`inset_drone_zoom`** — врезка «Дроны (крупно)» (масштаб только под дронов), чтобы на полной карте было видно смещение до целей на кольце; сценарий передаёт её для `viz_initial` / `viz_converged` (`scenarios/task_assignment_decentralized.py`).
- `task_assignment_decentralized`: `viz_converged.png` пишется **после** завершения полёта PID (позиции в common NED как у цикла обмена), не в момент решения задачи до движения дронов (`scenarios/task_assignment_decentralized.py`).
- `task_assignment_decentralized`: в метриках время до сходимости решателя — **`time_to_solver_converged_sec`**; итоговое **`time_to_converged_sec`** в конце прогона — время до попадания всех в круг (если миссия успешна; см. новую логику `converged`) (`scenarios/task_assignment_decentralized.py`).
- Batch YAML: сценарий `task_assignment_decentralized` и оси `task.*`, `swarm.num_drones` (пер-запусковое `-n`); пример серии «5 целей, центр (10,-5), R=5» в `scenarios/batch_parameters/task_assignment_fixed_targets_sweep.yaml` без отдельных скриптов (`core/batch/param_cli.py`, `core/batch/sitl_combo.py`, `launch_simulation.py`).
- Сценарий `task_assignment_decentralized`: фиксированное число целей `--num-targets`, центр окружности целей `--target-center-x` / `--target-center-y`; матрица стоимости N×K; при K≠N используется прямоугольный SciPy LSAP, децентрализованный Chopra — только при K=N; те же флаги в лончере (`scenarios/task_assignment_decentralized.py`, `launch_simulation.py`).
- `task_assignment_decentralized` + 2D viz: цели по UDP рисуются крестиками; в каталог эксперимента пишутся `viz_initial.png` и `viz_converged.png` (Agg); при `--with-2d-visualizer` лончер выставляет `DRONE_SWARM_WITH_2D_VIZ=1` — окно визуализатора дополнительно сохраняет те же пути по запросу (`visualizer/drone_position_visualizer.py`, `visualizer/ned_xy_snapshot.py`, `scenarios/task_assignment_decentralized.py`, `launch_simulation.py`).
- Сценарий `task_assignment_decentralized`: по умолчанию после сходимости решателя полёт до конца `--duration` с проверкой попадания в `--target-reach-radius-m`; короткое окно `--post-assign-fly-sec` — только с `--allow-converged-without-reaching-targets`; `--no-exit-on-converged` по-прежнему до полного `--duration`; лончер пробрасывает флаги (`scenarios/task_assignment_decentralized.py`, `launch_simulation.py`).
- Лончер: при **Ctrl+C** во время интервальных пауз между `sim_vehicle`/во время начального boot-wait — корректное завершение всех уже запущенных SITL (process group), вспомогательных процессов (например Webots) и `pkill` MAVProxy при нужном режиме; раньше обработчик SIGINT ставился только после старта сценария, из-за чего оставались окна MAVProxy (`launch_simulation.py`). по шаблонам `mavproxy.py` и `MAVProxy` (убирает зависшие экземпляры от прошлого прогона); cleanup после прогона без изменений (`launch_simulation.py`).
- Лончер: по умолчанию **не вызывает waf** — каждый `sim_vehicle` с `-N`; пересборка только по `--sitl-rebuild-waf` или `DRONE_SWARM_SITL_ALLOW_WAF_REBUILD=1` (`launch_simulation.py`).
- Документация для диссертации: децентрализованный венгерский Chopra 2018, слои репозитория, метрики и CLI; навигатор в `docs/Магистерская диссертация/` (paths: `docs/Децентрализованный_венгерский_Chopra2018.md`, `docs/Магистерская диссертация/Навигатор_диссертации.md`).
- Сценарий `task_assignment_decentralized`: обмен координатами, матрица стоимостей NED↔фиксированные цели с глобальными индексами колонок, децентрализованный венгерский (`core/assignment/`) или `--centralized-assignment` для отладки, PID к назначенным точкам, `assignment_metrics.json` и расширенный `metadata.json`; регистрация и пункт меню в `launch_simulation.py` (`scenarios/task_assignment_decentralized.py`, лончер).
- Тесты децентрализованного венгерского: `tests/test_decentralized_hungarian.py` (SciPy/degenerate optimum/metrics/payload); старый `test_decentralized_chopra.py` удалён как дубликат.
- Метрики assignment: убран неиспользуемый `scipy_delta_final`; в `finalize()` остаются `scipy_delta_abs` и `scipy_delta_per_round`; докстринги про интерпретацию; пояснён oracle/full-`E_y` vs Sec. IV lean exchange (`metrics.py`, `decentralized_chopra.py`); тесты n=1 и изолированный граф до `max_rounds`.
- Децентрализованный венгерский Chopra et al. 2018: синхронные раунды, `MetricsCollector`, сериализация сообщений и размер payload в байтах (`core/assignment/decentralized_chopra.py`, `metrics.py`, `serialization.py`).
- Зависимость `scipy`; пакет `core/assignment/` — типы (граф рёбер, локальное состояние, сообщения 𝓖^i), эталон LSAP через `scipy.optimize.linear_sum_assignment` (`solve_lsap_reference`, `LSAPResult`).

## Ранее в репозитории (сжатая сводка)

Ниже — смысловые блоки по уже существующим коммитам (порядок примерно от новых к старым в пределах блока).

### MAVLink, телеметрия, время

- Поза в SITL: потоки `SIM_STATE`, конвертация lat/lon в NED (`geo_ned`), опция `MAVLINK_TELEMETRY_MODE`.
- Снимки задержек RC→телеметрия: поля на контроллере, доп. колонки/логи рядом с CSV эксперимента, скрипт `plot_time_sync`.

### Симулятор и сценарии

- Лончер: MAVProxy-консоли по умолчанию для режима только SITL.
- Сценарии змейки: единый шаг PID при неограниченной частоте цикла.

### Скрипты и инфраструктура

- Добавлены `CHANGELOG.md` и правило его ведения в `general-project-rules.mdc`; скилл `rework-commits` может опираться на секцию Unreleased при плане коммитов.
- `plot_experiment_csv`: только trajectory CSV; `plot_time_sync` для диагностики time-sync.
- Gitignore для `logs/`, `plots/` и артефактов графиков.

---

### Как писать новые записи

1. Правка — в секции **Unreleased**, **новые сверху**.
2. Одна строка-буллет ≈ один смысловой коммит (как при `rework-commits`): что изменилось и зачем; при необходимости хвост `(core/mavlink/...)`.
3. Язык: русский или нейтральный английский, как в сообщениях коммитов репозитория; **без** брендинга инструментов в тексте.
4. Чисто косметические правки без эффекта на поведение — обычно не дублируем.
5. Для вехи можно перенести накопленное из Unreleased в блок `## YYYY-MM-DD` и очистить Unreleased.
