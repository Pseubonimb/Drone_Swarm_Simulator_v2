# Тесты Drone Swarm Simulator v2

Запуск из корня репозитория (с активированным `drone_env`):

```bash
source drone_env/bin/activate
python -m pytest tests/ -q
```

Ниже перечислены модули, целевой код и смысл проверок.

---

## `test_coord_exchange.py` — `core.network`

| Тест | Назначение |
|------|------------|
| `test_default_local_to_common_ned_y_shift` | Сдвиг локального NED в общий кадр по оси Y (`east_spacing_m`). |
| `test_compute_collision_flags` | Флаги столкновения для пары дронов при заданном радиусе. |
| `test_formation_metrics_step` | Ошибка формации, минимальная дистанция, флаг коллизии. |
| `test_noise_config_from_mapping` | Разбор `CoordExchangeNoiseConfig` из словаря. |
| `test_position_noise_zero_is_identity` | Нулевой шум позиции не меняет координаты. |

---

## `test_user_batch_params.py` — `core.batch.user_batch_params`

| Тест | Назначение |
|------|------------|
| `test_linear_range_inclusive_standard_grid` | Линейная сетка `start…stop` с шагом `step`. |
| `test_linear_range_inclusive_rejects_bad_stop` | Ошибка, если `stop` не на сетке. |
| `test_generate_single_axis_combinations` | Один параметр из YAML → список комбинаций. |
| `test_generate_combinations_accepts_end_alias` | Алиас `end` вместо `stop`. |
| `test_generate_cartesian_product_two_axes` | Декартово произведение двух осей. |
| `test_iterations_repeats_each_combo` | Повтор комбинаций по `iterations`. |
| `test_example_user_batch_params_file` | Согласованность с `scenarios/batch_parameters/user_batch_params.yaml`. |
| `test_load_missing_file` | `FileNotFoundError` для отсутствующего YAML. |

---

## `test_sitl_combo.py` — `core.batch.sitl_combo`

| Тест | Назначение |
|------|------------|
| `test_partition_combo` | Разделение параметров сценария и `sitl.*` для батча. |
| `test_write_sitl_overlay_angle_max` | Запись overlay `.parm` для `ANGLE_MAX`. |
| `test_write_sitl_unknown_key` | Неподдерживаемый ключ SITL → `ValueError`. |

---

## `test_experiment_csv_io.py` — `core.experiment.csv_io`

| Тест | Назначение |
|------|------------|
| `test_leader_follower_dx_series` | Ряд `Δx` на временной сетке лидера с интерполяцией преследователя. |
| `test_rms_tail` | RMS по хвосту временного окна. |

---

## `test_batch_param_cli.py` — `core.batch.param_cli`

| Тест | Назначение |
|------|------------|
| `test_batch_params_to_scenario_argv_leader` | Маппинг PID-параметров в argv сценария `leader_forward_back`. |
| `test_batch_params_derivative_alpha_argv` | Аргумент `--derivative-alpha`. |
| `test_batch_params_unknown_key` | Неизвестный ключ → `ValueError`. |
| `test_batch_params_unsupported_scenario` | Сценарий без PID-маппинга → `ValueError`. |
| `test_merge_batch_launch_from_yaml_only` | Слияние `launch` из YAML с namespace CLI. |
| `test_merge_batch_launch_cli_overrides_scenario` | CLI переопределяет `scenario`, YAML — остальное. |
| `test_batch_experiment_label_prefers_batch_id` | Метка эксперимента из `batch_id`. |
| `test_batch_experiment_label_fallback_stem` | Fallback — имя файла YAML. |
| `test_effective_launch_merges_batch_then_launch` | Блок `batch` + `launch` → итоговый launch. |
| `test_merge_batch_launch_uses_batch_block` | Использование верхнего блока `batch`. |

---

## `test_plotter_batch.py` — `plotter.plotter`

| Тест | Назначение |
|------|------------|
| `test_discover_and_load_run_dx` | Поиск каталогов батча, загрузка `Δx` по CSV. |
| `test_plot_time_overlay_writes_file` | `plot_time_overlay` создаёт файл изображения. |

---

## `test_measure_pursuit_x_gap.py` — `scripts.measure_pursuit_x_gap`

| Тест | Назначение |
|------|------------|
| `test_compute_pursuit_x_gap_passes_when_gap_small` | Малый зазор по X на хвосте окна. |
| `test_compute_pursuit_x_gap_fails_when_gap_large` | Большой зазор по X. |
| `test_interpolates_leader_at_follower_times` | Разная частота дискретизации лидера и преследователя, интерполяция. |

---

## `test_pursuit_snake_metrics.py` — `core.experiment.csv_io` (метрики «змейки» / цепочки)

Плоскостное евклидово расстояние по **X и Y**, ряды на сетке времени лидера и среднее по звеньям цепи. Без SITL и без моков MAVLink.

| Тест | Назначение |
|------|------------|
| `test_euclidean_xy_distance_colocated` | Нулевое расстояние для совпадающих точек. |
| `test_euclidean_xy_distance_3_4_5` | Известный треугольник 3–4–5. |
| `test_euclidean_xy_distance_vectorized` | Покомпонентный расчёт для массивов. |
| `test_interpolate_scalar_on_times_empty_target` | Пустой `t_target` → пустой результат. |
| `test_interpolate_scalar_on_times_insufficient_source` | `<2` точек в источнике → нули (контракт как у `interpolate_x_on_times`). |
| `test_interpolate_scalar_on_times_linear` | Линейная интерполяция на целевую сетку. |
| `test_leader_follower_distance_xy_series_basic` | Ряд дистанции XY при синхронных временах. |
| `test_leader_follower_distance_xy_matches_abs_dx_when_dy_zero` | При `Δy=0` дистанция совпадает с \|Δx\|. |
| `test_leader_follower_distance_xy_empty_leader` | Пустой CSV лидера → пустые ряды. |
| `test_leader_follower_distance_xy_interpolates_follower_on_leader_times` | Редкий лидер, плотный преследователь; значения на отметках лидера. |
| `test_chain_two_drones_matches_pair_series` | Цепь из двух дронов = пара `leader_follower_distance_xy_series`. |
| `test_chain_three_drones_mean_of_two_links` | Среднее двух звеньев (≥2 точки на дрон для интерполяции). |
| `test_chain_four_drones_three_links_mean` | Три звена, проверка среднего арифметического. |
| `test_chain_empty_or_short_returns_empty` | Пустой список, один дрон, пустой сосед → пустой результат. |
| `test_common_ned_xy_distance_matches_manual_y_shift` | Дистанция в общем NED после `default_local_to_common_ned` совпадает с ручным расчётом. |

---

## `replay/test_playback_controller.py` — `replay.playback_controller.PlaybackState`

Юнит-тесты (unittest): индекс, `seek_to_time`, скорость, `advance_index`, `get_state`, граничные случаи (`num_steps` 0 и 1). ROS не требуется.

| Класс | Тема |
|-------|------|
| `TestPlaybackStateIndexClamp` | `set_index` в пределах `[0, n-1]`. |
| `TestPlaybackStateSeekToTime` | Привязка к шагу по массиву времени. |
| `TestPlaybackStateSpeedClamp` | Ограничение скорости, `adjust_speed`. |
| `TestPlaybackStateAdvanceIndex` | Конец ряда и инкремент. |
| `TestPlaybackStateGetState` | `playing`, индекс, скорость, `toggle_playing`. |
| `TestPlaybackStateEdgeCases` | Нулевая длина и один кадр. |

---

## Зависимости новых тестов

- `interpolate_x_on_times` реализован через `interpolate_scalar_on_times` (обратная совместимость с `test_leader_follower_dx_series`).
