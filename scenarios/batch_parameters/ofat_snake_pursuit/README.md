# OFAT-настройка `snake_pursuit` (лидер +Y, фоловер — pitch/roll PID)

Пошаговый перебор **одной оси в YAML**, остальные коэффициенты задаются **флагами лаунчера** (см. ниже). После правки `launch_simulation.py` родительские `--kp`, `--ki`, `--kd`, `--ki-pitch`, `--derivative-alpha` пробрасываются в каждый дочерний прогон батча, если соответствующий ключ **не** входит в текущий sweep.

Если для корректного заполнения `drone_*.csv` нужен **MAVProxy** (а не прямое TCP к SITL), ко всем примерам ниже добавляйте **`--with-mavproxy-consoles`**: после каждого прогона батч-цикла лаунчер выполняет `pkill -f mavproxy.py`, чтобы следующая итерация поднялась на свободных портах.

## Целевой зазор и метрики (заполните под диплом)

В сценарии фоловер стремится держать **≈2 m** по оси **Y** в общей NED-сетке (`error_y = rel_y - 2` в `snake_pursuit.py`). Для отчёта зафиксируйте:

| Обозначение | Смысл | Пример |
|-------------|--------|--------|
| \(d^\*\) | целевой зазор по выбранной величине | 2.0 m по \|Δy\| между центрами в общей сетке |
| \(\varepsilon\) | допустимое отклонение «дошли до режима» | 0.1–0.2 m |
| \(T_\varepsilon\) | время до первого входа в \(\|d - d^\*\| \le \varepsilon\) (и удержания) | считать по `drone_*.csv` |
| IAE | \(\int \|d(t)-d^\*\|\,dt\) на \([0,T]\) | по тем же CSV |
| перерегулирование | \(\max_t \|d - d^\*\|\) или \(\max_t d\) | по CSV |

Сводная метрика (пример): \(J = T_\varepsilon + w_1 \cdot \mathrm{IAE} + w_2 \cdot \text{overshoot}\) — веса под ваш приоритет (скорость vs плавность).

## Шаблон таблицы результатов

Скопируйте `tuning_results_template.csv` и заполняйте после каждого этапа. Поля `T_epsilon_s`, `IAE_m_s`, `overshoot_m`, `J` считайте скриптом или вручную из логов.

```text
run_index,session_stamp,rel_exp_dir,kp,ki,kd,ki_pitch,derivative_alpha,angle_max_cd,d_star_m,epsilon_m,T_epsilon_s,IAE_m_s,overshoot_m,J,notes
1,2026-04-10_20-00-00,experiments/.../batch_snake_ofat_s1_kp_run_1,2,0,10,,0,3000,2.0,0.15,,,,,kp low oscillation
```

## Порядок этапов (рекомендация)

Дефолты сценария: `--kp 8`, `--ki 0`, `--kd 10`, `--derivative-alpha 0` (см. `snake_pursuit.py`).

### Этап 1 — только P (`pid.p_gain` → `--kp`)

```bash
python launch_simulation.py -s --batch-params \
  scenarios/batch_parameters/ofat_snake_pursuit/stage1_kp.yaml
```

Выберите **kp\*** с лучшим компромиссом \(T_\varepsilon\) / IAE / перерегулирование (и устойчивость).

### Этап 2 — D (`pid.d_gain` → `--kd`), P зафиксирован

Подставьте свой **kp\***:

```bash
python launch_simulation.py -s --kp <kp_star> --batch-params \
  scenarios/batch_parameters/ofat_snake_pursuit/stage2_kd.yaml
```

Получите **kd\***.

### Этап 3 — I по pitch (`pid.ki_pitch` → `--ki-pitch`), без лишнего I на roll

```bash
python launch_simulation.py -s --kp <kp_star> --kd <kd_star> --ki 0 --batch-params \
  scenarios/batch_parameters/ofat_snake_pursuit/stage3_ki_pitch.yaml
```

### Этап 4 — сглаживание D (`pid.derivative_alpha`)

```bash
python launch_simulation.py -s --kp <kp_star> --kd <kd_star> --ki 0 \
  --ki-pitch <ki_pitch_star> --batch-params \
  scenarios/batch_parameters/ofat_snake_pursuit/stage4_derivative_alpha.yaml
```

### Этап 5 (опционально) — 2D-сетка

Когда 1D ясность есть, отдельный YAML с **ровно двумя** осями в `parameter_sweeps` или несколько пар «лучший kp ± δ» × «лучший kd ± δ»; для визуализации: `plotter/plotter.py --mode heatmap --vary-x ... --vary-y ...`.

## Графики

```bash
python plotter/plotter.py --mode time_overlay --out-dir plots
# bar_metric / heatmap — см. plotter/README.md
```

`ANGLE_MAX` в этих OFAT-файлах **не перебирается** (фиксируется `iris.parm` / вашим `--param-file`). Чтобы исследовать лимит наклона, добавьте отдельную ось `sitl.ANGLE_MAX` **после** стабилизации PID.

## Файлы в этой папке

| Файл | Назначение |
|------|------------|
| `stage1_kp.yaml` | sweep `pid.p_gain` |
| `stage2_kd.yaml` | sweep `pid.d_gain` |
| `stage3_ki_pitch.yaml` | sweep `pid.ki_pitch` |
| `stage4_derivative_alpha.yaml` | sweep `pid.derivative_alpha` |
| `tuning_results_template.csv` | шаблон журнала прогонов |
