# Ozon Hackathon — трек 3: роботизированная сортировка товаров

Виртуальная ячейка предсортировки: depth-камера над непрерывно идущим конвейером
видит несколько товаров, сохраняет их `item_id`, классифицирует каждый и двумя
поворотными шиберами направляет его по склизам в зоны. Полный контур — подача →
восприятие → классификация → управление → физическая маршрутизация — замкнут в
Gazebo и воспроизводится одной командой.

**Категории.** `B` — подходит для основного сортировщика; `C` — не проходит по
габаритам; `D` — требует доупаковки (обнаружен круг в сечении, `K > 0.8`).
Правила задачи — `docs/md/task.md`, эталонные модели — `docs/md/models.md`.

**Текущий результат.** Три полных body-scored census дали `33/33 ×3`. По пяти
успешным 3-товарным потокам steady-state производительность — медиана
`18 товаров/мин`, p95-медленный такт — `11 товаров/мин`. Startup Gazebo и
soft-start ленты в эту метрику не входят.

**Стек.** Python 3.11+, ROS 2 Humble, Gazebo Fortress (`ros_gz`), NumPy/SciPy.
Ядро восприятия и классификации тестируется отдельно от ROS 2; ROS 2 связывает
узлы в контур, Gazebo даёт физику ленты, товаров, шиберов, склизов и камеры.

## Навигация

| Документ | Что внутри |
|---|---|
| `docs/md/task.md` | Условия задачи трека |
| `docs/md/criteries.md` | Критерии оценки жюри |
| `docs/md/models.md` | Анализ 11 тестовых моделей |
| `docs/md/organizer_faq_2026-07-16.md` | Официальные уточнения о приватном наборе, Docker и физических допущениях |
| `docs/report/` | Разделы итогового отчёта (архитектура, классификация, методология и ограничения) |
| `docs/report/layout.md` | Размерная компоновка шиберов, склизов, зон и сервисного доступа |
| `docs/report/validation.md` | Frozen real-Gazebo срезы: recall, ошибки габаритов/K и ограничения |
| `docs/report/failure_scenarios.md` | Нештатные ситуации: сигнал, безопасная реакция, восстановление и команды проверки |
| `docs/decisions.md` | Журнал инженерных решений с обоснованиями |
| `docs/experiments.md` | Журнал прогонов и метрик |
| `PLAN.md` | Стратегия и календарь этапа |
| `CLAUDE.md`, `GIT.md` | Правила работы над кодом и git-процесс |

## Требования

- **Docker** с плагином Compose — рекомендуемый путь: воспроизводит окружение
  целиком, включая ROS 2 и Gazebo, без установки их в систему.
- Либо **Ubuntu 22.04** (нативно или в WSL2) для запуска без Docker.
- Для запуска только Python-части (классификация, генерация моделей, тесты)
  достаточно Python 3.11+ на любой ОС — ROS 2 и Gazebo не нужны.

Все команды ниже выполняются из корня репозитория.

## Запуск в Docker (рекомендуется)

Полностью воспроизводимое окружение — то же, что разворачивается на сервере.

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run dev
```

Внутри контейнера доступен весь контур. `install/` и `sim/models/items/` —
build-артефакты, в гит они не попадают, поэтому на свежем checkout первые две
команды обязательны и идут именно в этом порядке:

```bash
colcon build --packages-select ros_msgs          # ROS 2 контракты -> install/
python3 scripts/build_item_models.py             # SDF-модели 11 товаров -> sim/models/
bash scripts/check_sdf.sh                         # валидность мира и моделей
WORLD=sim/worlds/cell_diverter.sdf \
  bash scripts/run_skeleton.sh box_300x200x200 B # один товар, финальный механизм
bash scripts/run_stream.sh                        # поток без перезапуска мира
```

GUI Gazebo пробрасывается через X11 (Linux) или WSLg (Windows); для проверок
без экрана используется headless-режим (`ign gazebo -s -r <world>`).

## Запуск Python-части (без ROS 2 / Gazebo)

Классификация, анализ моделей и генерация SDF работают на чистом Python.

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pytest -q                        # unit-тесты; ROS-node тесты гоняет CI и Docker-среда
python scripts/build_item_models.py
python scripts/measure_validation.py
python scripts/measure_hard_scenes.py
python scripts/measure_private_shapes.py
```

## Запуск в WSL2 (нативно, без Docker)

Альтернатива для локальной разработки на Windows. Подставьте свои имя дистрибутива
и путь к репозиторию (`<distro>`, `<repo-path>`).

```powershell
wsl --import <distro> <install-dir> <ubuntu-22.04-rootfs>.tar.gz --version 2
wsl -d <distro> -- bash <repo-path>/scripts/provision_wsl.sh   # ROS 2 + Gazebo + deps
wsl --terminate <distro>                                       # перезапуск для systemd
```

```bash
# внутри WSL, из корня репозитория. Первые две команды собирают build-артефакты
# (install/, sim/models/items/) — без них check_sdf.sh нечего проверять,
# а прогоны не находят ROS-контракты:
colcon build --packages-select ros_msgs  # ROS 2 контракты -> install/
python3 scripts/build_item_models.py     # SDF-модели 11 товаров -> sim/models/items/
bash scripts/check_sdf.sh                # валидность мира и 11 моделей
bash scripts/smoke_belt.sh               # короб физически едет по ленте 1 м/с
bash scripts/smoke_multi_item.sh         # два товара одновременно видны и имеют разные ID
```

Свежий WSL-дистрибутив нередко не резолвит DNS через WSL-шлюз (`apt-get update`
отдаёт сплошные `Ign`). Лечится один раз, до провижна:

```bash
printf '[boot]\nsystemd=true\n\n[network]\ngenerateResolvConf = false\n' > /etc/wsl.conf
printf 'nameserver 8.8.8.8\nnameserver 1.1.1.1\n' > /etc/resolv.conf
# затем из PowerShell: wsl --terminate <distro>
```

Headless-запуски Gazebo на программном GL требуют `LIBGL_ALWAYS_SOFTWARE=1`
(обоснование — `docs/decisions.md`); скрипты выставляют это сами.

## Воспроизведение прогонов

Вся случайность заведена через один явный seed, поэтому любой результат
повторяется одной командой. Команды ниже — Gazebo-прогоны: они запускаются
внутри среды из разделов выше (Docker или WSL2) и после её build-шагов
(`colcon build --packages-select ros_msgs`, `python3 scripts/build_item_models.py`).

```bash
# один товар, полный контур на финальном шибере без ручных вмешательств
WORLD=sim/worlds/cell_diverter.sdf bash scripts/run_skeleton.sh box_300x200x200 B

# несколько товаров без перезапуска Gazebo; план подачи печатается до старта
bash scripts/run_stream.sh

# матрица: все товары × N воспроизводимых ориентаций от заданного seed
WORLD=sim/worlds/cell_diverter.sdf bash scripts/run_matrix.sh 0 3

# медиана/p95 throughput и задержек по сохранённым потоковым прогонам
python3 scripts/measure_throughput.py runs/stream_run_1 runs/stream_run_2

# запись видео сквозного прогона (камера-«зритель» подсаживается на время записи)
bash scripts/record_skeleton_video.sh plate D
```

Ячейка матрицы однозначно задаётся тройкой `(seed, item_index, orient_index)`,
поэтому отдельный эпизод можно переиграть независимо. Крупные бинарники (видео,
веса) в репозиторий не кладутся — уходят в облако организаторов (`GIT.md`).

`run_stream.sh` сначала проверяет, что расстояния между товарами совместимы с
ходом ленты и временем удержания шибера. Каждый спавн объявляется контроллеру:
если товар не доехал до камеры за 8 с, система защёлкивает E-stop вместо тихого
пропуска. Каждый эпизод также сохраняет `resources.csv` и `resources.json` с
профилем CPU/RAM деревьев Gazebo, ROS и feeder. Подробная схема состояния и топиков — в
`docs/report/architecture.md`. Длинные серии, полные знаменатели надёжности,
ресурсный профиль и признаки загрязнённого Gazebo-стенда сведены в
`docs/report/soak.md`.

### Основные входные параметры прогонов

| Сценарий | Входы | Назначение |
|---|---|---|
| Один товар | `run_skeleton.sh <slug> <B\|C\|D> [spawn_x]`; `WORLD`, `SPAWN_Y`, `SPAWN_Z`, `ORIENT_{X,Y,Z,W}` | Точечный сквозной replay. Для финального механизма всегда задавать `WORLD=sim/worlds/cell_diverter.sdf` |
| Матрица | `run_matrix.sh [seed] [N] [start_item] [end_item]`; `LOGDIR`, `CELL_TIMEOUT=180` | Последовательный census 11 товаров × N ориентаций; диапазон индексов позволяет продолжить оборванный хвост |
| Поток | `run_stream.sh [slug:zone:gap_m ...]`; `SEED`, `ORIENT_INDEX`, `LOGDIR` | `gap_m` — расстояние позади предыдущего товара; первый gap равен 0. Небезопасный план отвергается до старта Gazebo |
| Потоковая матрица | `run_stream_suite.sh [seed] [start_orient] [end_orient]`; `SAME_ZONE_GAP_M` | Все 11 моделей двумя эпизодами на ориентацию; A/B-ручка меняет только same-zone gap, default 1.0 м |
| Диагностика | `MATRIX_DRY_RUN=1`, `STREAM_DRY_RUN=1`, `SAVE_DYNAMICS=1` | Показать точный план без Gazebo либо сохранить траектории ячеек матрицы |

`slug` должен совпадать с каталогом в `sim/models/items`, а ожидаемая зона — с
правилом B/C/D. `LOGDIR` лучше задавать явно для приёмочных серий: каталог
содержит план, терминальные результаты и данные для triage. Параметры механизма
(`HOLD_S`, `FIRE_LEAD_S`, `ENGAGE_CMD`) являются инженерной калибровкой, а не
обычными параметрами оператора; их изменение требует отдельного повторного
census и safety-прогона.

## Структура репозитория

```
src/          ядро perception / классификации / tracking и ROS 2 узлы
ros_msgs/     ROS 2 пакет с контрактами ItemMeasurement и ItemClassification
launch/       запуск полного контура одной командой
sim/          мир Gazebo, модели товаров, конфиг моста ros_gz
scripts/      генератор моделей, сквозные прогоны, утилиты анализа и записи
docker/       воспроизводимое окружение сборки и запуска
tests/        unit-тесты ядра и узлов
docs/         условия, решения, эксперименты, разделы отчёта
```
