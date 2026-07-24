# Ozon Hackathon — трек 3: роботизированная сортировка товаров

Виртуальная ячейка предсортировки: depth-камера над непрерывно идущим конвейером
видит несколько товаров, сохраняет их `item_id`, классифицирует каждый и двумя
поворотными шиберами направляет его по склизам в зоны. Полный контур — подача →
восприятие → классификация → управление → физическая маршрутизация — замкнут в
Gazebo и воспроизводится одной командой.

**Категории.** `B` — подходит для основного сортировщика; `C` — не проходит по
габаритам; `D` — требует доупаковки (обнаружен круг в сечении, `K > 0.8`).
Правила задачи — `docs/md/task.md`, эталонные модели — `docs/md/models.md`.

**Принятая версия.** `v0.5-stable-stream` — стабильная точка для отчёта и
демонстрации. Поверх неё 21.07 приняты два измеренно проверенных пакета:
feedback-gated восстановление после E-stop (Stage 26) и попиксельный пересчёт
depth-масштаба. После обоих изменений непрерывная перепись осталась **33/33**;
фикс масштаба поднял офлайн-точность габаритов с 8/11 до 11/11 и улучшил замеры
Шлема/Пуфика на кадрах Gazebo. Тег не переносится: актуальную точку называем
«v0.5 + Stage 26 + depth-scale fix». Физический occupied-smoke входит в
обязательный `e2e-smoke`; открытым остаётся resource/latency-gate двух новых
feedback-топиков на точном окружении организаторов.

**Текущий результат.** Три исторических pre-v0.5 body-scored census дали
`33/33 ×3`; два полных stream-suite v0.5 — `33/33 + 33/33`. На актуальном ядре
«v0.5 + Stage 26 + depth-scale fix» повторная непрерывная перепись также дала
**33/33**, таймаутов и execution-ошибок нет. Последний multi-seed замер
однокамерной конфигурации на depth-scale базе дал **163/165** (оба промаха —
K≈0.8, execution 0); точный `main` после последующих инфраструктурных правок
повторно проверен одним полным seed. Исторический run-to-run диапазон прежних
ревизий — 162–165/165. Устойчивая
производительность на **полной матрице всех 11 моделей** (потоковая суита
`33/33` доставлено, 6/6 эпизодов) — медиана
`10 товаров/мин` (диапазон по эпизодам `4–15`). Ранее полученные
`18 товаров/мин` — только короткий пиковый поток, а не sustained-режим, поэтому
как основная заявка не используется. Startup Gazebo и soft-start ленты в метрику
не входят (T0 = полная скорость ленты). Замер: `measure_throughput.py` по
`runs/stream_suite_*`.

**Стек.** Python 3.11+, ROS 2 Humble, Gazebo Fortress (`ros_gz`), NumPy/SciPy.
Ядро восприятия и классификации тестируется отдельно от ROS 2; ROS 2 связывает
узлы в контур, Gazebo даёт физику ленты, товаров, шиберов, склизов и камеры.

## Навигация

**Начать отсюда:** `docs/report/final_report.md` — цельный итоговый отчёт;
`docs/report/one_pager.md` — всё решение за 3 минуты.

**Видеодемонстрация:** `<ссылка на облако организаторов>` — *заполнить перед
сдачей* (крупные mp4 не в репозитории, `GIT.md`); постеры-кадры — в
`docs/report/video/`. **Презентация защиты:** структура — `docs/report/presentation_outline.md`.

| Документ | Что внутри |
|---|---|
| `docs/report/final_report.md` | ★ Итоговый отчёт: задача, обе части ПАК, связка, результаты и ограничения |
| `docs/report/one_pager.md` | ★ Резюме «прочти за 3 минуты» — точка входа для жюри |
| `docs/report/criteria_coverage.md` | Карта: каждый пункт `criteries.md` → артефакт репо |
| `docs/md/task.md` | Условия задачи трека |
| `docs/md/criteries.md` | Критерии оценки жюри |
| `docs/md/models.md` | Анализ 11 тестовых моделей |
| `docs/md/organizer_faq_2026-07-16.md` | Официальные уточнения о приватном наборе, Docker и физических допущениях |
| `docs/report/classification.md` | Правила классификации, K по проекциям, пограничные случаи, 33/33 |
| `docs/report/mechanism.md` | Выбор и обоснование исполнительного механизма (шибер), бережность |
| `docs/report/architecture.md` | Единый контур: перцепция → классификация → контроллер → шибер |
| `docs/report/calc_vs_sim.md` | Сведение расчёт ↔ симуляция (обоснование УГТ «исполнение 4») |
| `docs/report/layout.md` | Компоновка + сверка с официальной схемой участка |
| `docs/report/soak.md` | Длинные серии, надёжность, throughput, ресурсный профиль |
| `docs/report/safety.md` | E-stop, разделение зон, граница программного прототипа |
| `docs/report/physics_assumptions.md` | Что задано организаторами, а что принято по массе, трению и физике |
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

Проверка развёртывания одной командой (чистый checkout `HEAD`, сборка
запиненного образа, headless CPU сквозной прогон — тот же путь, что e2e-smoke
в CI; непроверенные локальные правки в неё не попадают — сначала коммит):

```bash
python scripts/check_submission.py   # preflight полного контура, артефактов и ссылок
bash scripts/check_clean_deploy.sh
```

До загрузки видео в облако первый preflight намеренно возвращает `BLOCKED` на
плейсхолдере ссылки в README. Это текущая человеко-задача H6, а не ошибка кода.

Версии зафиксированы двумя замками: базовый образ — по digest в
`docker/Dockerfile`, python-зависимости образа — `docker/pip-constraints.txt`
(контракт — `tests/test_deploy_pins.py`).

## Запуск Python-части (без ROS 2 / Gazebo)

Классификация, анализ моделей и генерация SDF работают на чистом Python.

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pytest -q                        # unit-тесты; ROS-node тесты — Docker/ручной CI
python scripts/build_item_models.py
python scripts/measure_validation.py
python scripts/measure_hard_scenes.py
python scripts/measure_private_shapes.py
```

### Классификация своей 3D-модели (STL)

Требование экспертной сессии (`docs/md/expert_session_qa.md`, `[35:29]`): загрузить
СВОЮ модель и прогнать полный контур — определение параметров → классификация по
правилам → вердикт. Точка входа:

```bash
# один или несколько путей к STL — печатает вердикт, эталон docs/md/models.md НЕ трогает
python scripts/analyze_models.py path/to/model.stl [другой.stl ...]

# на эталонном наборе организаторов (11 моделей из docs/Stl/):
python scripts/analyze_models.py docs/Stl/*.stl
```

Реальный вывод на наборе организаторов:

```
Бутылка: OBB [305.   91.3  91.2] K=0.996 watertight=True faces=6522 -> D: доупаковка (круг в сечении)
Короб 300х200х200: OBB [301.  200.5 200. ] K=0.720 watertight=True faces=592 -> B: подходит для сортировки
Короб 400х400х300: OBB [401.  400.  300.5] K=0.716 watertight=True faces=536 -> C: не подходит по габаритам
ЛанчБокс: OBB [201.  152.4  62.3] K=0.637 watertight=True faces=11574 -> B: подходит для сортировки
Мешок: OBB [201.6 176.2 169.6] K=0.800 watertight=False faces=21228 -> D: доупаковка (круг в сечении)
Моющее средство: OBB [279.6 259.8 179.2] K=0.708 watertight=True faces=72752 -> B: подходит для сортировки
Пуфик: OBB [488.9 488.9 264. ] K=0.994 watertight=False faces=12880 -> C: не подходит по габаритам
Ручка: OBB [148.5  13.1   9. ] K=0.591 watertight=False faces=40926 -> C: не подходит по габаритам
Тарелка: OBB [209.5 209.4  26.5] K=0.999 watertight=True faces=2504 -> D: доупаковка (круг в сечении)
Цилиндр: OBB [435.  50.  43.] K=0.749 watertight=True faces=2152 -> B: подходит для сортировки
Шлем: OBB [352.1 297.6 281.6] K=0.783 watertight=False faces=55159 -> B: подходит для сортировки
```

Под каждым товаром печатается **передача в исполнительную часть** — тот же контракт
категория→актуатор, что и на живом пути камера→классификатор→контроллер (`src/controller_node.py`):

```
    -> исполнительная часть: категория D — доупаковка, дивёртер зоны D (/pusher_d/cmd)
    -> исполнительная часть: категория C — дивёртер зоны C (/pusher_c/cmd)
    -> исполнительная часть: категория B — едет до конца ленты, без дивёрта
```

Так контур замыкается сквозь: загрузка STL → параметры → классификация → **команда
исполнительной части** (имитация, `docs/md/expert_session_qa.md` `[35:29]`–`[36:12]`).

Габариты — по OBB (мм, по убыванию), K — максимум `r_впис/R_опис` по трём проекциям
тем же оценщиком, что и прод (`src.perception._roundness_k`); категория — по правилам
`task.md` из порогов `src/constants.py`. Без аргументов скрипт вместо печати
**перегенерирует** `docs/md/models.md` (эталонный прогон) — для разовой проверки
всегда передавайте пути.

> **Расхождение STL-путь ↔ камера (честно).** Этот путь считает категорию по чистой
> геометрии меша. Прод-камера может решать иначе там, где вмешивается политика: напр.
> **Мешок** здесь `K=0.800 → D`, а прод сознательно держит его в **B** (мягкий ком не
> катится; провенанс в `docs/decisions.md`). Это задокументированное расхождение, а не
> рассинхрон: STL-путь показывает геометрию, прод — политику.

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

# Stage 26 поверх стабильной v0.5:
# занятый механизм → E-stop → ручная очистка → feedback-gated парковка → свежий B
bash scripts/smoke_estop_stream.sh

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
`docs/report/architecture.md`. Регрессионные серии, полные знаменатели,
ограниченный ресурсный прогон и признаки загрязнённого Gazebo-стенда сведены в
`docs/report/soak.md`.

Начиная со Stage 26 позиционный шибер удерживается по измеренному углу только
при наличии свежего `JointState`, полученного после последней команды. Если
такого подтверждения нет, лента остаётся остановленной, контроллер сохраняет
последнюю цель и явно сообщает, что физическое удержание не подтверждено.

### Воспроизведение заявленных результатов (одна команда на заявку)

Каждая цифра из «Текущий результат» воспроизводится одной командой + seed. Все
команды — внутри среды (Docker/WSL2) после build-шагов.

| Заявка | Команда | Проверка результата |
|---|---|---|
| Классификация + маршрутизация `33/33` | `WORLD=sim/worlds/cell_diverter.sdf bash scripts/run_matrix.sh 0 3` | census 11×3; `scripts/census_ruler_diff.py` |
| Потоковая матрица всех моделей `33/33`, 6/6 all-pass | `bash scripts/run_stream_suite.sh 0 0 2` | вердикты эпизодов; `runs/stream_suite_*` |
| Sustained производительность `медиана 10 шт/мин` | `python3 scripts/measure_throughput.py runs/stream_suite_<...>` | steady-state median/p95, `calc_vs_sim.md` якорь 5 |
| Камера→решение `0.030 с` (синхронизация) | тот же `measure_throughput.py` | per-stage latency, один sim-clock |
| Чистое развёртывание (headless CPU) | `bash scripts/check_clean_deploy.sh` | чистый checkout HEAD → образ → `ci_e2e.sh` PASS |

Тройка `(seed, item_index, orient_index)` однозначно задаёт ячейку матрицы —
любой отдельный эпизод переигрывается независимо.

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

### Структура решения по частям

Разбиение по требованию сдачи (`task.md` §4): что относится к классификации, к
исполнительной части, к связке и к сценариям проверки.

| Часть | Код / артефакты | Раздел отчёта |
|---|---|---|
| **Определение и классификация** | `src/perception.py` (габариты, K по проекциям из depth), `src/aggregation.py` (стабилизация по кадрам), `src/classification.py` (правила B/C/D), `src/constants.py` (пороги), узлы `perception_node`/`classifier_node` | `classification.md`, `validation.md` |
| **Исполнительная часть** | `sim/worlds/cell_diverter.sdf` (шиберы, склизы, зоны), `src/controller_node.py` (тайминг, возврат, E-stop, jam/feed-watchdog), `src/tracking.py` (dead-reckoning выстрела) | `mechanism.md`, `layout.md`, `safety.md` |
| **Связка (классификация → воздействие)** | контроллер подписан на `ItemClassification` → `plan_push` (зона→шибер, упреждение по скорости ленты) → команда механизму; контракты в `ros_msgs/` | `architecture.md`, `calc_vs_sim.md` |
| **Сценарии проверки** | `run_matrix.sh` (census), `run_stream_suite.sh` (поток), `check_clean_deploy.sh` (деплой), `smoke_estop*.sh` (безопасность), `measure_*.py` (метрики) | `soak.md`, `failure_scenarios.md`, `criteria_coverage.md` |
