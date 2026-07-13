# Ozon Hackathon — трек 3: роботизированная сортировка товаров

Виртуальная ячейка предсортировки: камера над конвейером классифицирует товар и
исполнительный механизм физически раскладывает его по зонам. Полный контур —
восприятие → классификация → управление → манипуляция — замкнут в симуляции и
воспроизводится одной командой.

**Категории.** `B` — подходит для основного сортировщика; `C` — не проходит по
габаритам; `D` — требует доупаковки (обнаружен круг в сечении, `K > 0.8`).
Правила задачи — `docs/md/task.md`, эталонные модели — `docs/md/models.md`.

**Стек.** Python 3.11+, ROS 2 Humble, Gazebo Fortress (`ros_gz`). Ядро восприятия
и классификации не зависит от OpenCV и тестируется автономно; ROS 2 связывает узлы
в контур, Gazebo даёт физику ленты, толкателей и камеры.

## Навигация

| Документ | Что внутри |
|---|---|
| `docs/md/task.md` | Условия задачи трека |
| `docs/md/criteries.md` | Критерии оценки жюри |
| `docs/md/models.md` | Анализ 11 тестовых моделей |
| `docs/report/` | Разделы итогового отчёта (архитектура, классификация, методология и ограничения) |
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

Внутри контейнера доступен весь контур:

```bash
python3 scripts/build_item_models.py            # SDF-модели 11 товаров -> sim/models/
bash scripts/check_sdf.sh                        # валидность мира и моделей
bash scripts/run_skeleton.sh box_300x200x200 B   # сквозной прогон одного товара
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
# внутри WSL, из корня репозитория:
bash scripts/check_sdf.sh     # валидность мира и 11 моделей
bash scripts/smoke_belt.sh    # короб физически едет по ленте 1 м/с
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
повторяется одной командой.

```bash
# один товар в заданной категории, полный контур без ручных вмешательств
bash scripts/run_skeleton.sh box_300x200x200 B

# матрица: все товары × N воспроизводимых ориентаций от заданного seed
WORLD=sim/worlds/cell_diverter.sdf bash scripts/run_matrix.sh 0 3

# запись видео сквозного прогона (камера-«зритель» подсаживается на время записи)
bash scripts/record_skeleton_video.sh plate D
```

Ячейка матрицы однозначно задаётся тройкой `(seed, item_index, orient_index)`,
поэтому отдельный эпизод можно переиграть независимо. Крупные бинарники (видео,
веса) в репозиторий не кладутся — уходят в облако организаторов (`GIT.md`).

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
