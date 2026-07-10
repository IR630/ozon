# Ozon Hackathon — трек 3: роботизированная сортировка товаров

Виртуальная ячейка предсортировки: камера над конвейером классифицирует товар
(B — сортируемый, C — не по габаритам, D — доупаковка) и исполнительный механизм
физически раскладывает его по зонам. Симуляция: Gazebo Fortress + ROS 2 Humble.

## Навигация

| Документ | Что внутри |
|---|---|
| `PLAN.md` | Стратегия, календарь этапа до 02.08, рабочие потоки |
| `PLAN-7DAYS.md` | Детальный план текущей недели |
| `CLAUDE.md` | Правила работы: код, тесты, ревью, принципы |
| `GIT.md` | Git-процесс: коммиты, ветки, откаты |
| `docs/md/` | Условия задачи, критерии, анализ тестовых моделей |
| `docs/decisions.md` | Журнал инженерных решений |
| `docs/experiments.md` | Журнал экспериментов и прогонов |

## Быстрый старт: Python-часть (классификация, анализ моделей)

```bash
python -m venv .venv
.venv/Scripts/activate        # Windows; на Linux: source .venv/bin/activate
pip install -r requirements.txt
pytest -q                     # 35 тестов зелёные
python scripts/build_item_models.py   # SDF-модели 11 товаров -> sim/models/items/
```

## Быстрый старт: симуляция (Docker)

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run dev
# внутри контейнера:
python3 scripts/build_item_models.py
ign gazebo sim/worlds/cell.sdf                  # мир участка (GUI)
ign gazebo -s -r sim/worlds/cell.sdf            # headless
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/ws/sim/bridge.yaml
python3 scripts/dump_camera.py --out out        # PNG кадры с камеры
```

## Структура

```
src/          ядро: константы домена и правила классификации (+ тесты в tests/)
scripts/      анализ STL, генератор SDF-моделей, утилиты
ros_msgs/     ROS 2 пакет с контрактом ItemClassification (CV -> контроллер)
sim/          мир Gazebo, модели, конфиг моста ros_gz
docker/       среда разработки и запуска
docs/         условия, решения, эксперименты, отчёт
```
