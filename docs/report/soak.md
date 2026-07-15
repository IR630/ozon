# Soak и регрессия полного контура

Этот раздел сводит длинные серии после принятия прозрачных ограждений и 3D
body-OBB. Разные сценарии не складываются в один «процент качества»: census
измеряет все модели и ориентации, stream — совместную работу товаров, safety-smoke
— безопасную реакцию. Для каждого набора сохраняется собственный знаменатель.

## Актуальная сводка

| Набор | Эпизоды | Товары / проверки | Результат | Сырые каталоги |
|---|---:|---:|---|---|
| Два census слитого main, 11 моделей × 3 ориентации | 2/2 all-pass | 66/66 routed; каждая модель 6/6; B 36/36, C 18/18, D 12/12 | 33/33 + 33/33; classification 0, execution 0, ни одна ячейка не разошлась | `runs/census_merged_1`, `runs/census_merged_2` |
| Пограничный replay Pouf/Pen/Plate/Cylinder/Helmet × 3 | 1/1 all-pass | 15/15 routed; каждая модель 3/3 | classification 0, execution 0, TIMEOUT 0 | `runs/week3_boundary_main_20260715` |
| Штатный поток, seeds 0/2/3/4 | 4/4 all-pass | 12/12 routed | камера→решение median 0.094 с, p95 0.186 с | `runs/stream_lat_seed*` |
| Плотный пяти-товарный контрольный план | 5/5 all-pass | 25/25 routed; каждый маршрут 5/5 | старый Lunchbox B→D в этой серии не воспроизведён и остаётся историческим ограничением | `runs/week3_dense_{baseline,repeat2..5}_20260715_seed0` |
| Safety после синхронизации main | 2/2 сценария | multi-item: 2/2 в B; occupied E-stop: 2 товара + engaged blade неподвижны | отдельные `item_id`; `FIRED` не растёт после E-stop | `runs/week3_post_sync_estop_20260715` |

Frozen real-frame набор отдельно даёт visible contract **8/8** и expected reject
**4/4** (`scripts/measure_validation.py`). Это регрессия perception, не физический
soak, поэтому она не включена в routed-знаменатели таблицы.

## Учёт отказов

В актуальных наборах таблицы терминальных физических FAIL/INVALID/TIMEOUT нет.
Это не стирает старые неудачные попытки: исправленный анализ сохранённых потоков
считает оборванные эпизоды в знаменателе и дал исторический аудит **6/10 all-pass,
27/36 routed**. Плотный Lunchbox B→D не выдан за исправленный без различающего
trace; штатный безопасный план и контрольная серия названы явно.

Перед двумя завершёнными `census_merged_*` одна попытка цепочки завершилась до
появления roster/status (нулевой лог, процессов уже не было). Это отдельный
инфраструктурный INVALID-запуск, а не физическая ячейка и не скрытый PASS; именно
после него длинный runner получил heartbeat/termination ledger. Два 33/33 —
числитель завершённых census, а не утверждение «все команды запуска успешны».

Для нового каталога обязательны три проверки:

```bash
python3 scripts/triage_matrix.py --logdir runs/<matrix>
python3 scripts/measure_throughput.py runs/<stream-root>
find runs/<matrix> -name 'matrix_*.log' | wc -l
```

`PASS/FAIL` означает измеренную физику. Сбой runner, feeder, create-service или
body-verdict помечается `INVALID`/`INCOMPLETE` и остаётся в знаменателе, а не
маскируется под физический FAIL или исчезает из статистики.

## Проверенная среда и длительность

- Ubuntu 22.04 (WSL2 или Docker), ROS 2 Humble, Gazebo Fortress 6.18,
  системный Python 3.10, NumPy/SciPy/OpenCV; host-only тесты поддерживают
  Python 3.11+. Точные пакеты задают `docker/Dockerfile` и
  `requirements.txt`.
- Headless-прогоны используют software rendering (`LIBGL_ALWAYS_SOFTWARE=1`).
- На этой WSL-машине пограничная матрица заняла 592.5 с на 15 ячеек — около
  39.5 с/ячейку; полный census 33 ячейки следует планировать примерно на
  22 минуты плюс triage.
- Одновременно допускается только один Gazebo-контур. Docker с
  `network_mode: host`, WSL и host-процесс разделяют Ignition Transport и могут
  отвечать на сервисы чужого эпизода.
- Минимум RAM/CPU как production-требование не заявляется: пик RSS отдельно не
  измерен. Репозиторий фиксирует воспроизводимую программную среду и измеренное
  время, а не выдуманный аппаратный минимум.

## Как распознать загрязнённый стенд

До старта на локальной машине должны быть пусты:

```bash
pgrep -af '^ign gazebo( |$)'
pgrep -af 'skeleton.launch|parameter_bridge|src\..*_node'
docker ps
```

Признаки загрязнения: несколько `ign gazebo`, неожиданный `Service call timed
out`, ответ create от другого мира, ранний TIMEOUT сразу после предыдущей ячейки,
расхождение напечатанного `world:` с планом или число `matrix_*.log`, не равное
roster. В таком случае серия не ретраится поверх того же каталога: текущий runner
останавливают, ждут пустых процессов, создают новый `LOGDIR` и повторяют весь
различающий набор. Глобальный `pkill` нельзя выполнять, если на этой машине
действительно идёт согласованный эксперимент другого участника.

## Воспроизведение

```bash
# Полный census: свежий Gazebo на каждую ячейку
LOGDIR=runs/census_repeat bash scripts/run_matrix.sh 0 3

# Пограничная хвостовая группа
LOGDIR=runs/boundary_repeat bash scripts/run_matrix.sh 0 3 6 10

# Физические safety-сценарии — строго последовательно
bash scripts/smoke_multi_item.sh
LOGDIR=runs/estop_repeat bash scripts/smoke_estop_stream.sh
```

Runtime-логи остаются в gitignored `runs/` или командном хранилище; условия,
знаменатели и выводы фиксируются в `docs/experiments.md`.
