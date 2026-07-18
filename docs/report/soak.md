# Soak и регрессия полного контура

Этот раздел сводит длинные серии после принятия прозрачных ограждений и 3D
body-OBB. Разные сценарии не складываются в один «процент качества»: census
измеряет все модели и ориентации, stream — совместную работу товаров, safety-smoke
— безопасную реакцию. Для каждого набора сохраняется собственный знаменатель.

## Актуальная сводка

| Набор | Эпизоды | Товары / проверки | Результат | Сырые каталоги |
|---|---:|---:|---|---|
| Stage 24 stability `main@f4d3540`: seed0 repeat + seed1/oi0 | 6/8 all-pass | **42/44** terminal; roster 44/44/44 | 2 execution FAIL: только Pouf seed0 oi0/oi1; TIMEOUT/JAM/E-stop 0; seed1/oi0 11/11 | `runs/week4_stability_{seed0_repeat,seed1_oi0}_f4d3540` |
| Helmet ID gate `main@c91c541`: Pouf→Helmet ×2 + census + occupied E-stop | 2/2 handoff + 1 census + 1 safety | handoff 4/4; census 33/33; E-stop 2 товара + engaged blade | roster 2/2/2 в обоих handoff; triage 0/0; `dx=0`, `FIRED 2→2` | `runs/week4_{pouf_helmet_idfix_replay*,idfix_census,idfix_estop}_c91c541` |
| Stream-suite `main@3f3a7ed`, 11 моделей × 3 ориентации | 6/6 all-pass | 33/33 terminal PASS; roster 33/33/33 | 0 FAIL/TIMEOUT/JAM; без phantom ID; поэпизодный темп 3–12 шт/мин | `runs/week4_suite_gate_3f3a7ed_seed0_oi{0,1,2}` |
| Stream-suite перемер `main@8a728f9` (throughput), 11 моделей × 3 ориентации | 6/6 all-pass | 33/33 terminal PASS; 27 интервалов | 0 FAIL; **sustained-медиана 10 шт/мин** (p95-медл. такт 4, поэпизодно 4–15); камера→решение медиана 0.030 с | `runs/stream_suite_20260717_123705_seed0` |
| Freeze-census текущего `main@6618ded`, 11 моделей × 3 ориентации | 1/1 all-pass | 33/33 routed; каждая модель 3/3 | classification 0, execution 0, TIMEOUT 0; triage 33/33 | `runs/week3_freeze_main_20260715` |
| Два census слитого main, 11 моделей × 3 ориентации | 2/2 all-pass | 66/66 routed; каждая модель 6/6; B 36/36, C 18/18, D 12/12 | 33/33 + 33/33; classification 0, execution 0, ни одна ячейка не разошлась | `runs/census_merged_1`, `runs/census_merged_2` |
| Пограничный replay Pouf/Pen/Plate/Cylinder/Helmet × 3 | 1/1 all-pass | 15/15 routed; каждая модель 3/3 | classification 0, execution 0, TIMEOUT 0 | `runs/week3_boundary_main_20260715` |
| Штатный поток, seeds 0/2/3/4 | 4/4 all-pass | 12/12 routed | камера→решение median 0.094 с, p95 0.186 с | `runs/stream_lat_seed*` |
| Плотный пяти-товарный контрольный план | 5/5 all-pass | 25/25 routed; каждый маршрут 5/5 | старый Lunchbox B→D в этой серии не воспроизведён и остаётся историческим ограничением | `runs/week3_dense_{baseline,repeat2..5}_20260715_seed0` |
| Safety после синхронизации main | 2/2 сценария | multi-item: 2/2 в B; occupied E-stop: 2 товара + engaged blade неподвижны | отдельные `item_id`; `FIRED` не растёт после E-stop | `runs/week3_post_sync_estop_20260715` |
| Safety freeze-кандидата | 2/2 сценария | multi-item: 2/2 в B; occupied E-stop: 2 товара + engaged blade неподвижны | `ids=[1,2]`; оба `dx=0`; C-blade 0.710→0.709 рад; `FIRED` 2→2 | `runs/week3_freeze_estop_20260715` |

Frozen real-frame набор отдельно даёт visible contract **8/8** и expected reject
**4/4** (`scripts/measure_validation.py`). Это регрессия perception, не физический
soak, поэтому она не включена в routed-знаменатели таблицы.

## Учёт отказов

В Stage 24 есть два терминальных физических FAIL, и оба оставлены в знаменателе.
Pouf seed0 oi0/oi1 имел полный roster, верную категорию C и `pusher_c FIRED`, но
не осел в body-окне зоны; oi2 и новый seed1/oi0 прошли. Поэтому прежний 33/33
остаётся фактом отдельного запуска, но не гарантией повторяемости. В остальных
актуальных наборах таблицы терминальных FAIL/INVALID/TIMEOUT нет. Старые неудачные
попытки также не стираются: исправленный аудит сохранённых потоков дал **6/10
all-pass, 27/36 routed**. Плотный Lunchbox B→D не выдан за исправленный без
различающего trace; каждый безопасный план назван явно.

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
- Численный минимум RAM/CPU как production-требование не заявляется. Каждый новый
  `run_stream.sh` снимает раз в секунду деревья Gazebo, ROS launch и feeder в
  `resources.csv`, а в `resources.json` сохраняет peak RSS, среднее/пиковое число
  занятых CPU-ядер и максимум процессов. На текущем Windows-хосте нет WSL/Docker,
  поэтому реальный профиль Fortress этим инструментом ещё не снят и Stage 24 не
  закрыт; репозиторий не подменяет отсутствующее измерение расчётным минимумом.

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
