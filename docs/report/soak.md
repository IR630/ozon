# Soak и регрессия полного контура

Этот раздел сводит регрессионные серии и ограниченный ресурсный прогон после
принятия прозрачных ограждений и 3D body-OBB. Разные сценарии не складываются
в один «процент качества»: census измеряет все модели и ориентации, stream —
совместную работу товаров, safety-smoke — безопасную реакцию. Для каждого
набора сохраняется собственный знаменатель.

## Актуальная сводка

| Набор | Эпизоды | Товары / проверки | Результат | Сырые каталоги |
|---|---:|---:|---|---|
| Однокамерный multi-seed на depth-scale базе (**22.07**, fallback-путь `feat/three-cameras@4581d8d`) | 5/5 census run | **163/165** routed (33/32/33/33/32) | 2 классификационных промаха, execution 0: Шлем oi=1 seed1 и Мешок oi=2 seed4, оба K≈0.8 → D. Двух- и трёхкамерный multi-seed не выполнялся | `runs/a2sweep_20260722_*_seed{0..4}` |
| Мульти-seed census (**21.07, ядро С E-stop** `bf891af`), seeds 0–4, 11 моделей × 3 ориентации | 5/5 census run | **162/165** routed (33/32/33/33/31) | **3 классификационных промаха** (execution 0), ВСЕ околопороговые по K: Шлем oi=1 seed1 и seed4, Мешок oi=2 seed4 — все K=0.80 → D у порога 0.8; **Pouf 15/15**. Перепрогон Шлема seed1 oi1 ×8 → B 7/8 (недетерм.) | `runs/estop_merge_census_seed0`, `runs/estop_sweep_seed{1,2,3,4}` |
| Мульти-seed census (20.07, ядро ДО E-stop `c12ba2b`), seeds 0–4 | 5/5 census run | **164/165** routed | 1 промах: Мешок oi=2 seed4 (K=0.8004→D, порог 0.8); execution 0; **Pouf 15/15** | `runs/baseline_main_seed0`, `runs/sweep_main_seed{1,2,3,4}` |
| Мульти-seed census (19.07), seeds 0–4 — на ядре ДО фикса K (`fa4d30e`, ветка-не-потомок) | 5/5 census all-pass | 165/165 routed (историческое, не текущее ядро) | classification 0, execution 0; **Pouf 15/15** | `runs/sweep_seed{0,1,2,3,4}` |
| Финальный gate ядра `v0.5-stable-stream` (`main@cafbbd4` + хвост): два полных stream-suite подряд + census | 2×6/6 all-pass + census | **33/33 + 33/33** terminal PASS; census 32/32 + helmet-хвост 3/3 после обрыва сессии | roster 5/5/5 mixed и 6/6/6 B во всех эпизодах; 0 FAIL/TIMEOUT/JAM; triage classification 0, execution 0 | `runs/pouf_fix_final_*_cafbbd4`, `runs/b_oi1_poll200_cafbbd4` |
| Ресурсный профиль худшего mixed oi1 + восстановление после E-stop (`main@424a0a2`) | 1 поток + 2 смока | 5/5 PASS; recovery 2/2 | peak RSS 944 MiB; CPU 2.8 ср. / 4.4 пик ядра; ≤12 процессов за 48 с; после сброса E-stop повторный soft-start и доезд в B без повторной защёлки | `runs/resource_mixed_oi1_424a0a2`, `runs/estop_recover_r{2,3}` |
| Stage 24 stability `main@f4d3540`: seed0 repeat + seed1/oi0 | 6/8 all-pass | **42/44** terminal; roster 44/44/44 | 2 execution FAIL: только Pouf seed0 oi0/oi1; TIMEOUT/JAM/E-stop 0; seed1/oi0 11/11 | `runs/week4_stability_{seed0_repeat,seed1_oi0}_f4d3540` |
| Helmet ID gate `main@c91c541`: Pouf→Helmet ×2 + census + occupied E-stop | 2/2 handoff + 1 census + 1 safety | handoff 4/4; census 33/33; E-stop 2 товара + engaged blade | roster 2/2/2 в обоих handoff; triage 0/0; `dx=0`, `FIRED 2→2` | `runs/week4_{pouf_helmet_idfix_replay*,idfix_census,idfix_estop}_c91c541` |
| Stream-suite `main@3f3a7ed`, 11 моделей × 3 ориентации | 6/6 all-pass | 33/33 terminal PASS; roster 33/33/33 | 0 FAIL/TIMEOUT/JAM; без phantom ID; поэпизодный темп 3–12 шт/мин | `runs/week4_suite_gate_3f3a7ed_seed0_oi{0,1,2}` |
| Stream-suite перемер `main@8a728f9` (throughput), 11 моделей × 3 ориентации | 6/6 all-pass | 33/33 terminal PASS; 27 интервалов | 0 FAIL; **sustained-медиана 10 шт/мин** (p95-медл. такт 4, поэпизодно 4–15); камера→решение медиана 0.030 с (steady-state, seed 0, полная suite) | `runs/stream_suite_20260717_123705_seed0` |
| Freeze-census `main@6618ded` (15.07), 11 моделей × 3 ориентации | 1/1 all-pass | 33/33 routed; каждая модель 3/3 | classification 0, execution 0, TIMEOUT 0; triage 33/33 | `runs/week3_freeze_main_20260715` |
| Два census слитого main, 11 моделей × 3 ориентации | 2/2 all-pass | 66/66 routed; каждая модель 6/6; B 36/36, C 18/18, D 12/12 | 33/33 + 33/33; classification 0, execution 0, ни одна ячейка не разошлась | `runs/census_merged_1`, `runs/census_merged_2` |
| Пограничный replay Pouf/Pen/Plate/Cylinder/Helmet × 3 | 1/1 all-pass | 15/15 routed; каждая модель 3/3 | classification 0, execution 0, TIMEOUT 0 | `runs/week3_boundary_main_20260715` |
| Штатный поток, seeds 0/2/3/4 | 4/4 all-pass | 12/12 routed | камера→решение median 0.094 с, p95 0.186 с (прямой 4-seed замер с хвостом; headline 0.030 с — steady-state seed 0, полная suite; согласование — §2.7 methodology) | `runs/stream_lat_seed*` |
| Плотный пяти-товарный контрольный план | 5/5 all-pass | 25/25 routed; каждый маршрут 5/5 | старый Lunchbox B→D в этой серии не воспроизведён и остаётся историческим ограничением | `runs/week3_dense_{baseline,repeat2..5}_20260715_seed0` |
| Safety после синхронизации main | 2/2 сценария | multi-item: 2/2 в B; occupied E-stop: 2 товара + engaged blade неподвижны | отдельные `item_id`; `FIRED` не растёт после E-stop | `runs/week3_post_sync_estop_20260715` |
| Safety freeze-кандидата | 2/2 сценария | multi-item: 2/2 в B; occupied E-stop: 2 товара + engaged blade неподвижны | `ids=[1,2]`; оба `dx=0`; C-blade 0.710→0.709 рад; `FIRED` 2→2 | `runs/week3_freeze_estop_20260715` |

Frozen real-frame набор отдельно даёт visible contract **8/8** и expected reject
**4/4** (`scripts/measure_validation.py`). Это регрессия perception, не физический
soak, поэтому она не включена в routed-знаменатели таблицы.

Термин `sustained` для headline-метрики означает шесть свежих полных эпизодов
suite с общим знаменателем 33/33, а не один непрерывный процесс на десятках
товаров. Same-process soak на 30–50 товарах остаётся backlog Stage 27. Финальный
census Stage 24 также не выдаётся за непрерывный: после 32 успешных результатов
сессия оборвалась, затем весь Helmet-хвост был повторён 3/3; объединение покрывает
33 уникальные ячейки, а повторные наблюдения не складываются в новый процент.

## Учёт отказов

В Stage 24 есть два терминальных физических FAIL, и оба оставлены в знаменателе.
Pouf seed0 oi0/oi1 имел полный roster, верную категорию C и `pusher_c FIRED`, но
не осел в body-окне зоны; oi2 и новый seed1/oi0 прошли. Поэтому прежний 33/33
остаётся фактом отдельного запуска, но не гарантией повторяемости.

Последний однокамерный multi-seed на depth-scale базе дал **163/165**; точный
последующий `main` отдельно подтверждён seed-0 переписью 33/33. Мульти-seed по
пяти seed’ам — величина **run-to-run, а не константа**: 165/165
(19.07, ядро ДО фикса K, `fa4d30e`), 164/165 (20.07, ядро до E-stop, `c12ba2b`),
**162/165 (21.07, ядро С E-stop, `bf891af`)**. Заявка честнее диапазоном
**162–165/165**, а не одним числом. Инвариант через все три прогона: **execution
0, а каждый промах — околопороговый по K товар** (Шлем K≈0.78, Мешок K=0.800),
у которого силуэтная K колеблется вокруг порога 0.8 из-за оседания Gazebo и
изредка уходит в D. На ядре с E-stop таких промахов три (Шлем seed1/seed4, Мешок
seed4) — и это **не регресс safety-пакета**: он трогает `controller_node.py`, а
`perception.py`, где считается K, между прогонами байт-в-байт тот же; seed 0/2/3
дали чистые 33/33.

Недетерминированность промахов доказана перепрогоном, а не заявлена:
**Мешок seed4 oi2 → B 8/8** (габариты осели `197×174×166` против `195×183×159` в
переписи) и **Шлем seed1 oi1 → B 7/8** — обе ячейки, провалившиеся в переписи,
при повторе проходят в подавляющем большинстве. То есть классификатор устойчив,
недетерминирована игра оседания Gazebo — та же «±2–3 ячейки шума», что и всегда.
Это НЕ то же самое, что детерминизм при
фиксированном seed — каждый seed прогонялся один раз, а Stage-24-отказ был
непостоянством одного и того же seed 0 между прогонами Gazebo. Две оси вместе
читаются так: routing Pouf корректен в подавляющем большинстве прогонов, с редким
недетерминированным промахом исполнения при конкретных ориентациях, который
консервативный зазор смены зон (3.1 м) и body-scored вердикт и призваны
поглощать. Чтобы закрыть и вторую ось, нужен повтор одного seed ×N, не сделан. В остальных
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
  Python 3.11+. Точные версии образа задают `docker/Dockerfile` и
  `docker/pip-constraints.txt`; bare-имена в `requirements.txt` оставлены для
  dev/CI и не являются lock-файлом образа.
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
  занятых CPU-ядер и максимум процессов. Реальный профиль Fortress снят 17.07 на
  худшем mixed oi1 (см. таблицу выше: peak RSS 944 MiB, CPU 2.8 ср. / 4.4 пик
  ядра; к концу прогона RSS наблюдался на плато 853 MiB); измеренный факт не
  превращается в заявленный минимум. Этого профиля достаточно для факта, но
  **недостаточно для вывода об отсутствии утечки** в длительном непрерывном
  процессе: нужен отдельный same-process soak на 30–50 товарах, он не сделан.

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
