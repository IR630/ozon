# Перечень допустимого программного обеспечения

> Конвертировано из `docs/software.pdf`. Проверка решений выполняется в серверной среде организаторов.

## 1. Разрешённое проприетарное ПО (закрытый перечень)

- **AnyLogic** — имитационное моделирование процессов (дискретно-событийное, агентное, системная динамика).
- **КОМПАС-3D** — трёхмерное проектирование (CAD), работа со сборками и чертежами; читает STEP/STL.
- **nanoCAD** — проектирование и инженерное черчение (CAD).

**Что это означает:** серверная среда проверки готовится под ПО из перечня — это предпочтительный путь. Иное проприетарное ПО — под ответственность команды; при невозможности воспроизведения решение может быть рассмотрено не в полном объёме. Для добавления ПО в перечень — заблаговременный запрос организаторам.

**Итоговые файлы — в универсальных форматах:**

| Тип | Форматы |
|---|---|
| 3D-модели и CAD | STEP, STL, IGES, glTF/OBJ |
| Документы, чертежи, отчёты | PDF |
| Данные, метрики, таблицы | CSV, JSON |
| Видеодемонстрации | MP4 |
| Модели машинного обучения | ONNX |
| Модели роботов и сцены симуляции | URDF, SDF, FMU/FMI |
| Изображения | PNG, JPG |

## 2. Рекомендуемое open-source ПО (открытый перечень)

**Имитационное моделирование процессов:** SimPy, Salabim, JaamSim, Ciw, DESMO-J, ProM, Google OR-Tools

**Трёхмерное проектирование (CAD):** FreeCAD, Blender

**Машинное зрение и машинное обучение (CV/ML):** OpenCV, PyTorch, TensorFlow / Keras, ONNX Runtime, scikit-image, scikit-learn, Ultralytics YOLO, RT-DETR, YOLOX, Detectron2, MMDetection, Segment Anything (SAM), GroundingDINO, DINOv2, CLIP, SigLIP, timm

**Инженерные расчёты, многотельная динамика и мехатроника:** OpenModelica + OMSimulator, Project Chrono / PyChrono, Drake, FreeDyn, CalculiX, Code_Aster

**Робототехника, симуляция и виртуальная наладка:** Gazebo, Webots, MuJoCo, PyBullet, ROS 2 + MoveIt, NVIDIA Isaac Sim

**Работа с 3D-моделями, геометрией и штрихкодами:** Open3D, Trimesh, pyzbar, ZXing

**Разметка данных:** CVAT, Label Studio

**Базовое ПО и инфраструктура:** Python, Docker, Git, инструменты сборки C/C++ (GCC/Clang, CMake), Node.js, Jupyter, NumPy / SciPy / pandas

**Электроника и схемотехника:** KiCad + ngspice, LTspice
