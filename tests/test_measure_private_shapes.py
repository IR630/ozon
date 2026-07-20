import measure_private_shapes as private_shapes


def test_procedural_private_set_uses_no_released_model_identity():
    cases = private_shapes.build_cases()

    assert len(cases) == 20
    assert {case.expected for case in cases} == {"B", "C", "D"}
    assert all("box" not in case.name and "pouf" not in case.name for case in cases)


def test_production_depth_pipeline_classifies_every_procedural_private_case():
    results = private_shapes.evaluate(private_shapes.build_cases())

    assert len(results) == 20
    assert all(result.actual == result.expected for result in results), results
    assert all(result.actual != "NO_DETECTION" for result in results)


def test_k_threshold_resolves_on_both_sides():
    """The K=0.8 roundness threshold (Шлем 0.78 / Цилиндр 0.74 neighborhood):
    a case measured just under stays B, one just over goes D."""
    by_name = {r.name: r for r in private_shapes.evaluate(private_shapes.build_cases())}

    below = by_name["k_just_below_threshold_B"]
    assert below.k <= 0.8 and below.actual == "B", below

    above = by_name["k_just_above_threshold_D"]
    assert above.k > 0.8 and above.actual == "D", above


def test_size_priority_overrides_roundness():
    """docs/md/task.md: an item that is round AND out of size limits is C, not D
    (габариты приоритетнее формы). This case is genuinely round (K > 0.8) yet its
    second dimension exceeds 320 mm, so it must route C."""
    by_name = {r.name: r for r in private_shapes.evaluate(private_shapes.build_cases())}

    trap = by_name["round_but_oversized_priority_C"]
    assert trap.k > 0.8, trap                 # the shape really is round
    assert max(trap.dims_mm) > 320.0, trap    # and really oversized
    assert trap.actual == "C", trap           # size wins over form
