import measure_private_shapes as private_shapes


def test_procedural_private_set_uses_no_released_model_identity():
    cases = private_shapes.build_cases()

    assert len(cases) == 12
    assert {case.expected for case in cases} == {"B", "C", "D"}
    assert all("box" not in case.name and "pouf" not in case.name for case in cases)


def test_production_depth_pipeline_classifies_every_procedural_private_case():
    results = private_shapes.evaluate(private_shapes.build_cases())

    assert len(results) == 12
    assert all(result.actual == result.expected for result in results), results
    assert all(result.actual != "NO_DETECTION" for result in results)
