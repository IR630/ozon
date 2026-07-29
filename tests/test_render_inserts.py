from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INSERTS = ROOT / "docs" / "report" / "video" / "inserts"


def test_every_insert_svg_has_a_rendered_png():
    # The SVG is the source and the PNG is what goes into the timeline. When the
    # two drift, the video keeps showing a number the documents have retired —
    # which is exactly what happened to the throughput row of the metrics card.
    for source in sorted(INSERTS.glob("*.svg")):
        png = source.with_suffix(".png")
        assert png.is_file(), f"{source.name} has no rendered PNG"
        assert png.stat().st_size > 10 * 1024, f"{png.name} looks empty"


def test_rendered_png_is_newer_than_its_source():
    for source in sorted(INSERTS.glob("*.svg")):
        png = source.with_suffix(".png")
        assert png.stat().st_mtime >= source.stat().st_mtime, (
            f"{png.name} is older than {source.name} — run scripts/render_inserts.py")
