from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_demo_assets_are_linked_and_valid():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    video = ROOT / "docs/assets/task2-demo.mp4"
    preview = ROOT / "docs/assets/task2-demo-preview.webp"
    preview_bytes = preview.read_bytes()

    assert "## Demo" in readme
    assert "[full demo video](docs/assets/task2-demo.mp4)" in readme
    assert "docs/assets/task2-demo-preview.webp" in readme
    assert video.read_bytes()[4:8] == b"ftyp"
    assert preview_bytes[:4] == b"RIFF"
    assert preview_bytes[8:12] == b"WEBP"
    assert preview_bytes.count(b"ANMF") > 1
