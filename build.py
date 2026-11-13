
from pathlib import Path
import sys

ROOT = Path(__file__).parent
SHARED = ROOT / "shared" / "driver.py"

PROJECTS = ["LeftMain", "RightMain"]
IMPORT_LINE = "from shared.driver import *"


def build(project):
    source = ROOT / project / "src" / "main.py"
    output = ROOT / project / "src" / "main_built.py"

    if not SHARED.exists():
        raise FileNotFoundError(SHARED)

    if not source.exists():
        raise FileNotFoundError(source)

    main_code = source.read_text()
    driver_code = SHARED.read_text()

    lines = main_code.splitlines(keepends=True)

    matches = [
        i for i, line in enumerate(lines)
        if line.strip() == IMPORT_LINE
    ]

    if len(matches) != 1:
        raise ValueError(
            f"{project}: expected exactly one "
            f"'{IMPORT_LINE}' import, found {len(matches)}"
        )

    index = matches[0]

    lines[index] = (
        "# --- Embedded shared driver code ---\n"
        + driver_code
        + "\n# --- End shared driver code ---\n"
    )

    output.write_text("".join(lines))

    print(f"Built {project}: {output.relative_to(ROOT)}")


if __name__ == "__main__":
    selected = sys.argv[1:]

    if not selected:
        selected = PROJECTS

    for project in selected:
        if project not in PROJECTS:
            raise ValueError(f"Unknown project: {project}")

        build(project)
