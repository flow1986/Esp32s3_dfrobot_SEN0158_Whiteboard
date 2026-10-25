import hashlib
from pathlib import Path

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
html = (project_dir / "data" / "index.html").read_text(encoding="utf-8")
delimiter = "W" + hashlib.sha256(html.encode("utf-8")).hexdigest()[:14]
header = (
    "#pragma once\n"
    "#include <Arduino.h>\n"
    f'const char WEB_UI_HTML[] PROGMEM = R"{delimiter}('
    + html
    + f'){delimiter}";\n'
)

output = project_dir / "include" / "web_ui.h"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(header, encoding="utf-8")