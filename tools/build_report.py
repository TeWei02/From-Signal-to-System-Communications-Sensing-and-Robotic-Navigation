#!/usr/bin/env python3
"""Convert report/report.tex into docs/report.html.

Only the constructs actually used in the report are handled: sections,
subsections, emphasis, inline maths, and paragraphs. No external assets are
pulled in, so the page keeps working offline like the rest of docs/.
"""

from __future__ import annotations

import html
import re
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TEX = REPO / "report" / "report.tex"
OUT = REPO / "docs" / "report.html"

MATH_REPLACEMENTS = [
    (r"\\text\{([^{}]*)\}", r"\1"),
    (r"\\mathbf\{([^{}]*)\}", r"\1"),
    (r"\\mathrm\{([^{}]*)\}", r"\1"),
    (r"\\mathbb\{([^{}]*)\}", r"\1"),
    (r"\\frac\{([^{}]*)\}\{([^{}]*)\}", r"\1/\2"),
    (r"\\mid", "|"),
    (r"\\propto", "\u221d"),
    (r"\\times", "\u00d7"),
    (r"\\approx", "\u2248"),
    (r"\\leq", "\u2264"),
    (r"\\geq", "\u2265"),
    (r"\\lambda", "\u03bb"),
    (r"\\alpha", "\u03b1"),
    (r"\\beta", "\u03b2"),
    (r"\\mu", "\u03bc"),
    (r"\\Delta", "\u0394"),
    (r"\\tau", "\u03c4"),
    (r"\\sigma", "\u03c3"),
    (r"\\pi", "\u03c0"),
    (r"\\ell", "\u2113"),
    (r"\\,", " "),
    (r"\\;", " "),
    (r"\\!", ""),
    (r"\\ ", " "),
    (r"\{,\}", ","),
    (r"\\left", ""),
    (r"\\right", ""),
]


def clean_math(s: str) -> str:
    for pattern, replacement in MATH_REPLACEMENTS:
        s = re.sub(pattern, replacement, s)
    s = re.sub(r"\s+", " ", s).strip()
    return s


def inline(s: str) -> str:
    s = html.escape(s, quote=False)
    s = re.sub(r"\\textbf\{([^{}]*)\}", r"<strong>\1</strong>", s)
    s = re.sub(r"\\emph\{([^{}]*)\}", r"<em>\1</em>", s)
    s = re.sub(r"\\texttt\{([^{}]*)\}", r"<code>\1</code>", s)
    s = re.sub(r"\\url\{([^{}]*)\}", r'<a href="\1">\1</a>', s)
    s = s.replace("---", "\u2014").replace("--", "\u2013")
    s = s.replace("~", " ")
    s = re.sub(r"\s+", " ", s)
    return s.strip()


def build() -> str:
    text = TEX.read_text(encoding="utf-8")

    title = re.search(r"\\title\{(.*?)\}\n", text, re.S)
    title = inline(title.group(1)) if title else "From Signal to System"
    author = re.search(r"\\author\{(.*?)\}", text)
    author = inline(author.group(1)) if author else ""

    # pull the abstract out before anything else
    abstract = ""
    match = re.search(r"\\begin\{abstract\}(.*?)\\end\{abstract\}", text, re.S)
    if match:
        abstract = match.group(1)
        text = text[: match.start()] + text[match.end():]

    # inline maths, which may span lines
    maths: list[str] = []

    def stash(m):
        maths.append(clean_math(m.group(1)))
        return f"\x00{len(maths) - 1}\x00"

    text = re.sub(r"\\\((.*?)\\\)", stash, text, flags=re.S)

    # drop comments and the preamble
    text = re.sub(r"^%.*$", "", text, flags=re.M)
    text = re.sub(r"\\documentclass.*?\n", "", text, count=1)
    text = re.sub(r"\\usepackage.*?\n", "", text)
    text = re.sub(r"\\title\{.*?\}\n", "", text, flags=re.S)
    text = re.sub(r"\\author\{.*?\}\n", "", text)
    text = re.sub(r"\\date\{.*?\}\n", "", text)
    text = re.sub(r"\\(maketitle|newcommand\{[^{}]*\}\{[^{}]*\})", "", text)

    blocks: list[str] = []
    paragraph: list[str] = []

    def flush():
        if paragraph:
            joined = " ".join(x.strip() for x in paragraph if x.strip())
            if joined:
                blocks.append(f"<p>{inline(joined)}</p>")
            paragraph.clear()

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            flush()
            continue
        sec = re.match(r"\\section\{(.*)\}", line)
        sub = re.match(r"\\subsection\*?\{(.*)\}", line)
        if sec:
            flush()
            blocks.append(f"<h2 id=\"s-{len(blocks)}\">{inline(sec.group(1))}</h2>")
            continue
        if sub:
            flush()
            blocks.append(f"<h3>{inline(sub.group(1))}</h3>")
            continue
        paragraph.append(line)
    flush()

    body = "\n".join(blocks)
    body = re.sub(
        r"\x00(\d+)\x00",
        lambda m: '<code class="math">' + html.escape(maths[int(m.group(1))]) + "</code>",
        body,
    )

    abstract_html = ""
    if abstract:
        joined = " ".join(x.strip() for x in abstract.splitlines() if x.strip())
        abstract_html = (
            '<section class="card tight abstract">\n<h2>Abstract</h2>\n'
            f"<p>{inline(joined)}</p>\n</section>\n"
        )

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<meta name="description" content="Design study on the information flow from raw sensor signals to robotic navigation: communications, sensing and on-board computation as one system.">
<meta name="theme-color" content="#0b111c">
<link rel="manifest" href="manifest.json">
<link rel="icon" href="icons/icon-192.png" sizes="192x192">
<link rel="apple-touch-icon" href="icons/icon-192.png">
<link rel="stylesheet" href="styles.css">
</head>
<body class="report">
<header class="hero">
  <div class="hero-inner">
    <div class="eyebrow"><a href="index.html">&larr; From Signal to System</a></div>
    <h1>{title}</h1>
    <p class="lede">{author}</p>
    <div class="tagrow">
      <span class="tag warn">設計研究：案例為假設情境，無量測數據</span>
      <span class="tag">對應 LaTeX 原始檔 report/report.tex</span>
    </div>
  </div>
</header>
<div class="wrap">
{abstract_html}{body}
<footer>
  {author} &middot; MIT License &middot;
  <a href="index.html">回到互動模型</a> &middot;
  <a href="https://github.com/TeWei02/From-Signal-to-System-Communications-Sensing-and-Robotic-Navigation">GitHub</a>
</footer>
</div>
</body>
</html>
"""


if __name__ == "__main__":
    html_text = build()
    OUT.write_text(html_text, encoding="utf-8")
    print(f"wrote {OUT} ({OUT.stat().st_size} bytes)")
