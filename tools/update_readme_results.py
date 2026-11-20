#!/usr/bin/env python3
"""Insert the latest benchmark and map-evaluation results into README.md.

Looks for:
  results/<run>/summary.md        (from analyze_results; newest run directory wins)
  results/map_eval/map_metrics.md (from compare_maps) and map_comparison.png
and replaces the block between <!-- RESULTS:START --> and <!-- RESULTS:END -->.
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
START, END = '<!-- RESULTS:START -->', '<!-- RESULTS:END -->'


def latest_summary():
    runs = [p for p in glob.glob(os.path.join(ROOT, 'results', '*', 'summary.md'))
            if os.sep + 'map_eval' + os.sep not in p]
    return max(runs, key=os.path.getmtime) if runs else None


def main():
    parts = []
    summary = latest_summary()
    if summary:
        rel_dir = os.path.relpath(os.path.dirname(summary), ROOT)
        text = open(summary).read()
        text = re.sub(r'^# .*\n', '', text, count=1)                       # drop H1
        text = re.sub(r'^## ', '### ', text, flags=re.M)                   # demote headings
        text = re.sub(r'!\[([^\]]*)\]\(([^)/]+\.png)\)',
                      lambda m: f'![{m.group(1)}]({rel_dir}/{m.group(2)})', text)
        parts.append(f'Run: `{rel_dir}`\n\n' + text.strip())

    map_md = os.path.join(ROOT, 'results', 'map_eval', 'map_metrics.md')
    if os.path.exists(map_md):
        block = '### SLAM map accuracy vs ground truth\n\n' + open(map_md).read().strip()
        if os.path.exists(os.path.join(ROOT, 'results', 'map_eval', 'map_comparison.png')):
            block += '\n\n![map comparison](results/map_eval/map_comparison.png)'
        parts.append(block)

    if not parts:
        sys.exit('no results found under results/ - run the benchmarks first')

    readme_path = os.path.join(ROOT, 'README.md')
    readme = open(readme_path).read()
    if START not in readme or END not in readme:
        sys.exit('README.md has no RESULTS markers')
    new = readme.split(START)[0] + START + '\n' + '\n\n'.join(parts) + '\n' + END + \
        readme.split(END)[1]
    open(readme_path, 'w').write(new)
    print('README.md results section updated')


if __name__ == '__main__':
    main()
