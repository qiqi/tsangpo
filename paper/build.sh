#!/usr/bin/env bash
# Reproducible build of the SciTech extended abstract.
#
# Run from anywhere:
#     bash paper/build.sh
#
# The script:
#   1. Regenerates per-phase sensitivity caches + plots for every config
#      (consumes the CFD digests already committed under post/out/).
#   2. Re-builds the equilibrium-grid table for v2 gap-low.
#   3. Re-renders every paper figure (airfoil stack, config montage,
#      combined-sens overlays, per-surface breakdown).
#   4. Runs pdflatex + bibtex + pdflatex + pdflatex to produce
#      paper/extended_abstract.pdf.
#
# Prerequisites (Debian/Ubuntu):
#     sudo apt-get install texlive-latex-base texlive-latex-extra \
#                          texlive-fonts-extra texlive-publishers \
#                          texlive-science
#     pip install numpy pandas pyvista pyyaml matplotlib
# The Flow360 SDK is NOT required — every step below reads the cached
# CSV / JSON files committed under post/out/, not the cloud.
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$REPO"

echo "=== 1. Refit per-phase sensitivities from cached sweep CSVs ==="
for fam in v2_continuous v2_continuous_high v2_gapped v2_gapped_high v3; do
    for ph in cruise takeoff landing; do
        if [[ -f "post/$fam/plot_${ph}_sensitivities.py" ]]; then
            python3 "post/$fam/plot_${ph}_sensitivities.py" \
                | tail -3
        fi
    done
done

echo
echo "=== 2. Equilibrium grid for gap-low ==="
python3 post/v2_gapped/equilibrium_grid.py | tail -3

echo
echo "=== 3. Re-render paper figures ==="
python3 paper/figures/_build_airfoil_stack.py
python3 paper/figures/_build_config_montage.py
python3 paper/figures/_build_combined_sens.py | tail -6
# _per_surface_breakdown.py and _retrieve_slices.py / _plot_slice.py
# pull from the cloud and are NOT part of the offline build pipeline.

echo
echo "=== 4. LaTeX build ==="
cd paper
pdflatex -interaction=nonstopmode extended_abstract.tex > /dev/null
bibtex extended_abstract                                > /dev/null || true
pdflatex -interaction=nonstopmode extended_abstract.tex > /dev/null
pdflatex -interaction=nonstopmode extended_abstract.tex > /dev/null
rm -f extended_abstract.{aux,log,bbl,blg,out}
echo "Output: $(pwd)/extended_abstract.pdf"
ls -la extended_abstract.pdf
