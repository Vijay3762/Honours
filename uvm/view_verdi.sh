#!/usr/bin/env bash
# =============================================================================
# view_verdi.sh - Launch Synopsys Verdi with dump.fsdb and serial signals
# =============================================================================

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
export PATH="${VERDI_HOME}/bin:${PATH}"

cd "${DIR}"

if [[ ! -f "dump.fsdb" ]]; then
  echo "Error: dump.fsdb not found in ${DIR}!"
  echo "Please run ./run.sh first to generate the simulation waveform."
  exit 1
fi

echo "Opening Verdi with dump.fsdb and design database..."
if [[ -d "simv.daidir" ]]; then
  verdi -simflow -dbdir simv.daidir -ssf dump.fsdb -sswr signals.rc &
else
  verdi -ssf dump.fsdb -sswr signals.rc &
fi
