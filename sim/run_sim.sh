#!/bin/sh
# Simulasi wrapper replay dengan Verilator (butuh: pip install verilator; g++ dengan C++20).
# Hasil: sim/log.txt; bandingkan dengan dataset lewat cg_replay.compare().
set -e
R=${RTL_DIR:-../../rtl}      # folder rtl/ dari rtl.zip (cg_pkg.sv, cg_top.sv, ...)
VB=$(python3 -c "import verilator,os;print(os.path.join(os.path.dirname(verilator.__file__),'bin'))")
export PATH=$PATH:$VB
python3 -c "
import sys; sys.path.insert(0,'../notebook'); import cg_replay as c
b=c.pack_stimulus('${DATASET_DIR:-../../dataset}')
open('stim.hex','w').write(''.join(' '.join(f'{int(x):08x}' for x in r)+'\n' for r in b))"
verilator-cli --binary --timing -CFLAGS -std=c++20 -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH -Wno-UNOPTFLAT \
  --top-module tb_replay --Mdir obj -o tb_replay \
  $R/cg_pkg.sv $R/cg_div.sv $R/cg_hdc_enc.sv $R/cg_m0_input.sv $R/cg_m1_preproc.sv $R/cg_m2_band_energy.sv \
  $R/cg_m3_rcontact.sv $R/cg_m4_thermal.sv $R/cg_m5a_ewma.sv $R/cg_m5b_hdc.sv $R/cg_m6_hdc_cls.sv \
  $R/cg_m7_fsm.sv $R/cg_m8_hardtrip.sv $R/cg_m9_regs.sv $R/cg_top.sv ../rtl/cg_replay_top.sv tb_replay.sv || true
# pada Verilator pip, flag PCH kosong: build ulang dengan override
make -C obj -f Vtb_replay.mk -j 4 CFG_CXXFLAGS_PCH_I=-include
./obj/tb_replay +in=stim.hex +out=log.txt
