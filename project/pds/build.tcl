# Run in the disposable prj directory created by tools/avm/run_pds.py.
set_arch -family Logos2 -device PG2L50H -speedgrade -6 -package FBG484
foreach avm_source [lsort [glob ../rtl/*.v]] {
    add_design -verilog $avm_source
}
add_constraint -logic -fdc avm_core.fdc
compile -top_module avm_core -defined_parameters {WIDTH 1280 HEIGHT 720 CACHE_PREFETCH_TILES 2}
set avm_synth_options {} ;# @SYNTH_OPTIONS@
eval synthesize -ads -selected_syn_tool_opt 2 -disable_io_insertion -frequency 100 -max_path 20 -nworst 2 $avm_synth_options
