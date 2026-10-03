# Run after opening core.wlf, or during a live small-frame simulation.
# Set avm_wave_dataset explicitly if multiple datasets are open.
if {![info exists avm_wave_dataset]} {
    set avm_wave_dataset [lindex [dataset list] 0]
}
set avm_wave_root "${avm_wave_dataset}:/tb_avm_frame"
add wave -divider {Frame and memory}
add wave $avm_wave_root/clk $avm_wave_root/rst_n $avm_wave_root/start
add wave $avm_wave_root/busy $avm_wave_root/done
add wave -radix hexadecimal $avm_wave_root/req_addr $avm_wave_root/req_len
add wave $avm_wave_root/req_valid $avm_wave_root/req_ready
add wave $avm_wave_root/rsp_valid $avm_wave_root/rsp_ready $avm_wave_root/rsp_last
add wave -divider {Raster output}
add wave $avm_wave_root/m_valid $avm_wave_root/m_ready $avm_wave_root/m_sof $avm_wave_root/m_eol
add wave -radix hexadecimal $avm_wave_root/m_pixel
add wave $avm_wave_root/config_version $avm_wave_root/count
configure wave -namecolwidth 240
wave zoom full
