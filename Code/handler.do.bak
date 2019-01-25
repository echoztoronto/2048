vlib work
vlog project_2048_game.v vga_adapter.v vga_address_translator.v vga_controller.v vga_pll.v
vsim datapath
log {/*}
add wave {/*}

	
#// Standard I/O
#input CLOCK_50,
#input resetn,

#input [3:0] sig_move,

#// Signals from control
#input sig_clearBoard,
#input sig_checkRandNum,
#input sig_spawnNumOnBoard,
#input sig_drawBoard,
#input sig_initDraw,
#input sig_gameDraw,
#input sig_gameEndDraw,
#input sig_resetIteration,
#input sig_iterationCheck,
#input sig_setCurrentPOS,
#input sig_setCurrentNextPOS,
#input sig_checkBound,
#input sig_calcMove,
#input sig_noMove,
#input sig_mergeUpdateNext,
#input sig_mergeUpdateCur,
#input sig_noMergeUpdateNext,
#input sig_noMergeUpdateCur,
#input sig_iterationIncre,
#input gameRAM_writeEn,
#input ld_randomNum,
#input ld_move,
#input ld_iterationCounter,
#input ld_gameBoard_cur_X,
#input ld_gameBoard_cur_Y,
#input ld_gameBoard_cur_Value,
#input ld_gameBoard_next_X,
#input ld_gameBoard_next_Y,

force -deposit /CLOCK_50 1 0, 0 {100 ps} -repeat 200

force resetn 1
run 1 ns
force resetn 0
run 1 ns
force resetn 1
run 2 ns
