vlib work
vlog project_2048_game.v vga_adapter.v vga_address_translator.v vga_controller.v vga_pll.v
vsim control
log {/*}
add wave {/*}

#  // Standard I/O
#	input CLOCK_50,
#	input resetn,
	
#	// Game control inputs
#	input [3:0] sig_move,
	
#	// Signals from datapath
#	input sig_clearBoard_DONE,
#	input sig_randNum_GOOD,
#	input sig_drawBoard_DONE,
#	input sig_doneProcess,
#	input sig_toNoMove,
#	input sig_toMergeMove,
#	input sig_toJustMove,
#	input sig_nextIteration,

force -deposit /CLOCK_50 1 0, 0 {100 ps} -repeat 200

force sig_move 					0000
force sig_clearBoard_DONE 		0
force sig_randNum_GOOD 			0
force sig_drawBoard_DONE		0
force sig_doneProcess			0
force sig_toNoMove				0
force sig_toMergeMove			0
force sig_toJustMove				0
force sig_nextIteration			0

force resetn 1
run 1 ns
force resetn 0
run 1 ns
force resetn 1
run 2 ns

force sig_clearBoard_DONE 		1
run 1 ns
force sig_clearBoard_DONE 		0
run 1 ns

force sig_randNum_GOOD 		1
run 1 ns
force sig_randNum_GOOD 		0
run 1 ns

force sig_drawBoard_DONE 		1
run 1 ns
force sig_drawBoard_DONE 		0
run 1 ns

force sig_move 		0001
run 1 ns
force sig_move 		0000
run 10 ns

force sig_nextIteration 1
run 1 ns
force sig_nextIteration 0
run 12 ns

force sig_nextIteration 1
force sig_doneProcess			1
run 1 ns
force sig_nextIteration 0
run 1 ns
force sig_doneProcess			0
run 10 ns

force sig_randNum_GOOD 		1
run 1 ns
force sig_randNum_GOOD 		0
run 10 ns

force sig_drawBoard_DONE 		1
run 1 ns
force sig_drawBoard_DONE 		0
run 10 ns


