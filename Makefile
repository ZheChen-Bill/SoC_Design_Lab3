fir:
	rm -rf xsim.dir/ *.log *.pb *.jou *.wdb *.vcd
	xvlog -f ./include.rtl.list.xsim ./tb/fir_tb.v
	xelab -top fir_tb -snapshot fir_tb_elab
	xsim fir_tb_elab -R

clean:
	rm -rf xsim.dir/ *.log *.pb *.jou *.wdb *.vcd