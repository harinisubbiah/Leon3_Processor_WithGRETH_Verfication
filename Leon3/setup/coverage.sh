vhdlan -vhdl93 testbench.vhd
vcs -cm line+cond+fsm+tgl+branch -cm_dir /path/to/systest.vdb tb_entity_name
./simv -cm line+cond+fsm+tgl+branch -cm_dir /path/to/systest.vdb 
urg -dir /path/to/systest.vdb -format both
