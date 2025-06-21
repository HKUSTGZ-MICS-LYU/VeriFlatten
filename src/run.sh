DIR=/home/ziyue/researchlib/Micro_Eletronic/VeriCodegen/bench/cascade_picorv32/pico.v
python3 main.py --file $DIR --top picorv32_mem_top --output $(dirname "$DIR")/f_pico.v
