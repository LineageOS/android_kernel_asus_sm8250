MOD=snt8100fsr
make clean
sudo rmmod ${MOD} &> /dev/null;
rm -f ${MOD}.ko && make -j2 && sudo insmod ${MOD}.ko

