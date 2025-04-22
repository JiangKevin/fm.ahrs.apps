clear
reset
# 
export FmDev=$(pwd)
# 
rm -rf build
cmake -S . -B build 
cmake --build build -j10