#! /bin/bash
exe="XSTracking" #发布的程序名称
des="paks" #创建文件夹的位置
build="build" 
mkdir -p $des
deplist=$(ldd ./$build/$exe | awk  '{if (match($3,"/")){ printf("%s "),$3 } }')  
cp $deplist $des
cp "./$build/$exe" $des

cd $des && rm -rf libc.so* libdl.so* libpthread.so* librt.so*
