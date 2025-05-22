# kernel_ckt72_w_kk-mt6572
 Kernel Sources for Doro Liberto 820 Mini and MT6572


Tutorial on how to set it up:

1. Open Terminal in the same directory as mk and makeMtk scripts

2. add toolchain to the PATH
         export PATH=/your directory/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin:$PATH

if something does not work and eabi is the fault also try
         export PATH=/your directory/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin:$PATH

3. Set up parameters
         export TARGET_PRODUCT=ckt72_w_kk

4. Now its time to rock
        ./makeMtk ckt72_w_kk n k

If it does not have permissions to execute give the file to do it, if you get errors in kernel building process give permissions for every folder and file in the folder or just execute GrantPerms.sh script with sudo

if it says "ld" is missing just install binutils and if "gcc" is missing or whatewer just install gcc

   Ubuntu example:

        sudo apt-get install binutils
        sudo apt-get install gcc

Have nice time ;)


   gdb'ing a non-running kernel currently fails because gdb (wrongly)
   disregards the starting offset for which the kernel is compiled.
