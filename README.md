# Modded-LK-ckt72_w_kk
 Little kernel modification attempt for Doro Liberto 820 Mini.

STATUS: Not booting to android and showing logo but fastboot seems to work, maybe other things work too, needs more testing.

All of this code is Mediatek Inc. Property.

Tutorial on how to set it up:

1. Open Terminal in the same directory as mk and makeMtk scripts and type this command
        ./makeMtk ckt72_w_kk n lk

If it does not have permissions to execute give the file to do it, if you get errors in kernel building process give permissions for every folder and file in the folder or just execute GrantPerms.sh script with sudo

if it says "ld" is missing just install binutils and if "gcc" is missing just install gcc

   Ubuntu example:

        sudo apt-get install binutils
        sudo apt-get install gcc

Have nice time ;)


   gdb'ing a non-running kernel currently fails because gdb (wrongly)
   disregards the starting offset for which the kernel is compiled.
