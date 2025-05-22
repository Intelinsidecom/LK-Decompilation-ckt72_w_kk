#!/usr/local/bin/perl -w

#You can change $MBR_Start_Address_KB, the size equals the BOOT1+BOOT2+RPMB of the max of eMMC chips you want use.
#You Can write a formula rather than a number calculated by yourself
# $MBR_Start_Address_KB = 1024+1024+128; is right
# $MBR_Start_Address_KB = 6*1024+128; is right

# 1.Reserved more PRELOADER partition size(12MB) for later adding new compatible eMCP
#   flash materials, to keep the same PRELOADER partition size for OTA upgrading success.
# 2.MBR_Start_Address_KB = BOOT1+BOOT2+RPMB
# 3.MBR_Start_Address_KB max size in mediatek/build/tools/emigen/MT6582/MemoryDeviceList_MT6582.xls
#   is (4096 + 4096 + 4096 = 12MB) to check emmc_region sheet.
# 4.Potential problem: the work around will fail if new flash info is more than (4096 + 4096 + 4096)
#$MBR_Start_Address_KB = 6144;
$MBR_Start_Address_KB = 12288;

print "[Ptgen in module] MBR_Start_Address_KB = $MBR_Start_Address_KB\n";

return $MBR_Start_Address_KB;
