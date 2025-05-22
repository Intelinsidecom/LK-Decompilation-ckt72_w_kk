#!/usr/bin/perl
($#ARGV != 0) && &Usage;
$prj = $ARGV[0];
$prjmk = "mediatek/config/${prj}/ProjectConfig.mk";
$prjmk = "mediatek/config/common/ProjectConfig.mk";

print "\n";
print "# begin mediatek build properties\n";
#pengfei.zhong-HOPE-23-porting from Doro820-Version number for factory
foreach $prjmk ("mediatek/config/common/ProjectConfig.mk", "mediatek/config/${prj}/ProjectConfig.mk","mediatek/config/${prj}/ProjectConfig_ckt.mk") {
  if (!-e $prjmk) {
#pengfei.zhong-HOPE-23-porting from Doro820-Version number for factory-start
    #die "#### Can't find $prjmk\n";
   unless ($_ eq "mediatek/config/${prj}/ProjectConfig_ckt.mk")
  	{
    	die "#### Can't find $prjmk\n";
    }
#pengfei.zhong-HOPE-23-porting from Doro820-Version number for factory-end
  } else {
    open (FILE_HANDLE, "<$prjmk") or die "cannot open $prjmk\n";
    while (<FILE_HANDLE>) {
      if (/^(\S+)\s*=\s*(\S+)/) {
        $$1 = $2;
      }
    }
    close FILE_HANDLE;
  }
}
#pengfei.zhong-HOPE-23-Internal version number for factory
#print "ro.mediatek.version.release=$MTK_BUILD_VERNO\n";
print "ro.mediatek.version.release=$ENV{CKT_BUILD_VERNO}\n";
print "ro.mediatek.platform=$MTK_PLATFORM\n";
print "ro.mediatek.chip_ver=$MTK_CHIP_VER\n";
print "ro.mediatek.version.branch=$MTK_BRANCH\n";
print "ro.mediatek.version.sdk=$PLATFORM_MTK_SDK_VERSION\n";
print "# end mediatek build properties\n";

exit 0;

