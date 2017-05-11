<!--udevadm info --query=all --name=sda-->   <!-- 查看端口ID_PATH -->
<!-- udevadm info -a -p $(udevadm info -q path -n /dev/video1)-->  <!-- 查看usb信息 -->

<!-- rule.d -->
<!--KERNEL=="video*", SUBSYSTEM=="video4linux", DRIVER=="",  ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:1:1.0", ATTR{index}=="0", ATTR{dev_debug}=="0", SYMLINK+="stereoRight"-->

<!--KERNEL=="video*", SUBSYSTEM=="video4linux", DRIVER=="",  ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:3:1.0", ATTR{index}=="0", ATTR{dev_debug}=="0", SYMLINK+="stereoLight"-->
