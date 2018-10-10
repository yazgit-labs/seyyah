# -*- coding: utf-8 -*-
'''
python 3.7 ve Linux 64 için miniconda indir :
https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh
Sonra indirdigin .sh dosyasını çalıştır
yönergeleri izleyerek kur
Kendi environmentini olustur :
conda create -n denemecv2 python
sonra gereken seyleri kur:
pip install opencv-python
pip install goprocam
pip install goprohero
pip install gopro
sonra once vava.py sonra lal.py calistir
'''
from goprocam import GoProCamera
from goprocam import constants

gopro = GoProCamera.GoPro()
#constants.Video.RESOLUTION = "13"
gopro.livestream("start")
gopro.livestream("stop")
gopro.video_settings("720p","30")
constants.Stream.WindowSize.R480
constants.Stream.BitRate.B4Mbps
#gopro.streamSettings(constants.Stream.BitRate.B4Mbps, constants.Stream.WindowSize.R480) #For HERO4



#constants.WINDOW_SIZE = "0"


#gopro.streamSettings("2400000","1")
#gpControlSet("62", 2400000)
#gpControlSet("64", 6)
gopro.stream("udp://127.0.0.1:10000")
