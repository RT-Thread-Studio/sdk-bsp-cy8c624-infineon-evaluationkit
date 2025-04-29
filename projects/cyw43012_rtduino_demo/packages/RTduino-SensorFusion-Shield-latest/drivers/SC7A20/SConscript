from building import *
import os

cwd     = GetCurrentDir()
src     = []
group   = []
CPPPATH = [cwd]

src += Glob('*.cpp')

group = group + DefineGroup('sc7a20', src, depend = ['PKG_RTDUINO_SENSORFUSION_SHIELD_SC7A20_DEMO'], CPPPATH = CPPPATH)

Return('group')
