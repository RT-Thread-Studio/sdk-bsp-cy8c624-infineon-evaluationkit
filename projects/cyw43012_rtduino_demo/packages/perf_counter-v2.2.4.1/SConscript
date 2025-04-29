Import('rtconfig')
from building import *
import os

cwd = GetCurrentDir()
src = ['perf_counter.c', os.path.join('os', 'perf_os_patch_rt_thread.c')]
path = [cwd]
CPPDEFINES = ['__perf_counter_printf__=rt_kprintf']

if GetDepend('PKG_PERF_COUNTER_USING_THREAD_STATISTIC'):
    CPPDEFINES += ['__PERF_CNT_USE_RTOS__']

CXXFLAGS = ''

if rtconfig.PLATFORM == 'armcc': # Keil AC5
    CXXFLAGS += ' --gnu'

group = DefineGroup('perf_counter', src, depend = ['PKG_USING_PERF_COUNTER'], CPPDEFINES = CPPDEFINES, CPPPATH = path, CXXFLAGS = CXXFLAGS)

Return('group')
