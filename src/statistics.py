import matplotlib.pyplot as plt;
import matplotlib as mpl;
import numpy as np;
import pandas as pd;

# print(plt.rcParams.keys())
# plt.rcParams['font.sans-serif']=['Droid Sans Fallback']
plt.rcParams['axes.unicode_minus']=False

cascade=[]
fdt=[]
uf=[]

with open('cmake-build-debug/devel/lib/pi_robot/cascade.txt')as f:
    cascade=[float(x[0:len(x)-1])*1000 for x in f.readlines()]

with open('cmake-build-debug/devel/lib/pi_robot/fdt.txt')as f:
    fdt=[float(x[0:len(x)-1])*1000 for x in f.readlines()]

with open('cmake-build-debug/devel/lib/pi_robot/uf.txt')as f:
    uf=[float(x[0:len(x)-1])*1000 for x in f.readlines()]

data={
    'cascade':cascade,
    'libfacedetection':fdt,
    'ultraface':uf
}
df=pd.DataFrame(data)
print(df.describe())
df.boxplot(meanline=False,showmeans=True,patch_artist=True)
plt.grid(linestyle='--',alpha=1)

zhfont=mpl.font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')
plt.title(u'人脸检测算法性能箱型图',fontproperties=zhfont)
plt.xlabel(u'算法',fontproperties=zhfont)
plt.ylabel(u'时间 /ms',fontproperties=zhfont)
plt.legend()
plt.show()