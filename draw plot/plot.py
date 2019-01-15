import matplotlib.pyplot as plt 
import numpy as np 

Y2016 = [ 4835291, 30324122, 66713068, 120083522]
Y2017 = [ 11245578, 71804071, 154378752, 259356303]
labels = [ 'ColorPoint', '10 times', '25 times', '50 times']
bar_width = 0.45


plt.bar(np.arange( 4), Y2016, label = 'Dandi', 
color = 'steelblue', alpha = 0.8, width = bar_width)
plt.bar(np.arange( 4)+bar_width, Y2017, label = 'Ali', 
color = 'indianred', alpha = 0.8, width = bar_width) 

plt.ylabel( 'points') 

plt.xticks(np.arange( 5)+bar_width,labels) 

plt.ylim([ 2000000, 400000000]) 

for x2016,y2016 in enumerate(Y2016): 
	plt.text(x2016, y2016+ 100, '%s'%y2016) 
for x2017,y2017 in enumerate(Y2017): 
	plt.text(x2017+bar_width, y2017+ 100, '%s'%y2017) 

plt.legend()

plt.show() 
