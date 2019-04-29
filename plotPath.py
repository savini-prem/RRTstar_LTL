import matplotlib.pyplot as plt
import numpy as np

plt.axes()
fig = plt.figure()
ax = fig.gca()
#ax.set_xticks(np.arange(0, 1, 0.1))
#ax.set_yticks(np.arange(0, 1, 0.1))
ax.set_axisbelow(True)
#ax.yaxis.grid(color='gray', linestyle='dashed')

#plot environment
env = plt.Polygon([[0,1],[0,0],[1,0],[1,1]], fill=None, edgecolor='k')
plt.gca().add_patch(env)

#plot obstacles
obst1 = plt.Polygon([[0.3, 0.2], [0.3, 0.0], [0.7, 0.0], [0.7, 0.2]], color='olive', alpha=0.5)
obst2 = plt.Polygon([[0.4, 1.0], [0.4, 0.7], [0.6, 0.7], [0.6, 1.0]], color='olive', alpha=0.5)
plt.gca().add_patch(obst1)
plt.gca().add_patch(obst2)
plt.text(0.48, 0.08, 'o1', fontsize=10)
plt.text(0.48, 0.83, 'o2', fontsize=10)

#plot regions
reg1 = plt.Polygon([[0.1, 0.9], [0.1, 0.7], [0.3, 0.7]], color='blue', alpha=0.5)
reg2 = plt.Polygon([[0.7, 0.9], [0.7, 0.7], [0.9, 0.7]], color='blue', alpha=0.5)
reg3 = plt.Polygon([[0.7, 0.5], [0.7, 0.3], [0.9, 0.3]], color='blue', alpha=0.5)
reg4 = plt.Polygon([[0.3, 0.5], [0.3, 0.3], [0.5, 0.3]], color='blue', alpha=0.5)
reg5 = plt.Polygon([[0.0, 0.3], [0.0, 0.1], [0.2, 0.1]], color='blue', alpha=0.5)
reg6 = plt.Polygon([[0.0, 0.6], [0.0, 0.4], [0.2, 0.4]], color='blue', alpha=0.5)
plt.gca().add_patch(reg1)
plt.gca().add_patch(reg2)
plt.gca().add_patch(reg3)
plt.gca().add_patch(reg4)
plt.gca().add_patch(reg5)
plt.gca().add_patch(reg6)
plt.text(0.15, 0.75, 'l1', fontsize=10)
plt.text(0.75, 0.75, 'l2', fontsize=10)
plt.text(0.75, 0.35, 'l3', fontsize=10)
plt.text(0.35, 0.35, 'l4', fontsize=10)
plt.text(0.05, 0.15, 'l5', fontsize=10)
plt.text(0.05, 0.45, 'l6', fontsize=10)

#plot paths
n = input("number of robots? ")
#OPEN PATH FILE & PLOT PATHS
for j in range(2):
	for k in range(n):
		file = open('path'+str(k)+'-'+str(j)+'.txt', 'r') 
		color = ['firebrick','darkorange']

		while True: 
			line = file.readline()
			if not line: break;

			line = line.rstrip(' \n')
			x = line.split(' ')
			x = map(float,x)

			line = file.readline()
			line = line.rstrip(' \n')
			y = line.split(' ')
			y = map(float,y)

			if j==0:
				for i in range(len(x)-2): #skip plotting of last point
					plt.arrow(x[i+2],y[i+2],x[i+1]-x[i+2],y[i+1]-y[i+2], color= color[j], linewidth=1, head_width=0.02)
			if j==1: 
				for i in range(len(x)-1): 
					plt.arrow(x[i+1],y[i+1],x[i]-x[i+1],y[i]-y[i+1], color= color[j], linewidth=1, head_width=0.02)


#legend


plt.axis('scaled')
plt.grid(color='gray', linestyle='dashed')


fig = plt.gcf()
fig.savefig('plot.png')

plt.show()
